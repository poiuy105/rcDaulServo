/**
 * @file radar_ld2452.c
 * @brief HLK-LD2452 高精度多目标识别传感器驱动实现
 *
 * 1T2R天线，纯上报型雷达，最多3个目标
 * 默认波特率: 9600
 * 帧格式: AA FF 03 00 [targets×3] 55 CC (30字节固定长度)
 *
 * 坐标编码: 自定义符号位（非标准补码）
 *   最高位1=正数，其余15位为绝对值
 *   最高位0=负数，其余15位为绝对值（需取反）
 */

#include "radar_ld2452.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "LD2452";

/* 事件基础定义 */
ESP_EVENT_DEFINE_BASE(LD2452_EVENT);

/* 设备结构 */
struct ld2452_dev {
    uart_port_t uart_num;
    int baud_rate;
    TaskHandle_t task_handle;
    esp_event_loop_handle_t event_loop;
};

/* 帧缓冲区 */
#define FRAME_BUF_SIZE 64

/**
 * @brief 解析LD2452自定义符号位编码为标准int16
 *
 * 编码规则：
 *   最高位(bit15)=1 → 正数，值 = raw & 0x7FFF
 *   最高位(bit15)=0 → 负数，值 = -(raw & 0x7FFF)
 */
static int16_t parse_signed_value(uint16_t raw)
{
    if (raw & 0x8000) {
        return (int16_t)(raw & 0x7FFF);
    } else {
        return -(int16_t)(raw & 0x7FFF);
    }
}

/**
 * @brief 从字节流中解析一个目标（8字节，小端）
 */
static void parse_target(const uint8_t *buf, ld2452_target_t *target)
{
    /* 小端读取原始值 */
    uint16_t x_raw     = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    uint16_t y_raw     = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t speed_raw = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    uint16_t dist_raw  = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

    /* 解析自定义符号位 */
    target->x        = parse_signed_value(x_raw) / 1000.0f;      /* mm → m */
    target->y        = parse_signed_value(y_raw) / 1000.0f;      /* mm → m */
    target->speed    = parse_signed_value(speed_raw) / 100.0f;   /* cm/s → m/s */
    target->distance = dist_raw / 1000.0f;                       /* mm → m */
}

/**
 * @brief 检查目标是否有效（非全零）
 */
static bool is_target_valid(const uint8_t *buf)
{
    for (int i = 0; i < LD2452_TARGET_DATA_SIZE; i++) {
        if (buf[i] != 0x00) {
            return true;
        }
    }
    return false;
}

/**
 * @brief UART接收任务 - 状态机解析固定长度帧
 */
static void ld2452_uart_task(void *arg)
{
    ld2452_handle_t dev = (ld2452_handle_t)arg;
    uint8_t buf[FRAME_BUF_SIZE];
    int pos = 0;

    while (1) {
        uint8_t ch;
        int len = uart_read_bytes(dev->uart_num, &ch, 1,
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        /* 帧头检测: AA FF 03 00 */
        if (pos == 0 && ch != LD2452_FRAME_HEADER_0) {
            continue;
        }
        if (pos == 1 && ch != LD2452_FRAME_HEADER_1) {
            pos = 0;
            if (ch == LD2452_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }
        if (pos == 2 && ch != LD2452_FRAME_HEADER_2) {
            pos = 0;
            if (ch == LD2452_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }
        if (pos == 3 && ch != LD2452_FRAME_HEADER_3) {
            pos = 0;
            if (ch == LD2452_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }

        buf[pos++] = ch;

        /* 收满一帧 (30字节) */
        if (pos >= LD2452_FRAME_SIZE) {
            /* 验证帧尾 */
            if (buf[LD2452_FRAME_SIZE - 2] == LD2452_FRAME_TAIL_0 &&
                buf[LD2452_FRAME_SIZE - 1] == LD2452_FRAME_TAIL_1) {

                ld2452_data_t data = {
                    .target_count = 0
                };

                /* 解析3个目标 */
                for (int i = 0; i < LD2452_MAX_TARGETS; i++) {
                    uint8_t *target_buf = &buf[4 + i * LD2452_TARGET_DATA_SIZE];
                    if (is_target_valid(target_buf)) {
                        parse_target(target_buf, &data.targets[data.target_count]);
                        data.target_count++;
                    }
                }

                /* 发布事件 */
                ld2452_event_t event = {
                    .id = LD2452_EVENT_TARGET_UPDATE,
                    .data = data
                };
                esp_event_post_to(dev->event_loop, LD2452_EVENT,
                                  event.id, &event.data,
                                  sizeof(event.data), portMAX_DELAY);
            }

            pos = 0;
        }

        /* 缓冲区溢出保护 */
        if (pos >= FRAME_BUF_SIZE) {
            pos = 0;
        }
    }
}

/* 初始化 */
ld2452_handle_t ld2452_init(const ld2452_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    ld2452_handle_t dev = calloc(1, sizeof(struct ld2452_dev));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate device");
        return NULL;
    }

    dev->uart_num = config->uart_num;
    dev->baud_rate = config->baud_rate;

    /* 配置UART */
    uart_config_t uart_cfg = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_param_config(config->uart_num, &uart_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        free(dev);
        return NULL;
    }

    if (uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        free(dev);
        return NULL;
    }

    if (uart_driver_install(config->uart_num, LD2452_UART_BUF_SIZE * 2,
                            0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        free(dev);
        return NULL;
    }

    /* 创建事件循环 */
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "ld2452_evt",
        .task_priority = 3,
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY,
    };

    if (esp_event_loop_create(&loop_args, &dev->event_loop) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop");
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    /* 创建接收任务 */
    if (xTaskCreate(ld2452_uart_task, "ld2452_uart", LD2452_TASK_STACK_SIZE,
                    dev, LD2452_TASK_PRIORITY, &dev->task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        esp_event_loop_delete(dev->event_loop);
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    ESP_LOGI(TAG, "LD2452 initialized, baud=%d", config->baud_rate);
    return dev;
}

/* 反初始化 */
esp_err_t ld2452_deinit(ld2452_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->task_handle != NULL) {
        vTaskDelete(handle->task_handle);
    }

    if (handle->event_loop != NULL) {
        esp_event_loop_delete(handle->event_loop);
    }

    uart_driver_delete(handle->uart_num);
    free(handle);

    ESP_LOGI(TAG, "LD2452 deinitialized");
    return ESP_OK;
}

/* 注册事件处理函数 */
esp_err_t ld2452_add_handler(ld2452_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *event_handler_arg)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(handle->event_loop,
                                           LD2452_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, event_handler_arg);
}

/* 注销事件处理函数 */
esp_err_t ld2452_remove_handler(ld2452_handle_t handle,
                                 esp_event_handler_t event_handler)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(handle->event_loop,
                                             LD2452_EVENT, ESP_EVENT_ANY_ID,
                                             event_handler);
}
