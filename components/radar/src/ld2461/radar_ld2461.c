/**
 * @file radar_ld2461.c
 * @brief HLK-LD2461 运动目标探测跟踪模组驱动实现
 *
 * 2T4R天线配置，支持最多3块区域过滤
 * 默认波特率: 9600
 * 数据格式: 大端模式
 */

#include "radar_ld2461.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "LD2461";

/* 事件基础定义 */
ESP_EVENT_DEFINE_BASE(LD2461_EVENT);

/* 设备结构 */
struct ld2461_dev {
    uart_port_t uart_num;
    int baud_rate;
    TaskHandle_t task_handle;
    SemaphoreHandle_t cmd_sem;
    uint8_t cmd_response[64];
    uint8_t cmd_response_len;
    bool cmd_waiting;
    esp_event_loop_handle_t event_loop;
};

/* 帧缓冲区 */
#define FRAME_BUF_SIZE 128

/* 辅助函数：计算校验和 */
static uint8_t calc_checksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* 辅助函数：发送命令帧 */
static esp_err_t send_command(ld2461_handle_t dev, uint8_t cmd,
                               const uint8_t *value, uint16_t value_len,
                               uint32_t timeout_ms)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[FRAME_BUF_SIZE];
    uint16_t pos = 0;

    /* 帧头 */
    frame[pos++] = LD2461_FRAME_HEADER_0;
    frame[pos++] = LD2461_FRAME_HEADER_1;
    frame[pos++] = LD2461_FRAME_HEADER_2;

    /* 数据长度（大端）= 命令字(1) + 命令值(N) */
    uint16_t data_len = 1 + value_len;
    frame[pos++] = (data_len >> 8) & 0xFF;
    frame[pos++] = data_len & 0xFF;

    /* 命令字 */
    frame[pos++] = cmd;

    /* 命令值 */
    if (value != NULL && value_len > 0) {
        memcpy(&frame[pos], value, value_len);
        pos += value_len;
    }

    /* 校验和（命令字 + 命令值） */
    uint8_t checksum_data[1 + 32];
    checksum_data[0] = cmd;
    if (value != NULL && value_len > 0) {
        memcpy(&checksum_data[1], value, value_len);
    }
    frame[pos++] = calc_checksum(checksum_data, 1 + value_len);

    /* 帧尾 */
    frame[pos++] = LD2461_FRAME_TAIL_0;
    frame[pos++] = LD2461_FRAME_TAIL_1;
    frame[pos++] = LD2461_FRAME_TAIL_2;

    /* 发送 */
    dev->cmd_waiting = true;
    dev->cmd_response_len = 0;

    uart_write_bytes(dev->uart_num, frame, pos);

    /* 等待响应 */
    if (xSemaphoreTake(dev->cmd_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        dev->cmd_waiting = false;
        ESP_LOGE(TAG, "Command 0x%02X timeout", cmd);
        return ESP_ERR_TIMEOUT;
    }

    dev->cmd_waiting = false;
    return ESP_OK;
}

/* 辅助函数：解析响应 */
static esp_err_t parse_response(const uint8_t *resp, uint8_t resp_len,
                                 uint8_t expected_cmd,
                                 uint8_t *out_data, uint8_t *out_len)
{
    if (resp_len < 9) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 检查帧头 */
    if (resp[0] != LD2461_FRAME_HEADER_0 ||
        resp[1] != LD2461_FRAME_HEADER_1 ||
        resp[2] != LD2461_FRAME_HEADER_2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 检查帧尾 */
    if (resp[resp_len - 3] != LD2461_FRAME_TAIL_0 ||
        resp[resp_len - 2] != LD2461_FRAME_TAIL_1 ||
        resp[resp_len - 1] != LD2461_FRAME_TAIL_2) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 获取数据长度 */
    uint16_t data_len = ((uint16_t)resp[3] << 8) | resp[4];

    /* 检查命令字 */
    uint8_t cmd = resp[5];
    if (cmd != expected_cmd) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* 验证校验和 */
    uint8_t calc_sum = calc_checksum(&resp[5], data_len);
    uint8_t recv_sum = resp[resp_len - 4];
    if (calc_sum != recv_sum) {
        ESP_LOGW(TAG, "Checksum mismatch: calc=0x%02X, recv=0x%02X", calc_sum, recv_sum);
        return ESP_ERR_INVALID_CRC;
    }

    /* 提取返回数据 */
    uint8_t ret_len = data_len - 1; /* 减去命令字 */
    if (out_data != NULL && out_len != NULL) {
        if (ret_len > *out_len) {
            ret_len = *out_len;
        }
        memcpy(out_data, &resp[6], ret_len);
        *out_len = ret_len;
    }

    return ESP_OK;
}

/* 解析上报帧 - 坐标数据 */
static void parse_coordinate_report(ld2461_handle_t dev, const uint8_t *data, uint8_t len)
{
    ld2461_event_t event = {
        .type = LD2461_EVENT_COORDINATES,
        .data.has_coordinate_data = true,
        .data.has_status_data = false,
        .data.target_count = 0
    };

    /* 每个目标2字节（X+Y） */
    uint8_t target_count = len / 2;
    if (target_count > 8) {
        target_count = 8;
    }

    for (uint8_t i = 0; i < target_count; i++) {
        int8_t x_raw = (int8_t)data[i * 2];
        int8_t y_raw = (int8_t)data[i * 2 + 1];

        event.data.targets[i].x = x_raw / 10.0f;
        event.data.targets[i].y = y_raw / 10.0f;
        event.data.target_count++;
    }

    /* 发布事件 */
    esp_event_post_to(dev->event_loop, LD2461_EVENT, event.type,
                      &event.data, sizeof(event.data), portMAX_DELAY);
}

/* 解析上报帧 - 区域状态 */
static void parse_status_report(ld2461_handle_t dev, const uint8_t *data, uint8_t len)
{
    if (len < 3) {
        return;
    }

    ld2461_event_t event = {
        .type = LD2461_EVENT_REGION_STATUS,
        .data.has_coordinate_data = false,
        .data.has_status_data = true,
        .data.target_count = 0
    };

    event.data.status.region1_occupied = (data[0] != 0);
    event.data.status.region2_occupied = (data[1] != 0);
    event.data.status.region3_occupied = (data[2] != 0);

    /* 发布事件 */
    esp_event_post_to(dev->event_loop, LD2461_EVENT, event.type,
                      &event.data, sizeof(event.data), portMAX_DELAY);
}

/* UART接收任务 */
static void ld2461_uart_task(void *arg)
{
    ld2461_handle_t dev = (ld2461_handle_t)arg;
    uint8_t buf[FRAME_BUF_SIZE];
    int pos = 0;

    while (1) {
        uint8_t ch;
        int len = uart_read_bytes(dev->uart_num, &ch, 1,
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        /* 帧头检测 */
        if (pos == 0 && ch != LD2461_FRAME_HEADER_0) {
            continue;
        }
        if (pos == 1 && ch != LD2461_FRAME_HEADER_1) {
            pos = 0;
            if (ch == LD2461_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }
        if (pos == 2 && ch != LD2461_FRAME_HEADER_2) {
            pos = 0;
            if (ch == LD2461_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }

        buf[pos++] = ch;

        /* 至少收到长度字段 */
        if (pos >= 5) {
            uint16_t data_len = ((uint16_t)buf[3] << 8) | buf[4];
            uint16_t frame_len = 3 + 2 + data_len + 1 + 3; /* 头+长度+数据+校验+尾 */

            if (pos >= frame_len) {
                /* 检查帧尾 */
                if (buf[frame_len - 3] == LD2461_FRAME_TAIL_0 &&
                    buf[frame_len - 2] == LD2461_FRAME_TAIL_1 &&
                    buf[frame_len - 1] == LD2461_FRAME_TAIL_2) {

                    uint8_t cmd = buf[5];

                    /* 命令响应 */
                    if (dev->cmd_waiting) {
                        memcpy(dev->cmd_response, buf, frame_len);
                        dev->cmd_response_len = frame_len;
                        xSemaphoreGive(dev->cmd_sem);
                    }
                    /* 主动上报 - 坐标数据 */
                    else if (cmd == LD2461_CMD_REPORT_COORD) {
                        uint8_t coord_len = data_len - 1;
                        parse_coordinate_report(dev, &buf[6], coord_len);
                    }
                    /* 主动上报 - 区域状态 */
                    else if (cmd == LD2461_CMD_REPORT_STATUS) {
                        uint8_t status_len = data_len - 1;
                        parse_status_report(dev, &buf[6], status_len);
                    }
                }

                pos = 0;
            }
        }

        /* 缓冲区溢出保护 */
        if (pos >= FRAME_BUF_SIZE) {
            pos = 0;
        }
    }
}

/* 初始化 */
ld2461_handle_t ld2461_init(const ld2461_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    ld2461_handle_t dev = calloc(1, sizeof(struct ld2461_dev));
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

    if (uart_driver_install(config->uart_num, LD2461_UART_BUF_SIZE * 2,
                            0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        free(dev);
        return NULL;
    }

    /* 创建信号量 */
    dev->cmd_sem = xSemaphoreCreateBinary();
    if (dev->cmd_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    /* 创建事件循环 */
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "ld2461_evt",
        .task_priority = 3,
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY,
    };

    if (esp_event_loop_create(&loop_args, &dev->event_loop) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop");
        vSemaphoreDelete(dev->cmd_sem);
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    /* 创建接收任务 */
    if (xTaskCreate(ld2461_uart_task, "ld2461_uart", LD2461_TASK_STACK_SIZE,
                    dev, LD2461_TASK_PRIORITY, &dev->task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        esp_event_loop_delete(dev->event_loop);
        vSemaphoreDelete(dev->cmd_sem);
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    ESP_LOGI(TAG, "LD2461 initialized, baud=%d", config->baud_rate);
    return dev;
}

/* 反初始化 */
esp_err_t ld2461_deinit(ld2461_handle_t handle)
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

    if (handle->cmd_sem != NULL) {
        vSemaphoreDelete(handle->cmd_sem);
    }

    uart_driver_delete(handle->uart_num);
    free(handle);

    ESP_LOGI(TAG, "LD2461 deinitialized");
    return ESP_OK;
}

/* 注册事件处理函数 */
esp_err_t ld2461_add_handler(ld2461_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *event_handler_arg)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(handle->event_loop,
                                           LD2461_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, event_handler_arg);
}

/* 注销事件处理函数 */
esp_err_t ld2461_remove_handler(ld2461_handle_t handle,
                                 esp_event_handler_t event_handler)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(handle->event_loop,
                                             LD2461_EVENT, ESP_EVENT_ANY_ID,
                                             event_handler);
}

/* 设置波特率 */
esp_err_t ld2461_set_baud_rate(ld2461_handle_t handle, int baud_rate)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 波特率编码（大端） */
    uint8_t baud_value[3];
    baud_value[0] = (baud_rate >> 16) & 0xFF;
    baud_value[1] = (baud_rate >> 8) & 0xFF;
    baud_value[2] = baud_rate & 0xFF;

    esp_err_t ret = send_command(handle, LD2461_CMD_BAUD_RATE,
                                  baud_value, 3, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[4];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_BAUD_RATE, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 1 && resp_data[0] == 1) {
        ESP_LOGI(TAG, "Baud rate set to %d", baud_rate);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* 查询版本号 */
esp_err_t ld2461_get_version(ld2461_handle_t handle, ld2461_version_t *version)
{
    if (handle == NULL || version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_val = 0x01;
    esp_err_t ret = send_command(handle, LD2461_CMD_VERSION,
                                  &cmd_val, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[16];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_VERSION, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 8) {
        /* 版本号解析 */
        uint8_t year_month = resp_data[0];
        version->year = (year_month >> 4) & 0x0F;
        version->month = year_month & 0x0F;
        version->day = resp_data[1];
        version->major = resp_data[2];
        version->minor = resp_data[3];

        /* 唯一ID（大端） */
        version->unique_id = ((uint32_t)resp_data[4] << 24) |
                             ((uint32_t)resp_data[5] << 16) |
                             ((uint32_t)resp_data[6] << 8) |
                             resp_data[7];

        return ESP_OK;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/* 设置区域 */
esp_err_t ld2461_set_region(ld2461_handle_t handle, ld2461_region_id_t region_id,
                            const ld2461_region_t *region)
{
    if (handle == NULL || region == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (region_id < LD2461_REGION_1 || region_id > LD2461_REGION_3) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 构建命令值 */
    uint8_t value[10];
    value[0] = region_id;

    /* 4个顶点坐标（每个顶点2字节） */
    for (int i = 0; i < 4; i++) {
        value[1 + i * 2] = (uint8_t)region->vertex[i].x;
        value[2 + i * 2] = (uint8_t)region->vertex[i].y;
    }

    /* 区域类型 */
    value[9] = region->type;

    esp_err_t ret = send_command(handle, LD2461_CMD_SET_REGION,
                                  value, 10, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[8];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_SET_REGION, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 3 && resp_data[2] == 1) {
        ESP_LOGI(TAG, "Region %d set successfully", region_id);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* 清除区域 */
esp_err_t ld2461_clear_region(ld2461_handle_t handle, ld2461_region_id_t region_id)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (region_id < LD2461_REGION_1 || region_id > LD2461_REGION_3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t value = region_id;
    esp_err_t ret = send_command(handle, LD2461_CMD_CLEAR_REGION,
                                  &value, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[4];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_CLEAR_REGION, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 2 && resp_data[1] == 1) {
        ESP_LOGI(TAG, "Region %d cleared successfully", region_id);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* 读取区域设置 */
esp_err_t ld2461_get_regions(ld2461_handle_t handle, ld2461_region_t regions[3])
{
    if (handle == NULL || regions == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_val = 0x01;
    esp_err_t ret = send_command(handle, LD2461_CMD_READ_REGION,
                                  &cmd_val, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[64];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_READ_REGION, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 清空区域数组 */
    memset(regions, 0, sizeof(ld2461_region_t) * 3);

    /* 解析3个区域（每个区域10字节） */
    uint8_t pos = 0;
    for (int i = 0; i < 3; i++) {
        if (pos + 10 > resp_len) {
            break;
        }

        uint8_t region_id = resp_data[pos++];
        if (region_id < 1 || region_id > 3) {
            /* 跳过无效区域 */
            pos += 9;
            continue;
        }

        ld2461_region_t *region = &regions[region_id - 1];
        region->enabled = true;
        region->type = resp_data[pos++];

        /* 4个顶点 */
        for (int j = 0; j < 4; j++) {
            region->vertex[j].x = (int8_t)resp_data[pos++];
            region->vertex[j].y = (int8_t)resp_data[pos++];
        }
    }

    return ESP_OK;
}

/* 设置上报格式 */
esp_err_t ld2461_set_report_format(ld2461_handle_t handle, ld2461_report_format_t format)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (format < LD2461_REPORT_COORDINATES || format > LD2461_REPORT_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t value = format;
    esp_err_t ret = send_command(handle, LD2461_CMD_REPORT_FORMAT,
                                  &value, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[4];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_REPORT_FORMAT, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 1 && resp_data[0] == 1) {
        ESP_LOGI(TAG, "Report format set to %d", format);
        return ESP_OK;
    }

    return ESP_FAIL;
}

/* 读取上报格式 */
esp_err_t ld2461_get_report_format(ld2461_handle_t handle, ld2461_report_format_t *format)
{
    if (handle == NULL || format == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_val = 0x01;
    esp_err_t ret = send_command(handle, LD2461_CMD_READ_REPORT_FMT,
                                  &cmd_val, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[4];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_READ_REPORT_FMT, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 1) {
        *format = (ld2461_report_format_t)resp_data[0];
        return ESP_OK;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/* 恢复出厂设置 */
esp_err_t ld2461_factory_reset(ld2461_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd_val = 0x01;
    esp_err_t ret = send_command(handle, LD2461_CMD_FACTORY_RESET,
                                  &cmd_val, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    uint8_t resp_data[4];
    uint8_t resp_len = sizeof(resp_data);
    ret = parse_response(handle->cmd_response, handle->cmd_response_len,
                         LD2461_CMD_FACTORY_RESET, resp_data, &resp_len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (resp_len >= 1 && resp_data[0] == 1) {
        ESP_LOGI(TAG, "Factory reset successful");
        return ESP_OK;
    }

    return ESP_FAIL;
}
