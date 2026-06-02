/**
 * @file radar_r60abd1.c
 * @brief R60ABD1 60GHz 呼吸睡眠雷达驱动实现
 *
 * 60GHz毫米波呼吸睡眠探测雷达
 * 波特率: 115200
 * 帧格式: 53 59 [control] [command] [len_h] [len_l] [data] [sum] 54 43
 */

#include "radar_r60abd1.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "R60ABD1";

/* 事件基础定义 */
ESP_EVENT_DEFINE_BASE(R60ABD1_EVENT);

/* 设备结构 */
struct r60abd1_dev {
    uart_port_t uart_num;
    int baud_rate;
    TaskHandle_t task_handle;
    SemaphoreHandle_t cmd_sem;
    uint8_t resp_buf[256];
    uint8_t resp_len;
    bool cmd_waiting;
    esp_event_loop_handle_t event_loop;
};

/* 帧缓冲区 */
#define FRAME_BUF_SIZE 512

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
static esp_err_t send_command(r60abd1_handle_t dev, uint8_t control,
                               uint8_t command, const uint8_t *data,
                               uint16_t data_len, uint32_t timeout_ms)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t frame[FRAME_BUF_SIZE];
    uint16_t pos = 0;

    /* 帧头 */
    frame[pos++] = R60ABD1_FRAME_HEADER_0;
    frame[pos++] = R60ABD1_FRAME_HEADER_1;

    /* 控制字 + 命令字 */
    frame[pos++] = control;
    frame[pos++] = command;

    /* 长度标识（大端） */
    frame[pos++] = (data_len >> 8) & 0xFF;
    frame[pos++] = data_len & 0xFF;

    /* 数据 */
    if (data != NULL && data_len > 0) {
        memcpy(&frame[pos], data, data_len);
        pos += data_len;
    }

    /* 校验和（帧头到数据） */
    frame[pos] = calc_checksum(frame, pos);
    pos++;

    /* 帧尾 */
    frame[pos++] = R60ABD1_FRAME_TAIL_0;
    frame[pos++] = R60ABD1_FRAME_TAIL_1;

    /* 发送 */
    dev->cmd_waiting = true;
    dev->resp_len = 0;

    uart_write_bytes(dev->uart_num, frame, pos);

    /* 等待响应 */
    if (xSemaphoreTake(dev->cmd_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        dev->cmd_waiting = false;
        ESP_LOGW(TAG, "Command 0x%02X timeout", command);
        return ESP_ERR_TIMEOUT;
    }

    dev->cmd_waiting = false;
    return ESP_OK;
}

/* 辅助函数：验证响应帧 */
static bool verify_frame(const uint8_t *frame, uint8_t len)
{
    if (len < 8) {
        return false;
    }

    /* 检查帧头 */
    if (frame[0] != R60ABD1_FRAME_HEADER_0 ||
        frame[1] != R60ABD1_FRAME_HEADER_1) {
        return false;
    }

    /* 检查帧尾 */
    if (frame[len - 2] != R60ABD1_FRAME_TAIL_0 ||
        frame[len - 1] != R60ABD1_FRAME_TAIL_1) {
        return false;
    }

    /* 获取数据长度 */
    uint16_t data_len = ((uint16_t)frame[4] << 8) | frame[5];

    /* 验证帧长度 */
    if (len != (6 + data_len + 1 + 2)) { /* 头+控制+命令+长度 + 数据 + 校验 + 尾 */
        return false;
    }

    /* 验证校验和 */
    uint8_t calc_sum = calc_checksum(frame, 6 + data_len);
    uint8_t recv_sum = frame[6 + data_len];
    if (calc_sum != recv_sum) {
        ESP_LOGW(TAG, "Checksum mismatch: calc=0x%02X, recv=0x%02X", calc_sum, recv_sum);
        return false;
    }

    return true;
}

/* 辅助函数：解析带符号的16位值（bit15为符号位） */
static int16_t parse_signed16(uint16_t raw)
{
    if (raw & 0x8000) {
        return (int16_t)(raw & 0x7FFF);
    } else {
        return -(int16_t)(raw & 0x7FFF);
    }
}

/* 解析上报数据 */
static void parse_report(r60abd1_handle_t dev, const uint8_t *frame, uint8_t len)
{
    if (!verify_frame(frame, len)) {
        return;
    }

    uint8_t control = frame[2];
    uint8_t command = frame[3];
    uint16_t data_len = ((uint16_t)frame[4] << 8) | frame[5];
    const uint8_t *data = &frame[6];

    r60abd1_data_t report = {0};

    switch (control) {
    case R60ABD1_CTRL_HUMAN:
        switch (command) {
        case R60ABD1_CMD_HUMAN_STATUS:
            if (data_len >= 1) {
                report.human = (r60abd1_human_status_t)data[0];
                report.has_human = true;
            }
            break;
        case R60ABD1_CMD_HUMAN_MOTION:
            if (data_len >= 1) {
                report.motion = (r60abd1_motion_status_t)data[0];
                report.has_motion = true;
            }
            break;
        case R60ABD1_CMD_HUMAN_MOVE:
            if (data_len >= 1) {
                report.move_param = data[0];
                report.has_move = true;
            }
            break;
        case R60ABD1_CMD_HUMAN_DISTANCE:
            if (data_len >= 2) {
                report.distance = ((uint16_t)data[0] << 8) | data[1];
                report.has_distance = true;
            }
            break;
        case R60ABD1_CMD_HUMAN_POSITION:
            if (data_len >= 6) {
                uint16_t x_raw = ((uint16_t)data[0] << 8) | data[1];
                uint16_t y_raw = ((uint16_t)data[2] << 8) | data[3];
                uint16_t z_raw = ((uint16_t)data[4] << 8) | data[5];
                report.position.x = parse_signed16(x_raw);
                report.position.y = parse_signed16(y_raw);
                report.position.z = parse_signed16(z_raw);
                report.has_position = true;
            }
            break;
        }
        break;

    case R60ABD1_CTRL_BREATH:
        switch (command) {
        case R60ABD1_CMD_BREATH_STATUS:
            if (data_len >= 1) {
                report.breath_status = (r60abd1_breath_status_t)data[0];
                report.has_breath_status = true;
            }
            break;
        case R60ABD1_CMD_BREATH_RATE:
            if (data_len >= 1) {
                report.breath_rate = data[0];
                report.has_breath_rate = true;
            }
            break;
        case R60ABD1_CMD_BREATH_WAVE:
            if (data_len >= 5) {
                memcpy(report.breath_wave.waveform, data, 5);
                report.has_breath_wave = true;
            }
            break;
        }
        break;

    case R60ABD1_CTRL_HEART_RATE:
        switch (command) {
        case R60ABD1_CMD_HEART_RATE:
            if (data_len >= 1) {
                report.heart_rate = data[0];
                report.has_heart_rate = true;
            }
            break;
        case R60ABD1_CMD_HEART_WAVE:
            if (data_len >= 5) {
                memcpy(report.heart_wave.waveform, data, 5);
                report.has_heart_wave = true;
            }
            break;
        }
        break;

    case R60ABD1_CTRL_SLEEP:
        switch (command) {
        case R60ABD1_CMD_SLEEP_BED:
            if (data_len >= 1) {
                report.bed = (r60abd1_bed_status_t)data[0];
                report.has_bed = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_STATE:
            if (data_len >= 1) {
                report.sleep_state = (r60abd1_sleep_state_t)data[0];
                report.has_sleep_state = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_WAKE_TIME:
            if (data_len >= 2) {
                report.wake_time = ((uint16_t)data[0] << 8) | data[1];
                report.has_wake_time = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_LIGHT_TIME:
            if (data_len >= 2) {
                report.light_time = ((uint16_t)data[0] << 8) | data[1];
                report.has_light_time = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_DEEP_TIME:
            if (data_len >= 2) {
                report.deep_time = ((uint16_t)data[0] << 8) | data[1];
                report.has_deep_time = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_SCORE:
            if (data_len >= 1) {
                report.sleep_score = data[0];
                report.has_sleep_score = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_COMPREHENSIVE:
            if (data_len >= 8) {
                report.sleep_comp.present = data[0] != 0;
                report.sleep_comp.state = (r60abd1_sleep_state_t)data[1];
                report.sleep_comp.breath_rate = data[2];
                report.sleep_comp.heart_rate = data[3];
                report.sleep_comp.turn_count = data[4];
                report.sleep_comp.large_move = data[5];
                report.sleep_comp.small_move = data[6];
                report.sleep_comp.apnea_count = data[7];
                report.has_sleep_comp = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_ANALYSIS:
            if (data_len >= 12) {
                report.sleep_analysis.score = data[0];
                report.sleep_analysis.total_time = ((uint16_t)data[1] << 8) | data[2];
                report.sleep_analysis.awake_ratio = data[3];
                report.sleep_analysis.light_ratio = data[4];
                report.sleep_analysis.deep_ratio = data[5];
                report.sleep_analysis.away_time = data[6];
                report.sleep_analysis.away_count = data[7];
                report.sleep_analysis.turn_count = data[8];
                report.sleep_analysis.avg_breath = data[9];
                report.sleep_analysis.avg_heart = data[10];
                report.sleep_analysis.apnea_count = data[11];
                report.has_sleep_analysis = true;
            }
            break;
        case R60ABD1_CMD_SLEEP_ABNORMAL:
            if (data_len >= 1) {
                report.abnormal = (r60abd1_sleep_abnormal_t)data[0];
                report.has_abnormal = true;
            }
            break;
        }
        break;

    case R60ABD1_CTRL_RANGE:
        if (command == R60ABD1_CMD_RANGE_STATUS && data_len >= 1) {
            report.range = (r60abd1_range_status_t)data[0];
            report.has_range = true;
        }
        break;
    }

    /* 发布事件 */
    if (report.has_human || report.has_motion || report.has_breath_rate ||
        report.has_heart_rate || report.has_bed || report.has_sleep_state) {
        esp_event_post_to(dev->event_loop, R60ABD1_EVENT,
                          R60ABD1_EVENT_DATA_UPDATE, &report,
                          sizeof(report), portMAX_DELAY);
    }
}

/* UART接收任务 */
static void r60abd1_uart_task(void *arg)
{
    r60abd1_handle_t dev = (r60abd1_handle_t)arg;
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
        if (pos == 0 && ch != R60ABD1_FRAME_HEADER_0) {
            continue;
        }
        if (pos == 1 && ch != R60ABD1_FRAME_HEADER_1) {
            pos = 0;
            if (ch == R60ABD1_FRAME_HEADER_0) {
                buf[pos++] = ch;
            }
            continue;
        }

        buf[pos++] = ch;

        /* 至少收到长度字段 */
        if (pos >= 6) {
            uint16_t data_len = ((uint16_t)buf[4] << 8) | buf[5];
            uint16_t frame_len = 6 + data_len + 1 + 2; /* 头+控制+命令+长度 + 数据 + 校验 + 尾 */

            if (pos >= frame_len && frame_len <= FRAME_BUF_SIZE) {
                /* 检查帧尾 */
                if (buf[frame_len - 2] == R60ABD1_FRAME_TAIL_0 &&
                    buf[frame_len - 1] == R60ABD1_FRAME_TAIL_1) {

                    /* 命令响应 */
                    if (dev->cmd_waiting) {
                        memcpy(dev->resp_buf, buf, frame_len);
                        dev->resp_len = frame_len;
                        xSemaphoreGive(dev->cmd_sem);
                    } else {
                        /* 主动上报 */
                        parse_report(dev, buf, frame_len);
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
r60abd1_handle_t r60abd1_init(const r60abd1_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    r60abd1_handle_t dev = calloc(1, sizeof(struct r60abd1_dev));
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

    if (uart_driver_install(config->uart_num, R60ABD1_UART_BUF_SIZE * 2,
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
        .task_name = "r60abd1_evt",
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
    if (xTaskCreate(r60abd1_uart_task, "r60abd1_uart", R60ABD1_TASK_STACK_SIZE,
                    dev, R60ABD1_TASK_PRIORITY, &dev->task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        esp_event_loop_delete(dev->event_loop);
        vSemaphoreDelete(dev->cmd_sem);
        uart_driver_delete(config->uart_num);
        free(dev);
        return NULL;
    }

    ESP_LOGI(TAG, "R60ABD1 initialized, baud=%d", config->baud_rate);
    return dev;
}

/* 反初始化 */
esp_err_t r60abd1_deinit(r60abd1_handle_t handle)
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

    ESP_LOGI(TAG, "R60ABD1 deinitialized");
    return ESP_OK;
}

/* 注册事件处理函数 */
esp_err_t r60abd1_add_handler(r60abd1_handle_t handle,
                                esp_event_handler_t event_handler,
                                void *event_handler_arg)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_register_with(handle->event_loop,
                                           R60ABD1_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, event_handler_arg);
}

/* 注销事件处理函数 */
esp_err_t r60abd1_remove_handler(r60abd1_handle_t handle,
                                   esp_event_handler_t event_handler)
{
    if (handle == NULL || event_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return esp_event_handler_unregister_with(handle->event_loop,
                                             R60ABD1_EVENT, ESP_EVENT_ANY_ID,
                                             event_handler);
}

/* 系统功能：复位 */
esp_err_t r60abd1_reset(r60abd1_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 0x0F;
    return send_command(handle, R60ABD1_CTRL_HEARTBEAT,
                        R60ABD1_CMD_RESET, &data, 1, 1000);
}

/* 系统功能：心跳包 */
esp_err_t r60abd1_heartbeat(r60abd1_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 0x0F;
    return send_command(handle, R60ABD1_CTRL_HEARTBEAT,
                        R60ABD1_CMD_HEARTBEAT_QUERY, &data, 1, 1000);
}

/* 产品信息：固件版本查询 */
esp_err_t r60abd1_get_firmware_version(r60abd1_handle_t handle, char *version, size_t len)
{
    if (handle == NULL || version == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 0x0F;
    esp_err_t ret = send_command(handle, R60ABD1_CTRL_PRODUCT,
                                  R60ABD1_CMD_FIRMWARE_VER, &data, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 解析响应 */
    if (handle->resp_len < 8) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t data_len = ((uint16_t)handle->resp_buf[4] << 8) | handle->resp_buf[5];
    if (data_len > 0 && data_len < len) {
        memcpy(version, &handle->resp_buf[6], data_len);
        version[data_len] = '\0';
        return ESP_OK;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/* 人体存在：开关设置 */
esp_err_t r60abd1_set_human_enable(r60abd1_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {enable ? 0x01 : 0x00, 0x00};
    return send_command(handle, R60ABD1_CTRL_HUMAN,
                        R60ABD1_CMD_HUMAN_ENABLE, data, 1, 1000);
}

/* 人体存在：状态查询 */
esp_err_t r60abd1_get_human_status(r60abd1_handle_t handle, r60abd1_human_status_t *status)
{
    if (handle == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 0x0F;
    esp_err_t ret = send_command(handle, R60ABD1_CTRL_HUMAN,
                                  R60ABD1_CMD_HUMAN_STATUS_Q, &data, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    if (handle->resp_len >= 7) {
        *status = (r60abd1_human_status_t)handle->resp_buf[6];
        return ESP_OK;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/* 呼吸检测：开关设置 */
esp_err_t r60abd1_set_breath_enable(r60abd1_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = enable ? 0x01 : 0x00;
    return send_command(handle, R60ABD1_CTRL_BREATH,
                        R60ABD1_CMD_BREATH_ENABLE, &data, 1, 1000);
}

/* 呼吸检测：状态查询 */
esp_err_t r60abd1_get_breath_status(r60abd1_handle_t handle, r60abd1_breath_status_t *status)
{
    if (handle == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = 0x0F;
    esp_err_t ret = send_command(handle, R60ABD1_CTRL_BREATH,
                                  R60ABD1_CMD_BREATH_STATUS_Q, &data, 1, 1000);
    if (ret != ESP_OK) {
        return ret;
    }

    if (handle->resp_len >= 7) {
        *status = (r60abd1_breath_status_t)handle->resp_buf[6];
        return ESP_OK;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

/* 心率监测：开关设置 */
esp_err_t r60abd1_set_heart_enable(r60abd1_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = enable ? 0x01 : 0x00;
    return send_command(handle, R60ABD1_CTRL_HEART_RATE,
                        R60ABD1_CMD_HEART_ENABLE, &data, 1, 1000);
}

/* 睡眠监测：开关设置 */
esp_err_t r60abd1_set_sleep_enable(r60abd1_handle_t handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = enable ? 0x01 : 0x00;
    return send_command(handle, R60ABD1_CTRL_SLEEP,
                        R60ABD1_CMD_SLEEP_ENABLE, &data, 1, 1000);
}

/* 睡眠监测：上报模式设置 */
esp_err_t r60abd1_set_report_mode(r60abd1_handle_t handle, r60abd1_report_mode_t mode)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data = (mode == R60ABD1_MODE_SLEEP) ? 0x01 : 0x00;
    return send_command(handle, R60ABD1_CTRL_SLEEP,
                        R60ABD1_CMD_SLEEP_MODE, &data, 1, 1000);
}
