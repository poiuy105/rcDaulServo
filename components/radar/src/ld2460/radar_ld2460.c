/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD2460 24GHz Millimeter Wave Radar Driver
 *
 * Features:
 * - Binary protocol frame parsing with state machine
 * - Async UART event-driven architecture
 * - Full command set: report control, installation, range, baudrate, etc.
 * - esp_event based notification to application layer
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "radar_ld2460.h"

static const char *TAG = "ld2460";

/* ============ Event base definition ============ */
ESP_EVENT_DEFINE_BASE(ESP_LD2460_EVENT);

/* ============ Internal constants ============ */
#define LD2460_CMD_ACK_LEN         12   /* Fixed ACK frame length */
#define LD2460_REPORT_HEADER_LEN   4
#define LD2460_REPORT_TAIL_LEN     4
#define LD2460_CMD_HEADER_LEN      4
#define LD2460_CMD_TAIL_LEN        4
#define LD2460_EVENT_LOOP_QUEUE    16
#define LD2460_FRAME_MAX_SIZE      64   /* Max possible frame size */

/* ============ Internal context structure ============ */
struct ld2460_context {
    uart_port_t       uart_port;
    uint32_t          baud_rate;
    uint8_t          *rx_buffer;
    uint16_t          rx_buffer_size;
    QueueHandle_t     event_queue;
    esp_event_loop_handle_t event_loop_hdl;
    TaskHandle_t      task_hdl;

    /* Frame parser state machine */
    uint16_t          parse_pos;       /* Current byte position in frame */
    uint8_t           parse_state;     /* Parser state */
    uint16_t          frame_len;       /* Expected total frame length */
    uint8_t           func_code;       /* Function code of current frame */

    /* Latest parsed data */
    ld2460_data_t     data;

    /* Command synchronization */
    SemaphoreHandle_t cmd_sem;
    uint8_t           cmd_ack_data[LD2460_CMD_ACK_LEN];
    bool              cmd_success;
};

/* ============ Parser states ============ */
typedef enum {
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_REPORT_HEADER,   /* Looking for F4 F3 F2 F1 */
    PARSE_STATE_REPORT_FUNC_LEN, /* Reading function code + length */
    PARSE_STATE_REPORT_DATA,     /* Reading target data */
    PARSE_STATE_REPORT_TAIL,     /* Reading F8 F7 F6 F5 */
    PARSE_STATE_CMD_HEADER,      /* Looking for FD FC FB FA (ACK) */
    PARSE_STATE_CMD_FUNC_LEN,    /* Reading ACK function code + length */
    PARSE_STATE_CMD_DATA,        /* Reading ACK data */
    PARSE_STATE_CMD_TAIL,        /* Reading 04 03 02 01 */
} parse_state_t;

/* ============ Helper: build command frame ============ */
static int build_cmd_frame(uint8_t *buf, uint8_t func_code,
                           const uint8_t *data, uint8_t data_len)
{
    int pos = 0;
    uint16_t total_len = LD2460_CMD_HEADER_LEN + 1 + 2 + data_len + LD2460_CMD_TAIL_LEN;

    /* Header */
    buf[pos++] = LD2460_CMD_HEADER_0;
    buf[pos++] = LD2460_CMD_HEADER_1;
    buf[pos++] = LD2460_CMD_HEADER_2;
    buf[pos++] = LD2460_CMD_HEADER_3;
    /* Function code */
    buf[pos++] = func_code;
    /* Data length (little-endian) */
    buf[pos++] = total_len & 0xFF;
    buf[pos++] = (total_len >> 8) & 0xFF;
    /* Data payload */
    if (data_len > 0 && data != NULL) {
        memcpy(buf + pos, data, data_len);
        pos += data_len;
    }
    /* Tail */
    buf[pos++] = LD2460_CMD_TAIL_0;
    buf[pos++] = LD2460_CMD_TAIL_1;
    buf[pos++] = LD2460_CMD_TAIL_2;
    buf[pos++] = LD2460_CMD_TAIL_3;

    return pos;
}

/* ============ Helper: send command and wait for ACK ============ */
static esp_err_t send_command(ld2460_handle_t handle,
                               uint8_t func_code,
                               const uint8_t *data, uint8_t data_len,
                               uint8_t *ack_payload, uint8_t *ack_payload_len)
{
    struct ld2460_context *ctx = (struct ld2460_context *)handle;
    uint8_t cmd_buf[LD2460_FRAME_MAX_SIZE];
    int cmd_len = build_cmd_frame(cmd_buf, func_code, data, data_len);

    /* Clear previous ACK state */
    ctx->cmd_success = false;
    memset(ctx->cmd_ack_data, 0, sizeof(ctx->cmd_ack_data));

    /* Send command */
    int written = uart_write_bytes(ctx->uart_port, cmd_buf, cmd_len);
    if (written != cmd_len) {
        ESP_LOGE(TAG, "Failed to send command (written %d/%d)", written, cmd_len);
        return ESP_FAIL;
    }

    /* Wait for ACK with timeout */
    if (xSemaphoreTake(ctx->cmd_sem, pdMS_TO_TICKS(LD2460_CMD_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Command 0x%02X timeout", func_code);
        return ESP_ERR_TIMEOUT;
    }

    if (!ctx->cmd_success) {
        ESP_LOGW(TAG, "Command 0x%02X failed", func_code);
        return ESP_FAIL;
    }

    /* Extract ACK payload if requested */
    if (ack_payload != NULL && ack_payload_len != NULL) {
        /* ACK frame: FD FC FB FA | func | lenL lenH | payload... | 04 03 02 01
         * Payload starts at offset 7, ends before the 4-byte tail */
        uint16_t ack_total = ctx->cmd_ack_data[5] | (ctx->cmd_ack_data[6] << 8);
        uint8_t payload_size = ack_total - LD2460_CMD_HEADER_LEN - 1 - 2 - LD2460_CMD_TAIL_LEN;
        if (payload_size > 0) {
            *ack_payload_len = payload_size;
            memcpy(ack_payload, ctx->cmd_ack_data + 7, payload_size);
        } else {
            *ack_payload_len = 0;
        }
    }

    return ESP_OK;
}

/* ============ Helper: simple command with 1-byte data ============ */
static esp_err_t send_simple_cmd(ld2460_handle_t handle, uint8_t func_code, uint8_t data_byte)
{
    return send_command(handle, func_code, &data_byte, 1, NULL, NULL);
}

/* ============ Helper: simple query command ============ */
static esp_err_t send_query_cmd(ld2460_handle_t handle, uint8_t func_code,
                                  uint8_t *ack_payload, uint8_t *ack_len)
{
    uint8_t query_data = 0x01;
    return send_command(handle, func_code, &query_data, 1, ack_payload, ack_len);
}

/* ============ Report frame parser (state machine) ============ */

static void parser_reset(struct ld2460_context *ctx)
{
    ctx->parse_state = PARSE_STATE_IDLE;
    ctx->parse_pos = 0;
    ctx->frame_len = 0;
    ctx->func_code = 0;
}

static void parse_report_frame(struct ld2460_context *ctx)
{
    ld2460_data_t *data = &ctx->data;
    uint8_t *buf = ctx->rx_buffer;

    /* Frame layout after header (offset 5):
     * [func=0x04] [lenL] [lenH] [target1_X_L] [target1_X_H] [target1_Y_L] [target1_Y_H] ...
     */
    uint16_t total_len = buf[5] | (buf[6] << 8);
    uint8_t data_len = total_len - 11; /* Subtract: 4(header) + 1(func) + 2(len) + 4(tail) */
    uint8_t target_count = data_len / 4;

    if (target_count > LD2460_MAX_TARGETS) {
        target_count = LD2460_MAX_TARGETS;
    }

    data->target_count = target_count;
    memset(data->targets, 0, sizeof(data->targets));

    for (uint8_t i = 0; i < target_count; i++) {
        uint16_t offset = 7 + i * 4;
        data->targets[i].x = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
        data->targets[i].y = (int16_t)(buf[offset + 2] | (buf[offset + 3] << 8));
    }

    /* Post event to application */
    esp_event_post_to(ctx->event_loop_hdl, ESP_LD2460_EVENT,
                      LD2460_EVENT_TARGET_UPDATE,
                      data, sizeof(ld2460_data_t),
                      pdMS_TO_TICKS(100));
}

static void parse_ack_frame(struct ld2460_context *ctx)
{
    uint8_t *buf = ctx->rx_buffer;

    /* Verify it's an ACK (function code matches a known command) */
    uint8_t func = buf[4];
    uint16_t total_len = buf[5] | (buf[6] << 8);

    /* Store ACK data for command synchronization */
    uint16_t copy_len = total_len;
    if (copy_len > sizeof(ctx->cmd_ack_data)) {
        copy_len = sizeof(ctx->cmd_ack_data);
    }
    memcpy(ctx->cmd_ack_data, buf, copy_len);

    /* Check ACK result: first byte of payload (offset 7) */
    /* For most commands: 0x00 = fail, 0x01 = success */
    uint8_t payload_size = total_len - 11;
    if (payload_size >= 1) {
        uint8_t result = buf[7];
        /* For report control (0x06): high nibble = result (1=success) */
        if (func == LD2460_FUNC_REPORT_CTRL) {
            ctx->cmd_success = ((result >> 4) & 0x0F) == 1;
        }
        /* For install mode (0x09): high nibble = result (1=success) */
        else if (func == LD2460_FUNC_SET_MODE) {
            ctx->cmd_success = ((result >> 4) & 0x0F) == 1;
        }
        /* For all other commands: 0x00 = fail, 0x01 = success */
        else {
            ctx->cmd_success = (result == 0x01);
        }
    } else {
        ctx->cmd_success = false;
    }

    /* Signal waiting command */
    xSemaphoreGive(ctx->cmd_sem);
}

static void process_uart_data(struct ld2460_context *ctx, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (ctx->parse_state) {

        case PARSE_STATE_IDLE:
            if (byte == LD2460_REPORT_HEADER_0) {
                ctx->rx_buffer[ctx->parse_pos++] = byte;
                ctx->parse_state = PARSE_STATE_REPORT_HEADER;
            } else if (byte == LD2460_CMD_HEADER_0) {
                ctx->rx_buffer[ctx->parse_pos++] = byte;
                ctx->parse_state = PARSE_STATE_CMD_HEADER;
            }
            break;

        /* ---- Report frame parsing ---- */
        case PARSE_STATE_REPORT_HEADER:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == LD2460_REPORT_HEADER_LEN) {
                /* Check full header: F4 F3 F2 F1 */
                if (ctx->rx_buffer[1] == LD2460_REPORT_HEADER_1 &&
                    ctx->rx_buffer[2] == LD2460_REPORT_HEADER_2 &&
                    ctx->rx_buffer[3] == LD2460_REPORT_HEADER_3) {
                    ctx->parse_state = PARSE_STATE_REPORT_FUNC_LEN;
                } else {
                    parser_reset(ctx);
                }
            }
            break;

        case PARSE_STATE_REPORT_FUNC_LEN:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == 7) {
                /* Have func code (offset 4) + length (offset 5-6) */
                ctx->func_code = ctx->rx_buffer[4];
                if (ctx->func_code != LD2460_FUNC_REPORT) {
                    ESP_LOGW(TAG, "Unexpected report func code: 0x%02X", ctx->func_code);
                    parser_reset(ctx);
                    break;
                }
                ctx->frame_len = ctx->rx_buffer[5] | (ctx->rx_buffer[6] << 8);
                if (ctx->frame_len < 11 || ctx->frame_len > LD2460_FRAME_MAX_SIZE) {
                    ESP_LOGW(TAG, "Invalid report frame length: %d", ctx->frame_len);
                    parser_reset(ctx);
                    break;
                }
                ctx->parse_state = PARSE_STATE_REPORT_DATA;
            }
            break;

        case PARSE_STATE_REPORT_DATA:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            /* Wait until we've read enough to reach the tail */
            if (ctx->parse_pos >= ctx->frame_len - LD2460_REPORT_TAIL_LEN) {
                ctx->parse_state = PARSE_STATE_REPORT_TAIL;
            }
            break;

        case PARSE_STATE_REPORT_TAIL:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == ctx->frame_len) {
                /* Verify tail: F8 F7 F6 F5 */
                uint16_t tail_start = ctx->frame_len - LD2460_REPORT_TAIL_LEN;
                if (ctx->rx_buffer[tail_start]     == LD2460_REPORT_TAIL_0 &&
                    ctx->rx_buffer[tail_start + 1] == LD2460_REPORT_TAIL_1 &&
                    ctx->rx_buffer[tail_start + 2] == LD2460_REPORT_TAIL_2 &&
                    ctx->rx_buffer[tail_start + 3] == LD2460_REPORT_TAIL_3) {
                    parse_report_frame(ctx);
                } else {
                    ESP_LOGW(TAG, "Report frame tail mismatch");
                }
                parser_reset(ctx);
            }
            break;

        /* ---- Command ACK frame parsing ---- */
        case PARSE_STATE_CMD_HEADER:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == LD2460_CMD_HEADER_LEN) {
                if (ctx->rx_buffer[1] == LD2460_CMD_HEADER_1 &&
                    ctx->rx_buffer[2] == LD2460_CMD_HEADER_2 &&
                    ctx->rx_buffer[3] == LD2460_CMD_HEADER_3) {
                    ctx->parse_state = PARSE_STATE_CMD_FUNC_LEN;
                } else {
                    parser_reset(ctx);
                }
            }
            break;

        case PARSE_STATE_CMD_FUNC_LEN:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == 7) {
                ctx->func_code = ctx->rx_buffer[4];
                ctx->frame_len = ctx->rx_buffer[5] | (ctx->rx_buffer[6] << 8);
                if (ctx->frame_len < LD2460_CMD_ACK_LEN || ctx->frame_len > LD2460_FRAME_MAX_SIZE) {
                    parser_reset(ctx);
                    break;
                }
                ctx->parse_state = PARSE_STATE_CMD_DATA;
            }
            break;

        case PARSE_STATE_CMD_DATA:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos >= ctx->frame_len - LD2460_CMD_TAIL_LEN) {
                ctx->parse_state = PARSE_STATE_CMD_TAIL;
            }
            break;

        case PARSE_STATE_CMD_TAIL:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == ctx->frame_len) {
                uint16_t tail_start = ctx->frame_len - LD2460_CMD_TAIL_LEN;
                if (ctx->rx_buffer[tail_start]     == LD2460_CMD_TAIL_0 &&
                    ctx->rx_buffer[tail_start + 1] == LD2460_CMD_TAIL_1 &&
                    ctx->rx_buffer[tail_start + 2] == LD2460_CMD_TAIL_2 &&
                    ctx->rx_buffer[tail_start + 3] == LD2460_CMD_TAIL_3) {
                    parse_ack_frame(ctx);
                } else {
                    ESP_LOGW(TAG, "ACK frame tail mismatch");
                }
                parser_reset(ctx);
            }
            break;

        default:
            parser_reset(ctx);
            break;
        }
    }
}

/* ============ UART event task ============ */

static void ld2460_task_entry(void *arg)
{
    struct ld2460_context *ctx = (struct ld2460_context *)arg;
    uart_event_t event;
    uint8_t *tmp_buf = malloc(ctx->rx_buffer_size);
    if (!tmp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(ctx->event_queue, &event, pdMS_TO_TICKS(200))) {
            switch (event.type) {
            case UART_DATA:
                /* Read available data and feed to parser */
                {
                    size_t buffered_len = 0;
                    uart_get_buffered_data_len(ctx->uart_port, &buffered_len);
                    if (buffered_len > 0) {
                        int read_len = uart_read_bytes(ctx->uart_port, tmp_buf,
                                                       buffered_len,
                                                       pdMS_TO_TICKS(100));
                        if (read_len > 0) {
                            process_uart_data(ctx, tmp_buf, read_len);
                        }
                    }
                }
                break;

            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "HW FIFO Overflow");
                uart_flush_input(ctx->uart_port);
                xQueueReset(ctx->event_queue);
                parser_reset(ctx);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "Ring Buffer Full");
                uart_flush_input(ctx->uart_port);
                xQueueReset(ctx->event_queue);
                parser_reset(ctx);
                break;

            case UART_BREAK:
                ESP_LOGW(TAG, "Rx Break");
                parser_reset(ctx);
                break;

            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "Parity Error");
                parser_reset(ctx);
                break;

            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "Frame Error");
                parser_reset(ctx);
                break;

            default:
                break;
            }
        }
        /* Drive the event loop */
        esp_event_loop_run(ctx->event_loop_hdl, pdMS_TO_TICKS(50));
    }

    free(tmp_buf);
    vTaskDelete(NULL);
}

/* ============================================================ */
/* ============ Public API Implementation ===================== */
/* ============================================================ */

ld2460_handle_t ld2460_init(const ld2460_config_t *config)
{
    struct ld2460_context *ctx = calloc(1, sizeof(struct ld2460_context));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }

    /* Allocate RX buffer */
    ctx->rx_buffer_size = LD2460_FRAME_MAX_SIZE;
    ctx->rx_buffer = calloc(1, ctx->rx_buffer_size);
    if (!ctx->rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        free(ctx);
        return NULL;
    }

    /* Store config */
    ctx->uart_port = config->uart_port;
    ctx->baud_rate = config->baud_rate > 0 ? config->baud_rate : LD2460_DEFAULT_BAUD_RATE;

    /* Create command semaphore */
    ctx->cmd_sem = xSemaphoreCreateBinary();
    if (!ctx->cmd_sem) {
        ESP_LOGE(TAG, "Failed to create command semaphore");
        free(ctx->rx_buffer);
        free(ctx);
        return NULL;
    }

    /* Install UART driver */
    uart_config_t uart_config = {
        .baud_rate = ctx->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uint32_t ring_buf_size = config->ring_buffer_size > 0 ? config->ring_buffer_size : 1024;
    uint32_t event_queue_size = config->event_queue_size > 0 ? config->event_queue_size : 16;

    if (uart_driver_install(ctx->uart_port, ring_buf_size, ring_buf_size,
                            event_queue_size, &ctx->event_queue, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        goto err_uart;
    }

    if (uart_param_config(ctx->uart_port, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART");
        goto err_uart;
    }

    int tx_pin = (config->tx_pin >= 0) ? config->tx_pin : UART_PIN_NO_CHANGE;
    if (uart_set_pin(ctx->uart_port, tx_pin, config->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        goto err_uart;
    }

    uart_flush(ctx->uart_port);

    /* Create event loop */
    esp_event_loop_args_t loop_args = {
        .queue_size = LD2460_EVENT_LOOP_QUEUE,
        .task_name = NULL,
    };
    if (esp_event_loop_create(&loop_args, &ctx->event_loop_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop");
        goto err_eloop;
    }

    /* Create parser task */
    uint32_t stack_size = config->task_stack_size > 0 ? config->task_stack_size : 4096;
    int priority = config->task_priority;
    BaseType_t ret = xTaskCreate(ld2460_task_entry, "ld2460_parse",
                                  stack_size, ctx, priority, &ctx->task_hdl);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create parser task");
        goto err_task;
    }

    ESP_LOGI(TAG, "LD2460 driver initialized (UART%d, %lu baud, TX=GPIO%d, RX=GPIO%d)",
             ctx->uart_port, ctx->baud_rate, tx_pin, config->rx_pin);
    return (ld2460_handle_t)ctx;

err_task:
    esp_event_loop_delete(ctx->event_loop_hdl);
err_eloop:
    uart_driver_delete(ctx->uart_port);
err_uart:
    vSemaphoreDelete(ctx->cmd_sem);
    free(ctx->rx_buffer);
    free(ctx);
    return NULL;
}

esp_err_t ld2460_deinit(ld2460_handle_t handle)
{
    struct ld2460_context *ctx = (struct ld2460_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;

    vTaskDelete(ctx->task_hdl);
    esp_event_loop_delete(ctx->event_loop_hdl);
    uart_driver_delete(ctx->uart_port);
    vSemaphoreDelete(ctx->cmd_sem);
    free(ctx->rx_buffer);
    free(ctx);
    return ESP_OK;
}

esp_err_t ld2460_add_handler(ld2460_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *handler_args)
{
    struct ld2460_context *ctx = (struct ld2460_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_register_with(ctx->event_loop_hdl,
                                           ESP_LD2460_EVENT,
                                           ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t ld2460_remove_handler(ld2460_handle_t handle,
                                 esp_event_handler_t event_handler)
{
    struct ld2460_context *ctx = (struct ld2460_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_unregister_with(ctx->event_loop_hdl,
                                             ESP_LD2460_EVENT,
                                             ESP_EVENT_ANY_ID,
                                             event_handler);
}

/* ============ Report Control ============ */

esp_err_t ld2460_enable_report(ld2460_handle_t handle, bool enable)
{
    return send_simple_cmd(handle, LD2460_FUNC_REPORT_CTRL, enable ? 0x01 : 0x00);
}

/* ============ Installation ============ */

esp_err_t ld2460_set_install_mode(ld2460_handle_t handle, ld2460_install_mode_t mode)
{
    return send_simple_cmd(handle, LD2460_FUNC_SET_MODE, (uint8_t)mode);
}

esp_err_t ld2460_get_install_mode(ld2460_handle_t handle, ld2460_install_mode_t *mode)
{
    uint8_t payload[8];
    uint8_t payload_len = 0;
    esp_err_t err = send_query_cmd(handle, LD2460_FUNC_GET_MODE, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 1 && mode != NULL) {
        *mode = (ld2460_install_mode_t)payload[0];
    }
    return err;
}

esp_err_t ld2460_set_install_params(ld2460_handle_t handle, float height, float angle)
{
    /* Height: meters * 100, Angle: degrees * 100 */
    uint16_t h = (uint16_t)(height * 100.0f);
    uint16_t a = (uint16_t)(angle * 100.0f);
    uint8_t data[4] = {
        h & 0xFF, (h >> 8) & 0xFF,
        a & 0xFF, (a >> 8) & 0xFF
    };
    return send_command(handle, LD2460_FUNC_SET_INSTALL, data, 4, NULL, NULL);
}

esp_err_t ld2460_get_install_params(ld2460_handle_t handle, ld2460_install_params_t *params)
{
    uint8_t payload[8];
    uint8_t payload_len = 0;
    esp_err_t err = send_query_cmd(handle, LD2460_FUNC_GET_INSTALL, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 4 && params != NULL) {
        uint16_t h = payload[0] | (payload[1] << 8);
        uint16_t a = payload[2] | (payload[3] << 8);
        params->height = h / 100.0f;
        params->angle = a / 100.0f;
    }
    return err;
}

/* ============ Detection Range ============ */

esp_err_t ld2460_set_detection_range(ld2460_handle_t handle,
                                      float distance, float angle_start, float angle_end)
{
    uint8_t d = (uint8_t)(distance * 10.0f);
    int16_t as = (int16_t)(angle_start * 10.0f);
    int16_t ae = (int16_t)(angle_end * 10.0f);
    uint8_t data[5] = {
        d,
        as & 0xFF, (as >> 8) & 0xFF,
        ae & 0xFF, (ae >> 8) & 0xFF
    };
    return send_command(handle, LD2460_FUNC_SET_RANGE, data, 5, NULL, NULL);
}

esp_err_t ld2460_get_detection_range(ld2460_handle_t handle, ld2460_range_t *range)
{
    uint8_t payload[8];
    uint8_t payload_len = 0;
    esp_err_t err = send_query_cmd(handle, LD2460_FUNC_GET_RANGE, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 5 && range != NULL) {
        range->distance = payload[0] / 10.0f;
        int16_t as = (int16_t)(payload[1] | (payload[2] << 8));
        int16_t ae = (int16_t)(payload[3] | (payload[4] << 8));
        range->angle_start = as / 10.0f;
        range->angle_end = ae / 10.0f;
    }
    return err;
}

/* ============ System ============ */

esp_err_t ld2460_get_firmware_version(ld2460_handle_t handle, ld2460_version_t *version)
{
    uint8_t payload[8];
    uint8_t payload_len = 0;
    esp_err_t err = send_query_cmd(handle, LD2460_FUNC_GET_VERSION, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 5 && version != NULL) {
        version->mode = (ld2460_install_mode_t)payload[0];
        version->year = payload[1];
        version->month = payload[2];
        version->version_major = payload[3];
        version->version_minor = payload[4];
    }
    return err;
}

esp_err_t ld2460_restart(ld2460_handle_t handle)
{
    return send_simple_cmd(handle, LD2460_FUNC_RESTART, 0x01);
}

esp_err_t ld2460_set_baud_rate(ld2460_handle_t handle, ld2460_baud_rate_index_t index)
{
    return send_simple_cmd(handle, LD2460_FUNC_SET_BAUDRATE, (uint8_t)index);
}

esp_err_t ld2460_factory_reset(ld2460_handle_t handle)
{
    return send_simple_cmd(handle, LD2460_FUNC_FACTORY_RESET, 0x01);
}

/* ============ Sensitivity (Reserved) ============ */

esp_err_t ld2460_set_sensitivity(ld2460_handle_t handle, ld2460_sensitivity_t level)
{
    return send_simple_cmd(handle, LD2460_FUNC_SET_SENSITIVITY, (uint8_t)level);
}

esp_err_t ld2460_get_sensitivity(ld2460_handle_t handle, ld2460_sensitivity_t *level)
{
    uint8_t payload[8];
    uint8_t payload_len = 0;
    esp_err_t err = send_query_cmd(handle, LD2460_FUNC_GET_SENSITIVITY, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 1 && level != NULL) {
        *level = (ld2460_sensitivity_t)payload[0];
    }
    return err;
}
