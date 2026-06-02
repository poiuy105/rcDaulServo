/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD2450 24GHz Millimeter Wave Radar Driver
 *
 * Features:
 * - Binary protocol frame parsing with state machine
 * - Async UART event-driven architecture
 * - Full command set: tracking mode, region filter, baudrate, etc.
 * - Config mode auto-wrap for commands requiring it
 * - esp_event based notification to application layer
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "radar_ld2450.h"

static const char *TAG = "ld2450";

/* ============ Event base definition ============ */
ESP_EVENT_DEFINE_BASE(ESP_LD2450_EVENT);

/* ============ Internal constants ============ */
#define LD2450_CMD_ACK_LEN         12   /* Fixed minimum ACK frame length */
#define LD2450_REPORT_HEADER_LEN   4
#define LD2450_REPORT_TAIL_LEN     2
#define LD2450_CMD_HEADER_LEN      4
#define LD2450_CMD_TAIL_LEN        4
#define LD2450_EVENT_LOOP_QUEUE    16
#define LD2450_FRAME_MAX_SIZE      64   /* Max possible frame size */
#define LD2450_TARGET_DATA_SIZE    8    /* Each target: X(2) + Y(2) + Speed(2) + Resolution(2) */

/* ============ Internal context structure ============ */
struct ld2450_context {
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

    /* Latest parsed data */
    ld2450_data_t     data;

    /* Command synchronization */
    SemaphoreHandle_t cmd_sem;
    uint8_t           cmd_ack_data[LD2450_FRAME_MAX_SIZE];
    uint16_t          cmd_ack_len;
    bool              cmd_success;
    bool              in_config_mode;  /* Track config mode state */
};

/* ============ Parser states ============ */
typedef enum {
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_REPORT_HEADER,   /* Looking for AA FF 03 00 */
    PARSE_STATE_REPORT_DATA,     /* Reading 24 bytes of target data */
    PARSE_STATE_REPORT_TAIL,     /* Looking for 55 CC */
    PARSE_STATE_CMD_HEADER,      /* Looking for FD FC FB FA (ACK) */
    PARSE_STATE_CMD_FUNC_LEN,    /* Reading ACK function code + length */
    PARSE_STATE_CMD_DATA,        /* Reading ACK data */
    PARSE_STATE_CMD_TAIL,        /* Reading 04 03 02 01 */
} parse_state_t;

/* ============ Helper: build command frame ============ */
static int build_cmd_frame(uint8_t *buf, uint16_t func_code,
                           const uint8_t *data, uint8_t data_len)
{
    int pos = 0;
    uint16_t total_len = LD2450_CMD_HEADER_LEN + 2 + 2 + data_len + LD2450_CMD_TAIL_LEN;

    /* Header: FD FC FB FA */
    buf[pos++] = LD2450_CMD_HEADER_0;
    buf[pos++] = LD2450_CMD_HEADER_1;
    buf[pos++] = LD2450_CMD_HEADER_2;
    buf[pos++] = LD2450_CMD_HEADER_3;

    /* Function code (little-endian) */
    buf[pos++] = func_code & 0xFF;
    buf[pos++] = (func_code >> 8) & 0xFF;

    /* Data length (little-endian) */
    buf[pos++] = total_len & 0xFF;
    buf[pos++] = (total_len >> 8) & 0xFF;

    /* Data payload */
    if (data_len > 0 && data != NULL) {
        memcpy(buf + pos, data, data_len);
        pos += data_len;
    }

    /* Tail: 04 03 02 01 */
    buf[pos++] = LD2450_CMD_TAIL_0;
    buf[pos++] = LD2450_CMD_TAIL_1;
    buf[pos++] = LD2450_CMD_TAIL_2;
    buf[pos++] = LD2450_CMD_TAIL_3;

    return pos;
}

/* ============ Helper: send raw command and wait for ACK ============ */
static esp_err_t send_raw_command(ld2450_handle_t handle,
                                   uint16_t func_code,
                                   const uint8_t *data, uint8_t data_len,
                                   uint8_t *ack_payload, uint8_t *ack_payload_len)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    uint8_t cmd_buf[LD2450_FRAME_MAX_SIZE];
    int cmd_len = build_cmd_frame(cmd_buf, func_code, data, data_len);

    /* Clear previous ACK state */
    ctx->cmd_success = false;
    ctx->cmd_ack_len = 0;
    memset(ctx->cmd_ack_data, 0, sizeof(ctx->cmd_ack_data));

    /* Send command */
    int written = uart_write_bytes(ctx->uart_port, cmd_buf, cmd_len);
    if (written != cmd_len) {
        ESP_LOGE(TAG, "Failed to send command (written %d/%d)", written, cmd_len);
        return ESP_FAIL;
    }

    /* Wait for ACK with timeout */
    if (xSemaphoreTake(ctx->cmd_sem, pdMS_TO_TICKS(LD2450_CMD_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "Command 0x%04X timeout", func_code);
        return ESP_ERR_TIMEOUT;
    }

    if (!ctx->cmd_success) {
        ESP_LOGW(TAG, "Command 0x%04X failed", func_code);
        return ESP_FAIL;
    }

    /* Extract ACK payload if requested */
    if (ack_payload != NULL && ack_payload_len != NULL) {
        /* ACK frame: FD FC FB FA | funcL funcH | lenL lenH | payload... | 04 03 02 01
         * Payload starts at offset 8, ends before the 4-byte tail */
        uint16_t ack_total = ctx->cmd_ack_data[6] | (ctx->cmd_ack_data[7] << 8);
        uint8_t payload_size = ack_total - LD2450_CMD_HEADER_LEN - 2 - 2 - LD2450_CMD_TAIL_LEN;
        if (payload_size > 0 && payload_size <= *ack_payload_len) {
            *ack_payload_len = payload_size;
            memcpy(ack_payload, ctx->cmd_ack_data + 8, payload_size);
        } else {
            *ack_payload_len = 0;
        }
    }

    return ESP_OK;
}

/* ============ Helper: send command with config mode auto-wrap ============ */
static esp_err_t send_config_command(ld2450_handle_t handle,
                                      uint16_t func_code,
                                      const uint8_t *data, uint8_t data_len,
                                      uint8_t *ack_payload, uint8_t *ack_payload_len)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    esp_err_t err;

    /* Enable config mode if not already in it */
    if (!ctx->in_config_mode) {
        err = send_raw_command(handle, LD2450_FUNC_ENABLE_CONFIG, NULL, 0, NULL, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable config mode");
            return err;
        }
        ctx->in_config_mode = true;
    }

    /* Send the actual command */
    err = send_raw_command(handle, func_code, data, data_len, ack_payload, ack_payload_len);

    /* End config mode */
    esp_err_t end_err = send_raw_command(handle, LD2450_FUNC_DISABLE_CONFIG, NULL, 0, NULL, NULL);
    if (end_err == ESP_OK) {
        ctx->in_config_mode = false;
    }

    return err;
}

/* ============ Helper: simple config command ============ */
static esp_err_t send_simple_config_cmd(ld2450_handle_t handle, uint16_t func_code)
{
    return send_config_command(handle, func_code, NULL, 0, NULL, NULL);
}

/* ============ Report frame parser (state machine) ============ */

static void parser_reset(struct ld2450_context *ctx)
{
    ctx->parse_state = PARSE_STATE_IDLE;
    ctx->parse_pos = 0;
    ctx->frame_len = 0;
}

static void parse_report_frame(struct ld2450_context *ctx)
{
    ld2450_data_t *data = &ctx->data;
    uint8_t *buf = ctx->rx_buffer;

    /* Frame layout:
     * AA FF 03 00 | [24 bytes: 3 targets × 8 bytes] | 55 CC
     * Each target: X(int16 LE mm) + Y(int16 LE mm) + Speed(int16 LE cm/s) + Resolution(uint16 LE mm)
     */
    data->target_count = LD2450_MAX_TARGETS;
    memset(data->targets, 0, sizeof(data->targets));

    for (uint8_t i = 0; i < LD2450_MAX_TARGETS; i++) {
        uint16_t offset = LD2450_REPORT_HEADER_LEN + i * LD2450_TARGET_DATA_SIZE;
        data->targets[i].x = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
        data->targets[i].y = (int16_t)(buf[offset + 2] | (buf[offset + 3] << 8));
        data->targets[i].speed = (int16_t)(buf[offset + 4] | (buf[offset + 5] << 8));
        data->targets[i].resolution = (uint16_t)(buf[offset + 6] | (buf[offset + 7] << 8));
    }

    /* Post event to application */
    esp_event_post_to(ctx->event_loop_hdl, ESP_LD2450_EVENT,
                      LD2450_EVENT_TARGET_UPDATE,
                      data, sizeof(ld2450_data_t),
                      pdMS_TO_TICKS(100));
}

static void parse_ack_frame(struct ld2450_context *ctx)
{
    uint8_t *buf = ctx->rx_buffer;

    /* Verify it's an ACK */
    uint16_t func = buf[4] | (buf[5] << 8);
    uint16_t total_len = buf[6] | (buf[7] << 8);

    /* Store ACK data for command synchronization */
    uint16_t copy_len = total_len;
    if (copy_len > sizeof(ctx->cmd_ack_data)) {
        copy_len = sizeof(ctx->cmd_ack_data);
    }
    memcpy(ctx->cmd_ack_data, buf, copy_len);
    ctx->cmd_ack_len = copy_len;

    /* Check ACK result: first byte of payload (offset 8)
     * For most commands: 0x00 = fail, 0x01 = success
     * For enable/disable config: result is in the payload */
    uint8_t payload_size = total_len - LD2450_CMD_HEADER_LEN - 2 - 2 - LD2450_CMD_TAIL_LEN;
    if (payload_size >= 1) {
        uint8_t result = buf[8];
        ctx->cmd_success = (result == 0x01 || result == 0x00);
        /* Special case: enable config returns 0x01 on success */
        if (func == LD2450_FUNC_ENABLE_CONFIG) {
            ctx->cmd_success = (result == 0x01);
        }
        /* Disable config returns 0x00 on success */
        else if (func == LD2450_FUNC_DISABLE_CONFIG) {
            ctx->cmd_success = true; /* Always succeeds */
        }
        /* Other commands: 0x01 = success */
        else {
            ctx->cmd_success = (result == 0x01);
        }
    } else {
        /* No payload means success for some commands */
        ctx->cmd_success = true;
    }

    /* Signal waiting command */
    xSemaphoreGive(ctx->cmd_sem);
}

static void process_uart_data(struct ld2450_context *ctx, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        switch (ctx->parse_state) {

        case PARSE_STATE_IDLE:
            if (byte == LD2450_REPORT_HEADER_0) {
                ctx->rx_buffer[ctx->parse_pos++] = byte;
                ctx->parse_state = PARSE_STATE_REPORT_HEADER;
            } else if (byte == LD2450_CMD_HEADER_0) {
                ctx->rx_buffer[ctx->parse_pos++] = byte;
                ctx->parse_state = PARSE_STATE_CMD_HEADER;
            }
            break;

        /* ---- Report frame parsing ---- */
        case PARSE_STATE_REPORT_HEADER:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == LD2450_REPORT_HEADER_LEN) {
                /* Check full header: AA FF 03 00 */
                if (ctx->rx_buffer[1] == LD2450_REPORT_HEADER_1 &&
                    ctx->rx_buffer[2] == LD2450_REPORT_HEADER_2 &&
                    ctx->rx_buffer[3] == LD2450_REPORT_HEADER_3) {
                    ctx->frame_len = LD2450_FRAME_SIZE; /* 30 bytes total */
                    ctx->parse_state = PARSE_STATE_REPORT_DATA;
                } else {
                    parser_reset(ctx);
                }
            }
            break;

        case PARSE_STATE_REPORT_DATA:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            /* After header (4) + target data (24) = 28 bytes, expect tail */
            if (ctx->parse_pos >= LD2450_REPORT_HEADER_LEN + LD2450_MAX_TARGETS * LD2450_TARGET_DATA_SIZE) {
                ctx->parse_state = PARSE_STATE_REPORT_TAIL;
            }
            break;

        case PARSE_STATE_REPORT_TAIL:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == ctx->frame_len) {
                /* Verify tail: 55 CC */
                uint16_t tail_start = ctx->frame_len - LD2450_REPORT_TAIL_LEN;
                if (ctx->rx_buffer[tail_start]     == LD2450_REPORT_TAIL_0 &&
                    ctx->rx_buffer[tail_start + 1] == LD2450_REPORT_TAIL_1) {
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
            if (ctx->parse_pos == LD2450_CMD_HEADER_LEN) {
                if (ctx->rx_buffer[1] == LD2450_CMD_HEADER_1 &&
                    ctx->rx_buffer[2] == LD2450_CMD_HEADER_2 &&
                    ctx->rx_buffer[3] == LD2450_CMD_HEADER_3) {
                    ctx->parse_state = PARSE_STATE_CMD_FUNC_LEN;
                } else {
                    parser_reset(ctx);
                }
            }
            break;

        case PARSE_STATE_CMD_FUNC_LEN:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == 8) {
                /* Have func code (offset 4-5) + length (offset 6-7) */
                ctx->frame_len = ctx->rx_buffer[6] | (ctx->rx_buffer[7] << 8);
                if (ctx->frame_len < LD2450_CMD_ACK_LEN || ctx->frame_len > LD2450_FRAME_MAX_SIZE) {
                    ESP_LOGW(TAG, "Invalid ACK frame length: %d", ctx->frame_len);
                    parser_reset(ctx);
                    break;
                }
                ctx->parse_state = PARSE_STATE_CMD_DATA;
            }
            break;

        case PARSE_STATE_CMD_DATA:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos >= ctx->frame_len - LD2450_CMD_TAIL_LEN) {
                ctx->parse_state = PARSE_STATE_CMD_TAIL;
            }
            break;

        case PARSE_STATE_CMD_TAIL:
            ctx->rx_buffer[ctx->parse_pos++] = byte;
            if (ctx->parse_pos == ctx->frame_len) {
                uint16_t tail_start = ctx->frame_len - LD2450_CMD_TAIL_LEN;
                if (ctx->rx_buffer[tail_start]     == LD2450_CMD_TAIL_0 &&
                    ctx->rx_buffer[tail_start + 1] == LD2450_CMD_TAIL_1 &&
                    ctx->rx_buffer[tail_start + 2] == LD2450_CMD_TAIL_2 &&
                    ctx->rx_buffer[tail_start + 3] == LD2450_CMD_TAIL_3) {
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

static void ld2450_task_entry(void *arg)
{
    struct ld2450_context *ctx = (struct ld2450_context *)arg;
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

ld2450_handle_t ld2450_init(const ld2450_config_t *config)
{
    struct ld2450_context *ctx = calloc(1, sizeof(struct ld2450_context));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }

    /* Allocate RX buffer */
    ctx->rx_buffer_size = LD2450_FRAME_MAX_SIZE;
    ctx->rx_buffer = calloc(1, ctx->rx_buffer_size);
    if (!ctx->rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        free(ctx);
        return NULL;
    }

    /* Store config */
    ctx->uart_port = config->uart_port;
    ctx->baud_rate = config->baud_rate > 0 ? config->baud_rate : LD2450_DEFAULT_BAUD_RATE;
    ctx->in_config_mode = false;

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
        .queue_size = LD2450_EVENT_LOOP_QUEUE,
        .task_name = NULL,
    };
    if (esp_event_loop_create(&loop_args, &ctx->event_loop_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop");
        goto err_eloop;
    }

    /* Create parser task */
    uint32_t stack_size = config->task_stack_size > 0 ? config->task_stack_size : 4096;
    int priority = config->task_priority;
    BaseType_t ret = xTaskCreate(ld2450_task_entry, "ld2450_parse",
                                  stack_size, ctx, priority, &ctx->task_hdl);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create parser task");
        goto err_task;
    }

    ESP_LOGI(TAG, "LD2450 driver initialized (UART%d, %lu baud, TX=GPIO%d, RX=GPIO%d)",
             ctx->uart_port, ctx->baud_rate, tx_pin, config->rx_pin);
    return (ld2450_handle_t)ctx;

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

esp_err_t ld2450_deinit(ld2450_handle_t handle)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;

    vTaskDelete(ctx->task_hdl);
    esp_event_loop_delete(ctx->event_loop_hdl);
    uart_driver_delete(ctx->uart_port);
    vSemaphoreDelete(ctx->cmd_sem);
    free(ctx->rx_buffer);
    free(ctx);
    return ESP_OK;
}

esp_err_t ld2450_add_handler(ld2450_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *handler_args)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_register_with(ctx->event_loop_hdl,
                                           ESP_LD2450_EVENT,
                                           ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t ld2450_remove_handler(ld2450_handle_t handle,
                                 esp_event_handler_t event_handler)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_unregister_with(ctx->event_loop_hdl,
                                             ESP_LD2450_EVENT,
                                             ESP_EVENT_ANY_ID,
                                             event_handler);
}

/* ============ Config Mode API ============ */

esp_err_t ld2450_enable_config(ld2450_handle_t handle)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (ctx->in_config_mode) {
        return ESP_OK; /* Already in config mode */
    }

    esp_err_t err = send_raw_command(handle, LD2450_FUNC_ENABLE_CONFIG, NULL, 0, NULL, NULL);
    if (err == ESP_OK) {
        ctx->in_config_mode = true;
    }
    return err;
}

esp_err_t ld2450_end_config(ld2450_handle_t handle)
{
    struct ld2450_context *ctx = (struct ld2450_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;

    if (!ctx->in_config_mode) {
        return ESP_OK; /* Not in config mode */
    }

    esp_err_t err = send_raw_command(handle, LD2450_FUNC_DISABLE_CONFIG, NULL, 0, NULL, NULL);
    if (err == ESP_OK) {
        ctx->in_config_mode = false;
    }
    return err;
}

/* ============ Tracking Mode API ============ */

esp_err_t ld2450_set_single_target(ld2450_handle_t handle)
{
    return send_simple_config_cmd(handle, LD2450_FUNC_SET_SINGLE_TARGET);
}

esp_err_t ld2450_set_multi_target(ld2450_handle_t handle)
{
    return send_simple_config_cmd(handle, LD2450_FUNC_SET_MULTI_TARGET);
}

esp_err_t ld2450_get_tracking_mode(ld2450_handle_t handle, ld2450_tracking_mode_t *mode)
{
    uint8_t payload[8];
    uint8_t payload_len = sizeof(payload);
    esp_err_t err = send_config_command(handle, LD2450_FUNC_GET_TRACKING_MODE,
                                         NULL, 0, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 1 && mode != NULL) {
        *mode = (ld2450_tracking_mode_t)payload[0];
    }
    return err;
}

/* ============ System API ============ */

esp_err_t ld2450_get_firmware_version(ld2450_handle_t handle, ld2450_version_t *version)
{
    uint8_t payload[8];
    uint8_t payload_len = sizeof(payload);
    esp_err_t err = send_config_command(handle, LD2450_FUNC_GET_VERSION,
                                         NULL, 0, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 4 && version != NULL) {
        version->major = payload[0] | (payload[1] << 8);
        version->minor = payload[2] | (payload[3] << 8) | ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
    }
    return err;
}

esp_err_t ld2450_set_baud_rate(ld2450_handle_t handle, ld2450_baud_index_t index)
{
    uint8_t data = (uint8_t)index;
    return send_config_command(handle, LD2450_FUNC_SET_BAUDRATE, &data, 1, NULL, NULL);
}

esp_err_t ld2450_factory_reset(ld2450_handle_t handle)
{
    return send_simple_config_cmd(handle, LD2450_FUNC_FACTORY_RESET);
}

esp_err_t ld2450_restart(ld2450_handle_t handle)
{
    return send_simple_config_cmd(handle, LD2450_FUNC_RESTART);
}

/* ============ Bluetooth API ============ */

esp_err_t ld2450_set_bluetooth(ld2450_handle_t handle, bool enable)
{
    uint8_t data = enable ? 0x01 : 0x00;
    return send_config_command(handle, LD2450_FUNC_SET_BLUETOOTH, &data, 1, NULL, NULL);
}

esp_err_t ld2450_get_mac_address(ld2450_handle_t handle, uint8_t *mac)
{
    if (!mac) return ESP_ERR_INVALID_ARG;

    uint8_t payload[8];
    uint8_t payload_len = sizeof(payload);
    esp_err_t err = send_config_command(handle, LD2450_FUNC_GET_MAC,
                                         NULL, 0, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 6) {
        memcpy(mac, payload, 6);
    }
    return err;
}

/* ============ Region Filter API ============ */

esp_err_t ld2450_set_region_filter(ld2450_handle_t handle, const ld2450_region_filter_t *filter)
{
    if (!filter) return ESP_ERR_INVALID_ARG;

    /* Build region filter data:
     * Byte 0: filter type (0=disable, 1=include, 2=exclude)
     * Bytes 1-24: 3 regions × 8 bytes (x1L, x1H, y1L, y1H, x2L, x2H, y2L, y2H)
     */
    uint8_t data[25];
    data[0] = filter->type;

    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        int offset = 1 + i * 8;
        data[offset + 0] = filter->regions[i].x1 & 0xFF;
        data[offset + 1] = (filter->regions[i].x1 >> 8) & 0xFF;
        data[offset + 2] = filter->regions[i].y1 & 0xFF;
        data[offset + 3] = (filter->regions[i].y1 >> 8) & 0xFF;
        data[offset + 4] = filter->regions[i].x2 & 0xFF;
        data[offset + 5] = (filter->regions[i].x2 >> 8) & 0xFF;
        data[offset + 6] = filter->regions[i].y2 & 0xFF;
        data[offset + 7] = (filter->regions[i].y2 >> 8) & 0xFF;
    }

    return send_config_command(handle, LD2450_FUNC_SET_REGION_FILTER, data, 25, NULL, NULL);
}

esp_err_t ld2450_get_region_filter(ld2450_handle_t handle, ld2450_region_filter_t *filter)
{
    if (!filter) return ESP_ERR_INVALID_ARG;

    uint8_t payload[32];
    uint8_t payload_len = sizeof(payload);
    esp_err_t err = send_config_command(handle, LD2450_FUNC_GET_REGION_FILTER,
                                         NULL, 0, payload, &payload_len);
    if (err == ESP_OK && payload_len >= 25) {
        filter->type = payload[0];
        for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
            int offset = 1 + i * 8;
            filter->regions[i].x1 = (int16_t)(payload[offset + 0] | (payload[offset + 1] << 8));
            filter->regions[i].y1 = (int16_t)(payload[offset + 2] | (payload[offset + 3] << 8));
            filter->regions[i].x2 = (int16_t)(payload[offset + 4] | (payload[offset + 5] << 8));
            filter->regions[i].y2 = (int16_t)(payload[offset + 6] | (payload[offset + 7] << 8));
        }
    }
    return err;
}
