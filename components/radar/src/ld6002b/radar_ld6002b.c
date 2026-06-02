/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD6002B 3D Millimeter Wave Radar Driver
 *
 * Features:
 * - TinyFrame (TF) protocol frame parsing with state machine
 * - Async UART event-driven architecture
 * - Full command set: control, area, sensitivity, low power, etc.
 * - esp_event based notification to application layer
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "radar_ld6002b.h"

static const char *TAG = "ld6002b";

/* ============ Event base definition ============ */
ESP_EVENT_DEFINE_BASE(ESP_LD6002B_EVENT);

/* ============ Internal constants ============ */
#define LD6002B_TF_HEADER_LEN       7   /* SOF(1) + ID(2) + LEN(2) + TYPE(2) */
#define LD6002B_EVENT_LOOP_QUEUE    16
#define LD6002B_RX_BUFFER_SIZE      1024

/* ============ TF Parser states ============ */
typedef enum {
    TF_PARSE_IDLE = 0,
    TF_PARSE_ID_HI,         /* Expecting ID high byte (byte 1, after SOF) */
    TF_PARSE_ID_LO,         /* Expecting ID low byte (byte 2) */
    TF_PARSE_LEN_HI,        /* Expecting LEN high byte (byte 3) */
    TF_PARSE_LEN_LO,        /* Expecting LEN low byte (byte 4) */
    TF_PARSE_TYPE_HI,       /* Expecting TYPE high byte (byte 5) */
    TF_PARSE_TYPE_LO,       /* Expecting TYPE low byte (byte 6) */
    TF_PARSE_HEAD_CKSUM,    /* Expecting HEAD_CKSUM byte (byte 7) */
    TF_PARSE_DATA,          /* Reading DATA bytes */
    TF_PARSE_DATA_CKSUM,    /* Expecting DATA_CKSUM byte */
} tf_parse_state_t;

/* ============ Internal context structure ============ */
struct ld6002b_context {
    uart_port_t       uart_port;
    uint32_t          baud_rate;
    uint8_t          *rx_buffer;
    QueueHandle_t     event_queue;
    esp_event_loop_handle_t event_loop_hdl;
    TaskHandle_t      task_hdl;

    /* TF frame parser state machine */
    tf_parse_state_t  parse_state;
    uint8_t           frame_header[LD6002B_TF_HEADER_LEN]; /* SOF + ID + LEN + TYPE */
    uint16_t          frame_id;
    uint16_t          frame_len;       /* DATA length only */
    uint16_t          frame_type;
    uint16_t          data_pos;        /* Bytes of DATA read so far */
    uint8_t          *frame_data;      /* Buffer for DATA payload */
    uint16_t          frame_data_buf_size; /* Allocated size of frame_data */

    /* Command synchronization */
    SemaphoreHandle_t cmd_sem;
    bool              cmd_success;
    uint16_t          cmd_resp_type;   /* Expected response TYPE */
    bool              cmd_expect_data; /* Whether response should carry DATA */
    uint16_t          resp_type;       /* Actual response TYPE */
    uint8_t           resp_data[LD6002B_RX_BUFFER_SIZE]; /* Response DATA */
    uint16_t          resp_data_len;   /* Response DATA length */

    /* Frame ID counter (incrementing) */
    uint16_t          frame_id_counter;
};

/* ============ Helper: TinyFrame checksum ============ */
static uint8_t tf_cksum(const uint8_t *data, size_t len)
{
    uint8_t ret = 0;
    for (size_t i = 0; i < len; i++) {
        ret ^= data[i];
    }
    return ~ret;
}

/* ============ Helper: read float (little-endian) ============ */
static float read_float_le(const uint8_t *buf)
{
    uint32_t val = buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    float f;
    memcpy(&f, &val, sizeof(f));
    return f;
}

/* ============ Helper: read int32 (little-endian) ============ */
static int32_t read_int32_le(const uint8_t *buf)
{
    return (int32_t)(buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

/* ============ Helper: write uint16 big-endian ============ */
static void write_u16_be(uint8_t *buf, uint16_t val)
{
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

/* ============ Helper: write int32 little-endian ============ */
static void write_i32_le(uint8_t *buf, int32_t val)
{
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
}

/* ============ Helper: write float little-endian ============ */
static void write_float_le(uint8_t *buf, float f)
{
    uint32_t val;
    memcpy(&val, &f, sizeof(val));
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
}

/* ============ Helper: build TF frame ============ */
static int build_tf_frame(struct ld6002b_context *ctx, uint8_t *buf,
                           uint16_t msg_type, const uint8_t *data, uint16_t data_len)
{
    int pos = 0;

    /* SOF */
    buf[pos++] = LD6002B_TF_SOF;

    /* ID (big-endian, incrementing) */
    ctx->frame_id_counter++;
    write_u16_be(buf + pos, ctx->frame_id_counter);
    pos += 2;

    /* LEN = data length (big-endian) */
    write_u16_be(buf + pos, data_len);
    pos += 2;

    /* TYPE (big-endian) */
    write_u16_be(buf + pos, msg_type);
    pos += 2;

    /* HEAD_CKSUM: XOR of bytes[0..6], then NOT */
    uint8_t hck = tf_cksum(buf, LD6002B_TF_HEADER_LEN);
    buf[pos++] = hck;

    /* DATA (little-endian fields) */
    if (data_len > 0 && data != NULL) {
        memcpy(buf + pos, data, data_len);
        pos += data_len;
    }

    /* DATA_CKSUM: XOR of all DATA bytes, then NOT */
    if (data_len > 0) {
        uint8_t dck = tf_cksum(data, data_len);
        buf[pos++] = dck;
    } else {
        /* No data: DATA_CKSUM = ~0 = 0xFF */
        buf[pos++] = 0xFF;
    }

    return pos;
}

/* ============ Helper: send TF command and wait for response ============ */
static esp_err_t send_tf_command(ld6002b_handle_t handle,
                                  uint16_t msg_type,
                                  const uint8_t *data, uint16_t data_len,
                                  bool expect_data)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    uint8_t cmd_buf[LD6002B_MAX_FRAME_SIZE];
    int cmd_len = build_tf_frame(ctx, cmd_buf, msg_type, data, data_len);

    /* Setup response expectation */
    ctx->cmd_success = false;
    ctx->cmd_resp_type = msg_type;
    ctx->cmd_expect_data = expect_data;
    ctx->resp_data_len = 0;

    /* Send command */
    int written = uart_write_bytes(ctx->uart_port, cmd_buf, cmd_len);
    if (written != cmd_len) {
        ESP_LOGE(TAG, "Failed to send TF command 0x%04X (written %d/%d)",
                 msg_type, written, cmd_len);
        return ESP_FAIL;
    }

    /* Wait for response with timeout */
    if (xSemaphoreTake(ctx->cmd_sem, pdMS_TO_TICKS(LD6002B_CMD_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "TF command 0x%04X timeout", msg_type);
        return ESP_ERR_TIMEOUT;
    }

    if (!ctx->cmd_success) {
        ESP_LOGW(TAG, "TF command 0x%04X failed", msg_type);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ============ Helper: send 0x0201 sub-command (SET, no data response) ============ */
static esp_err_t send_control_cmd(ld6002b_handle_t handle, int32_t sub_cmd)
{
    uint8_t data[4];
    write_i32_le(data, sub_cmd);
    return send_tf_command(handle, LD6002B_MSG_SET_CONTROL_CMD, data, 4, false);
}

/* ============ Helper: send 0x0201 sub-command with 1-byte parameter ============ */
static esp_err_t send_control_cmd_with_param(ld6002b_handle_t handle,
                                              int32_t sub_cmd, uint8_t param)
{
    uint8_t data[5];
    write_i32_le(data, sub_cmd);
    data[4] = param;
    return send_tf_command(handle, LD6002B_MSG_SET_CONTROL_CMD, data, 5, false);
}

/* ============ Helper: send 0x0201 query sub-command (expects data response) ============ */
static esp_err_t send_control_query(ld6002b_handle_t handle, int32_t sub_cmd)
{
    uint8_t data[4];
    write_i32_le(data, sub_cmd);
    return send_tf_command(handle, LD6002B_MSG_SET_CONTROL_CMD, data, 4, true);
}

/* ============ TF Parser: reset state machine ============ */
static void tf_parser_reset(struct ld6002b_context *ctx)
{
    ctx->parse_state = TF_PARSE_IDLE;
    ctx->data_pos = 0;
    ctx->frame_len = 0;
    ctx->frame_type = 0;
    ctx->frame_id = 0;
}

/* ============ TF Parser: dispatch completed frame ============ */
static void tf_dispatch_frame(struct ld6002b_context *ctx)
{
    uint16_t type = ctx->frame_type;

    /* Check if this is a response to a pending command */
    if (ctx->cmd_resp_type != 0 && type == ctx->cmd_resp_type) {
        if (ctx->cmd_expect_data) {
            /* Query response: should have DATA */
            if (ctx->frame_len > 0) {
                ctx->cmd_success = true;
                ctx->resp_type = type;
                ctx->resp_data_len = ctx->frame_len;
                if (ctx->frame_len > sizeof(ctx->resp_data)) {
                    ctx->resp_data_len = sizeof(ctx->resp_data);
                }
                memcpy(ctx->resp_data, ctx->frame_data, ctx->resp_data_len);
            } else {
                ESP_LOGW(TAG, "Expected data in response 0x%04X but got none", type);
                ctx->cmd_success = false;
            }
        } else {
            /* ACK response: should have LEN=0 (no DATA) */
            ctx->cmd_success = true;
            ctx->resp_type = type;
            ctx->resp_data_len = 0;
        }
        ctx->cmd_resp_type = 0;
        xSemaphoreGive(ctx->cmd_sem);
        return;
    }

    /* ---- Report dispatch ---- */

    if (type == LD6002B_MSG_REPORT_TARGET) {
        /* TYPE 0x0A04: Target data
         * DATA layout: target_count(1B) + per target: x(4f) y(4f) z(4f) dop_idx(4i) cluster_id(4i)
         * = 1 + target_count * 20
         */
        ld6002b_data_t report;
        memset(&report, 0, sizeof(report));

        if (ctx->frame_len >= 1) {
            report.target_count = ctx->frame_data[0];
            if (report.target_count > LD6002B_MAX_TARGETS) {
                report.target_count = LD6002B_MAX_TARGETS;
            }

            uint16_t offset = 1;
            for (uint8_t i = 0; i < report.target_count; i++) {
                if (offset + 20 <= ctx->frame_len) {
                    report.targets[i].x = read_float_le(ctx->frame_data + offset);
                    offset += 4;
                    report.targets[i].y = read_float_le(ctx->frame_data + offset);
                    offset += 4;
                    report.targets[i].z = read_float_le(ctx->frame_data + offset);
                    offset += 4;
                    report.targets[i].dop_idx = read_int32_le(ctx->frame_data + offset);
                    offset += 4;
                    report.targets[i].cluster_id = read_int32_le(ctx->frame_data + offset);
                    offset += 4;
                } else {
                    ESP_LOGW(TAG, "Target %d data truncated", i);
                    break;
                }
            }
        }

        esp_event_post_to(ctx->event_loop_hdl, ESP_LD6002B_EVENT,
                          LD6002B_EVENT_TARGET_UPDATE,
                          &report, sizeof(ld6002b_data_t),
                          pdMS_TO_TICKS(100));
    }
    else if (type == LD6002B_MSG_REPORT_AREA_STATE) {
        /* TYPE 0x0A0A: Area state (detection areas) */
        ld6002b_area_state_t area_state;
        memset(&area_state, 0, sizeof(area_state));

        if (ctx->frame_len >= 8) {
            memcpy(area_state.states, ctx->frame_data, 8);
        } else if (ctx->frame_len > 0) {
            memcpy(area_state.states, ctx->frame_data, ctx->frame_len);
        }

        esp_event_post_to(ctx->event_loop_hdl, ESP_LD6002B_EVENT,
                          LD6002B_EVENT_AREA_STATE,
                          &area_state, sizeof(ld6002b_area_state_t),
                          pdMS_TO_TICKS(100));
    }
    else {
        /* Other report types: log for debugging */
        ESP_LOGD(TAG, "Unhandled report TYPE 0x%04X, LEN=%d", type, ctx->frame_len);
    }
}

/* ============ TF Parser: process one byte ============ */
static void tf_parser_feed_byte(struct ld6002b_context *ctx, uint8_t byte)
{
    switch (ctx->parse_state) {

    case TF_PARSE_IDLE:
        if (byte == LD6002B_TF_SOF) {
            ctx->frame_header[0] = byte;  /* SOF at index 0 */
            ctx->parse_state = TF_PARSE_ID_HI;
        }
        break;

    case TF_PARSE_ID_HI:
        /* Byte 1: ID high byte */
        ctx->frame_header[1] = byte;
        ctx->parse_state = TF_PARSE_ID_LO;
        break;

    case TF_PARSE_ID_LO:
        /* Byte 2: ID low byte */
        ctx->frame_header[2] = byte;
        ctx->parse_state = TF_PARSE_LEN_HI;
        break;

    case TF_PARSE_LEN_HI:
        /* Byte 3: LEN high byte */
        ctx->frame_header[3] = byte;
        ctx->parse_state = TF_PARSE_LEN_LO;
        break;

    case TF_PARSE_LEN_LO:
        /* Byte 4: LEN low byte */
        ctx->frame_header[4] = byte;
        ctx->parse_state = TF_PARSE_TYPE_HI;
        break;

    case TF_PARSE_TYPE_HI:
        /* Byte 5: TYPE high byte */
        ctx->frame_header[5] = byte;
        ctx->parse_state = TF_PARSE_TYPE_LO;
        break;

    case TF_PARSE_TYPE_LO:
        /* Byte 6: TYPE low byte */
        ctx->frame_header[6] = byte;
        ctx->parse_state = TF_PARSE_HEAD_CKSUM;
        break;

    case TF_PARSE_HEAD_CKSUM:
        /* Byte 7: HEAD_CKSUM */
        {
            uint8_t expected_hck = tf_cksum(ctx->frame_header, LD6002B_TF_HEADER_LEN);
            if (byte != expected_hck) {
                ESP_LOGW(TAG, "Header checksum mismatch: got 0x%02X, expected 0x%02X",
                         byte, expected_hck);
                tf_parser_reset(ctx);
                break;
            }
        }

        /* Parse header fields */
        ctx->frame_id   = ((uint16_t)ctx->frame_header[1] << 8) | ctx->frame_header[2];
        ctx->frame_len  = ((uint16_t)ctx->frame_header[3] << 8) | ctx->frame_header[4];
        ctx->frame_type = ((uint16_t)ctx->frame_header[5] << 8) | ctx->frame_header[6];

        /* Validate frame length */
        if (ctx->frame_len > LD6002B_MAX_FRAME_SIZE) {
            ESP_LOGW(TAG, "Frame LEN too large: %d, ignoring", ctx->frame_len);
            tf_parser_reset(ctx);
            break;
        }

        /* Ensure frame_data buffer is large enough */
        if (ctx->frame_len > 0) {
            if (ctx->frame_len > ctx->frame_data_buf_size) {
                uint8_t *new_buf = realloc(ctx->frame_data, ctx->frame_len);
                if (!new_buf) {
                    ESP_LOGW(TAG, "Failed to allocate frame data buffer (%d bytes)", ctx->frame_len);
                    tf_parser_reset(ctx);
                    break;
                }
                ctx->frame_data = new_buf;
                ctx->frame_data_buf_size = ctx->frame_len;
            }
            ctx->data_pos = 0;
            ctx->parse_state = TF_PARSE_DATA;
        } else {
            /* No DATA, go directly to DATA_CKSUM */
            ctx->parse_state = TF_PARSE_DATA_CKSUM;
        }
        break;

    case TF_PARSE_DATA:
        if (ctx->data_pos < ctx->frame_len) {
            ctx->frame_data[ctx->data_pos++] = byte;
            if (ctx->data_pos >= ctx->frame_len) {
                ctx->parse_state = TF_PARSE_DATA_CKSUM;
            }
        } else {
            /* Should not happen, reset to be safe */
            tf_parser_reset(ctx);
        }
        break;

    case TF_PARSE_DATA_CKSUM:
        {
            uint8_t expected_dck;
            if (ctx->frame_len > 0 && ctx->frame_data != NULL) {
                expected_dck = tf_cksum(ctx->frame_data, ctx->frame_len);
            } else {
                /* No data: expected checksum = ~0 = 0xFF */
                expected_dck = 0xFF;
            }
            if (byte != expected_dck) {
                ESP_LOGW(TAG, "Data checksum mismatch: got 0x%02X, expected 0x%02X",
                         byte, expected_dck);
                tf_parser_reset(ctx);
                break;
            }
        }

        /* Frame complete, dispatch */
        tf_dispatch_frame(ctx);
        tf_parser_reset(ctx);
        break;

    default:
        tf_parser_reset(ctx);
        break;
    }
}

/* ============ Process UART data: feed bytes to TF parser ============ */
static void process_uart_data(struct ld6002b_context *ctx, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        tf_parser_feed_byte(ctx, data[i]);
    }
}

/* ============ UART event task ============ */

static void ld6002b_task_entry(void *arg)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)arg;
    uart_event_t event;
    uint8_t *tmp_buf = malloc(LD6002B_RX_BUFFER_SIZE);
    if (!tmp_buf) {
        ESP_LOGE(TAG, "Failed to allocate temp buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(ctx->event_queue, &event, pdMS_TO_TICKS(200))) {
            switch (event.type) {
            case UART_DATA:
                {
                    size_t buffered_len = 0;
                    uart_get_buffered_data_len(ctx->uart_port, &buffered_len);
                    if (buffered_len > 0) {
                        size_t read_len = buffered_len;
                        if (read_len > LD6002B_RX_BUFFER_SIZE) {
                            read_len = LD6002B_RX_BUFFER_SIZE;
                        }
                        int actual = uart_read_bytes(ctx->uart_port, tmp_buf,
                                                     read_len, pdMS_TO_TICKS(100));
                        if (actual > 0) {
                            process_uart_data(ctx, tmp_buf, (size_t)actual);
                        }
                    }
                }
                break;

            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "HW FIFO Overflow");
                uart_flush_input(ctx->uart_port);
                xQueueReset(ctx->event_queue);
                tf_parser_reset(ctx);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "Ring Buffer Full");
                uart_flush_input(ctx->uart_port);
                xQueueReset(ctx->event_queue);
                tf_parser_reset(ctx);
                break;

            case UART_BREAK:
                ESP_LOGW(TAG, "Rx Break");
                tf_parser_reset(ctx);
                break;

            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "Parity Error");
                tf_parser_reset(ctx);
                break;

            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "Frame Error");
                tf_parser_reset(ctx);
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

ld6002b_handle_t ld6002b_init(const ld6002b_config_t *config)
{
    if (!config) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }

    struct ld6002b_context *ctx = calloc(1, sizeof(struct ld6002b_context));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate context");
        return NULL;
    }

    /* Allocate RX buffer */
    ctx->rx_buffer = calloc(1, LD6002B_RX_BUFFER_SIZE);
    if (!ctx->rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        free(ctx);
        return NULL;
    }

    /* Store config */
    ctx->uart_port = config->uart_port;
    ctx->baud_rate = config->baud_rate > 0 ? config->baud_rate : LD6002B_DEFAULT_BAUD_RATE;

    /* Initialize parser */
    ctx->parse_state = TF_PARSE_IDLE;
    ctx->frame_id_counter = 0;

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
        .queue_size = LD6002B_EVENT_LOOP_QUEUE,
        .task_name = NULL,
    };
    if (esp_event_loop_create(&loop_args, &ctx->event_loop_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop");
        goto err_eloop;
    }

    /* Create parser task */
    uint32_t stack_size = config->task_stack_size > 0 ? config->task_stack_size : 4096;
    int priority = config->task_priority;
    BaseType_t ret = xTaskCreate(ld6002b_task_entry, "ld6002b_parse",
                                  stack_size, ctx, priority, &ctx->task_hdl);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "Failed to create parser task");
        goto err_task;
    }

    ESP_LOGI(TAG, "LD6002B driver initialized (UART%d, %lu baud, TX=GPIO%d, RX=GPIO%d)",
             ctx->uart_port + 1, (unsigned long)ctx->baud_rate, tx_pin, config->rx_pin);
    return (ld6002b_handle_t)ctx;

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

esp_err_t ld6002b_deinit(ld6002b_handle_t handle)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;

    vTaskDelete(ctx->task_hdl);
    esp_event_loop_delete(ctx->event_loop_hdl);
    uart_driver_delete(ctx->uart_port);
    vSemaphoreDelete(ctx->cmd_sem);
    if (ctx->frame_data) {
        free(ctx->frame_data);
    }
    free(ctx->rx_buffer);
    free(ctx);
    return ESP_OK;
}

esp_err_t ld6002b_add_handler(ld6002b_handle_t handle,
                               esp_event_handler_t event_handler,
                               void *handler_args)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_register_with(ctx->event_loop_hdl,
                                           ESP_LD6002B_EVENT,
                                           ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t ld6002b_remove_handler(ld6002b_handle_t handle,
                                  esp_event_handler_t event_handler)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx) return ESP_ERR_INVALID_ARG;
    return esp_event_handler_unregister_with(ctx->event_loop_hdl,
                                             ESP_LD6002B_EVENT,
                                             ESP_EVENT_ANY_ID,
                                             event_handler);
}

/* ============================================================ */
/* ============ Control Command API (0x0201 sub-commands) ===== */
/* ============================================================ */

esp_err_t ld6002b_auto_gen_noise(ld6002b_handle_t handle)
{
    return send_control_cmd(handle, LD6002B_CMD_AUTO_GEN_NOISE);
}

esp_err_t ld6002b_get_areas(ld6002b_handle_t handle)
{
    return send_control_query(handle, LD6002B_CMD_GET_AREAS);
}

esp_err_t ld6002b_clear_noise(ld6002b_handle_t handle)
{
    return send_control_cmd(handle, LD6002B_CMD_CLEAR_NOISE);
}

esp_err_t ld6002b_reset_detection(ld6002b_handle_t handle)
{
    return send_control_cmd(handle, LD6002B_CMD_RESET_DETECTION);
}

esp_err_t ld6002b_get_hold_delay(ld6002b_handle_t handle)
{
    return send_control_query(handle, LD6002B_CMD_GET_HOLD_DELAY);
}

esp_err_t ld6002b_set_point_cloud(ld6002b_handle_t handle, bool enable)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_POINT_CLOUD,
                                        enable ? 1 : 0);
}

esp_err_t ld6002b_set_target_display(ld6002b_handle_t handle, uint8_t mode)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_TARGET_DISPLAY, mode);
}

esp_err_t ld6002b_set_sensitivity(ld6002b_handle_t handle, ld6002b_sensitivity_t level)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_SENSITIVITY, (uint8_t)level);
}

esp_err_t ld6002b_get_sensitivity(ld6002b_handle_t handle, ld6002b_sensitivity_t *level)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx || !level) return ESP_ERR_INVALID_ARG;

    esp_err_t err = send_control_query(handle, LD6002B_CMD_GET_SENSITIVITY);
    if (err == ESP_OK && ctx->resp_data_len >= 1) {
        *level = (ld6002b_sensitivity_t)ctx->resp_data[0];
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "Sensitivity response too short (%d bytes)", ctx->resp_data_len);
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t ld6002b_set_trigger_speed(ld6002b_handle_t handle, ld6002b_trigger_speed_t speed)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_TRIGGER_SPEED, (uint8_t)speed);
}

esp_err_t ld6002b_get_trigger_speed(ld6002b_handle_t handle, ld6002b_trigger_speed_t *speed)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx || !speed) return ESP_ERR_INVALID_ARG;

    esp_err_t err = send_control_query(handle, LD6002B_CMD_GET_TRIGGER_SPEED);
    if (err == ESP_OK && ctx->resp_data_len >= 1) {
        *speed = (ld6002b_trigger_speed_t)ctx->resp_data[0];
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "Trigger speed response too short (%d bytes)", ctx->resp_data_len);
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t ld6002b_get_z_range(ld6002b_handle_t handle)
{
    return send_control_query(handle, LD6002B_CMD_GET_Z_RANGE);
}

esp_err_t ld6002b_set_install_mode(ld6002b_handle_t handle, ld6002b_install_mode_t mode)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_INSTALL_MODE, (uint8_t)mode);
}

esp_err_t ld6002b_get_install_mode(ld6002b_handle_t handle, ld6002b_install_mode_t *mode)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx || !mode) return ESP_ERR_INVALID_ARG;

    esp_err_t err = send_control_query(handle, LD6002B_CMD_GET_INSTALL_MODE);
    if (err == ESP_OK && ctx->resp_data_len >= 1) {
        *mode = (ld6002b_install_mode_t)ctx->resp_data[0];
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "Install mode response too short (%d bytes)", ctx->resp_data_len);
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t ld6002b_set_low_power_mode(ld6002b_handle_t handle, bool enable)
{
    return send_control_cmd_with_param(handle, LD6002B_CMD_SET_LOW_POWER, enable ? 1 : 0);
}

esp_err_t ld6002b_get_low_power_mode(ld6002b_handle_t handle, bool *enabled)
{
    struct ld6002b_context *ctx = (struct ld6002b_context *)handle;
    if (!ctx || !enabled) return ESP_ERR_INVALID_ARG;

    esp_err_t err = send_control_query(handle, LD6002B_CMD_GET_LOW_POWER);
    if (err == ESP_OK && ctx->resp_data_len >= 1) {
        *enabled = (ctx->resp_data[0] != 0);
    } else if (err == ESP_OK) {
        ESP_LOGW(TAG, "Low power mode response too short (%d bytes)", ctx->resp_data_len);
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t ld6002b_get_low_power_time(ld6002b_handle_t handle)
{
    return send_control_query(handle, LD6002B_CMD_GET_LOW_POWER_TIME);
}

esp_err_t ld6002b_reset_unoccupied(ld6002b_handle_t handle)
{
    return send_control_cmd(handle, LD6002B_CMD_RESET_UNOCCUPIED);
}

/* ============================================================ */
/* ============ Area Configuration API (0x0202) ============== */
/* ============================================================ */

esp_err_t ld6002b_set_area(ld6002b_handle_t handle, const ld6002b_area_t *area)
{
    if (!area) return ESP_ERR_INVALID_ARG;

    /* DATA: 6 floats in little-endian (x_min, x_max, y_min, y_max, z_min, z_max) */
    uint8_t data[24];
    write_float_le(data + 0,  area->x_min);
    write_float_le(data + 4,  area->x_max);
    write_float_le(data + 8,  area->y_min);
    write_float_le(data + 12, area->y_max);
    write_float_le(data + 16, area->z_min);
    write_float_le(data + 20, area->z_max);
    return send_tf_command(handle, LD6002B_MSG_SET_AREA, data, 24, false);
}

/* ============================================================ */
/* ============ Separate SET Command API ====================== */
/* ============================================================ */

esp_err_t ld6002b_set_hold_delay(ld6002b_handle_t handle, uint8_t delay)
{
    return send_tf_command(handle, LD6002B_MSG_SET_HOLD_DELAY, &delay, 1, false);
}

esp_err_t ld6002b_set_z_range(ld6002b_handle_t handle, float z_min, float z_max)
{
    uint8_t data[8];
    write_float_le(data + 0, z_min);
    write_float_le(data + 4, z_max);
    return send_tf_command(handle, LD6002B_MSG_SET_Z_RANGE, data, 8, false);
}

esp_err_t ld6002b_set_low_power_time(ld6002b_handle_t handle, uint8_t time)
{
    return send_tf_command(handle, LD6002B_MSG_SET_LOW_POWER, &time, 1, false);
}
