/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD2460 24GHz Millimeter Wave Radar Driver
 * Multi-target tracking radar with UART interface
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_event.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============ Constants ============ */

/** Maximum number of targets tracked simultaneously */
#define LD2460_MAX_TARGETS         8

/** Default UART baud rate */
#define LD2460_DEFAULT_BAUD_RATE   115200

/** Default UART port */
#define LD2460_DEFAULT_UART_NUM    UART_NUM_1

/** Report frame header: F4 F3 F2 F1 */
#define LD2460_REPORT_HEADER_0     0xF4
#define LD2460_REPORT_HEADER_1     0xF3
#define LD2460_REPORT_HEADER_2     0xF2
#define LD2460_REPORT_HEADER_3     0xF1

/** Report frame tail: F8 F7 F6 F5 */
#define LD2460_REPORT_TAIL_0       0xF8
#define LD2460_REPORT_TAIL_1       0xF7
#define LD2460_REPORT_TAIL_2       0xF6
#define LD2460_REPORT_TAIL_3       0xF5

/** Command/ACK frame header: FD FC FB FA */
#define LD2460_CMD_HEADER_0        0xFD
#define LD2460_CMD_HEADER_1        0xFC
#define LD2460_CMD_HEADER_2        0xFB
#define LD2460_CMD_HEADER_3        0xFA

/** Command/ACK frame tail: 04 03 02 01 */
#define LD2460_CMD_TAIL_0          0x04
#define LD2460_CMD_TAIL_1          0x03
#define LD2460_CMD_TAIL_2          0x02
#define LD2460_CMD_TAIL_3          0x01

/** Function codes */
#define LD2460_FUNC_REPORT         0x04
#define LD2460_FUNC_REPORT_CTRL    0x06
#define LD2460_FUNC_SET_INSTALL    0x07
#define LD2460_FUNC_GET_INSTALL    0x08
#define LD2460_FUNC_SET_MODE       0x09
#define LD2460_FUNC_GET_MODE       0x0A
#define LD2460_FUNC_GET_VERSION    0x0B
#define LD2460_FUNC_RESTART        0x0D
#define LD2460_FUNC_SET_BAUDRATE   0x0E
#define LD2460_FUNC_FACTORY_RESET  0x10
#define LD2460_FUNC_SET_RANGE      0x11
#define LD2460_FUNC_GET_RANGE      0x12
#define LD2460_FUNC_SET_SENSITIVITY 0x13
#define LD2460_FUNC_GET_SENSITIVITY 0x14

/** Command timeout in milliseconds */
#define LD2460_CMD_TIMEOUT_MS      1000

/* ============ Data Structures ============ */

/**
 * @brief Single target information from LD2460
 */
typedef struct {
    int16_t x;    /*!< X coordinate, raw value. Actual = x * 0.1 (unit: 0.1m) */
    int16_t y;    /*!< Y coordinate, raw value. Actual = y * 0.1 (unit: 0.1m) */
} ld2460_target_t;

/**
 * @brief LD2460 report data (full frame payload)
 */
typedef struct {
    uint8_t target_count;                            /*!< Number of valid targets */
    ld2460_target_t targets[LD2460_MAX_TARGETS];     /*!< Target array */
} ld2460_data_t;

/**
 * @brief Installation mode
 */
typedef enum {
    LD2460_INSTALL_SIDE = 0x01,  /*!< Side mount (侧装) */
    LD2460_INSTALL_TOP  = 0x02,  /*!< Top mount (顶装) */
} ld2460_install_mode_t;

/**
 * @brief Installation parameters
 */
typedef struct {
    float height;   /*!< Installation height in meters */
    float angle;    /*!< Installation angle in degrees */
} ld2460_install_params_t;

/**
 * @brief Detection range parameters
 */
typedef struct {
    float distance;     /*!< Detection distance in meters */
    float angle_start;  /*!< Start angle in degrees (can be negative for side mount) */
    float angle_end;    /*!< End angle in degrees */
} ld2460_range_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    ld2460_install_mode_t mode;  /*!< Installation mode */
    uint8_t year;                /*!< Year (e.g. 25 for 2025) */
    uint8_t month;               /*!< Month (1-12) */
    uint8_t version_major;       /*!< Major version */
    uint8_t version_minor;       /*!< Minor version */
} ld2460_version_t;

/**
 * @brief Sensitivity level
 */
typedef enum {
    LD2460_SENSITIVITY_HIGH = 0x01,  /*!< High sensitivity */
    LD2460_SENSITIVITY_MID  = 0x02,  /*!< Medium sensitivity */
    LD2460_SENSITIVITY_LOW  = 0x03,  /*!< Low sensitivity */
} ld2460_sensitivity_t;

/**
 * @brief Baud rate index
 */
typedef enum {
    LD2460_BAUD_9600   = 0x00,
    LD2460_BAUD_19200  = 0x01,
    LD2460_BAUD_38400  = 0x02,
    LD2460_BAUD_57600  = 0x03,
    LD2460_BAUD_115200 = 0x04,  /*!< Factory default */
    LD2460_BAUD_230400 = 0x05,
    LD2460_BAUD_256000 = 0x06,
    LD2460_BAUD_460800 = 0x07,
} ld2460_baud_rate_index_t;

/* ============ Event Definitions ============ */

ESP_EVENT_DECLARE_BASE(ESP_LD2460_EVENT);

/**
 * @brief LD2460 event IDs
 */
typedef enum {
    LD2460_EVENT_TARGET_UPDATE,   /*!< New target data received */
    LD2460_EVENT_CMD_ACK,         /*!< Command acknowledged (internal use) */
} ld2460_event_id_t;

/* ============ Configuration ============ */

/**
 * @brief LD2460 driver configuration
 */
typedef struct {
    uart_port_t uart_port;        /*!< UART port number */
    int tx_pin;                   /*!< UART TX pin (-1 = not used) */
    int rx_pin;                   /*!< UART RX pin */
    uint32_t baud_rate;           /*!< UART baud rate (0 = use default 115200) */
    uint32_t event_queue_size;    /*!< UART event queue size */
    uint32_t ring_buffer_size;    /*!< UART ring buffer size */
    uint32_t task_stack_size;     /*!< Parser task stack size */
    int task_priority;            /*!< Parser task priority */
} ld2460_config_t;

/**
 * @brief Default configuration macro
 */
#define LD2460_CONFIG_DEFAULT()               \
    {                                         \
        .uart_port = UART_NUM_1,              \
        .tx_pin = CONFIG_RADAR_UART_RX_PIN,   /* 互换: TX->RX */ \
        .rx_pin = CONFIG_RADAR_UART_TX_PIN,   /* 互换: RX->TX */ \
        .baud_rate = 0,                       \
        .event_queue_size = 16,               \
        .ring_buffer_size = 1024,             \
        .task_stack_size = 4096,              \
        .task_priority = 10,                  \
    }

/* ============ Handle ============ */

/** Opaque LD2460 driver handle */
typedef struct ld2460_context *ld2460_handle_t;

/* ============ Lifecycle API ============ */

/**
 * @brief Initialize LD2460 driver
 *
 * @param config  Pointer to configuration
 * @return LD2460 handle on success, NULL on failure
 */
ld2460_handle_t ld2460_init(const ld2460_config_t *config);

/**
 * @brief Deinitialize LD2460 driver
 *
 * @param handle  LD2460 handle
 * @return ESP_OK on success
 */
esp_err_t ld2460_deinit(ld2460_handle_t handle);

/* ============ Event Handler API ============ */

/**
 * @brief Register event handler for target data updates
 *
 * @param handle         LD2460 handle
 * @param event_handler  User event handler
 * @param handler_args   Handler arguments
 * @return ESP_OK on success
 */
esp_err_t ld2460_add_handler(ld2460_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *handler_args);

/**
 * @brief Unregister event handler
 *
 * @param handle         LD2460 handle
 * @param event_handler  User event handler to remove
 * @return ESP_OK on success
 */
esp_err_t ld2460_remove_handler(ld2460_handle_t handle,
                                 esp_event_handler_t event_handler);

/* ============ Report Control API ============ */

/**
 * @brief Enable or disable radar report
 *
 * @param handle  LD2460 handle
 * @param enable  true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t ld2460_enable_report(ld2460_handle_t handle, bool enable);

/* ============ Installation API ============ */

/**
 * @brief Set installation mode (side/top mount)
 *
 * @param handle  LD2460 handle
 * @param mode    Installation mode
 * @return ESP_OK on success
 */
esp_err_t ld2460_set_install_mode(ld2460_handle_t handle, ld2460_install_mode_t mode);

/**
 * @brief Get current installation mode
 *
 * @param handle  LD2460 handle
 * @param mode    Output: current installation mode
 * @return ESP_OK on success
 */
esp_err_t ld2460_get_install_mode(ld2460_handle_t handle, ld2460_install_mode_t *mode);

/**
 * @brief Set installation parameters (height and angle)
 * Only applicable in side-mount mode. Parameters persist after power-off.
 *
 * @param handle  LD2460 handle
 * @param height  Installation height in meters
 * @param angle   Installation angle in degrees
 * @return ESP_OK on success
 */
esp_err_t ld2460_set_install_params(ld2460_handle_t handle, float height, float angle);

/**
 * @brief Get current installation parameters
 *
 * @param handle  LD2460 handle
 * @param params  Output: installation parameters
 * @return ESP_OK on success
 */
esp_err_t ld2460_get_install_params(ld2460_handle_t handle, ld2460_install_params_t *params);

/* ============ Detection Range API ============ */

/**
 * @brief Set detection range (distance and angle)
 * Side mount: max 6m, ±60°. Top mount: max 4m, 0-360°.
 *
 * @param handle      LD2460 handle
 * @param distance    Detection distance in meters
 * @param angle_start Start angle in degrees
 * @param angle_end   End angle in degrees
 * @return ESP_OK on success
 */
esp_err_t ld2460_set_detection_range(ld2460_handle_t handle,
                                      float distance,
                                      float angle_start,
                                      float angle_end);

/**
 * @brief Get current detection range
 *
 * @param handle      LD2460 handle
 * @param range       Output: detection range
 * @return ESP_OK on success
 */
esp_err_t ld2460_get_detection_range(ld2460_handle_t handle, ld2460_range_t *range);

/* ============ System API ============ */

/**
 * @brief Get firmware version
 *
 * @param handle  LD2460 handle
 * @param version Output: firmware version info
 * @return ESP_OK on success
 */
esp_err_t ld2460_get_firmware_version(ld2460_handle_t handle, ld2460_version_t *version);

/**
 * @brief Restart radar module
 *
 * @param handle  LD2460 handle
 * @return ESP_OK on success
 */
esp_err_t ld2460_restart(ld2460_handle_t handle);

/**
 * @brief Set UART baud rate (persists after power-off, takes effect after restart)
 *
 * @param handle  LD2460 handle
 * @param index   Baud rate index (see ld2460_baud_rate_index_t)
 * @return ESP_OK on success
 */
esp_err_t ld2460_set_baud_rate(ld2460_handle_t handle, ld2460_baud_rate_index_t index);

/**
 * @brief Factory reset (baudrate→115200, side mount, height 2.6m, angle 30°)
 *
 * @param handle  LD2460 handle
 * @return ESP_OK on success
 */
esp_err_t ld2460_factory_reset(ld2460_handle_t handle);

/* ============ Sensitivity API (Reserved) ============ */

/**
 * @brief Set sensitivity level
 *
 * @param handle  LD2460 handle
 * @param level   Sensitivity level
 * @return ESP_OK on success
 */
esp_err_t ld2460_set_sensitivity(ld2460_handle_t handle, ld2460_sensitivity_t level);

/**
 * @brief Get current sensitivity level
 *
 * @param handle  LD2460 handle
 * @param level   Output: current sensitivity level
 * @return ESP_OK on success
 */
esp_err_t ld2460_get_sensitivity(ld2460_handle_t handle, ld2460_sensitivity_t *level);

#ifdef __cplusplus
}
#endif
