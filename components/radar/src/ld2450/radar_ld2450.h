/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD2450 24GHz Millimeter Wave Radar Driver
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
#define LD2450_MAX_TARGETS         3

/** Default UART baud rate */
#define LD2450_DEFAULT_BAUD_RATE   256000

/** Default UART port */
#define LD2450_DEFAULT_UART_NUM    UART_NUM_1

/** Report frame size in bytes */
#define LD2450_FRAME_SIZE          30

/** Report frame header: AA FF 03 00 */
#define LD2450_REPORT_HEADER_0     0xAA
#define LD2450_REPORT_HEADER_1     0xFF
#define LD2450_REPORT_HEADER_2     0x03
#define LD2450_REPORT_HEADER_3     0x00

/** Report frame tail: 55 CC */
#define LD2450_REPORT_TAIL_0       0x55
#define LD2450_REPORT_TAIL_1       0xCC

/** Command/ACK frame header: FD FC FB FA */
#define LD2450_CMD_HEADER_0        0xFD
#define LD2450_CMD_HEADER_1        0xFC
#define LD2450_CMD_HEADER_2        0xFB
#define LD2450_CMD_HEADER_3        0xFA

/** Command/ACK frame tail: 04 03 02 01 */
#define LD2450_CMD_TAIL_0          0x04
#define LD2450_CMD_TAIL_1          0x03
#define LD2450_CMD_TAIL_2          0x02
#define LD2450_CMD_TAIL_3          0x01

/** Function codes */
#define LD2450_FUNC_ENABLE_CONFIG      0x00FF  /*!< Enable config mode */
#define LD2450_FUNC_DISABLE_CONFIG     0x00FE  /*!< Disable config mode */
#define LD2450_FUNC_FACTORY_RESET      0x0080  /*!< Factory reset */
#define LD2450_FUNC_SET_SINGLE_TARGET  0x0090  /*!< Set single target mode */
#define LD2450_FUNC_SET_MULTI_TARGET   0x0091  /*!< Set multi target mode */
#define LD2450_FUNC_GET_VERSION        0x00A0  /*!< Get firmware version */
#define LD2450_FUNC_SET_BAUDRATE       0x00A1  /*!< Set baud rate */
#define LD2450_FUNC_RESTART            0x00A2  /*!< Restart module */
#define LD2450_FUNC_SET_BLUETOOTH      0x00A3  /*!< Set bluetooth on/off */
#define LD2450_FUNC_GET_MAC            0x00A4  /*!< Get MAC address */
#define LD2450_FUNC_GET_TRACKING_MODE  0x00A5  /*!< Get tracking mode */
#define LD2450_FUNC_SET_REGION_FILTER  0x00C1  /*!< Set region filter */
#define LD2450_FUNC_GET_REGION_FILTER  0x00C2  /*!< Get region filter */

/** Command timeout in milliseconds */
#define LD2450_CMD_TIMEOUT_MS      1000

/* ============ Data Structures ============ */

/**
 * @brief Single target information from LD2450
 * @note X, Y in mm; Speed in cm/s; Resolution in mm
 */
typedef struct {
    int16_t x;          /*!< X coordinate in mm */
    int16_t y;          /*!< Y coordinate in mm */
    int16_t speed;      /*!< Speed in cm/s */
    uint16_t resolution; /*!< Resolution in mm */
} ld2450_target_t;

/**
 * @brief LD2450 report data (full frame payload)
 */
typedef struct {
    uint8_t target_count;                         /*!< Number of valid targets */
    ld2450_target_t targets[LD2450_MAX_TARGETS];  /*!< Target array */
} ld2450_data_t;

/**
 * @brief Rectangular region definition (two diagonal corners)
 */
typedef struct {
    int16_t x1;  /*!< First corner X in mm */
    int16_t y1;  /*!< First corner Y in mm */
    int16_t x2;  /*!< Second corner X in mm */
    int16_t y2;  /*!< Second corner Y in mm */
} ld2450_region_t;

/**
 * @brief Region filter configuration
 * @note Up to 3 regions can be configured
 */
typedef struct {
    uint8_t type;                              /*!< Filter type: 0=disable, 1=include, 2=exclude */
    ld2450_region_t regions[LD2450_MAX_TARGETS]; /*!< Region array (max 3 regions) */
} ld2450_region_filter_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint16_t major;  /*!< Major version */
    uint32_t minor;  /*!< Minor version */
} ld2450_version_t;

/**
 * @brief Tracking mode
 */
typedef enum {
    LD2450_TRACKING_SINGLE = 0x01,  /*!< Single target tracking mode */
    LD2450_TRACKING_MULTI  = 0x02,  /*!< Multi target tracking mode */
} ld2450_tracking_mode_t;

/**
 * @brief Baud rate index
 */
typedef enum {
    LD2450_BAUD_9600   = 0x01,
    LD2450_BAUD_19200  = 0x02,
    LD2450_BAUD_38400  = 0x03,
    LD2450_BAUD_57600  = 0x04,
    LD2450_BAUD_115200 = 0x05,
    LD2450_BAUD_230400 = 0x06,
    LD2450_BAUD_256000 = 0x07,  /*!< Factory default */
    LD2450_BAUD_460800 = 0x08,
} ld2450_baud_index_t;

/* ============ Event Definitions ============ */

ESP_EVENT_DECLARE_BASE(ESP_LD2450_EVENT);

/**
 * @brief LD2450 event IDs
 */
typedef enum {
    LD2450_EVENT_TARGET_UPDATE,   /*!< New target data received */
    LD2450_EVENT_CMD_ACK,         /*!< Command acknowledged (internal use) */
} ld2450_event_id_t;

/* ============ Configuration ============ */

/**
 * @brief LD2450 driver configuration
 */
typedef struct {
    uart_port_t uart_port;        /*!< UART port number */
    int tx_pin;                   /*!< UART TX pin (-1 = not used) */
    int rx_pin;                   /*!< UART RX pin */
    uint32_t baud_rate;           /*!< UART baud rate (0 = use default 256000) */
    uint32_t event_queue_size;    /*!< UART event queue size */
    uint32_t ring_buffer_size;    /*!< UART ring buffer size */
    uint32_t task_stack_size;     /*!< Parser task stack size */
    int task_priority;            /*!< Parser task priority */
} ld2450_config_t;

/**
 * @brief Default configuration macro
 */
#define LD2450_CONFIG_DEFAULT()               \
    {                                         \
        .uart_port = UART_NUM_1,              \
        .tx_pin = CONFIG_RADAR_UART_TX_PIN,   \
        .rx_pin = CONFIG_RADAR_UART_RX_PIN,   \
        .baud_rate = 0,                       \
        .event_queue_size = 16,               \
        .ring_buffer_size = 1024,             \
        .task_stack_size = 4096,              \
        .task_priority = 10,                  \
    }

/* ============ Handle ============ */

/** Opaque LD2450 driver handle */
typedef struct ld2450_context *ld2450_handle_t;

/* ============ Lifecycle API ============ */

/**
 * @brief Initialize LD2450 driver
 *
 * @param config  Pointer to configuration
 * @return LD2450 handle on success, NULL on failure
 */
ld2450_handle_t ld2450_init(const ld2450_config_t *config);

/**
 * @brief Deinitialize LD2450 driver
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_deinit(ld2450_handle_t handle);

/* ============ Event Handler API ============ */

/**
 * @brief Register event handler for target data updates
 *
 * @param handle         LD2450 handle
 * @param event_handler  User event handler
 * @param handler_args   Handler arguments
 * @return ESP_OK on success
 */
esp_err_t ld2450_add_handler(ld2450_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *handler_args);

/**
 * @brief Unregister event handler
 *
 * @param handle         LD2450 handle
 * @param event_handler  User event handler to remove
 * @return ESP_OK on success
 */
esp_err_t ld2450_remove_handler(ld2450_handle_t handle,
                                 esp_event_handler_t event_handler);

/* ============ Config Mode API ============ */

/**
 * @brief Enable configuration mode
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_enable_config(ld2450_handle_t handle);

/**
 * @brief End configuration mode
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_end_config(ld2450_handle_t handle);

/* ============ Tracking Mode API ============ */

/**
 * @brief Set single target tracking mode
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_single_target(ld2450_handle_t handle);

/**
 * @brief Set multi target tracking mode
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_multi_target(ld2450_handle_t handle);

/**
 * @brief Get current tracking mode
 *
 * @param handle  LD2450 handle
 * @param mode    Output: current tracking mode
 * @return ESP_OK on success
 */
esp_err_t ld2450_get_tracking_mode(ld2450_handle_t handle, ld2450_tracking_mode_t *mode);

/* ============ System API ============ */

/**
 * @brief Get firmware version
 *
 * @param handle  LD2450 handle
 * @param version Output: firmware version info
 * @return ESP_OK on success
 */
esp_err_t ld2450_get_firmware_version(ld2450_handle_t handle, ld2450_version_t *version);

/**
 * @brief Set UART baud rate (persists after power-off, takes effect after restart)
 *
 * @param handle  LD2450 handle
 * @param index   Baud rate index (see ld2450_baud_index_t)
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_baud_rate(ld2450_handle_t handle, ld2450_baud_index_t index);

/**
 * @brief Factory reset
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_factory_reset(ld2450_handle_t handle);

/**
 * @brief Restart radar module
 *
 * @param handle  LD2450 handle
 * @return ESP_OK on success
 */
esp_err_t ld2450_restart(ld2450_handle_t handle);

/* ============ Bluetooth API ============ */

/**
 * @brief Set bluetooth on/off
 *
 * @param handle  LD2450 handle
 * @param enable  true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_bluetooth(ld2450_handle_t handle, bool enable);

/**
 * @brief Get MAC address
 *
 * @param handle     LD2450 handle
 * @param mac        Output: MAC address buffer (6 bytes)
 * @return ESP_OK on success
 */
esp_err_t ld2450_get_mac_address(ld2450_handle_t handle, uint8_t *mac);

/* ============ Region Filter API ============ */

/**
 * @brief Set region filter
 *
 * @param handle  LD2450 handle
 * @param filter  Region filter configuration
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_region_filter(ld2450_handle_t handle, const ld2450_region_filter_t *filter);

/**
 * @brief Get current region filter configuration
 *
 * @param handle  LD2450 handle
 * @param filter  Output: region filter configuration
 * @return ESP_OK on success
 */
esp_err_t ld2450_get_region_filter(ld2450_handle_t handle, ld2450_region_filter_t *filter);

#ifdef __cplusplus
}
#endif
