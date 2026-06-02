/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD6002B 3D Millimeter Wave Radar Driver
 * 3D radar with TinyFrame (TF) protocol, UART interface
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

/** TinyFrame Start-of-Frame byte */
#define LD6002B_TF_SOF                0x01

/** Maximum number of targets tracked simultaneously */
#define LD6002B_MAX_TARGETS           3

/** Default UART baud rate */
#define LD6002B_DEFAULT_BAUD_RATE     115200

/** Maximum TinyFrame frame size (bytes) */
#define LD6002B_MAX_FRAME_SIZE        1024

/** Command timeout in milliseconds */
#define LD6002B_CMD_TIMEOUT_MS        1000

/* ============ TinyFrame Message TYPE Constants ============ */

/*
 * TF frame layout:
 *   SOF(1) + ID(2,BE) + LEN(2,BE) + TYPE(2,BE) + HEAD_CKSUM(1) + DATA(N,LE) + DATA_CKSUM(1)
 * Checksum: XOR all bytes then NOT
 */

/* --- SET message types (host -> radar) --- */
#define LD6002B_MSG_SET_CONTROL_CMD   0x0201  /*!< Control command (with sub-cmd) */
#define LD6002B_MSG_SET_AREA          0x0202  /*!< Set detection area */
#define LD6002B_MSG_SET_HOLD_DELAY    0x0203  /*!< Set hold delay */
#define LD6002B_MSG_SET_Z_RANGE       0x0204  /*!< Set Z-axis detection range */
#define LD6002B_MSG_SET_LOW_POWER     0x0205  /*!< Set low-power sleep time */

/* --- REPORT message types (radar -> host) --- */
#define LD6002B_MSG_REPORT_TARGET     0x0A04  /*!< Target data (x/y/z + dop_idx + cluster_id) */
#define LD6002B_MSG_REPORT_POINT_CLOUD 0x0A08 /*!< Point cloud data */
#define LD6002B_MSG_REPORT_AREA_STATE 0x0A0A  /*!< Area state (detection areas) */
#define LD6002B_MSG_REPORT_AREA_1     0x0A0B  /*!< Area 1 coordinates */
#define LD6002B_MSG_REPORT_AREA_2     0x0A0C  /*!< Area 2 coordinates */
#define LD6002B_MSG_REPORT_HOLD_DELAY 0x0A0D  /*!< Hold delay value */
#define LD6002B_MSG_REPORT_SENSITIVITY 0x0A0E /*!< Sensitivity level */
#define LD6002B_MSG_REPORT_TRIGGER_SPEED 0x0A0F /*!< Trigger speed level */
#define LD6002B_MSG_REPORT_Z_RANGE    0x0A10  /*!< Z-axis range value */
#define LD6002B_MSG_REPORT_INSTALL_MODE 0x0A11 /*!< Installation mode */
#define LD6002B_MSG_REPORT_WORK_MODE  0x0A12  /*!< Work mode */
#define LD6002B_MSG_REPORT_LOW_POWER_TIME 0x0A13 /*!< Low power time */
#define LD6002B_MSG_REPORT_LOW_POWER_STATE 0x0A14 /*!< Low power state */

/* ============ Control Command (0x0201) Sub-Commands ============ */
/* LD6002B supports sub-commands 0x01-0x1A (26 commands) */

#define LD6002B_CMD_AUTO_GEN_NOISE    0x01  /*!< Auto-generate noise floor */
#define LD6002B_CMD_GET_AREAS         0x02  /*!< Get all detection areas */
#define LD6002B_CMD_CLEAR_NOISE       0x03  /*!< Clear noise floor */
#define LD6002B_CMD_RESET_DETECTION   0x04  /*!< Reset detection state */
#define LD6002B_CMD_GET_HOLD_DELAY    0x05  /*!< Get hold delay */
#define LD6002B_CMD_SET_POINT_CLOUD   0x06  /*!< Enable/disable point cloud output */
#define LD6002B_CMD_SET_TARGET_DISPLAY 0x07 /*!< Set target display mode */
#define LD6002B_CMD_SET_SENSITIVITY   0x08  /*!< Set sensitivity level */
#define LD6002B_CMD_GET_SENSITIVITY   0x09  /*!< Get sensitivity level */
#define LD6002B_CMD_SET_TRIGGER_SPEED 0x0A  /*!< Set trigger speed level */
#define LD6002B_CMD_GET_TRIGGER_SPEED 0x0B  /*!< Get trigger speed level */
#define LD6002B_CMD_GET_Z_RANGE       0x0C  /*!< Get Z-axis range */
#define LD6002B_CMD_SET_INSTALL_MODE  0x0D  /*!< Set installation mode */
#define LD6002B_CMD_GET_INSTALL_MODE  0x0E  /*!< Get installation mode */
#define LD6002B_CMD_RESERVED_0F       0x0F  /*!< Reserved */
#define LD6002B_CMD_RESERVED_10       0x10  /*!< Reserved */
#define LD6002B_CMD_GET_LOW_POWER_TIME 0x11 /*!< Get low power time */
#define LD6002B_CMD_RESET_UNOCCUPIED  0x12  /*!< Reset unoccupied state */
#define LD6002B_CMD_RESERVED_13       0x13  /*!< Reserved */
#define LD6002B_CMD_RESERVED_14       0x14  /*!< Reserved */
#define LD6002B_CMD_RESERVED_15       0x15  /*!< Reserved */
#define LD6002B_CMD_SET_LOW_POWER     0x16  /*!< Set low power mode (on/off) */
#define LD6002B_CMD_GET_LOW_POWER     0x17  /*!< Get low power mode state */
#define LD6002B_CMD_RESERVED_18       0x18  /*!< Reserved */
#define LD6002B_CMD_RESERVED_19       0x19  /*!< Reserved */
#define LD6002B_CMD_RESERVED_1A       0x1A  /*!< Reserved */

/* ============ Data Structures ============ */

/**
 * @brief Single target information from LD6002B
 */
typedef struct {
    float x;          /*!< X coordinate in meters (float, LE) */
    float y;          /*!< Y coordinate in meters (float, LE) */
    float z;          /*!< Z coordinate in meters (float, LE) */
    int32_t dop_idx;  /*!< Doppler index */
    int32_t cluster_id; /*!< Cluster ID */
} ld6002b_target_t;

/**
 * @brief LD6002B target data report payload
 */
typedef struct {
    uint8_t target_count;                          /*!< Number of valid targets (0-3) */
    ld6002b_target_t targets[LD6002B_MAX_TARGETS]; /*!< Target array */
} ld6002b_data_t;

/**
 * @brief Detection area definition (3D bounding box)
 */
typedef struct {
    float x_min;   /*!< X-axis minimum (meters) */
    float x_max;   /*!< X-axis maximum (meters) */
    float y_min;   /*!< Y-axis minimum (meters) */
    float y_max;   /*!< Y-axis maximum (meters) */
    float z_min;   /*!< Z-axis minimum (meters) */
    float z_max;   /*!< Z-axis maximum (meters) */
} ld6002b_area_t;

/**
 * @brief Area state report (detection areas)
 */
typedef struct {
    uint8_t states[8]; /*!< State of each area (0 = unoccupied, 1 = occupied) */
} ld6002b_area_state_t;

/**
 * @brief Installation mode
 */
typedef enum {
    LD6002B_INSTALL_TOP  = 0,  /*!< Top mount (ceiling) */
    LD6002B_INSTALL_SIDE = 1,  /*!< Side mount (wall) */
} ld6002b_install_mode_t;

/**
 * @brief Sensitivity level
 */
typedef enum {
    LD6002B_SENSITIVITY_LOW  = 0,  /*!< Low sensitivity */
    LD6002B_SENSITIVITY_MID  = 1,  /*!< Medium sensitivity */
    LD6002B_SENSITIVITY_HIGH = 2,  /*!< High sensitivity */
} ld6002b_sensitivity_t;

/**
 * @brief Trigger speed level
 */
typedef enum {
    LD6002B_TRIGGER_SPEED_SLOW = 0,  /*!< Slow trigger */
    LD6002B_TRIGGER_SPEED_MID  = 1,  /*!< Medium trigger */
    LD6002B_TRIGGER_SPEED_FAST = 2,  /*!< Fast trigger */
} ld6002b_trigger_speed_t;

/* ============ Event Definitions ============ */

ESP_EVENT_DECLARE_BASE(ESP_LD6002B_EVENT);

/**
 * @brief LD6002B event IDs
 */
typedef enum {
    LD6002B_EVENT_TARGET_UPDATE,   /*!< New target data received */
    LD6002B_EVENT_AREA_STATE,      /*!< Area state changed */
} ld6002b_event_id_t;

/* ============ Configuration ============ */

/**
 * @brief LD6002B driver configuration
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
} ld6002b_config_t;

/**
 * @brief Default configuration macro
 */
#define LD6002B_CONFIG_DEFAULT()               \
    {                                           \
        .uart_port = UART_NUM_1,                \
        .tx_pin = CONFIG_RADAR_UART_TX_PIN,     \
        .rx_pin = CONFIG_RADAR_UART_RX_PIN,     \
        .baud_rate = 0,                         \
        .event_queue_size = 16,                 \
        .ring_buffer_size = 1024,               \
        .task_stack_size = 4096,                \
        .task_priority = 10,                    \
    }

/* ============ Handle ============ */

/** Opaque LD6002B driver handle */
typedef struct ld6002b_context *ld6002b_handle_t;

/* ============ Lifecycle API ============ */

/**
 * @brief Initialize LD6002B driver
 *
 * @param config  Pointer to configuration
 * @return LD6002B handle on success, NULL on failure
 */
ld6002b_handle_t ld6002b_init(const ld6002b_config_t *config);

/**
 * @brief Deinitialize LD6002B driver
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_deinit(ld6002b_handle_t handle);

/* ============ Event Handler API ============ */

/**
 * @brief Register event handler for radar events
 *
 * @param handle         LD6002B handle
 * @param event_handler  User event handler
 * @param handler_args   Handler arguments
 * @return ESP_OK on success
 */
esp_err_t ld6002b_add_handler(ld6002b_handle_t handle,
                               esp_event_handler_t event_handler,
                               void *handler_args);

/**
 * @brief Unregister event handler
 *
 * @param handle         LD6002B handle
 * @param event_handler  User event handler to remove
 * @return ESP_OK on success
 */
esp_err_t ld6002b_remove_handler(ld6002b_handle_t handle,
                                  esp_event_handler_t event_handler);

/* ============ Control Command API (0x0201 sub-commands) ============ */

/**
 * @brief Auto-generate noise floor
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_auto_gen_noise(ld6002b_handle_t handle);

/**
 * @brief Get all detection areas
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_areas(ld6002b_handle_t handle);

/**
 * @brief Clear noise floor
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_clear_noise(ld6002b_handle_t handle);

/**
 * @brief Reset detection state
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_reset_detection(ld6002b_handle_t handle);

/**
 * @brief Get hold delay value
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_hold_delay(ld6002b_handle_t handle);

/**
 * @brief Enable or disable point cloud output
 *
 * @param handle  LD6002B handle
 * @param enable  true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_point_cloud(ld6002b_handle_t handle, bool enable);

/**
 * @brief Set target display mode
 *
 * @param handle  LD6002B handle
 * @param mode    Display mode value
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_target_display(ld6002b_handle_t handle, uint8_t mode);

/**
 * @brief Set sensitivity level
 *
 * @param handle  LD6002B handle
 * @param level   Sensitivity level (see ld6002b_sensitivity_t)
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_sensitivity(ld6002b_handle_t handle, ld6002b_sensitivity_t level);

/**
 * @brief Get sensitivity level
 *
 * @param handle  LD6002B handle
 * @param level   Output: current sensitivity level
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_sensitivity(ld6002b_handle_t handle, ld6002b_sensitivity_t *level);

/**
 * @brief Set trigger speed level
 *
 * @param handle  LD6002B handle
 * @param speed   Trigger speed level (see ld6002b_trigger_speed_t)
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_trigger_speed(ld6002b_handle_t handle, ld6002b_trigger_speed_t speed);

/**
 * @brief Get trigger speed level
 *
 * @param handle  LD6002B handle
 * @param speed   Output: current trigger speed level
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_trigger_speed(ld6002b_handle_t handle, ld6002b_trigger_speed_t *speed);

/**
 * @brief Get Z-axis detection range
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_z_range(ld6002b_handle_t handle);

/**
 * @brief Set installation mode (top/side mount)
 *
 * @param handle  LD6002B handle
 * @param mode    Installation mode (see ld6002b_install_mode_t)
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_install_mode(ld6002b_handle_t handle, ld6002b_install_mode_t mode);

/**
 * @brief Get installation mode
 *
 * @param handle  LD6002B handle
 * @param mode    Output: current installation mode
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_install_mode(ld6002b_handle_t handle, ld6002b_install_mode_t *mode);

/**
 * @brief Set low power mode
 *
 * @param handle  LD6002B handle
 * @param enable  true to enable low power mode, false to disable
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_low_power_mode(ld6002b_handle_t handle, bool enable);

/**
 * @brief Get low power mode state
 *
 * @param handle  LD6002B handle
 * @param enabled Output: true if low power mode is enabled
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_low_power_mode(ld6002b_handle_t handle, bool *enabled);

/**
 * @brief Get low power time
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_get_low_power_time(ld6002b_handle_t handle);

/**
 * @brief Reset unoccupied state
 *
 * @param handle  LD6002B handle
 * @return ESP_OK on success
 */
esp_err_t ld6002b_reset_unoccupied(ld6002b_handle_t handle);

/* ============ Area Configuration API (0x0202) ============ */

/**
 * @brief Set detection area (3D bounding box)
 *
 * @param handle  LD6002B handle
 * @param area    Area definition
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_area(ld6002b_handle_t handle, const ld6002b_area_t *area);

/* ============ Separate SET Command API ============ */

/**
 * @brief Set hold delay
 *
 * @param handle  LD6002B handle
 * @param delay   Hold delay value
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_hold_delay(ld6002b_handle_t handle, uint8_t delay);

/**
 * @brief Set Z-axis detection range
 *
 * @param handle  LD6002B handle
 * @param z_min   Z-axis minimum (meters)
 * @param z_max   Z-axis maximum (meters)
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_z_range(ld6002b_handle_t handle, float z_min, float z_max);

/**
 * @brief Set low power sleep time
 *
 * @param handle  LD6002B handle
 * @param time    Low power time value
 * @return ESP_OK on success
 */
esp_err_t ld6002b_set_low_power_time(ld6002b_handle_t handle, uint8_t time);

#ifdef __cplusplus
}
#endif
