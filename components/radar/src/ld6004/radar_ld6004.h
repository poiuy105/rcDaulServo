/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * HLK-LD6004 3D Millimeter Wave Radar Driver
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
#define LD6004_TF_SOF                0x01

/** Maximum number of targets tracked simultaneously */
#define LD6004_MAX_TARGETS           3

/** Default UART baud rate */
#define LD6004_DEFAULT_BAUD_RATE     115200

/** Maximum TinyFrame frame size (bytes) */
#define LD6004_MAX_FRAME_SIZE        1024

/** Command timeout in milliseconds */
#define LD6004_CMD_TIMEOUT_MS        1000

/* ============ TinyFrame Message TYPE Constants ============ */

/*
 * TF frame layout:
 *   SOF(1) + ID(2,BE) + LEN(2,BE) + TYPE(2,BE) + HEAD_CKSUM(1) + DATA(N,LE) + DATA_CKSUM(1)
 * Checksum: XOR all bytes then NOT
 */

/* --- SET message types (host -> radar) --- */
#define LD6004_MSG_SET_CONTROL_CMD   0x0201  /*!< Control command (with sub-cmd) */
#define LD6004_MSG_SET_AREA          0x0202  /*!< Set detection area */
#define LD6004_MSG_SET_HOLD_DELAY    0x0203  /*!< Set hold delay */
#define LD6004_MSG_SET_Z_RANGE       0x0204  /*!< Set Z-axis detection range */
#define LD6004_MSG_SET_LOW_POWER     0x0205  /*!< Set low-power sleep time */
#define LD6004_MSG_SET_STAY_LIFE     0x0206  /*!< Set stay life */
#define LD6004_MSG_SET_OUTPUT_INTERVAL 0x0207 /*!< Set output interval */
#define LD6004_MSG_SET_BAUD_RATE     0x0F0F  /*!< Set baud rate */

/* --- REPORT message types (radar -> host) --- */
#define LD6004_MSG_REPORT_TARGET     0x0A04  /*!< Target data (x/y/z + dop_idx + cluster_id) */
#define LD6004_MSG_REPORT_POINT_CLOUD 0x0A08 /*!< Point cloud data */
#define LD6004_MSG_REPORT_AREA_STATE 0x0A0A  /*!< Area state (4 detection areas) */
#define LD6004_MSG_REPORT_AREA_1     0x0A0B  /*!< Area 1 coordinates */
#define LD6004_MSG_REPORT_AREA_2     0x0A0C  /*!< Area 2 coordinates */
#define LD6004_MSG_REPORT_AREA_3     0x0A16  /*!< Area 3 coordinates */
#define LD6004_MSG_REPORT_HOLD_DELAY 0x0A0D  /*!< Hold delay value */
#define LD6004_MSG_REPORT_SENSITIVITY 0x0A0E /*!< Sensitivity level */
#define LD6004_MSG_REPORT_TRIGGER_SPEED 0x0A0F /*!< Trigger speed level */
#define LD6004_MSG_REPORT_Z_RANGE    0x0A10  /*!< Z-axis range value */
#define LD6004_MSG_REPORT_INSTALL_MODE 0x0A11 /*!< Installation mode */
#define LD6004_MSG_REPORT_WORK_MODE  0x0A12  /*!< Work mode */
#define LD6004_MSG_REPORT_LOW_POWER_TIME 0x0A13 /*!< Low power time */
#define LD6004_MSG_REPORT_LOW_POWER_STATE 0x0A14 /*!< Low power state */
#define LD6004_MSG_REPORT_GPIO_STATE 0x0A15 /*!< GPIO state */
#define LD6004_MSG_REPORT_STAY_LIFE  0x0A17  /*!< Stay life value */
#define LD6004_MSG_REPORT_OUTPUT_INTERVAL 0x0A18 /*!< Output interval value */

/* --- GENERAL message types --- */
#define LD6004_MSG_GENERAL_FW_VERSION 0xFFFF /*!< Firmware version query/response */

/* ============ Control Command (0x0201) Sub-Commands ============ */

#define LD6004_CMD_AUTO_GEN_NOISE    0x01  /*!< Auto-generate noise floor */
#define LD6004_CMD_GET_AREAS         0x02  /*!< Get all detection areas */
#define LD6004_CMD_CLEAR_NOISE       0x03  /*!< Clear noise floor */
#define LD6004_CMD_RESET_DETECTION   0x04  /*!< Reset detection state */
#define LD6004_CMD_GET_HOLD_DELAY    0x05  /*!< Get hold delay */
#define LD6004_CMD_SET_POINT_CLOUD   0x06  /*!< Enable/disable point cloud output */
#define LD6004_CMD_SET_TARGET_DISPLAY 0x07 /*!< Set target display mode */
#define LD6004_CMD_SET_SENSITIVITY   0x08  /*!< Set sensitivity level */
#define LD6004_CMD_GET_SENSITIVITY   0x09  /*!< Get sensitivity level */
#define LD6004_CMD_SET_TRIGGER_SPEED 0x0A  /*!< Set trigger speed level */
#define LD6004_CMD_GET_TRIGGER_SPEED 0x0B  /*!< Get trigger speed level */
#define LD6004_CMD_GET_Z_RANGE       0x0C  /*!< Get Z-axis range */
#define LD6004_CMD_SET_INSTALL_MODE  0x0D  /*!< Set installation mode */
#define LD6004_CMD_GET_INSTALL_MODE  0x0E  /*!< Get installation mode */
#define LD6004_CMD_SET_WORK_MODE     0x0F  /*!< Set work mode */
#define LD6004_CMD_GET_WORK_MODE     0x10  /*!< Get work mode */
#define LD6004_CMD_GET_LOW_POWER_TIME 0x11 /*!< Get low power time */
#define LD6004_CMD_RESET_UNOCCUPIED  0x12  /*!< Reset unoccupied state */
#define LD6004_CMD_SET_GPIO_MODE     0x13  /*!< Set GPIO mode */
#define LD6004_CMD_GET_GPIO_MODE     0x14  /*!< Get GPIO mode */
#define LD6004_CMD_CLEAR_STAY_AREAS  0x15  /*!< Clear stay areas */
#define LD6004_CMD_GET_STAY_LIFE     0x16  /*!< Get stay life */
#define LD6004_CMD_GET_OUTPUT_INTERVAL 0x17 /*!< Get output interval */

/* ============ Data Structures ============ */

/**
 * @brief Single target information from LD6004
 */
typedef struct {
    float x;          /*!< X coordinate in meters (float, LE) */
    float y;          /*!< Y coordinate in meters (float, LE) */
    float z;          /*!< Z coordinate in meters (float, LE) */
    int32_t dop_idx;  /*!< Doppler index */
    int32_t cluster_id; /*!< Cluster ID */
} ld6004_target_t;

/**
 * @brief LD6004 target data report payload
 */
typedef struct {
    uint8_t target_count;                        /*!< Number of valid targets (0-3) */
    ld6004_target_t targets[LD6004_MAX_TARGETS]; /*!< Target array */
} ld6004_data_t;

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
} ld6004_area_t;

/**
 * @brief Area state report (4 detection areas)
 */
typedef struct {
    uint8_t states[4]; /*!< State of each area (0 = unoccupied, 1 = occupied) */
} ld6004_area_state_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint8_t project;   /*!< Project code */
    uint8_t major;     /*!< Major version */
    uint8_t sub;       /*!< Sub version */
    uint8_t modified;  /*!< Modified flag */
} ld6004_version_t;

/**
 * @brief Installation mode
 */
typedef enum {
    LD6004_INSTALL_TOP  = 0,  /*!< Top mount (ceiling) */
    LD6004_INSTALL_SIDE = 1,  /*!< Side mount (wall) */
} ld6004_install_mode_t;

/**
 * @brief Work mode
 */
typedef enum {
    LD6004_WORK_MODE_NORMAL          = 0,  /*!< Normal mode */
    LD6004_WORK_MODE_LOW_POWER       = 1,  /*!< Low power mode */
    LD6004_WORK_MODE_OFF_HIGH        = 2,  /*!< Off - high sensitivity */
    LD6004_WORK_MODE_OFF_LOW         = 3,  /*!< Off - low sensitivity */
    LD6004_WORK_MODE_STRONG_REFLECTION = 4, /*!< Strong reflection suppression */
} ld6004_work_mode_t;

/**
 * @brief Sensitivity level
 */
typedef enum {
    LD6004_SENSITIVITY_LOW  = 0,  /*!< Low sensitivity */
    LD6004_SENSITIVITY_MID  = 1,  /*!< Medium sensitivity */
    LD6004_SENSITIVITY_HIGH = 2,  /*!< High sensitivity */
} ld6004_sensitivity_t;

/**
 * @brief Trigger speed level
 */
typedef enum {
    LD6004_TRIGGER_SPEED_SLOW = 0,  /*!< Slow trigger */
    LD6004_TRIGGER_SPEED_MID  = 1,  /*!< Medium trigger */
    LD6004_TRIGGER_SPEED_FAST = 2,  /*!< Fast trigger */
} ld6004_trigger_speed_t;

/**
 * @brief GPIO output mode
 */
typedef enum {
    LD6004_GPIO_MODE_0 = 0,  /*!< GPIO mode 0 */
    LD6004_GPIO_MODE_1 = 1,  /*!< GPIO mode 1 */
    LD6004_GPIO_MODE_2 = 2,  /*!< GPIO mode 2 */
    LD6004_GPIO_MODE_3 = 3,  /*!< GPIO mode 3 */
    LD6004_GPIO_MODE_4 = 4,  /*!< GPIO mode 4 */
    LD6004_GPIO_MODE_5 = 5,  /*!< GPIO mode 5 */
} ld6004_gpio_mode_t;

/**
 * @brief Baud rate index
 */
typedef enum {
    LD6004_BAUD_9600   = 0,  /*!< 9600 bps */
    LD6004_BAUD_19200  = 1,  /*!< 19200 bps */
    LD6004_BAUD_38400  = 2,  /*!< 38400 bps */
    LD6004_BAUD_57600  = 3,  /*!< 57600 bps */
    LD6004_BAUD_115200 = 4,  /*!< 115200 bps (default) */
    LD6004_BAUD_256000 = 5,  /*!< 256000 bps */
} ld6004_baud_index_t;

/* ============ Event Definitions ============ */

ESP_EVENT_DECLARE_BASE(ESP_LD6004_EVENT);

/**
 * @brief LD6004 event IDs
 */
typedef enum {
    LD6004_EVENT_TARGET_UPDATE,   /*!< New target data received */
    LD6004_EVENT_AREA_STATE,      /*!< Area state changed */
} ld6004_event_id_t;

/* ============ Configuration ============ */

/**
 * @brief LD6004 driver configuration
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
} ld6004_config_t;

/**
 * @brief Default configuration macro
 */
#define LD6004_CONFIG_DEFAULT()               \
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

/** Opaque LD6004 driver handle */
typedef struct ld6004_context *ld6004_handle_t;

/* ============ Lifecycle API ============ */

/**
 * @brief Initialize LD6004 driver
 *
 * @param config  Pointer to configuration
 * @return LD6004 handle on success, NULL on failure
 */
ld6004_handle_t ld6004_init(const ld6004_config_t *config);

/**
 * @brief Deinitialize LD6004 driver
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_deinit(ld6004_handle_t handle);

/* ============ Event Handler API ============ */

/**
 * @brief Register event handler for radar events
 *
 * @param handle         LD6004 handle
 * @param event_handler  User event handler
 * @param handler_args   Handler arguments
 * @return ESP_OK on success
 */
esp_err_t ld6004_add_handler(ld6004_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *handler_args);

/**
 * @brief Unregister event handler
 *
 * @param handle         LD6004 handle
 * @param event_handler  User event handler to remove
 * @return ESP_OK on success
 */
esp_err_t ld6004_remove_handler(ld6004_handle_t handle,
                                 esp_event_handler_t event_handler);

/* ============ Control Command API (0x0201 sub-commands) ============ */

/**
 * @brief Auto-generate noise floor
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_auto_gen_noise(ld6004_handle_t handle);

/**
 * @brief Get all detection areas
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_areas(ld6004_handle_t handle);

/**
 * @brief Clear noise floor
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_clear_noise(ld6004_handle_t handle);

/**
 * @brief Reset detection state
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_reset_detection(ld6004_handle_t handle);

/**
 * @brief Get hold delay value
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_hold_delay(ld6004_handle_t handle);

/**
 * @brief Enable or disable point cloud output
 *
 * @param handle  LD6004 handle
 * @param enable  true to enable, false to disable
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_point_cloud(ld6004_handle_t handle, bool enable);

/**
 * @brief Set target display mode
 *
 * @param handle  LD6004 handle
 * @param mode    Display mode value
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_target_display(ld6004_handle_t handle, uint8_t mode);

/**
 * @brief Set sensitivity level
 *
 * @param handle  LD6004 handle
 * @param level   Sensitivity level (see ld6004_sensitivity_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_sensitivity(ld6004_handle_t handle, ld6004_sensitivity_t level);

/**
 * @brief Get sensitivity level
 *
 * @param handle  LD6004 handle
 * @param level   Output: current sensitivity level
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_sensitivity(ld6004_handle_t handle, ld6004_sensitivity_t *level);

/**
 * @brief Set trigger speed level
 *
 * @param handle  LD6004 handle
 * @param speed   Trigger speed level (see ld6004_trigger_speed_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_trigger_speed(ld6004_handle_t handle, ld6004_trigger_speed_t speed);

/**
 * @brief Get trigger speed level
 *
 * @param handle  LD6004 handle
 * @param speed   Output: current trigger speed level
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_trigger_speed(ld6004_handle_t handle, ld6004_trigger_speed_t *speed);

/**
 * @brief Get Z-axis detection range
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_z_range(ld6004_handle_t handle);

/**
 * @brief Set installation mode (top/side mount)
 *
 * @param handle  LD6004 handle
 * @param mode    Installation mode (see ld6004_install_mode_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_install_mode(ld6004_handle_t handle, ld6004_install_mode_t mode);

/**
 * @brief Get installation mode
 *
 * @param handle  LD6004 handle
 * @param mode    Output: current installation mode
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_install_mode(ld6004_handle_t handle, ld6004_install_mode_t *mode);

/**
 * @brief Set work mode
 *
 * @param handle  LD6004 handle
 * @param mode    Work mode (see ld6004_work_mode_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_work_mode(ld6004_handle_t handle, ld6004_work_mode_t mode);

/**
 * @brief Get work mode
 *
 * @param handle  LD6004 handle
 * @param mode    Output: current work mode
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_work_mode(ld6004_handle_t handle, ld6004_work_mode_t *mode);

/**
 * @brief Get low power time
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_low_power_time(ld6004_handle_t handle);

/**
 * @brief Reset unoccupied state
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_reset_unoccupied(ld6004_handle_t handle);

/**
 * @brief Set GPIO output mode
 *
 * @param handle  LD6004 handle
 * @param mode    GPIO mode (see ld6004_gpio_mode_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_gpio_mode(ld6004_handle_t handle, ld6004_gpio_mode_t mode);

/**
 * @brief Get GPIO output mode
 *
 * @param handle  LD6004 handle
 * @param mode    Output: current GPIO mode
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_gpio_mode(ld6004_handle_t handle, ld6004_gpio_mode_t *mode);

/**
 * @brief Clear stay areas
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_clear_stay_areas(ld6004_handle_t handle);

/**
 * @brief Get stay life value
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_stay_life(ld6004_handle_t handle);

/**
 * @brief Get output interval value
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_get_output_interval(ld6004_handle_t handle);

/* ============ Area Configuration API (0x0202) ============ */

/**
 * @brief Set detection area (3D bounding box)
 *
 * @param handle  LD6004 handle
 * @param area    Area definition
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_area(ld6004_handle_t handle, const ld6004_area_t *area);

/* ============ Separate SET Command API ============ */

/**
 * @brief Set hold delay
 *
 * @param handle  LD6004 handle
 * @param delay   Hold delay value
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_hold_delay(ld6004_handle_t handle, uint8_t delay);

/**
 * @brief Set Z-axis detection range
 *
 * @param handle  LD6004 handle
 * @param z_min   Z-axis minimum (meters)
 * @param z_max   Z-axis maximum (meters)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_z_range(ld6004_handle_t handle, float z_min, float z_max);

/**
 * @brief Set low power sleep time
 *
 * @param handle  LD6004 handle
 * @param time    Low power time value
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_low_power_time(ld6004_handle_t handle, uint8_t time);

/**
 * @brief Set stay life value
 *
 * @param handle  LD6004 handle
 * @param life    Stay life value
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_stay_life(ld6004_handle_t handle, uint8_t life);

/**
 * @brief Set output interval
 *
 * @param handle   LD6004 handle
 * @param interval Output interval value
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_output_interval(ld6004_handle_t handle, uint8_t interval);

/**
 * @brief Set UART baud rate
 *
 * @param handle  LD6004 handle
 * @param index   Baud rate index (see ld6004_baud_index_t)
 * @return ESP_OK on success
 */
esp_err_t ld6004_set_baud_rate(ld6004_handle_t handle, ld6004_baud_index_t index);

/* ============ General API ============ */

/**
 * @brief Query firmware version
 *
 * @param handle  LD6004 handle
 * @return ESP_OK on success
 */
esp_err_t ld6004_query_firmware_version(ld6004_handle_t handle);

#ifdef __cplusplus
}
#endif
