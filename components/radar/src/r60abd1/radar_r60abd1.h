/**
 * @file radar_r60abd1.h
 * @brief R60ABD1 60GHz 呼吸睡眠雷达驱动头文件
 *
 * 60GHz毫米波呼吸睡眠探测雷达
 * 波特率: 115200
 * 帧格式: 53 59 [control] [command] [len_h] [len_l] [data] [sum] 54 43
 *
 * @copyright Copyright (c) 2024
 */

#ifndef RADAR_R60ABD1_H
#define RADAR_R60ABD1_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART默认配置 */
#define R60ABD1_DEFAULT_BAUD_RATE   115200
#define R60ABD1_DEFAULT_UART_NUM    UART_NUM_1
#define R60ABD1_DEFAULT_TX_PIN      1
#define R60ABD1_DEFAULT_RX_PIN      2
#define R60ABD1_UART_BUF_SIZE       512
#define R60ABD1_TASK_STACK_SIZE     4096
#define R60ABD1_TASK_PRIORITY       5

/* 帧定义 */
#define R60ABD1_FRAME_HEADER_0      0x53
#define R60ABD1_FRAME_HEADER_1      0x59
#define R60ABD1_FRAME_TAIL_0        0x54
#define R60ABD1_FRAME_TAIL_1        0x43

/* 控制字 */
#define R60ABD1_CTRL_HEARTBEAT      0x01
#define R60ABD1_CTRL_PRODUCT        0x02
#define R60ABD1_CTRL_OTA            0x03
#define R60ABD1_CTRL_WORK_STATE     0x05
#define R60ABD1_CTRL_RANGE          0x07
#define R60ABD1_CTRL_HUMAN          0x80
#define R60ABD1_CTRL_BREATH         0x81
#define R60ABD1_CTRL_SLEEP          0x84
#define R60ABD1_CTRL_HEART_RATE     0x85

/* 命令字 - 系统功能 */
#define R60ABD1_CMD_HEARTBEAT       0x01
#define R60ABD1_CMD_RESET           0x02
#define R60ABD1_CMD_HEARTBEAT_QUERY 0x80

/* 命令字 - 产品信息 */
#define R60ABD1_CMD_PRODUCT_MODEL   0xA1
#define R60ABD1_CMD_PRODUCT_ID      0xA2
#define R60ABD1_CMD_HARDWARE_VER    0xA3
#define R60ABD1_CMD_FIRMWARE_VER    0xA4

/* 命令字 - 工作状态 */
#define R60ABD1_CMD_INIT_COMPLETE   0x01
#define R60ABD1_CMD_INIT_QUERY      0x81

/* 命令字 - 探测范围 */
#define R60ABD1_CMD_RANGE_STATUS    0x07
#define R60ABD1_CMD_RANGE_QUERY     0x87

/* 命令字 - 人体存在 */
#define R60ABD1_CMD_HUMAN_ENABLE    0x00
#define R60ABD1_CMD_HUMAN_STATUS    0x01
#define R60ABD1_CMD_HUMAN_MOTION    0x02
#define R60ABD1_CMD_HUMAN_MOVE      0x03
#define R60ABD1_CMD_HUMAN_DISTANCE  0x04
#define R60ABD1_CMD_HUMAN_POSITION  0x05
#define R60ABD1_CMD_HUMAN_ENABLE_Q  0x80
#define R60ABD1_CMD_HUMAN_STATUS_Q  0x81
#define R60ABD1_CMD_HUMAN_MOTION_Q  0x82
#define R60ABD1_CMD_HUMAN_MOVE_Q    0x83
#define R60ABD1_CMD_HUMAN_DIST_Q    0x84
#define R60ABD1_CMD_HUMAN_POS_Q     0x85

/* 命令字 - 呼吸检测 */
#define R60ABD1_CMD_BREATH_ENABLE   0x00
#define R60ABD1_CMD_BREATH_STATUS   0x01
#define R60ABD1_CMD_BREATH_RATE     0x02
#define R60ABD1_CMD_BREATH_WAVE     0x05
#define R60ABD1_CMD_BREATH_ENABLE_Q 0x80
#define R60ABD1_CMD_BREATH_STATUS_Q 0x81
#define R60ABD1_CMD_BREATH_RATE_Q   0x82
#define R60ABD1_CMD_BREATH_WAVE_Q   0x85

/* 命令字 - 心率监测 */
#define R60ABD1_CMD_HEART_ENABLE    0x00
#define R60ABD1_CMD_HEART_RATE      0x02
#define R60ABD1_CMD_HEART_WAVE      0x05
#define R60ABD1_CMD_HEART_ENABLE_Q  0x80
#define R60ABD1_CMD_HEART_RATE_Q    0x82
#define R60ABD1_CMD_HEART_WAVE_Q    0x85

/* 命令字 - 睡眠监测 */
#define R60ABD1_CMD_SLEEP_ENABLE    0x00
#define R60ABD1_CMD_SLEEP_BED       0x01
#define R60ABD1_CMD_SLEEP_STATE     0x02
#define R60ABD1_CMD_SLEEP_WAKE_TIME 0x03
#define R60ABD1_CMD_SLEEP_LIGHT_TIME 0x04
#define R60ABD1_CMD_SLEEP_DEEP_TIME 0x05
#define R60ABD1_CMD_SLEEP_SCORE     0x06
#define R60ABD1_CMD_SLEEP_COMPREHENSIVE 0x0C
#define R60ABD1_CMD_SLEEP_ANALYSIS  0x0D
#define R60ABD1_CMD_SLEEP_ABNORMAL  0x0E
#define R60ABD1_CMD_SLEEP_MODE      0x0F
#define R60ABD1_CMD_SLEEP_ENABLE_Q  0x80
#define R60ABD1_CMD_SLEEP_BED_Q     0x81
#define R60ABD1_CMD_SLEEP_STATE_Q   0x82
#define R60ABD1_CMD_SLEEP_WAKE_Q    0x83
#define R60ABD1_CMD_SLEEP_LIGHT_Q   0x84
#define R60ABD1_CMD_SLEEP_DEEP_Q    0x85
#define R60ABD1_CMD_SLEEP_SCORE_Q   0x86
#define R60ABD1_CMD_SLEEP_MODE_Q    0x8C
#define R60ABD1_CMD_SLEEP_COMP_Q    0x8D
#define R60ABD1_CMD_SLEEP_ABN_Q     0x8E
#define R60ABD1_CMD_SLEEP_STATS_Q   0x8F

/* 状态枚举 */
typedef enum {
    R60ABD1_HUMAN_NONE = 0x00,
    R60ABD1_HUMAN_PRESENT = 0x01,
} r60abd1_human_status_t;

typedef enum {
    R60ABD1_MOTION_NONE = 0x00,
    R60ABD1_MOTION_STATIC = 0x01,
    R60ABD1_MOTION_ACTIVE = 0x02,
} r60abd1_motion_status_t;

typedef enum {
    R60ABD1_BREATH_NORMAL = 0x01,
    R60ABD1_BREATH_HIGH = 0x02,
    R60ABD1_BREATH_LOW = 0x03,
    R60ABD1_BREATH_NONE = 0x04,
} r60abd1_breath_status_t;

typedef enum {
    R60ABD1_SLEEP_DEEP = 0x00,
    R60ABD1_SLEEP_LIGHT = 0x01,
    R60ABD1_SLEEP_AWAKE = 0x02,
    R60ABD1_SLEEP_NONE = 0x03,
} r60abd1_sleep_state_t;

typedef enum {
    R60ABD1_BED_AWAY = 0x00,
    R60ABD1_BED_IN = 0x01,
    R60ABD1_BED_NONE = 0x02,
} r60abd1_bed_status_t;

typedef enum {
    R60ABD1_ABNORMAL_SHORT = 0x00,
    R60ABD1_ABNORMAL_LONG = 0x01,
    R60ABD1_ABNORMAL_NOBODY = 0x02,
    R60ABD1_ABNORMAL_NONE = 0x03,
} r60abd1_sleep_abnormal_t;

typedef enum {
    R60ABD1_MODE_REALTIME = 0x00,
    R60ABD1_MODE_SLEEP = 0x01,
} r60abd1_report_mode_t;

typedef enum {
    R60ABD1_RANGE_OUT = 0x00,
    R60ABD1_RANGE_IN = 0x01,
} r60abd1_range_status_t;

/* 数据结构 */
typedef struct {
    uint8_t model[32];
    uint8_t id[32];
    uint8_t hardware[32];
    uint8_t firmware[32];
} r60abd1_product_info_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} r60abd1_position_t;

typedef struct {
    uint8_t waveform[5];
} r60abd1_waveform_t;

typedef struct {
    bool present;
    r60abd1_sleep_state_t state;
    uint8_t breath_rate;
    uint8_t heart_rate;
    uint8_t turn_count;
    uint8_t large_move;
    uint8_t small_move;
    uint8_t apnea_count;
} r60abd1_sleep_comprehensive_t;

typedef struct {
    uint8_t score;
    uint16_t total_time;
    uint8_t awake_ratio;
    uint8_t light_ratio;
    uint8_t deep_ratio;
    uint8_t away_time;
    uint8_t away_count;
    uint8_t turn_count;
    uint8_t avg_breath;
    uint8_t avg_heart;
    uint8_t apnea_count;
} r60abd1_sleep_analysis_t;

typedef struct {
    r60abd1_human_status_t human;
    r60abd1_motion_status_t motion;
    uint8_t move_param;
    uint16_t distance;
    r60abd1_position_t position;
    r60abd1_breath_status_t breath_status;
    uint8_t breath_rate;
    r60abd1_waveform_t breath_wave;
    uint8_t heart_rate;
    r60abd1_waveform_t heart_wave;
    r60abd1_bed_status_t bed;
    r60abd1_sleep_state_t sleep_state;
    uint16_t wake_time;
    uint16_t light_time;
    uint16_t deep_time;
    uint8_t sleep_score;
    r60abd1_sleep_comprehensive_t sleep_comp;
    r60abd1_sleep_analysis_t sleep_analysis;
    r60abd1_sleep_abnormal_t abnormal;
    r60abd1_range_status_t range;
    bool has_human;
    bool has_motion;
    bool has_move;
    bool has_distance;
    bool has_position;
    bool has_breath_status;
    bool has_breath_rate;
    bool has_breath_wave;
    bool has_heart_rate;
    bool has_heart_wave;
    bool has_bed;
    bool has_sleep_state;
    bool has_wake_time;
    bool has_light_time;
    bool has_deep_time;
    bool has_sleep_score;
    bool has_sleep_comp;
    bool has_sleep_analysis;
    bool has_abnormal;
    bool has_range;
} r60abd1_data_t;

/* 事件ID */
typedef enum {
    R60ABD1_EVENT_DATA_UPDATE = 0,
} r60abd1_event_id_t;

/* 前向声明 */
typedef struct r60abd1_dev r60abd1_dev_t;
typedef r60abd1_dev_t* r60abd1_handle_t;

/* 配置结构 */
typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baud_rate;
} r60abd1_config_t;

/* 默认配置 */
#define R60ABD1_DEFAULT_CONFIG() { \
    .uart_num = R60ABD1_DEFAULT_UART_NUM, \
    .tx_pin = R60ABD1_DEFAULT_TX_PIN, \
    .rx_pin = R60ABD1_DEFAULT_RX_PIN, \
    .baud_rate = R60ABD1_DEFAULT_BAUD_RATE, \
}

/* 事件基础声明 */
ESP_EVENT_DECLARE_BASE(R60ABD1_EVENT);

/* 生命周期 */
r60abd1_handle_t r60abd1_init(const r60abd1_config_t *config);
esp_err_t r60abd1_deinit(r60abd1_handle_t handle);

/* 事件处理 */
esp_err_t r60abd1_add_handler(r60abd1_handle_t handle,
                                esp_event_handler_t event_handler,
                                void *event_handler_arg);
esp_err_t r60abd1_remove_handler(r60abd1_handle_t handle,
                                   esp_event_handler_t event_handler);

/* 系统功能 */
esp_err_t r60abd1_reset(r60abd1_handle_t handle);
esp_err_t r60abd1_heartbeat(r60abd1_handle_t handle);

/* 产品信息 */
esp_err_t r60abd1_get_product_model(r60abd1_handle_t handle, char *model, size_t len);
esp_err_t r60abd1_get_product_id(r60abd1_handle_t handle, char *id, size_t len);
esp_err_t r60abd1_get_hardware_version(r60abd1_handle_t handle, char *version, size_t len);
esp_err_t r60abd1_get_firmware_version(r60abd1_handle_t handle, char *version, size_t len);

/* 工作状态 */
esp_err_t r60abd1_get_init_status(r60abd1_handle_t handle, bool *completed);

/* 探测范围 */
esp_err_t r60abd1_get_range_status(r60abd1_handle_t handle, r60abd1_range_status_t *status);

/* 人体存在功能 */
esp_err_t r60abd1_set_human_enable(r60abd1_handle_t handle, bool enable);
esp_err_t r60abd1_get_human_enable(r60abd1_handle_t handle, bool *enable);
esp_err_t r60abd1_get_human_status(r60abd1_handle_t handle, r60abd1_human_status_t *status);
esp_err_t r60abd1_get_motion_status(r60abd1_handle_t handle, r60abd1_motion_status_t *status);
esp_err_t r60abd1_get_move_param(r60abd1_handle_t handle, uint8_t *param);
esp_err_t r60abd1_get_distance(r60abd1_handle_t handle, uint16_t *distance);
esp_err_t r60abd1_get_position(r60abd1_handle_t handle, r60abd1_position_t *position);

/* 呼吸检测功能 */
esp_err_t r60abd1_set_breath_enable(r60abd1_handle_t handle, bool enable);
esp_err_t r60abd1_get_breath_enable(r60abd1_handle_t handle, bool *enable);
esp_err_t r60abd1_get_breath_status(r60abd1_handle_t handle, r60abd1_breath_status_t *status);
esp_err_t r60abd1_get_breath_rate(r60abd1_handle_t handle, uint8_t *rate);
esp_err_t r60abd1_get_breath_waveform(r60abd1_handle_t handle, r60abd1_waveform_t *wave);

/* 心率监测功能 */
esp_err_t r60abd1_set_heart_enable(r60abd1_handle_t handle, bool enable);
esp_err_t r60abd1_get_heart_enable(r60abd1_handle_t handle, bool *enable);
esp_err_t r60abd1_get_heart_rate(r60abd1_handle_t handle, uint8_t *rate);
esp_err_t r60abd1_get_heart_waveform(r60abd1_handle_t handle, r60abd1_waveform_t *wave);

/* 睡眠监测功能 */
esp_err_t r60abd1_set_sleep_enable(r60abd1_handle_t handle, bool enable);
esp_err_t r60abd1_get_sleep_enable(r60abd1_handle_t handle, bool *enable);
esp_err_t r60abd1_get_bed_status(r60abd1_handle_t handle, r60abd1_bed_status_t *status);
esp_err_t r60abd1_get_sleep_state(r60abd1_handle_t handle, r60abd1_sleep_state_t *state);
esp_err_t r60abd1_get_wake_time(r60abd1_handle_t handle, uint16_t *minutes);
esp_err_t r60abd1_get_light_time(r60abd1_handle_t handle, uint16_t *minutes);
esp_err_t r60abd1_get_deep_time(r60abd1_handle_t handle, uint16_t *minutes);
esp_err_t r60abd1_get_sleep_score(r60abd1_handle_t handle, uint8_t *score);
esp_err_t r60abd1_set_report_mode(r60abd1_handle_t handle, r60abd1_report_mode_t mode);
esp_err_t r60abd1_get_report_mode(r60abd1_handle_t handle, r60abd1_report_mode_t *mode);
esp_err_t r60abd1_get_sleep_comprehensive(r60abd1_handle_t handle, r60abd1_sleep_comprehensive_t *comp);
esp_err_t r60abd1_get_sleep_analysis(r60abd1_handle_t handle, r60abd1_sleep_analysis_t *analysis);
esp_err_t r60abd1_get_sleep_abnormal(r60abd1_handle_t handle, r60abd1_sleep_abnormal_t *abnormal);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_R60ABD1_H */
