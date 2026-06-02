/**
 * @file radar_ld2452.h
 * @brief HLK-LD2452 高精度多目标识别传感器驱动头文件
 *
 * 1T2R天线，纯上报型雷达，最多3个目标
 * 默认波特率: 9600
 * 帧格式: AA FF 03 00 [targets] 55 CC
 *
 * @copyright Copyright (c) 2024
 */

#ifndef RADAR_LD2452_H
#define RADAR_LD2452_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART默认配置 */
#define LD2452_DEFAULT_BAUD_RATE    9600
#define LD2452_DEFAULT_UART_NUM     UART_NUM_1
#define LD2452_DEFAULT_TX_PIN       1
#define LD2452_DEFAULT_RX_PIN       2
#define LD2452_UART_BUF_SIZE        512
#define LD2452_TASK_STACK_SIZE      4096
#define LD2452_TASK_PRIORITY        5

/* 帧定义 */
#define LD2452_FRAME_HEADER_0       0xAA
#define LD2452_FRAME_HEADER_1       0xFF
#define LD2452_FRAME_HEADER_2       0x03
#define LD2452_FRAME_HEADER_3       0x00
#define LD2452_FRAME_TAIL_0         0x55
#define LD2452_FRAME_TAIL_1         0xCC

/* 帧长度: 4(头) + 8*3(目标) + 2(尾) = 30 */
#define LD2452_FRAME_SIZE           30
#define LD2452_TARGET_DATA_SIZE     8
#define LD2452_MAX_TARGETS          3

/* 目标数据 */
typedef struct {
    int16_t x;          /*!< X坐标，单位mm（自定义符号位编码） */
    int16_t y;          /*!< Y坐标，单位mm（自定义符号位编码） */
    int16_t speed;      /*!< 速度，单位cm/s（自定义符号位编码，靠近雷达为负） */
    uint16_t distance;  /*!< 像素距离，单位mm */
} ld2452_target_raw_t;

/* 解析后的目标数据（物理单位） */
typedef struct {
    float x;            /*!< X坐标，单位m */
    float y;            /*!< Y坐标，单位m */
    float speed;        /*!< 速度，单位m/s */
    float distance;     /*!< 像素距离，单位m */
} ld2452_target_t;

/* 雷达数据 */
typedef struct {
    ld2452_target_t targets[LD2452_MAX_TARGETS];   /*!< 目标数组 */
    uint8_t target_count;                           /*!< 有效目标数量 */
} ld2452_data_t;

/* 事件ID */
typedef enum {
    LD2452_EVENT_TARGET_UPDATE = 0,     /*!< 目标数据更新 */
} ld2452_event_id_t;

/* 事件数据 */
typedef struct {
    ld2452_event_id_t id;
    ld2452_data_t data;
} ld2452_event_t;

/* 前向声明 */
typedef struct ld2452_dev ld2452_dev_t;
typedef ld2452_dev_t* ld2452_handle_t;

/* 配置结构 */
typedef struct {
    uart_port_t uart_num;           /*!< UART端口号 */
    int tx_pin;                     /*!< TX引脚 */
    int rx_pin;                     /*!< RX引脚 */
    int baud_rate;                  /*!< 波特率 */
} ld2452_config_t;

/* 默认配置 */
#define LD2452_DEFAULT_CONFIG() { \
    .uart_num = LD2452_DEFAULT_UART_NUM, \
    .tx_pin = LD2452_DEFAULT_TX_PIN, \
    .rx_pin = LD2452_DEFAULT_RX_PIN, \
    .baud_rate = LD2452_DEFAULT_BAUD_RATE, \
}

/* 事件基础声明 */
ESP_EVENT_DECLARE_BASE(LD2452_EVENT);

/**
 * @brief 初始化LD2452雷达
 *
 * @param config 配置参数
 * @return 雷达句柄，失败返回NULL
 */
ld2452_handle_t ld2452_init(const ld2452_config_t *config);

/**
 * @brief 反初始化LD2452雷达
 *
 * @param handle 雷达句柄
 * @return ESP_OK成功
 */
esp_err_t ld2452_deinit(ld2452_handle_t handle);

/**
 * @brief 注册事件处理函数
 *
 * @param handle 雷达句柄
 * @param event_handler 事件处理函数
 * @param event_handler_arg 用户参数
 * @return ESP_OK成功
 */
esp_err_t ld2452_add_handler(ld2452_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *event_handler_arg);

/**
 * @brief 注销事件处理函数
 *
 * @param handle 雷达句柄
 * @param event_handler 事件处理函数
 * @return ESP_OK成功
 */
esp_err_t ld2452_remove_handler(ld2452_handle_t handle,
                                 esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_LD2452_H */
