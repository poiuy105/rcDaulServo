/**
 * @file radar_ld2461.h
 * @brief HLK-LD2461 运动目标探测跟踪模组驱动头文件
 *
 * 2T4R天线配置，支持最多3块区域过滤
 * 默认波特率: 9600
 * 数据格式: 大端模式
 *
 * @copyright Copyright (c) 2024
 */

#ifndef RADAR_LD2461_H
#define RADAR_LD2461_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART默认配置 */
#define LD2461_DEFAULT_BAUD_RATE    9600
#define LD2461_DEFAULT_UART_NUM     UART_NUM_1
#define LD2461_DEFAULT_TX_PIN       1
#define LD2461_DEFAULT_RX_PIN       2
#define LD2461_UART_BUF_SIZE        512
#define LD2461_TASK_STACK_SIZE      4096
#define LD2461_TASK_PRIORITY        5

/* 帧定义 */
#define LD2461_FRAME_HEADER_0       0xFF
#define LD2461_FRAME_HEADER_1       0xEE
#define LD2461_FRAME_HEADER_2       0xDD
#define LD2461_FRAME_TAIL_0         0xDD
#define LD2461_FRAME_TAIL_1         0xEE
#define LD2461_FRAME_TAIL_2         0xFF

/* 命令字 */
#define LD2461_CMD_BAUD_RATE        0x01
#define LD2461_CMD_REPORT_FORMAT    0x02
#define LD2461_CMD_READ_REPORT_FMT  0x03
#define LD2461_CMD_SET_REGION       0x04
#define LD2461_CMD_CLEAR_REGION     0x05
#define LD2461_CMD_READ_REGION      0x06
#define LD2461_CMD_REPORT_COORD     0x07
#define LD2461_CMD_REPORT_STATUS    0x08
#define LD2461_CMD_VERSION          0x09
#define LD2461_CMD_FACTORY_RESET    0x0A

/* 上报格式 */
typedef enum {
    LD2461_REPORT_COORDINATES = 0x01,   /*!< 只显示点迹坐标值 */
    LD2461_REPORT_STATUS = 0x02,        /*!< 只显示区域内是否有目标（默认） */
    LD2461_REPORT_BOTH = 0x03,          /*!< 显示点迹坐标值和区域内是否有目标 */
} ld2461_report_format_t;

/* 区域类型 */
typedef enum {
    LD2461_REGION_DETECT_ONLY = 0x00,   /*!< 只检测区域内的目标 */
    LD2461_REGION_EXCLUDE = 0x01,       /*!< 不检测区域内的目标 */
} ld2461_region_type_t;

/* 区域编号 */
typedef enum {
    LD2461_REGION_1 = 0x01,
    LD2461_REGION_2 = 0x02,
    LD2461_REGION_3 = 0x03,
} ld2461_region_id_t;

/* 坐标点（精度0.1m，乘以10存储） */
typedef struct {
    int8_t x;   /*!< X坐标，范围-80~+80（-8.0m~+8.0m） */
    int8_t y;   /*!< Y坐标，范围0~+80（0m~+8.0m） */
} ld2461_point_t;

/* 区域定义（四边形，4个顶点） */
typedef struct {
    ld2461_point_t vertex[4];   /*!< 4个顶点，按顺序 */
    ld2461_region_type_t type;  /*!< 区域类型 */
    bool enabled;               /*!< 是否启用 */
} ld2461_region_t;

/* 目标坐标 */
typedef struct {
    float x;    /*!< X坐标，单位m */
    float y;    /*!< Y坐标，单位m */
} ld2461_target_t;

/* 区域状态 */
typedef struct {
    bool region1_occupied;      /*!< 区域1是否有人 */
    bool region2_occupied;      /*!< 区域2是否有人 */
    bool region3_occupied;      /*!< 区域3是否有人 */
} ld2461_region_status_t;

/* 版本信息 */
typedef struct {
    uint8_t year;       /*!< 年（实际年份 = 2020 + year） */
    uint8_t month;      /*!< 月 */
    uint8_t day;        /*!< 日 */
    uint8_t major;      /*!< 主版本 */
    uint8_t minor;      /*!< 次版本 */
    uint32_t unique_id; /*!< 唯一ID */
} ld2461_version_t;

/* 雷达数据 */
typedef struct {
    ld2461_target_t targets[8];     /*!< 目标坐标数组 */
    uint8_t target_count;           /*!< 目标数量 */
    ld2461_region_status_t status;  /*!< 区域状态 */
    bool has_coordinate_data;       /*!< 是否包含坐标数据 */
    bool has_status_data;           /*!< 是否包含区域状态数据 */
} ld2461_data_t;

/* 事件类型 */
typedef enum {
    LD2461_EVENT_COORDINATES,       /*!< 坐标数据上报 */
    LD2461_EVENT_REGION_STATUS,     /*!< 区域状态上报 */
} ld2461_event_type_t;

/* 事件数据 */
typedef struct {
    ld2461_event_type_t type;
    union {
        ld2461_data_t data;
    };
} ld2461_event_t;

/* 前向声明 */
typedef struct ld2461_dev ld2461_dev_t;
typedef ld2461_dev_t* ld2461_handle_t;

/* 配置结构 */
typedef struct {
    uart_port_t uart_num;           /*!< UART端口号 */
    int tx_pin;                     /*!< TX引脚 */
    int rx_pin;                     /*!< RX引脚 */
    int baud_rate;                  /*!< 波特率 */
} ld2461_config_t;

/* 默认配置 */
#define LD2461_DEFAULT_CONFIG() { \
    .uart_num = LD2461_DEFAULT_UART_NUM, \
    .tx_pin = LD2461_DEFAULT_TX_PIN, \
    .rx_pin = LD2461_DEFAULT_RX_PIN, \
    .baud_rate = LD2461_DEFAULT_BAUD_RATE, \
}

/* 事件基础声明 */
ESP_EVENT_DECLARE_BASE(LD2461_EVENT);

/**
 * @brief 初始化LD2461雷达
 *
 * @param config 配置参数
 * @return 雷达句柄，失败返回NULL
 */
ld2461_handle_t ld2461_init(const ld2461_config_t *config);

/**
 * @brief 反初始化LD2461雷达
 *
 * @param handle 雷达句柄
 * @return ESP_OK成功
 */
esp_err_t ld2461_deinit(ld2461_handle_t handle);

/**
 * @brief 注册事件处理函数
 *
 * @param handle 雷达句柄
 * @param event_handler 事件处理函数
 * @param event_handler_arg 用户参数
 * @return ESP_OK成功
 */
esp_err_t ld2461_add_handler(ld2461_handle_t handle,
                              esp_event_handler_t event_handler,
                              void *event_handler_arg);

/**
 * @brief 注销事件处理函数
 *
 * @param handle 雷达句柄
 * @param event_handler 事件处理函数
 * @return ESP_OK成功
 */
esp_err_t ld2461_remove_handler(ld2461_handle_t handle,
                                 esp_event_handler_t event_handler);

/**
 * @brief 设置波特率
 *
 * @param handle 雷达句柄
 * @param baud_rate 波特率值（9600/19200/38400/57600/115200/256000）
 * @return ESP_OK成功
 */
esp_err_t ld2461_set_baud_rate(ld2461_handle_t handle, int baud_rate);

/**
 * @brief 查询版本号和唯一ID
 *
 * @param handle 雷达句柄
 * @param version 版本信息输出
 * @return ESP_OK成功
 */
esp_err_t ld2461_get_version(ld2461_handle_t handle, ld2461_version_t *version);

/**
 * @brief 设置区域
 *
 * @param handle 雷达句柄
 * @param region_id 区域编号（1-3）
 * @param region 区域定义
 * @return ESP_OK成功
 */
esp_err_t ld2461_set_region(ld2461_handle_t handle, ld2461_region_id_t region_id,
                            const ld2461_region_t *region);

/**
 * @brief 清除区域
 *
 * @param handle 雷达句柄
 * @param region_id 区域编号（1-3）
 * @return ESP_OK成功
 */
esp_err_t ld2461_clear_region(ld2461_handle_t handle, ld2461_region_id_t region_id);

/**
 * @brief 读取所有区域设置
 *
 * @param handle 雷达句柄
 * @param regions 区域数组（3个元素）
 * @return ESP_OK成功
 */
esp_err_t ld2461_get_regions(ld2461_handle_t handle, ld2461_region_t regions[3]);

/**
 * @brief 设置上报格式
 *
 * @param handle 雷达句柄
 * @param format 上报格式
 * @return ESP_OK成功
 */
esp_err_t ld2461_set_report_format(ld2461_handle_t handle, ld2461_report_format_t format);

/**
 * @brief 读取上报格式
 *
 * @param handle 雷达句柄
 * @param format 上报格式输出
 * @return ESP_OK成功
 */
esp_err_t ld2461_get_report_format(ld2461_handle_t handle, ld2461_report_format_t *format);

/**
 * @brief 恢复出厂设置
 *
 * @param handle 雷达句柄
 * @return ESP_OK成功
 */
esp_err_t ld2461_factory_reset(ld2461_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_LD2461_H */
