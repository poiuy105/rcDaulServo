/**
 * @file owl_protocol.h
 * @brief 猫头鹰玩具 BLE 通讯协议定义
 * @version 1.0
 * @date 2026-05-23
 */

#ifndef OWL_PROTOCOL_H
#define OWL_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 *                              UUID 定义
 *============================================================================*/

/** 猫头鹰服务 UUID */
#define OWL_SERVICE_UUID            0xFF00

/** 控制通道特征值 UUID (遥控器 → 猫头鹰) */
#define OWL_CHAR_CONTROL_UUID       0xFF01

/** 反馈通道特征值 UUID (猫头鹰 → 遥控器) */
#define OWL_CHAR_FEEDBACK_UUID      0xFF02

/** 系统命令特征值 UUID (双向) */
#define OWL_CHAR_COMMAND_UUID       0xFF03

/** 设备名称 */
#define OWL_DEVICE_NAME             "OWL_TOY"

/** 绑定信息 Magic */
#define OWL_BINDING_MAGIC           0xAB

/** NVS 命名空间 */
#define OWL_NVS_NAMESPACE           "owl_binding"
#define OWL_KEY_REMOTE_MAC          "remote_mac"

/*============================================================================
 *                              协议版本
 *============================================================================*/

#define OWL_PROTOCOL_VERSION        0x10    // 版本 1.0

/*============================================================================
 *                              包类型定义
 *============================================================================*/

/** 控制通道包类型 (遥控器 → 猫头鹰) */
#define OWL_PKT_JOYSTICK            0x01    // 摇杆数据
#define OWL_PKT_SWITCH              0x02    // 开关状态
#define OWL_PKT_HEARTBEAT           0x03    // 心跳/保活
#define OWL_PKT_COMMAND             0x04    // 系统命令

/** 反馈通道包类型 (猫头鹰 → 遥控器) */
#define OWL_PKT_STATUS              0x81    // 周期性状态反馈
#define OWL_PKT_EVENT               0x82    // 事件通知
#define OWL_PKT_ACK                 0x83    // 命令确认
#define OWL_PKT_ERROR               0x84    // 错误报告

/** 雷达状态包类型 */
#define OWL_PKT_RADAR_STATUS      0x18    // 雷达目标状态上报

/*============================================================================
 *                              命令码定义
 *============================================================================*/

#define OWL_CMD_PAIR_REQUEST        0x01    // 配对请求
#define OWL_CMD_CALIBRATE           0x02    // 校准模式
#define OWL_CMD_SERVO_CENTER        0x03    // 舵机中点设置
#define OWL_CMD_RESET               0x04    // 复位
#define OWL_CMD_SLEEP               0x05    // 休眠
#define OWL_CMD_WAKEUP              0x06    // 唤醒
#define OWL_CMD_LIGHT_BRIGHTNESS    0x07    // 灯光亮度
#define OWL_CMD_VOLUME              0x08    // 音量设置
#define OWL_CMD_EMERGENCY_STOP      0x09    // 紧急停止
#define OWL_CMD_OTA_START           0x0A    // OTA 开始

/* 工作模式命令码 */
#define OWL_CMD_MODE_SWITCH         0x10    // 切换工作模式 (param: 0=遥控, 1=安防, 2=预设)
#define OWL_CMD_PRESET_START        0x11    // 开始执行预设 (param: 槽位0-3)
#define OWL_CMD_PRESET_STOP         0x12    // 停止执行预设
#define OWL_CMD_RECORD_START       0x13    // 开始录制 (param: 槽位0-3)
#define OWL_CMD_RECORD_STOP        0x14    // 停止录制并保存
#define OWL_CMD_RECORD_DELETE      0x15    // 删除预设 (param: 槽位0-3)

/*============================================================================
 *                              事件类型定义
 *============================================================================*/

#define OWL_EVENT_FIRE_COMPLETE     0x01    // 开炮完成
#define OWL_EVENT_SOUND_COMPLETE    0x02    // 声音播放完成
#define OWL_EVENT_LOW_BATTERY       0x03    // 低电量警告
#define OWL_EVENT_SERVO_STALL       0x04    // 舵机堵转
#define OWL_EVENT_SLEEP             0x05    // 进入休眠
#define OWL_EVENT_WAKEUP            0x06    // 被唤醒

/* 工作模式事件类型 */
#define OWL_EVENT_INTRUSION_DETECTED  0x10  // 检测到入侵
#define OWL_EVENT_INTRUSION_LOST      0x11  // 入侵者消失
#define OWL_EVENT_PRESET_COMPLETED    0x12  // 预设执行完成
#define OWL_EVENT_RECORD_STARTED      0x13  // 录制开始
#define OWL_EVENT_RECORD_STOPPED      0x14  // 录制停止并保存
#define OWL_EVENT_MODE_CHANGED        0x15  // 模式已切换

/*============================================================================
 *                              错误码定义
 *============================================================================*/

#define OWL_ERR_NONE                0x00    // 正常
#define OWL_ERR_LOW_BATTERY         0x01    // 低电量
#define OWL_ERR_SERVO_X_STALL       0x02    // X舵机堵转
#define OWL_ERR_SERVO_Y_STALL       0x03    // Y舵机堵转
#define OWL_ERR_OVERHEAT            0x04    // 过热
#define OWL_ERR_COMM_TIMEOUT        0x05    // 通讯超时
#define OWL_ERR_INVALID_CMD         0x06    // 无效命令
#define OWL_ERR_EXEC_FAILED         0x07    // 执行失败

/*============================================================================
 *                              数据结构定义
 *============================================================================*/

/**
 * @brief 包头结构 (所有包共用)
 */
typedef struct __attribute__((packed)) {
    uint8_t version_type;   // Bit 7-4: 协议版本, Bit 3-0: 包类型
    uint8_t seq;            // 序列号 (0-255, 循环)
    uint8_t timestamp;      // 时间戳低字节 (ms % 256)
} owl_packet_header_t;

/**
 * @brief 摇杆数据包 (类型 0x01)
 * @note 总长度: 6 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t x_axis;         // X轴 (0-255, 128=中点)
    uint8_t y_axis;         // Y轴 (0-255, 128=中点)
    uint8_t button_flags;   // Bit 0: 按键, Bit 4-7: 摇杆ID
} owl_joystick_pkt_t;

/**
 * @brief 开关数据包 (类型 0x02)
 * @note 总长度: 6 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t switch1;        // 开关组 #1 (Bit 0-4: 中/右/左/下/上)
    uint8_t switch2;        // 开关组 #2
    uint8_t change_flags;   // 变化标志 (Bit 0: 组1, Bit 1: 组2)
} owl_switch_pkt_t;

/**
 * @brief 心跳数据包 (类型 0x03)
 * @note 总长度: 5 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t status;         // Bit 0: 低电量, Bit 1: 信号弱
    uint8_t battery;        // 电池电压 (0.1V 单位)
} owl_heartbeat_pkt_t;

/**
 * @brief 系统命令数据包 (类型 0x04)
 * @note 总长度: 6 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t cmd;            // 命令码
    uint8_t param;          // 命令参数
    uint8_t need_ack;       // 是否需要确认 (1=需要)
} owl_command_pkt_t;

/**
 * @brief 状态反馈数据包 (类型 0x81)
 * @note 总长度: 9 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t servo_status;   // Bit 0: X到位, Bit 1: Y到位, Bit 2: X运动中, Bit 3: Y运动中
    uint8_t x_angle;        // X舵机当前角度 (0-180)
    uint8_t y_angle;        // Y舵机当前角度 (0-180)
    uint8_t relay_state;    // Bit 0: 眼睛灯, Bit 1: 声音, Bit 2: 开炮
    uint8_t battery;        // 电池电压 (0.1V 单位)
    int8_t rssi;            // 信号强度 (dBm, 负数)
} owl_status_pkt_t;

/**
 * @brief 事件通知数据包 (类型 0x82)
 * @note 总长度: 7 bytes (header 3 + event_type 1 + p1 1 + p2 1 + p3 1)
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t event_type;     // 事件类型
    uint8_t p1;             // 参数1 (如: 槽位号、模式值)
    uint8_t p2;             // 参数2 (如: 帧数高字节)
    uint8_t p3;             // 参数3 (如: 帧数低字节、目标数)
} owl_event_pkt_t;

/**
 * @brief 命令确认数据包 (类型 0x83)
 * @note 总长度: 6 bytes
 */
typedef struct __attribute__((packed)) {
    owl_packet_header_t header;
    uint8_t cmd;            // 命令码 (回显)
    uint8_t result;         // 执行结果
    uint8_t data;           // 结果数据
} owl_ack_pkt_t;

/*============================================================================
 *                              辅助宏定义
 *============================================================================*/

/** 构建包头 */
#define OWL_MAKE_HEADER(pkt_type) \
    ((OWL_PROTOCOL_VERSION & 0xF0) | (pkt_type & 0x0F))

/** 获取包类型 */
#define OWL_GET_PKT_TYPE(header) \
    ((header)->version_type & 0x0F)

/** 获取协议版本 */
#define OWL_GET_VERSION(header) \
    ((header)->version_type >> 4)

/** 开关位定义 */
#define OWL_SW_CENTER   (1 << 0)
#define OWL_SW_RIGHT    (1 << 1)
#define OWL_SW_LEFT     (1 << 2)
#define OWL_SW_DOWN     (1 << 3)
#define OWL_SW_UP       (1 << 4)

/** 舵机状态位定义 */
#define OWL_SERVO_X_REACHED     (1 << 0)
#define OWL_SERVO_Y_REACHED     (1 << 1)
#define OWL_SERVO_X_MOVING      (1 << 2)
#define OWL_SERVO_Y_MOVING      (1 << 3)

/** 继电器状态位定义 */
#define OWL_RELAY_EYE_LIGHT     (1 << 0)
#define OWL_RELAY_SOUND         (1 << 1)
#define OWL_RELAY_CANNON        (1 << 2)

/*============================================================================
 *                              包大小定义
 *============================================================================*/

#define OWL_PKT_JOYSTICK_SIZE   sizeof(owl_joystick_pkt_t)
#define OWL_PKT_SWITCH_SIZE     sizeof(owl_switch_pkt_t)
#define OWL_PKT_HEARTBEAT_SIZE  sizeof(owl_heartbeat_pkt_t)
#define OWL_PKT_COMMAND_SIZE    sizeof(owl_command_pkt_t)
#define OWL_PKT_STATUS_SIZE     sizeof(owl_status_pkt_t)
#define OWL_PKT_EVENT_SIZE      sizeof(owl_event_pkt_t)
#define OWL_PKT_ACK_SIZE        sizeof(owl_ack_pkt_t)

/*============================================================================
 *                              工作模式定义
 *============================================================================*/

/** 工作模式枚举 */
typedef enum {
    OWL_MODE_REMOTE,     // 遥控模式（默认）
    OWL_MODE_SECURITY,   // 安防模式
    OWL_MODE_PRESET,     // 预设模式
    OWL_MODE_RECORD,     // 录制模式（预设模式的子模式）
} owl_mode_t;

/** 安防子状态枚举 */
typedef enum {
    SEC_STATE_SCANNING,  // 巡逻扫描
    SEC_STATE_DETECTED,  // 检测到入侵
    SEC_STATE_TRACKING,  // 持续跟踪
    SEC_STATE_LOST,      // 目标丢失
} owl_security_state_t;

/** 预设槽位数量 */
#define OWL_PRESET_SLOT_COUNT   4
#define OWL_PRESET_MAX_FRAMES   4000   // 每个槽位最大帧数
#define OWL_PRESET_RECORD_MAX_MS 60000  // 录制最长60秒

/** 预设 NVS 命名空间 */
#define OWL_PRESETS_NVS_NAMESPACE  "owl_presets"

/** 预设动作帧格式 (8字节) */
typedef struct __attribute__((packed)) {
    uint16_t delay_ms;     // 与上一帧的间隔时间 (0-60000ms)
    uint8_t  servo_x;      // X舵机角度 (0-180, 255=不变)
    uint8_t  servo_y;      // Y舵机角度 (0-180, 255=不变)
    uint8_t  relay_flags;  // 继电器状态 (Bit0:灯 Bit1:声 Bit2:炮)
    uint8_t  reserved[2];  // 保留，填充为0
} preset_frame_t;

#define OWL_PRESET_FRAME_SIZE  sizeof(preset_frame_t)

/** servo_x/servo_y 不变的标记值 */
#define OWL_SERVO_NO_CHANGE     255

#ifdef __cplusplus
}
#endif

#endif /* OWL_PROTOCOL_H */
