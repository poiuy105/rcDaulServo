# 猫头鹰多工作模式设计文档

**日期**: 2026-06-03
**状态**: Draft
**方案**: 集中式状态机（方案A）

---

## 1. 背景与目标

当前猫头鹰（gatt_server）没有显式的模式管理，雷达跟踪和摇杆控制并行生效、互相冲突。本设计引入三种工作模式，通过模式分发层实现互斥控制。

### 目标模式

| 模式 | 触发方式 | 核心行为 |
|------|----------|----------|
| **遥控模式** | 遥控器按键切换（默认） | 摇杆控制舵机，按键控制继电器，与现有行为一致 |
| **安防模式** | 遥控器按键切换 | 雷达跟踪入侵者 + 声音灯光报警 + BLE上报 + 威慑动作 |
| **预设模式** | 遥控器按键切换 | 执行NVS中存储的预设动作序列，支持遥控器录制 |

### 设计原则

- 模式间互斥，同一时间只有一个模式活跃
- 模式切换通过现有 COMMAND 包（0x04）实现
- 遥控器端通过矩阵按键组合键触发模式切换
- 最小化对现有代码的侵入

---

## 2. 模式定义

```c
typedef enum {
    OWL_MODE_REMOTE,     // 遥控模式（默认）
    OWL_MODE_SECURITY,   // 安防模式
    OWL_MODE_PRESET,     // 预设模式
    OWL_MODE_RECORD,     // 录制模式（预设模式的子模式）
} owl_mode_t;
```

全局状态：
```c
static owl_mode_t g_current_mode = OWL_MODE_REMOTE;
```

---

## 3. 协议扩展

### 3.1 新增命令码（OWL_PKT_COMMAND, 类型0x04）

| 命令码 | 名称 | 参数 | 说明 |
|--------|------|------|------|
| 0x10 | MODE_SWITCH | 模式编号(1=安防,2=预设) | 切换工作模式 |
| 0x11 | PRESET_START | 预设编号(0-3) | 开始执行预设 |
| 0x12 | PRESET_STOP | - | 停止执行预设 |
| 0x13 | RECORD_START | 预设槽位(0-3) | 开始录制 |
| 0x14 | RECORD_STOP | - | 停止录制并保存 |
| 0x15 | RECORD_DELETE | 预设槽位(0-3) | 删除预设 |

### 3.2 新增事件包（OWL_PKT_EVENT, 类型0x82）

猫头鹰 → 遥控器，通过反馈通道(0xFF02) Notify 发送：

```c
#define OWL_EVENT_INTRUSION_DETECTED  0x01  // 检测到入侵
#define OWL_EVENT_INTRUSION_LOST      0x02  // 入侵者消失
#define OWL_EVENT_PRESET_COMPLETED    0x03  // 预设执行完成
#define OWL_EVENT_RECORD_STARTED      0x04  // 录制开始
#define OWL_EVENT_RECORD_STOPPED      0x05  // 录制停止并保存
#define OWL_EVENT_MODE_CHANGED        0x06  // 模式已切换

typedef struct __attribute__((packed)) {
    owl_packet_header_t header;   // type = 0x82
    uint8_t event_code;           // 事件类型
    uint8_t param1;               // 参数1（如入侵者X坐标）
    uint8_t param2;               // 参数2（如入侵者Y坐标）
    uint8_t param3;               // 参数3（如目标数量）
} owl_event_pkt_t;
```

### 3.3 ACK 确认包增强（OWL_PKT_ACK, 类型0x83）

```c
#define OWL_ACK_MODE_SWITCH    0x10  // 模式切换确认
#define OWL_ACK_PRESET_START   0x11  // 预设开始确认
#define OWL_ACK_PRESET_STOP    0x12  // 预设停止确认
#define OWL_ACK_RECORD_START   0x13  // 录制开始确认
#define OWL_ACK_RECORD_STOP    0x14  // 录制停止确认
```

---

## 4. 遥控器端按键映射

利用矩阵按键的**组合键**（两键同时按下）切换模式：

| 按键组合 | 功能 | 发送的命令 |
|----------|------|------------|
| 上+下 | 切换到安防模式 | MODE_SWITCH(1) |
| 左+右 | 切换到遥控模式 | MODE_SWITCH(0) |
| 上+中 | 切换到预设模式 | MODE_SWITCH(2) |
| 下+中 | 开始/停止录制 | RECORD_START / RECORD_STOP |
| 左+中 | 执行预设1 | PRESET_START(0) |
| 右+中 | 执行预设2 | PRESET_START(1) |

组合键检测逻辑：在矩阵按键扫描中，如果同一行扫描周期内检测到两个及以上按键按下，则判定为组合键。

---

## 5. 各模式行为详解

### 5.1 遥控模式（默认）

- JOYSTICK 包 → 直接驱动舵机（与现有逻辑一致）
- SWITCH 包 → 直接控制继电器（与现有逻辑一致）
- 雷达事件 → **忽略**（不驱动舵机）
- COMMAND 包 → 处理模式切换等命令
- 遥控器 LED0 = 绿色

### 5.2 安防模式

**进入条件**: 收到 MODE_SWITCH(1) 命令

**安防子状态机**:
```
SCANNING(巡逻扫描) → DETECTED(跟踪+报警) → TRACKING(持续跟踪) → LOST(目标丢失) → SCANNING
```

**各子状态行为**:

| 子状态 | 舵机 | 灯光(IO10) | 声音(IO7) | 开炮(IO8) | BLE上报 |
|--------|------|-----------|-----------|-----------|---------|
| SCANNING | 左右慢速扫描(45°-135°) | 关闭 | 关闭 | 关闭 | - |
| DETECTED | 转向入侵者 | 1Hz闪烁 | 间歇响(3s周期) | 100ms脉冲 | 入侵事件 |
| TRACKING | 持续跟踪入侵者 | 1Hz闪烁 | 间歇响 | 关闭 | 持续上报位置 |
| LOST | 保持最后方向3秒 | 关闭 | 关闭 | 关闭 | 入侵消失事件 |

**雷达坐标映射**:
- X坐标 → 舵机X角度: `angle_x = 90 + (radar_x * 45.0)`
- Y坐标 → 舵机Y角度: `angle_y = 90 + (radar_y * 45.0)`
- 无目标时 → 舵机回中点(90, 90)

**巡逻扫描**: 无目标时舵机在45°-135°之间以10°/秒的速度来回扫描

**遥控器 LED0 = 红色**，收到入侵事件时 LED2 闪红色

### 5.3 预设模式

**进入条件**: 收到 MODE_SWITCH(2) + PRESET_START(slot) 命令

**动作帧格式**（存储在 NVS，每帧8字节）:
```c
typedef struct __attribute__((packed)) {
    uint16_t delay_ms;     // 与上一帧的间隔时间 (0-60000ms)
    uint8_t  servo_x;      // X舵机角度 (0-180, 255=不变)
    uint8_t  servo_y;      // Y舵机角度 (0-180, 255=不变)
    uint8_t  relay_flags;  // 继电器状态 (Bit0:灯 Bit1:声 Bit2:炮)
    uint8_t  reserved[2];  // 保留，填充为0
} preset_frame_t;
```

**存储方案**:
- NVS 命名空间: `"owl_presets"`
- Key: `"preset_0"` ~ `"preset_3"`（4个槽位）
- 每个槽位最大存储: NVS blob 最大 32KB / 8字节 = ~4000帧
- 录制时长限制: 60秒

**执行逻辑**:
1. 从 NVS 读取指定槽位的帧数据
2. 创建预设执行任务 `preset_player_task`
3. 按帧间隔依次执行舵机/继电器动作
4. 执行中忽略 JOYSTICK/SWITCH 包
5. 收到 PRESET_STOP 或 MODE_SWITCH 命令时停止
6. 执行完毕发送 OWL_EVENT_PRESET_COMPLETED 事件

**遥控器 LED0 = 蓝色**

### 5.4 录制模式

**进入条件**: 收到 RECORD_START(slot) 命令

**录制流程**:
1. 猫头鹰端进入录制模式，初始化帧缓冲区
2. 每收到 JOYSTICK 包 → 记录一帧（servo_x, servo_y, delay_ms）
3. 每收到 SWITCH 包 → 记录一帧（relay_flags, delay_ms）
4. delay_ms 通过 esp_timer_get_time() 计算帧间时间差
5. 帧缓冲区满或录制超过60秒 → 自动停止录制
6. 收到 RECORD_STOP → 将缓冲区写入 NVS 指定槽位
7. 发送 OWL_EVENT_RECORD_STOPPED 事件确认保存

**录制中**:
- JOYSTICK/SWITCH 包**同时**驱动硬件（用户可以看到录制效果）
- 雷达事件忽略
- COMMAND 包仍处理（可停止录制）

**遥控器 LED3 = 黄色常亮**

---

## 6. 模式切换流程

```
遥控器检测组合键
    → 构造 COMMAND 包 (cmd=MODE_SWITCH, param=模式编号)
    → BLE Write 到猫头鹰
    → 猫头鹰 parse_command_packet() 处理
    → 调用 mode_switch(new_mode)
        ├── mode_exit(g_current_mode)    // 退出当前模式清理
        │     ├── 遥控模式: 无需清理
        │     ├── 安防模式: 停止报警、舵机回中
        │     ├── 预设模式: 停止执行任务
        │     └── 录制模式: 保存/丢弃录制数据
        ├── g_current_mode = new_mode
        ├── mode_enter(new_mode)         // 进入新模式初始化
        │     ├── 遥控模式: 无需初始化
        │     ├── 安防模式: 启动安防状态机
        │     ├── 预设模式: 等待 PRESET_START 命令
        │     └── 录制模式: 初始化帧缓冲区
        └── 发送 ACK 确认 + MODE_CHANGED 事件
    → 遥控器收到 ACK，更新 LED 指示
```

---

## 7. 代码架构

### 7.1 gatt_server 端改动

在 `gatts_demo.c` 中新增以下模块化函数组：

```
// 模式管理核心
static owl_mode_t g_current_mode = OWL_MODE_REMOTE;
static void mode_switch(owl_mode_t new_mode);
static void mode_exit(owl_mode_t mode);
static void mode_enter(owl_mode_t mode);

// 模式分发层（修改现有 parse_control_packet）
static void parse_control_packet(uint8_t *data, uint16_t len) {
    // 根据包类型，先判断当前模式是否允许处理
    // 再分发到对应处理函数
}

// 安防模式
static void security_enter(void);
static void security_exit(void);
static void security_radar_handler(radar_data_t *data);
static void security_update_state(void);  // 定时调用，驱动子状态机
static owl_security_state_t g_security_state;

// 预设模式
static void preset_enter(void);
static void preset_exit(void);
static void preset_start(uint8_t slot);
static void preset_stop(void);
static void preset_player_task(void *arg);  // FreeRTOS 任务

// 录制模式
static void record_enter(uint8_t slot);
static void record_exit(void);
static void record_add_joystick_frame(owl_joystick_pkt_t *pkt);
static void record_add_switch_frame(owl_switch_pkt_t *pkt);
static void record_save(void);
```

### 7.2 gatt_client 端改动

在 `gattc_demo.c` 中新增：

```
// 组合键检测
static void check_combo_keys(uint8_t switch1, uint8_t switch2);

// 模式管理
static owl_mode_t g_current_mode = OWL_MODE_REMOTE;
static void send_mode_switch(uint8_t mode);
static void send_preset_command(uint8_t cmd, uint8_t param);

// 事件处理（解析猫头鹰发来的 EVENT 包）
static void handle_event_packet(uint8_t *data, uint16_t len);

// LED 模式指示更新
static void update_mode_led(owl_mode_t mode);
```

### 7.3 protocol/owl_protocol.h 改动

新增命令码常量、事件类型常量、preset_frame_t 结构体、owl_event_pkt_t 结构体。

---

## 8. NVS 存储规划

| 命名空间 | Key | 类型 | 大小 | 说明 |
|----------|-----|------|------|------|
| `owl_binding` | `remote_mac` | blob | 6B | 绑定的遥控器MAC（已有） |
| `owl_presets` | `preset_0` | blob | 最大32KB | 预设槽位0 |
| `owl_presets` | `preset_1` | blob | 最大32KB | 预设槽位1 |
| `owl_presets` | `preset_2` | blob | 最大32KB | 预设槽位2 |
| `owl_presets` | `preset_3` | blob | 最大32KB | 预设槽位3 |

---

## 9. 错误处理

| 场景 | 处理方式 |
|------|----------|
| 模式切换命令参数无效 | 发送 OWL_PKT_ERROR，忽略命令 |
| 预设槽位不存在或为空 | 发送 OWL_PKT_ERROR，不执行 |
| 录制缓冲区满 | 自动停止录制，保存已有数据，发送事件 |
| NVS 写入失败 | 发送 OWL_PKT_ERROR，通知遥控器 |
| 安防模式下BLE断开 | 保持安防模式运行（雷达+报警仍工作） |
| 预设执行中BLE断开 | 继续执行完当前预设 |

---

## 10. 实现优先级

建议分3个阶段实现：

**阶段1 - 基础框架 + 遥控模式**:
- 模式枚举和状态管理
- 模式分发层
- 模式切换命令（协议扩展）
- 确保遥控模式行为与现有一致（回归测试）

**阶段2 - 安防模式**:
- 安防子状态机
- 雷达驱动舵机跟踪
- 报警行为（灯光闪烁、声音、威慑动作）
- BLE 事件上报
- 遥控器端组合键和LED指示

**阶段3 - 预设模式 + 录制**:
- 预设帧格式和NVS存储
- 预设执行任务
- 录制模式
- 遥控器端录制UI
