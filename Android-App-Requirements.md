# BLE猫头鹰玩具 - Android App 开发需求文档

## 项目概述

开发Android App替代ESP32C3遥控器，通过手机BLE与猫头鹰ESP32C3通信，实现完整遥控功能并增加手机特有的增强功能。

---

## 一、功能需求

### 1.1 基础功能（完全替代ESP32C3）

| 功能模块 | 需求描述 | 优先级 |
|---------|---------|--------|
| BLE连接 | 扫描、连接、断开、重连 | P0 |
| 虚拟摇杆 | 双轴摇杆控制舵机X/Y | P0 |
| 虚拟按键 | 5向开关（上/下/左/右/中） | P0 |
| 心跳维持 | 定时发送心跳包 | P0 |
| MAC绑定 | 首次连接自动绑定 | P0 |
| 状态显示 | 连接状态、电量、信号强度 | P0 |

### 1.2 增强功能（手机特有）

| 功能模块 | 需求描述 | 优先级 |
|---------|---------|--------|
| 双摇杆模式 | 左摇杆舵机，右摇杆预留 | P1 |
| 手势控制 | 陀螺仪体感控制 | P2 |
| 语音控制 | 语音指令触发动作 | P2 |
| 编程模式 | 录制动作序列并回放 | P2 |
| 摄像头追踪 | 视觉识别自动瞄准 | P3 |
| 多设备管理 | 保存多个猫头鹰配置 | P1 |
| 固件OTA | 通过手机更新猫头鹰固件 | P2 |
| 数据记录 | 操作日志、使用统计 | P3 |
| 振动反馈 | 按键/触发时振动 | P1 |
| 音效反馈 | 按键音、连接提示音 | P2 |

---

## 二、BLE通信协议详解

### 2.1 服务与特征值

```kotlin
// 猫头鹰服务
val OWL_SERVICE_UUID = UUID.fromString("00001815-0000-1000-8000-00805f9b34fb")

// 特征值
val CHAR_CONTROL_UUID = UUID.fromString("00002a52-0000-1000-8000-00805f9b34fb")   // 写（无响应）
val CHAR_FEEDBACK_UUID = UUID.fromString("00002a56-0000-1000-8000-00805f9b34fb")  // 通知
val CHAR_COMMAND_UUID = UUID.fromString("00002a3d-0000-1000-8000-00805f9b34fb")   // 写+指示
val CCC_DESCRIPTOR_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb") // 客户端配置描述符
```

### 2.2 数据包格式

#### JOYSTICK包（0x11）
```kotlin
// 6字节
// [0] = 类型 (0x11)
// [1] = 序列号 (0-255循环)
// [2] = 时间戳 (毫秒低8位)
// [3] = X轴 (0-255, 128为中点)
// [4] = Y轴 (0-255, 128为中点)
// [5] = 按键标志 (bit0=摇杆按键)

fun buildJoystickPacket(seq: Int, x: Int, y: Int, button: Boolean): ByteArray {
    return byteArrayOf(
        0x11.toByte(),                    // 类型
        (seq and 0xFF).toByte(),          // 序列号
        (System.currentTimeMillis() and 0xFF).toByte(), // 时间戳
        x.coerceIn(0, 255).toByte(),      // X轴
        y.coerceIn(0, 255).toByte(),      // Y轴
        if (button) 0x01 else 0x00        // 按键
    )
}
```

#### SWITCH包（0x12）
```kotlin
// 6字节
// [0] = 类型 (0x12)
// [1] = 序列号
// [2] = 时间戳
// [3] = 开关组1 (bit0=上, bit1=下, bit2=左, bit3=右, bit4=中)
// [4] = 开关组2
// [5] = 变化标志

// 开关位定义
const val SW_UP = 0x01
const val SW_DOWN = 0x02
const val SW_LEFT = 0x04
const val SW_RIGHT = 0x08
const val SW_CENTER = 0x10

fun buildSwitchPacket(seq: Int, sw1: Int, sw2: Int): ByteArray {
    return byteArrayOf(
        0x12.toByte(),
        (seq and 0xFF).toByte(),
        (System.currentTimeMillis() and 0xFF).toByte(),
        sw1.toByte(),
        sw2.toByte(),
        0x03.toByte()  // 两组都有变化
    )
}
```

#### HEARTBEAT包（0x13）
```kotlin
// 5字节
// [0] = 类型 (0x13)
// [1] = 序列号
// [2] = 时间戳
// [3] = 状态 (0x00=正常)
// [4] = 电池电压 (37=3.7V)

fun buildHeartbeatPacket(seq: Int, batteryPercent: Int): ByteArray {
    val batteryVoltage = (batteryPercent * 37 / 100).coerceIn(30, 42)
    return byteArrayOf(
        0x13.toByte(),
        (seq and 0xFF).toByte(),
        (System.currentTimeMillis() and 0xFF).toByte(),
        0x00.toByte(),  // 状态正常
        batteryVoltage.toByte()
    )
}
```

#### COMMAND包（0x14）
```kotlin
// 6字节
// [0] = 类型 (0x14)
// [1] = 序列号
// [2] = 时间戳
// [3] = 命令码
// [4] = 参数
// [5] = 需确认标志

// 命令码定义
const val CMD_EMERGENCY_STOP = 0x01  // 紧急停止
const val CMD_RESET = 0x02           // 系统复位
const val CMD_CALIBRATE = 0x03       // 校准模式
const val CMD_OTA_START = 0x10       // OTA开始
const val CMD_OTA_DATA = 0x11        // OTA数据
const val CMD_OTA_END = 0x12         // OTA结束

fun buildCommandPacket(seq: Int, cmd: Int, param: Int, needAck: Boolean): ByteArray {
    return byteArrayOf(
        0x14.toByte(),
        (seq and 0xFF).toByte(),
        (System.currentTimeMillis() and 0xFF).toByte(),
        cmd.toByte(),
        param.toByte(),
        if (needAck) 0x01 else 0x00
    )
}
```

### 2.3 通信时序

```
连接建立:
App扫描 → 发现猫头鹰 → 连接 → 发现服务 → 使能通知 → 发送绑定确认

正常运行:
每500ms: JOYSTICK包 + HEARTBEAT包交替发送
按键变化: 立即发送SWITCH包
心跳超时: 3秒未收到服务端心跳 → 标记断线 → 自动重连

断开连接:
App主动断开 或 心跳超时 → 清理资源 → 开始扫描重连
```

### 2.4 序列号管理

```kotlin
class SequenceManager {
    private var joystickSeq = 0
    private var heartbeatSeq = 0
    private var switchSeq = 0
    private var commandSeq = 0
    
    fun getJoystickSeq(): Int = joystickSeq++ and 0xFF
    fun getHeartbeatSeq(): Int = heartbeatSeq++ and 0xFF
    fun getSwitchSeq(): Int = switchSeq++ and 0xFF
    fun getCommandSeq(): Int = commandSeq++ and 0xFF
    
    fun reset() {
        joystickSeq = 0
        heartbeatSeq = 0
        switchSeq = 0
        commandSeq = 0
    }
}
```

---

## 三、UI界面设计

### 3.1 主界面布局

```
┌─────────────────────────────────────┐
│  [连接状态] [电池: 80%] [信号: -65dBm] │  ← 状态栏
├─────────────────────────────────────┤
│                                     │
│         ┌─────────┐                 │
│         │  摄像头  │                 │  ← 预留摄像头区域
│         │  (预留)  │                 │
│         └─────────┘                 │
│                                     │
├─────────────────────────────────────┤
│                                     │
│    ┌─────┐         ┌─────┐         │
│    │ 摇杆 │         │ 摇杆 │         │  ← 双摇杆
│    │  X  │         │  Y  │         │
│    └─────┘         └─────┘         │
│                                     │
├─────────────────────────────────────┤
│  [上] [下] [左] [右] [中]           │  ← 方向键
│                                     │
│  [灯光] [声音] [开炮] [编程]        │  ← 功能键
│                                     │
└─────────────────────────────────────┘
```

### 3.2 设置界面

- BLE设备管理（扫描、绑定、删除）
- 摇杆灵敏度调节
- 按键映射自定义
- 振动/音效开关
- 陀螺仪校准
- 关于/帮助

---

## 四、技术架构

### 4.1 项目结构

```
app/
├── src/main/java/com/example/owlremote/
│   ├── data/
│   │   ├── ble/
│   │   │   ├── BleManager.kt           # BLE管理器
│   │   │   ├── BleService.kt           # BLE服务封装
│   │   │   ├── PacketBuilder.kt        # 数据包构建
│   │   │   └── PacketParser.kt         # 数据包解析
│   │   ├── model/
│   │   │   ├── DeviceInfo.kt           # 设备信息
│   │   │   ├── ConnectionState.kt      # 连接状态
│   │   │   └── ControlData.kt          # 控制数据
│   │   └── repository/
│   │       └── DeviceRepository.kt     # 设备数据仓库
│   ├── ui/
│   │   ├── main/
│   │   │   ├── MainActivity.kt
│   │   │   ├── MainViewModel.kt
│   │   │   └── MainScreen.kt           # Compose界面
│   │   ├── components/
│   │   │   ├── VirtualJoystick.kt      # 虚拟摇杆组件
│   │   │   ├── DirectionPad.kt         # 方向键组件
│   │   │   └── StatusBar.kt            # 状态栏组件
│   │   └── settings/
│   │       └── SettingsScreen.kt
│   └── utils/
│       ├── Permissions.kt              # 权限管理
│       └── Constants.kt                # 常量定义
├── src/main/res/
└── build.gradle.kts
```

### 4.2 核心类设计

```kotlin
// BLE管理器
class BleManager(context: Context) {
    private val bluetoothAdapter: BluetoothAdapter
    private var gatt: BluetoothGatt? = null
    private val scope = CoroutineScope(Dispatchers.IO)
    
    // 状态流
    val connectionState: StateFlow<ConnectionState>
    val receivedData: SharedFlow<ByteArray>
    
    // 方法
    fun startScan(): Flow<BluetoothDevice>
    fun connect(device: BluetoothDevice)
    fun disconnect()
    fun sendPacket(packet: ByteArray)
    fun enableNotifications()
}

// 虚拟摇杆
class VirtualJoystickView(context: Context) : View(context) {
    var onMove: ((x: Int, y: Int) -> Unit)? = null
    var onRelease: (() -> Unit)? = null
    
    // 绘制摇杆UI
    // 处理触摸事件
    // 映射到0-255范围
}

// 主ViewModel
class MainViewModel : ViewModel() {
    private val bleManager: BleManager
    private val seqManager = SequenceManager()
    
    // UI状态
    val joystickX = mutableStateOf(128)
    val joystickY = mutableStateOf(128)
    val isConnected = mutableStateOf(false)
    val batteryLevel = mutableStateOf(0)
    
    // 定时发送
    private fun startHeartbeatTimer() {
        viewModelScope.launch {
            while (isActive) {
                if (isConnected.value) {
                    sendJoystickPacket()
                    delay(500)
                    sendHeartbeatPacket()
                    delay(500)
                }
            }
        }
    }
}
```

---

## 五、权限需求

### 5.1 Android权限

```xml
<!-- AndroidManifest.xml -->
<uses-permission android:name="android.permission.BLUETOOTH" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
<uses-permission android:name="android.permission.VIBRATE" />
<uses-permission android:name="android.permission.RECORD_AUDIO" /> <!-- 语音控制 -->
<uses-permission android:name="android.permission.CAMERA" /> <!-- 摄像头追踪 -->

<uses-feature android:name="android.hardware.bluetooth_le" android:required="true" />
```

### 5.2 运行时权限申请

- Android 12+ (API 31+): `BLUETOOTH_SCAN`, `BLUETOOTH_CONNECT`
- Android 6-11: `ACCESS_FINE_LOCATION`
- 语音控制: `RECORD_AUDIO`
- 摄像头: `CAMERA`

---

## 六、开发计划

### 阶段1：MVP（2周）
- [ ] 项目搭建 + BLE基础框架
- [ ] 虚拟摇杆UI
- [ ] 连接/断开功能
- [ ] JOYSTICK包发送
- [ ] 状态显示

### 阶段2：完整功能（2周）
- [ ] 虚拟按键
- [ ] SWITCH包发送
- [ ] HEARTBEAT维持
- [ ] 自动重连
- [ ] MAC绑定
- [ ] 振动反馈

### 阶段3：增强功能（2周）
- [ ] 设置界面
- [ ] 多设备管理
- [ ] 手势控制（陀螺仪）
- [ ] 语音控制
- [ ] 编程模式

### 阶段4：优化发布（1周）
- [ ] UI美化
- [ ] 性能优化
- [ ] 测试修复
- [ ] 发布Google Play

---

## 七、测试清单

### 7.1 功能测试
- [ ] BLE扫描发现设备
- [ ] 连接/断开/重连
- [ ] 摇杆控制舵机
- [ ] 按键控制继电器
- [ ] 心跳超时检测
- [ ] MAC绑定验证
- [ ] 后台运行稳定性

### 7.2 兼容性测试
- [ ] Android 8.0 (API 26)
- [ ] Android 10 (API 29)
- [ ] Android 12 (API 31)
- [ ] Android 14 (API 34)
- [ ] 不同品牌手机（小米、华为、三星）

### 7.3 性能测试
- [ ] 摇杆响应延迟 < 50ms
- [ ] 连续运行1小时无断连
- [ ] 10米距离通信稳定
- [ ] 电池消耗 < 5%/小时

---

## 八、风险与应对

| 风险 | 影响 | 应对措施 |
|------|------|---------|
| BLE连接不稳定 | 高 | 实现自动重连机制，增加连接超时处理 |
| 不同手机BLE差异 | 中 | 多机型测试，适配主流品牌 |
| 后台被杀 | 中 | 使用前台服务，引导用户设置白名单 |
| 陀螺仪精度差 | 低 | 提供校准功能，允许灵敏度调节 |
| 语音识别不准 | 低 | 使用离线识别，限定指令集 |

---

## 九、参考资源

### 官方文档
- [Android BLE官方文档](https://developer.android.com/guide/topics/connectivity/bluetooth/ble-overview)
- [Kotlin协程](https://kotlinlang.org/docs/coroutines-overview.html)
- [Jetpack Compose](https://developer.android.com/jetpack/compose)

### 开源库
- [Android-BLE-Library](https://github.com/NordicSemiconductor/Android-BLE-Library)
- [compose-joystick](https://github.com/...) (虚拟摇杆组件)

### 现有项目参考
- 当前ESP32C3遥控器代码
- `gatt_client/main/gattc_demo.c`
- `protocol/owl_protocol.h`

---

## 十、附录

### A. 猫头鹰MAC地址
```
MAC: 48:f6:ee:55:00:e8
```

### B. 通信日志示例
```
[发送] JOYSTICK: 11 2a 8f 97 96 00
[发送] HEARTBEAT: 13 2a 83 00 25
[接收] FEEDBACK: 15 2a 7b 01 50
```

### C. 版本规划
| 版本 | 功能 | 时间 |
|------|------|------|
| v1.0 | 基础遥控功能 | 2周 |
| v1.1 | 增强功能（振动、音效） | 1周 |
| v1.2 | 编程模式 | 1周 |
| v2.0 | 摄像头追踪 | 2周 |

---

**文档版本**: v1.0  
**创建日期**: 2025-05-23  
**作者**: AI Assistant
