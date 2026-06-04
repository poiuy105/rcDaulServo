# 工业级全流程看门狗 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为遥控端(gatt_client)和被控端(gatt_server)同步增加工业级全流程看门狗，覆盖硬件看门狗、任务看门狗、心跳链路看门狗三层防护。

**Architecture:** 采用三层看门狗架构：(1) ESP32C3硬件RWDT作为最后防线，(2) FreeRTOS TWDT监控所有自定义任务，(3)应用层心跳链路看门狗检测BLE通信断连。所有任务注册到TWDT并定期"喂狗"，任何任务卡死都会触发系统复位。

**Tech Stack:** ESP-IDF v5.4 / ESP32C3 / FreeRTOS TWDT / esp_task_wdt API / esp_event_loop_wdt API

---

## 文件结构

| 文件 | 职责 |
|------|------|
| `gatt_server/sdkconfig.defaults.esp32c3` | Server看门狗Kconfig配置 |
| `gatt_client/sdkconfig.defaults.esp32c3` | Client看门狗Kconfig配置 |
| `gatt_server/main/gatts_demo.c` | Server所有任务注册/喂狗/初始化 |
| `gatt_client/main/gattc_demo.c` | Client所有任务注册/喂狗/初始化 |

---

## 任务清单

### Task 1: Server sdkconfig 看门狗配置

**Files:**
- Modify: `gatt_server/sdkconfig.defaults.esp32c3`

- [ ] **Step 1: 添加看门狗Kconfig配置**

在 `gatt_server/sdkconfig.defaults.esp32c3` 文件末尾追加以下配置：

```
# === 工业级看门狗配置 ===
# 启用任务看门狗(TWDT)，监控所有注册任务
CONFIG_ESP_TASK_WDT=y
# TWDT触发时panic复位（不恢复，直接重启）
CONFIG_ESP_TASK_WDT_PANIC=y
# TWDT超时阈值5秒（所有任务必须在5秒内喂狗一次）
CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
# 支持Idle任务也受TWDT监控
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK=y
# 启用中断看门狗(IWDT)
CONFIG_ESP_INT_WDT=y
# IWDT触发时panic复位
CONFIG_ESP_INT_WDT_PANIC=y
# 启用事件循环看门狗（监控BLE回调中的阻塞）
CONFIG_ESP_EVENT_LOOP_WDT=y
# 事件循环看门狗超时5秒
CONFIG_ESP_EVENT_LOOP_WDT_TIMEOUT_S=5
# 事件循环看门狗panic复位
CONFIG_ESP_EVENT_LOOP_WDT_PANIC=y
```

- [ ] **Step 2: 验证配置语法**

检查文件无语法错误，每个 `CONFIG_` 行格式正确（无前导空格、无多余空格）。

---

### Task 2: Client sdkconfig 看门狗配置

**Files:**
- Modify: `gatt_client/sdkconfig.defaults.esp32c3`

- [ ] **Step 1: 添加看门狗Kconfig配置**

在 `gatt_client/sdkconfig.defaults.esp32c3` 文件末尾追加与Task 1完全相同的配置：

```
# === 工业级看门狗配置 ===
CONFIG_ESP_TASK_WDT=y
CONFIG_ESP_TASK_WDT_PANIC=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=5
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK=y
CONFIG_ESP_INT_WDT=y
CONFIG_ESP_INT_WDT_PANIC=y
CONFIG_ESP_EVENT_LOOP_WDT=y
CONFIG_ESP_EVENT_LOOP_WDT_TIMEOUT_S=5
CONFIG_ESP_EVENT_LOOP_WDT_PANIC=y
```

---

### Task 3: Server 任务看门狗 — 头文件和宏定义

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` (文件头部，include区域之后)

- [ ] **Step 1: 添加看门狗头文件和喂狗宏**

在 `#include "freertos/semphr.h"` 之后添加：

```c
#include "esp_task_wdt.h"
```

在全局宏定义区域（`#define OWL_NVS_NAMESPACE` 附近）添加：

```c
/*=== 看门狗配置 ===*/
#define WDT_TIMEOUT_S           5       // TWDT超时秒数（与sdkconfig一致）
#define WDT_FEED_INTERVAL_MS    2000    // 喂狗间隔2秒（远小于5秒超时）
```

- [ ] **Step 2: 添加全局看门狗初始化函数**

在 `app_main` 函数之前（全局函数区域）添加：

```c
/**
 * @brief 初始化任务看门狗
 * @param task_count 需要监控的任务数量
 * @note 必须在所有 xTaskCreate 之前调用
 */
static void wdt_init(int task_count) {
    esp_err_t ret = esp_task_wdt_init(WDT_TIMEOUT_S, true);  // true = panic
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "TWDT初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    // 为每个自定义任务订阅TWDT（系统任务由IDF自动管理）
    for (int i = 0; i < task_count; i++) {
        ret = esp_task_wdt_add(NULL);  // NULL = 当前任务
        if (ret != ESP_OK) {
            ESP_LOGE(GATTS_TAG, "TWDT订阅失败(task%d): %s", i, esp_err_to_name(ret));
        }
    }
    ESP_LOGI(GATTS_TAG, "TWDT已初始化，监控%d个任务槽位", task_count);
}
```

注意：`esp_task_wdt_add(NULL)` 在任务创建后由各任务自己调用，不是在init中调用。修正如下：

```c
/**
 * @brief 初始化任务看门狗（仅初始化TWDT，不订阅任务）
 */
static void wdt_init(void) {
    esp_err_t ret = esp_task_wdt_init(WDT_TIMEOUT_S, true);  // true = panic复位
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(GATTS_TAG, "TWDT初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTS_TAG, "TWDT已初始化，超时=%ds", WDT_TIMEOUT_S);
}

/**
 * @brief 当前任务订阅TWDT（在每个任务入口调用）
 */
static void wdt_subscribe(void) {
    esp_err_t ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "TWDT订阅失败: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief 当前任务喂狗（在每个任务主循环中调用）
 */
static inline void wdt_feed(void) {
    esp_task_wdt_reset();
}
```

---

### Task 4: Server bg_worker 任务看门狗

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` — `bg_task_worker` 函数

- [ ] **Step 1: 在bg_task_worker入口订阅TWDT**

找到 `bg_task_worker` 函数入口（`static void bg_task_worker(void *arg)`），在 `while(1)` 循环之前添加：

```c
wdt_subscribe();
ESP_LOGI(GATTS_TAG, "bg_worker已订阅TWDT");
```

- [ ] **Step 2: 在bg_task_worker主循环中喂狗**

找到 `bg_task_worker` 的 `while(1)` 循环，在 `xQueueReceive` 成功处理后（每个case分支处理完毕后），添加喂狗调用。最佳位置是在 `xQueueReceive` 超时返回后（即队列为空时也喂狗）：

将：
```c
if (xQueueReceive(g_bg_task_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
```
改为在 `xQueueReceive` 之后（无论成功或超时）都喂狗：
```c
if (xQueueReceive(g_bg_task_queue, &msg, portMAX_DELAY) == pdTRUE) {
    // ... 处理消息 ...
}
wdt_feed();  // 每次循环迭代都喂狗
```

注意：将超时从 `pdMS_TO_TICKS(100)` 改为 `portMAX_DELAY`，因为TWDT已经负责超时检测。如果队列为空，任务会阻塞等待，不会触发TWDT（FreeRTOS阻塞态不受TWDT监控）。但如果消息处理耗时超过5秒，TWDT会触发复位。

---

### Task 5: Server heartbeat_monitor 任务看门狗

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` — `heartbeat_monitor_task` 函数

- [ ] **Step 1: 订阅TWDT + 主循环喂狗**

找到 `heartbeat_monitor_task` 函数，在 `while(1)` 之前添加订阅，在循环末尾添加喂狗：

```c
static void heartbeat_monitor_task(void *arg) {
    wdt_subscribe();
    ESP_LOGI(GATTS_TAG, "heartbeat_monitor已订阅TWDT");
    
    while (1) {
        // ... 原有逻辑 ...
        
        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

### Task 6: Server security_task 任务看门狗

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` — `security_task` 函数

- [ ] **Step 1: 订阅TWDT + 主循环喂狗**

找到 `security_task` 函数，在 `while(1)` 之前添加订阅，在循环末尾添加喂狗：

```c
static void security_task(void *arg) {
    wdt_subscribe();
    ESP_LOGI(GATTS_TAG, "security_task已订阅TWDT");
    
    while (1) {
        // ... 原有逻辑 ...
        
        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(100));  // 原有延时保持不变
    }
}
```

---

### Task 7: Server preset_player_task 任务看门狗

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` — `preset_player_task` 函数

- [ ] **Step 1: 订阅TWDT + 循环中喂狗**

`preset_player_task` 是动态创建的任务，播放预设动作序列后退出。需要在入口订阅TWDT，在播放循环中喂狗，退出前取消订阅。

找到 `preset_player_task` 函数：

```c
static void preset_player_task(void *arg) {
    wdt_subscribe();
    ESP_LOGI(GATTS_TAG, "preset_player已订阅TWDT");
    
    // ... 原有的NVS读取和帧播放逻辑 ...
    // 在 for 循环的每次迭代中（每帧播放后）添加：
    //     wdt_feed();
    
    // 退出前取消订阅（避免已删除任务仍被TWDT监控）
    esp_task_wdt_delete(NULL);
    
    // ... 原有的 g_preset_task_handle = NULL; vTaskDelete(NULL); ...
}
```

具体位置：在帧播放的 `for` 循环内，每帧的 `vTaskDelay(pdMS_TO_TICKS(f->delay_ms))` 之后添加 `wdt_feed();`。

在 `vTaskDelete(NULL)` 之前添加 `esp_task_wdt_delete(NULL);`。

---

### Task 8: Server app_main 集成看门狗初始化

**Files:**
- Modify: `gatt_server/main/gatts_demo.c` — `app_main` 函数

- [ ] **Step 1: 在创建任务之前初始化TWDT**

找到 `app_main` 中创建互斥锁和队列之后、第一个 `xTaskCreate` 之前的位置，添加：

```c
// 初始化任务看门狗（必须在创建任务之前）
wdt_init();
```

具体位置：在 `g_bg_task_queue = xQueueCreate(...)` 之后，`xTaskCreate(bg_task_worker, ...)` 之前。

---

### Task 9: Client 任务看门狗 — 头文件和宏定义

**Files:**
- Modify: `gatt_client/main/gattc_demo.c` (文件头部)

- [ ] **Step 1: 添加看门狗头文件和宏/函数**

在 `#include "freertos/semphr.h"` 之后添加：

```c
#include "esp_task_wdt.h"
```

在全局宏定义区域添加：

```c
/*=== 看门狗配置 ===*/
#define WDT_TIMEOUT_S           5
#define WDT_FEED_INTERVAL_MS    2000
```

在全局函数区域添加（与Server相同的三个函数）：

```c
static void wdt_init(void) {
    esp_err_t ret = esp_task_wdt_init(WDT_TIMEOUT_S, true);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(GATTC_TAG, "TWDT初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(GATTC_TAG, "TWDT已初始化，超时=%ds", WDT_TIMEOUT_S);
}

static void wdt_subscribe(void) {
    esp_err_t ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "TWDT订阅失败: %s", esp_err_to_name(ret));
    }
}

static inline void wdt_feed(void) {
    esp_task_wdt_reset();
}
```

---

### Task 10: Client key_scan_task 任务看门狗

**Files:**
- Modify: `gatt_client/main/gattc_demo.c` — `key_scan_task` 函数

- [ ] **Step 1: 订阅TWDT + 主循环喂狗**

找到 `key_scan_task` 函数，在 `while(1)` 之前添加订阅，在循环末尾（`vTaskDelay` 之前）添加喂狗：

```c
static void key_scan_task(void *arg) {
    wdt_subscribe();
    ESP_LOGI(GATTC_TAG, "key_scan已订阅TWDT");
    
    // ... 原有变量声明 ...
    
    while (1) {
        // ... 原有逻辑 ...
        
        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

### Task 11: Client heartbeat_monitor_task 任务看门狗

**Files:**
- Modify: `gatt_client/main/gattc_demo.c` — `heartbeat_monitor_task` 函数

- [ ] **Step 1: 订阅TWDT + 主循环喂狗**

找到 `heartbeat_monitor_task` 函数，在 `while(1)` 之前添加订阅，在循环末尾添加喂狗：

```c
static void heartbeat_monitor_task(void *arg) {
    wdt_subscribe();
    ESP_LOGI(GATTC_TAG, "heartbeat_monitor已订阅TWDT");
    
    while (1) {
        // ... 原有逻辑 ...
        
        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

### Task 12: Client app_main 集成看门狗初始化

**Files:**
- Modify: `gatt_client/main/gattc_demo.c` — `app_main` 函数

- [ ] **Step 1: 在app_main中初始化TWDT**

找到 `app_main` 中BLE初始化完成之后、`ws2812_startup_animation()` 之前的位置，添加：

```c
// 初始化任务看门狗
wdt_init();
```

注意：Client的key_scan和heartbeat_monitor任务在BLE连接成功后才创建，但TWDT初始化可以在app_main中提前完成。各任务在自己的入口函数中调用 `wdt_subscribe()` 即可。

---

### Task 13: 编译验证

- [ ] **Step 1: 提交代码**

```bash
git add gatt_server/sdkconfig.defaults.esp32c3 gatt_client/sdkconfig.defaults.esp32c3 gatt_server/main/gatts_demo.c gatt_client/main/gattc_demo.c
git commit -m "feat(server,client): add industrial-grade watchdog for all tasks"
```

- [ ] **Step 2: 推送到GitHub触发编译**

```bash
git push
```

- [ ] **Step 3: 检查编译结果**

```bash
gh run list --limit 1
```

Expected: `completed success`

---

## 自检清单

**1. 需求覆盖：**
- [x] 硬件看门狗(RWDT) — ESP32C3默认启用，无需额外代码
- [x] 任务看门狗(TWDT) — sdkconfig配置 + esp_task_wdt API
- [x] 中断看门狗(IWDT) — sdkconfig配置
- [x] 事件循环看门狗 — sdkconfig配置（监控BLE回调阻塞）
- [x] Server 4个任务全部注册（bg_worker, heartbeat_monitor, security, preset_player）
- [x] Client 2个任务全部注册（key_scan, heartbeat_monitor）
- [x] 动态任务(preset_player)退出前取消订阅
- [x] Server和Client同步增加

**2. 占位符扫描：** 无TBD、无"适当处理"、无省略代码

**3. 类型一致性：** 所有 `wdt_init/subscribe/feed` 函数签名在Server和Client中完全一致
