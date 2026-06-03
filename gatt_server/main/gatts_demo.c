/*
 * 猫头鹰玩具 BLE GATT Server (猫头鹰端)
 * 基于 ESP-IDF 官方示例修改
 * 
 * 功能：
 * - 接收遥控器的控制指令（摇杆、开关、心跳）
 * - 发送状态反馈
 * - 验证 BLE 通讯协议
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "owl_protocol.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "radar.h"

#define GATTS_TAG "OWL_SERVER"

// 心跳超时配置
#define HEARTBEAT_TIMEOUT_MS    5000    // 5秒无心跳认为断线

// 心跳检测变量
static int64_t last_heartbeat_time = 0;
static esp_gatt_if_t gatts_if_global = 0;
static uint16_t conn_id_global = 0;

// LD6004 Radar State
static radar_handle_t radar_handle = NULL;
static bool radar_enabled = false;
static uint8_t radar_notify_seq = 0;

/*============================================================================
 *                              工作模式管理
 *============================================================================*/
static owl_mode_t g_current_mode = OWL_MODE_REMOTE;
static uint8_t feedback_seq = 0;

// 安防模式状态
static owl_security_state_t g_sec_state = SEC_STATE_SCANNING;
static int64_t g_sec_state_enter_time = 0;
static uint8_t g_sec_scan_angle = 45;
static int8_t g_sec_scan_dir = 1;  // 1=正向, -1=反向
static bool g_sec_alarm_on = false;
static int64_t g_sec_last_alarm_time = 0;

// 预设模式状态
static TaskHandle_t g_preset_task_handle = NULL;
static volatile bool g_preset_running = false;
static uint8_t g_preset_slot = 0;

// 录制模式状态
static preset_frame_t *g_record_buffer = NULL;
static uint16_t g_record_count = 0;
static uint16_t g_record_capacity = 0;
static uint8_t g_record_slot = 0;
static int64_t g_record_start_time = 0;
static int64_t g_record_last_frame_time = 0;
static volatile bool g_recording = false;

/*============================================================================
 *                              舵机 PWM 配置
 *============================================================================*/

// GPIO 定义
#define SERVO_X_GPIO        3
#define SERVO_Y_GPIO        4

// LEDC 配置
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_X_CHANNEL      LEDC_CHANNEL_0
#define LEDC_Y_CHANNEL      LEDC_CHANNEL_1
#define LEDC_DUTY_RES       LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY      50  // 50Hz = 20ms 周期

// 舵机角度映射 (MG996R: 0.5ms-2.5ms 脉宽)
// 12bit分辨率: 0-4095
// 50Hz时, 20ms = 4095 ticks
// 0.5ms = 102 ticks (2.5%)
// 1.5ms = 307 ticks (7.5%)
// 2.5ms = 512 ticks (12.5%)
#define SERVO_MIN_DUTY      102   // 0°
#define SERVO_MAX_DUTY      512   // 180°
#define SERVO_MID_DUTY      307   // 90°

// 当前舵机角度
static uint8_t servo_x_angle = 90;
static uint8_t servo_y_angle = 90;

/*============================================================================
 *                              继电器 GPIO 配置
 *============================================================================*/

// 继电器 GPIO 定义
#define RELAY_LIGHT_GPIO    10   // 灯光（眼睛）
#define RELAY_SOUND_GPIO    7    // 声音（躯干）
#define RELAY_CANNON_GPIO   8    // 开炮（嘴巴）

// 继电器状态
static bool relay_light_on = false;
static bool relay_sound_on = false;
static bool relay_cannon_on = false;

// 初始化继电器 GPIO
static void relay_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_LIGHT_GPIO) | (1ULL << RELAY_SOUND_GPIO) | (1ULL << RELAY_CANNON_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // 初始状态：全部关闭（低电平）
    gpio_set_level(RELAY_LIGHT_GPIO, 0);
    gpio_set_level(RELAY_SOUND_GPIO, 0);
    gpio_set_level(RELAY_CANNON_GPIO, 0);
    
    ESP_LOGI(GATTS_TAG, "继电器 GPIO 初始化完成");
    ESP_LOGI(GATTS_TAG, "  灯光: GPIO%d", RELAY_LIGHT_GPIO);
    ESP_LOGI(GATTS_TAG, "  声音: GPIO%d", RELAY_SOUND_GPIO);
    ESP_LOGI(GATTS_TAG, "  开炮: GPIO%d", RELAY_CANNON_GPIO);
}

// 设置继电器状态
static void relay_set(uint32_t gpio_num, bool on) {
    gpio_set_level(gpio_num, on ? 1 : 0);
}

/*============================================================================
 *                              GATT 定义
 *============================================================================*/

#define PROFILE_NUM             1
#define PROFILE_APP_ID          0
#define GATTS_NUM_HANDLE        10    // 1 service + 3 chars + 3 descrs

// 服务和特征值 UUID
#define OWL_SERVICE_UUID        0xFF00
#define OWL_CHAR_CONTROL_UUID   0xFF01
#define OWL_CHAR_FEEDBACK_UUID  0xFF02
#define OWL_CHAR_COMMAND_UUID   0xFF03

// 设备名称
static char owl_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = OWL_DEVICE_NAME;

// MTU
static uint16_t local_mtu = 23;

// 序列号
static uint8_t last_seq = 0;

// 心跳检测任务
static void heartbeat_monitor_task(void *arg) {
    while (1) {
        if (last_heartbeat_time > 0) {
            int64_t now = esp_timer_get_time() / 1000;
            
            // 检查心跳超时
            if ((now - last_heartbeat_time) > HEARTBEAT_TIMEOUT_MS) {
                ESP_LOGW(GATTS_TAG, "⚠️ 心跳超时！客户端可能已断线");
                // 主动断开连接，让客户端重新连接
                if (conn_id_global > 0) {
                    esp_ble_gatts_close(gatts_if_global, conn_id_global);
                }
                last_heartbeat_time = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*============================================================================
 *                              MAC 绑定功能
 *============================================================================*/

// 绑定状态
static bool g_is_bound = false;
static uint8_t g_bound_mac[6] = {0};

// 加载绑定的 MAC 地址
static void load_bound_mac(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t length = 6;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(GATTS_TAG, "NVS 未初始化，无绑定信息");
        g_is_bound = false;
        return;
    }
    
    err = nvs_get_blob(nvs_handle, OWL_KEY_REMOTE_MAC, g_bound_mac, &length);
    if (err == ESP_OK && length == 6) {
        g_is_bound = true;
        ESP_LOGI(GATTS_TAG, "已绑定设备 MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(g_bound_mac));
    } else {
        g_is_bound = false;
        ESP_LOGI(GATTS_TAG, "无绑定设备");
    }
    
    nvs_close(nvs_handle);
}

// 保存绑定的 MAC 地址
static void save_bound_mac(uint8_t *mac) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "NVS 打开失败");
        return;
    }
    
    err = nvs_set_blob(nvs_handle, OWL_KEY_REMOTE_MAC, mac, 6);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            memcpy(g_bound_mac, mac, 6);
            g_is_bound = true;
            ESP_LOGI(GATTS_TAG, "绑定设备 MAC 已保存: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(mac));
        }
    }
    
    nvs_close(nvs_handle);
}

// 清除绑定
static void clear_bound_mac(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return;
    
    err = nvs_erase_key(nvs_handle, OWL_KEY_REMOTE_MAC);
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
    }
    
    nvs_close(nvs_handle);
    g_is_bound = false;
    memset(g_bound_mac, 0, 6);
    ESP_LOGI(GATTS_TAG, "绑定信息已清除");
}

// 验证 MAC 地址
static bool verify_mac(uint8_t *mac) {
    if (!g_is_bound) {
        return true;  // 未绑定状态，允许任何设备连接
    }
    return (memcmp(mac, g_bound_mac, 6) == 0);
}

/*============================================================================
 *                              广播数据
 *============================================================================*/

static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,  // UUID: 0xFF00
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = ESP_BLE_GAP_CONN_ITVL_MS(20),
    .max_interval = ESP_BLE_GAP_CONN_ITVL_MS(30),
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .adv_int_max = ESP_BLE_GAP_ADV_ITVL_MS(40),
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

/*============================================================================
 *                              Profile 结构
 *============================================================================*/

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    
    // 三个特征值的句柄
    uint16_t char_control_handle;   // 控制通道 (Write)
    uint16_t char_feedback_handle;  // 反馈通道 (Notify)
    uint16_t char_command_handle;   // 系统命令 (Write + Indicate)
    
    uint16_t descr_control_handle;
    uint16_t descr_feedback_handle;
    uint16_t descr_command_handle;
};

static struct gatts_profile_inst gl_profile = {
    .gatts_cb = NULL,
    .gatts_if = ESP_GATT_IF_NONE,
    .app_id = PROFILE_APP_ID,
};

/*============================================================================
 *                              协议解析函数
 *============================================================================*/

// 初始化舵机 PWM
static void servo_init(void) {
    // 配置定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置X轴舵机通道
    ledc_channel_config_t ledc_x_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_X_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_X_GPIO,
        .duty           = SERVO_MID_DUTY,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_x_channel));

    // 配置Y轴舵机通道
    ledc_channel_config_t ledc_y_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_Y_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_Y_GPIO,
        .duty           = SERVO_MID_DUTY,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_y_channel));

    ESP_LOGI(GATTS_TAG, "舵机 PWM 初始化完成");
    ESP_LOGI(GATTS_TAG, "  X轴: GPIO%d, 初始角度 90°", SERVO_X_GPIO);
    ESP_LOGI(GATTS_TAG, "  Y轴: GPIO%d, 初始角度 90°", SERVO_Y_GPIO);
}

// 设置舵机角度
static void servo_set_angle(uint8_t channel, uint8_t angle) {
    // 限制角度范围 0-180
    if (angle > 180) angle = 180;
    
    // 角度映射到duty
    uint32_t duty = SERVO_MIN_DUTY + (angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180);
    
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

/*============================================================================
 *                              LD6004 雷达功能
 *============================================================================*/

// 雷达坐标到舵机角度映射
static uint16_t radar_coord_to_angle(float coord, float gain) {
    int angle = 90 + (int)(coord * gain);
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    return (uint16_t)angle;
}

// BLE 上报雷达状态
static void send_radar_notify(uint8_t target_count, int16_t x, int16_t y, int16_t z) {
    if (gl_profile.conn_id == 0 || !radar_enabled) {
        return;
    }

    uint8_t pkt[11];
    pkt[0] = OWL_PROTOCOL_VERSION | 0x18;  // version_type: 雷达状态
    pkt[1] = radar_notify_seq++;
    pkt[2] = (uint8_t)(esp_log_timestamp() & 0xFF);
    pkt[3] = target_count;
    memcpy(&pkt[4], &x, 2);  // x (LE)
    memcpy(&pkt[6], &y, 2);  // y (LE)
    memcpy(&pkt[8], &z, 2);  // z (LE)

    esp_ble_gatts_send_indicate(gl_profile.gatts_if,
                                 gl_profile.conn_id,
                                 gl_profile.descr_feedback_handle,
                                 sizeof(pkt), pkt, false);
}

/*============================================================================
 *                              BLE 反馈发送辅助函数
 *============================================================================*/

// 发送事件包（通过反馈通道 Notify）
static void send_event_notify(uint8_t event_code, uint8_t p1, uint8_t p2, uint8_t p3) {
    if (gl_profile.conn_id == 0) return;
    
    owl_event_pkt_t pkt;
    pkt.header.version_type = OWL_MAKE_HEADER(OWL_PKT_EVENT);
    pkt.header.seq = feedback_seq++;
    pkt.header.timestamp = (uint8_t)(esp_log_timestamp() & 0xFF);
    pkt.event_type = event_code;
    pkt.event_param = p1;
    
    // 使用原始Notify发送7字节（header 3字节 + event_type + event_param + 2 extra bytes）
    uint8_t buf[7];
    memcpy(buf, &pkt, sizeof(owl_event_pkt_t));
    buf[5] = p2;
    buf[6] = p3;
    
    esp_ble_gatts_send_indicate(gl_profile.gatts_if,
                                 gl_profile.conn_id,
                                 gl_profile.descr_feedback_handle,
                                 sizeof(buf), buf, false);
}

// 发送ACK包
static void send_ack(uint8_t cmd, uint8_t result) {
    if (gl_profile.conn_id == 0) return;
    
    owl_ack_pkt_t pkt;
    pkt.header.version_type = OWL_MAKE_HEADER(OWL_PKT_ACK);
    pkt.header.seq = feedback_seq++;
    pkt.header.timestamp = (uint8_t)(esp_log_timestamp() & 0xFF);
    pkt.cmd = cmd;
    pkt.result = result;
    pkt.data = 0;
    
    esp_ble_gatts_send_indicate(gl_profile.gatts_if,
                                 gl_profile.conn_id,
                                 gl_profile.descr_feedback_handle,
                                 sizeof(pkt), (uint8_t*)&pkt, false);
}

/*============================================================================
 *                              安防模式
 *============================================================================*/

static void security_enter(void) {
    g_sec_state = SEC_STATE_SCANNING;
    g_sec_state_enter_time = esp_timer_get_time() / 1000;
    g_sec_scan_angle = 45;
    g_sec_scan_dir = 1;
    g_sec_alarm_on = false;
    g_sec_last_alarm_time = 0;
    ESP_LOGI(GATTS_TAG, "[安防模式] 进入，开始巡逻扫描");
}

static void security_exit(void) {
    g_sec_alarm_on = false;
    relay_set(RELAY_LIGHT_GPIO, false);
    relay_set(RELAY_SOUND_GPIO, false);
    relay_set(RELAY_CANNON_GPIO, false);
    relay_light_on = false;
    relay_sound_on = false;
    relay_cannon_on = false;
    // 舵机回中
    servo_set_angle(LEDC_X_CHANNEL, 90);
    servo_set_angle(LEDC_Y_CHANNEL, 90);
    servo_x_angle = 90;
    servo_y_angle = 90;
    ESP_LOGI(GATTS_TAG, "[安防模式] 退出，舵机回中");
}

// 安防模式定时更新（100ms调用一次）
static void security_update(void) {
    int64_t now = esp_timer_get_time() / 1000;
    
    switch (g_sec_state) {
    case SEC_STATE_SCANNING:
        // 巡逻扫描：45°-135°，10°/秒
        g_sec_scan_angle += g_sec_scan_dir;
        if (g_sec_scan_angle >= 135) { g_sec_scan_angle = 135; g_sec_scan_dir = -1; }
        if (g_sec_scan_angle <= 45)  { g_sec_scan_angle = 45;  g_sec_scan_dir = 1; }
        servo_set_angle(LEDC_X_CHANNEL, (uint8_t)g_sec_scan_angle);
        servo_set_angle(LEDC_Y_CHANNEL, 90);
        servo_x_angle = (uint8_t)g_sec_scan_angle;
        servo_y_angle = 90;
        break;
        
    case SEC_STATE_DETECTED:
        // 报警：灯光1Hz闪烁，声音间歇响，开炮100ms脉冲
        if (!g_sec_alarm_on) {
            g_sec_alarm_on = true;
            g_sec_last_alarm_time = now;
            // 开炮威慑：100ms脉冲
            relay_set(RELAY_CANNON_GPIO, true);
            vTaskDelay(pdMS_TO_TICKS(100));
            relay_set(RELAY_CANNON_GPIO, false);
        }
        // 灯光闪烁（1Hz）
        relay_set(RELAY_LIGHT_GPIO, (now % 1000 < 500) ? true : false);
        relay_light_on = (now % 1000 < 500);
        // 声音间歇（3秒周期：响1秒，停2秒）
        relay_set(RELAY_SOUND_GPIO, (now % 3000 < 1000) ? true : false);
        relay_sound_on = (now % 3000 < 1000);
        break;
        
    case SEC_STATE_TRACKING:
        // 持续跟踪，灯光闪烁
        relay_set(RELAY_LIGHT_GPIO, (now % 1000 < 500) ? true : false);
        relay_light_on = (now % 1000 < 500);
        relay_set(RELAY_SOUND_GPIO, (now % 3000 < 1000) ? true : false);
        relay_sound_on = (now % 3000 < 1000);
        break;
        
    case SEC_STATE_LOST:
        // 保持最后方向3秒
        relay_set(RELAY_LIGHT_GPIO, false);
        relay_set(RELAY_SOUND_GPIO, false);
        relay_light_on = false;
        relay_sound_on = false;
        if ((now - g_sec_state_enter_time) > 3000) {
            g_sec_state = SEC_STATE_SCANNING;
            g_sec_state_enter_time = now;
            ESP_LOGI(GATTS_TAG, "[安防] 目标丢失超时，回到巡逻");
        }
        break;
    }
}

// 安防模式雷达事件处理
static void security_radar_handler(radar_data_t *data) {
    if (data->target_count > 0) {
        ld6004_target_t *target = &data->targets[0];
        uint16_t angle_x = radar_coord_to_angle(target->x, 45.0f);
        uint16_t angle_y = radar_coord_to_angle(target->y, 45.0f);
        
        servo_set_angle(LEDC_X_CHANNEL, (uint8_t)angle_x);
        servo_set_angle(LEDC_Y_CHANNEL, (uint8_t)angle_y);
        servo_x_angle = (uint8_t)angle_x;
        servo_y_angle = (uint8_t)angle_y;
        
        if (g_sec_state == SEC_STATE_SCANNING || g_sec_state == SEC_STATE_LOST) {
            g_sec_state = SEC_STATE_DETECTED;
            g_sec_state_enter_time = esp_timer_get_time() / 1000;
            ESP_LOGI(GATTS_TAG, "[安防] 检测到入侵! x=%.2f y=%.2f", target->x, target->y);
            send_event_notify(OWL_EVENT_INTRUSION_DETECTED,
                            (uint8_t)(target->x * 100 + 128),
                            (uint8_t)(target->y * 100 + 128),
                            data->target_count);
        } else {
            g_sec_state = SEC_STATE_TRACKING;
        }
        
        // BLE上报位置
        send_radar_notify(data->target_count,
                         (int16_t)(target->x * 100),
                         (int16_t)(target->y * 100),
                         (int16_t)(target->z * 100));
    } else {
        if (g_sec_state == SEC_STATE_DETECTED || g_sec_state == SEC_STATE_TRACKING) {
            g_sec_state = SEC_STATE_LOST;
            g_sec_state_enter_time = esp_timer_get_time() / 1000;
            ESP_LOGI(GATTS_TAG, "[安防] 入侵者消失");
            send_event_notify(OWL_EVENT_INTRUSION_LOST, 0, 0, 0);
        }
        send_radar_notify(0, 0, 0, 0);
    }
}

/*============================================================================
 *                              预设模式
 *============================================================================*/

static void preset_player_task(void *arg) {
    ESP_LOGI(GATTS_TAG, "[预设] 开始执行预设槽位 %d", g_preset_slot);
    
    // 从NVS读取预设数据
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", g_preset_slot);
    
    if (nvs_open(OWL_PRESETS_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "[预设] NVS打开失败");
        g_preset_running = false;
        g_preset_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    // 先获取数据长度
    size_t blob_len = 0;
    esp_err_t err = nvs_get_blob(nvs, key, NULL, &blob_len);
    if (err != ESP_OK || blob_len == 0 || blob_len % OWL_PRESET_FRAME_SIZE != 0) {
        ESP_LOGE(GATTS_TAG, "[预设] 槽位 %d 无数据或格式错误 (len=%d)", g_preset_slot, blob_len);
        nvs_close(nvs);
        g_preset_running = false;
        g_preset_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    uint16_t frame_count = blob_len / OWL_PRESET_FRAME_SIZE;
    preset_frame_t *frames = malloc(blob_len);
    if (!frames) {
        ESP_LOGE(GATTS_TAG, "[预设] 内存分配失败");
        nvs_close(nvs);
        g_preset_running = false;
        g_preset_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    nvs_get_blob(nvs, key, frames, &blob_len);
    nvs_close(nvs);
    
    ESP_LOGI(GATTS_TAG, "[预设] 读取到 %d 帧数据", frame_count);
    
    // 逐帧执行
    for (uint16_t i = 0; i < frame_count && g_preset_running; i++) {
        preset_frame_t *f = &frames[i];
        
        // 等待帧间隔
        if (f->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(f->delay_ms));
        }
        
        if (!g_preset_running) break;
        
        // 执行舵机动作
        if (f->servo_x != OWL_SERVO_NO_CHANGE) {
            servo_x_angle = f->servo_x;
            servo_set_angle(LEDC_X_CHANNEL, servo_x_angle);
        }
        if (f->servo_y != OWL_SERVO_NO_CHANGE) {
            servo_y_angle = f->servo_y;
            servo_set_angle(LEDC_Y_CHANNEL, servo_y_angle);
        }
        
        // 执行继电器动作
        relay_set(RELAY_LIGHT_GPIO, (f->relay_flags & OWL_RELAY_EYE_LIGHT) ? true : false);
        relay_set(RELAY_SOUND_GPIO, (f->relay_flags & OWL_RELAY_SOUND) ? true : false);
        relay_set(RELAY_CANNON_GPIO, (f->relay_flags & OWL_RELAY_CANNON) ? true : false);
        relay_light_on = (f->relay_flags & OWL_RELAY_EYE_LIGHT) != 0;
        relay_sound_on = (f->relay_flags & OWL_RELAY_SOUND) != 0;
        relay_cannon_on = (f->relay_flags & OWL_RELAY_CANNON) != 0;
    }
    
    free(frames);
    
    if (g_preset_running) {
        ESP_LOGI(GATTS_TAG, "[预设] 执行完成");
        send_event_notify(OWL_EVENT_PRESET_COMPLETED, g_preset_slot, 0, 0);
    }
    
    g_preset_running = false;
    g_preset_task_handle = NULL;
    vTaskDelete(NULL);
}

static void preset_enter(void) {
    g_preset_running = false;
    g_preset_task_handle = NULL;
    ESP_LOGI(GATTS_TAG, "[预设模式] 进入，等待执行命令");
}

static void preset_exit(void) {
    g_preset_running = false;
    // 等待任务结束
    if (g_preset_task_handle != NULL) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    g_preset_task_handle = NULL;
    // 继电器全部关闭
    relay_set(RELAY_LIGHT_GPIO, false);
    relay_set(RELAY_SOUND_GPIO, false);
    relay_set(RELAY_CANNON_GPIO, false);
    relay_light_on = false;
    relay_sound_on = false;
    relay_cannon_on = false;
    ESP_LOGI(GATTS_TAG, "[预设模式] 退出");
}

static void preset_start(uint8_t slot) {
    if (slot >= OWL_PRESET_SLOT_COUNT) {
        send_ack(OWL_CMD_PRESET_START, OWL_ERR_INVALID_CMD);
        return;
    }
    if (g_preset_running) {
        preset_stop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    g_preset_slot = slot;
    g_preset_running = true;
    xTaskCreate(preset_player_task, "preset_player", 4096, NULL, 5, &g_preset_task_handle);
    send_ack(OWL_CMD_PRESET_START, OWL_ERR_NONE);
}

static void preset_stop(void) {
    g_preset_running = false;
    send_ack(OWL_CMD_PRESET_STOP, OWL_ERR_NONE);
}

/*============================================================================
 *                              录制模式
 *============================================================================*/

static void record_enter(uint8_t slot) {
    if (slot >= OWL_PRESET_SLOT_COUNT) {
        send_ack(OWL_CMD_RECORD_START, OWL_ERR_INVALID_CMD);
        return;
    }
    g_record_slot = slot;
    g_record_capacity = OWL_PRESET_MAX_FRAMES;
    g_record_buffer = malloc(g_record_capacity * OWL_PRESET_FRAME_SIZE);
    if (!g_record_buffer) {
        ESP_LOGE(GATTS_TAG, "[录制] 内存分配失败");
        send_ack(OWL_CMD_RECORD_START, OWL_ERR_EXEC_FAILED);
        return;
    }
    g_record_count = 0;
    g_record_start_time = esp_timer_get_time() / 1000;
    g_record_last_frame_time = g_record_start_time;
    g_recording = true;
    ESP_LOGI(GATTS_TAG, "[录制] 开始录制到槽位 %d", slot);
    send_event_notify(OWL_EVENT_RECORD_STARTED, slot, 0, 0);
    send_ack(OWL_CMD_RECORD_START, OWL_ERR_NONE);
}

static void record_exit(void) {
    g_recording = false;
    if (g_record_buffer) {
        free(g_record_buffer);
        g_record_buffer = NULL;
    }
    g_record_count = 0;
    ESP_LOGI(GATTS_TAG, "[录制] 退出录制模式");
}

static void record_add_joystick_frame(owl_joystick_pkt_t *pkt) {
    if (!g_recording || !g_record_buffer) return;
    if (g_record_count >= g_record_capacity) {
        ESP_LOGW(GATTS_TAG, "[录制] 缓冲区满，自动停止");
        record_save();
        return;
    }
    // 检查录制时长
    int64_t now = esp_timer_get_time() / 1000;
    if ((now - g_record_start_time) > OWL_PRESET_RECORD_MAX_MS) {
        ESP_LOGW(GATTS_TAG, "[录制] 超时60秒，自动停止");
        record_save();
        return;
    }
    
    preset_frame_t *f = &g_record_buffer[g_record_count];
    f->delay_ms = (uint16_t)(now - g_record_last_frame_time);
    f->servo_x = joystick_to_angle(pkt->x_axis);
    f->servo_y = joystick_to_angle(pkt->y_axis);
    f->relay_flags = (relay_light_on ? OWL_RELAY_EYE_LIGHT : 0) |
                      (relay_sound_on ? OWL_RELAY_SOUND : 0) |
                      (relay_cannon_on ? OWL_RELAY_CANNON : 0);
    f->reserved[0] = 0;
    f->reserved[1] = 0;
    g_record_last_frame_time = now;
    g_record_count++;
}

static void record_add_switch_frame(owl_switch_pkt_t *pkt) {
    if (!g_recording || !g_record_buffer) return;
    if (g_record_count >= g_record_capacity) {
        ESP_LOGW(GATTS_TAG, "[录制] 缓冲区满，自动停止");
        record_save();
        return;
    }
    int64_t now = esp_timer_get_time() / 1000;
    if ((now - g_record_start_time) > OWL_PRESET_RECORD_MAX_MS) {
        ESP_LOGW(GATTS_TAG, "[录制] 超时60秒，自动停止");
        record_save();
        return;
    }
    
    preset_frame_t *f = &g_record_buffer[g_record_count];
    f->delay_ms = (uint16_t)(now - g_record_last_frame_time);
    f->servo_x = OWL_SERVO_NO_CHANGE;
    f->servo_y = OWL_SERVO_NO_CHANGE;
    // 记录目标继电器状态（不是变化，而是录制时刻的实际状态）
    f->relay_flags = (pkt->switch1 & OWL_SW_CENTER) ? OWL_RELAY_EYE_LIGHT : 0;
    if (pkt->switch1 & OWL_SW_UP) f->relay_flags |= OWL_RELAY_SOUND;
    if (pkt->switch1 & OWL_SW_DOWN) f->relay_flags |= OWL_RELAY_CANNON;
    f->reserved[0] = 0;
    f->reserved[1] = 0;
    g_record_last_frame_time = now;
    g_record_count++;
}

static void record_save(void) {
    if (!g_recording) return;
    g_recording = false;
    
    if (g_record_count == 0) {
        ESP_LOGW(GATTS_TAG, "[录制] 无数据，不保存");
        send_ack(OWL_CMD_RECORD_STOP, OWL_ERR_EXEC_FAILED);
        return;
    }
    
    nvs_handle_t nvs;
    if (nvs_open(OWL_PRESETS_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "[录制] NVS打开失败");
        send_ack(OWL_CMD_RECORD_STOP, OWL_ERR_EXEC_FAILED);
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", g_record_slot);
    
    esp_err_t err = nvs_set_blob(nvs, key, g_record_buffer, g_record_count * OWL_PRESET_FRAME_SIZE);
    if (err == ESP_OK) {
        nvs_commit(nvs);
        ESP_LOGI(GATTS_TAG, "[录制] 保存 %d 帧到槽位 %d", g_record_count, g_record_slot);
        send_event_notify(OWL_EVENT_RECORD_STOPPED, g_record_slot, 
                          (uint8_t)(g_record_count >> 8), (uint8_t)(g_record_count & 0xFF));
        send_ack(OWL_CMD_RECORD_STOP, OWL_ERR_NONE);
    } else {
        ESP_LOGE(GATTS_TAG, "[录制] NVS写入失败");
        send_ack(OWL_CMD_RECORD_STOP, OWL_ERR_EXEC_FAILED);
    }
    
    nvs_close(nvs);
    g_record_count = 0;
}

static void record_delete(uint8_t slot) {
    if (slot >= OWL_PRESET_SLOT_COUNT) {
        send_ack(OWL_CMD_RECORD_DELETE, OWL_ERR_INVALID_CMD);
        return;
    }
    nvs_handle_t nvs;
    if (nvs_open(OWL_PRESETS_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    
    char key[16];
    snprintf(key, sizeof(key), "preset_%d", slot);
    nvs_erase_key(nvs, key);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(GATTS_TAG, "[录制] 已删除预设槽位 %d", slot);
    send_ack(OWL_CMD_RECORD_DELETE, OWL_ERR_NONE);
}

/*============================================================================
 *                              模式切换核心
 *============================================================================*/

static const char *mode_name(owl_mode_t mode) {
    switch (mode) {
        case OWL_MODE_REMOTE:   return "遥控模式";
        case OWL_MODE_SECURITY: return "安防模式";
        case OWL_MODE_PRESET:   return "预设模式";
        case OWL_MODE_RECORD:   return "录制模式";
        default: return "未知";
    }
}

static void mode_switch(owl_mode_t new_mode) {
    if (new_mode == g_current_mode) return;
    
    ESP_LOGI(GATTS_TAG, "模式切换: %s → %s", mode_name(g_current_mode), mode_name(new_mode));
    
    // 退出当前模式
    switch (g_current_mode) {
        case OWL_MODE_SECURITY: security_exit(); break;
        case OWL_MODE_PRESET:   preset_exit();   break;
        case OWL_MODE_RECORD:   record_exit();   break;
        default: break;
    }
    
    g_current_mode = new_mode;
    
    // 进入新模式
    switch (new_mode) {
        case OWL_MODE_REMOTE:   ESP_LOGI(GATTS_TAG, "[遥控模式] 摇杆+按键控制"); break;
        case OWL_MODE_SECURITY: security_enter(); break;
        case OWL_MODE_PRESET:   preset_enter();   break;
        case OWL_MODE_RECORD:   /* record_enter需要slot参数，由command处理 */ break;
        default: break;
    }
    
    send_event_notify(OWL_EVENT_MODE_CHANGED, (uint8_t)new_mode, 0, 0);
}

// 安防模式定时更新任务
static void security_task(void *arg) {
    while (1) {
        if (g_current_mode == OWL_MODE_SECURITY) {
            security_update();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 雷达事件处理器
static void radar_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
    if (event_id != RADAR_EVENT_TARGET) {
        return;
    }

    radar_data_t *data = (radar_data_t *)event_data;

    // 仅在安防模式下处理雷达事件
    if (g_current_mode == OWL_MODE_SECURITY) {
        security_radar_handler(data);
    } else {
        // 其他模式下仅上报雷达数据，不驱动舵机
        if (data->target_count > 0) {
            ld6004_target_t *target = &data->targets[0];
            send_radar_notify(data->target_count,
                             (int16_t)(target->x * 100),
                             (int16_t)(target->y * 100),
                             (int16_t)(target->z * 100));
        } else {
            send_radar_notify(0, 0, 0, 0);
        }
    }
}

// JOYSTICK值(0-255)映射到角度(0-180)
static uint8_t joystick_to_angle(uint8_t joystick_val) {
    return (joystick_val * 180) / 255;
}

static void parse_joystick_packet(owl_joystick_pkt_t *pkt) {
    ESP_LOGI(GATTS_TAG, "=== JOYSTICK 包 ===");
    ESP_LOGI(GATTS_TAG, "  序列号: %d", pkt->header.seq);
    ESP_LOGI(GATTS_TAG, "  时间戳: %d", pkt->header.timestamp);
    ESP_LOGI(GATTS_TAG, "  X轴: %d (0-255, 中点=128)", pkt->x_axis);
    ESP_LOGI(GATTS_TAG, "  Y轴: %d (0-255, 中点=128)", pkt->y_axis);
    ESP_LOGI(GATTS_TAG, "  按键: %s", (pkt->button_flags & 0x01) ? "按下" : "松开");
    
    // 检测丢包
    int8_t seq_diff = pkt->header.seq - last_seq;
    if (seq_diff > 1) {
        ESP_LOGW(GATTS_TAG, "  ⚠️ 检测到丢包! 丢失 %d 个包", seq_diff - 1);
    }
    last_seq = pkt->header.seq;
    
    // 更新舵机角度
    uint8_t new_x_angle = joystick_to_angle(pkt->x_axis);
    uint8_t new_y_angle = joystick_to_angle(pkt->y_axis);
    
    if (new_x_angle != servo_x_angle) {
        servo_x_angle = new_x_angle;
        servo_set_angle(LEDC_X_CHANNEL, servo_x_angle);
        ESP_LOGI(GATTS_TAG, "  → X舵机更新: %d°", servo_x_angle);
    }
    
    if (new_y_angle != servo_y_angle) {
        servo_y_angle = new_y_angle;
        servo_set_angle(LEDC_Y_CHANNEL, servo_y_angle);
        ESP_LOGI(GATTS_TAG, "  → Y舵机更新: %d°", servo_y_angle);
    }
}

static void parse_switch_packet(owl_switch_pkt_t *pkt) {
    ESP_LOGI(GATTS_TAG, "=== SWITCH 包 ===");
    ESP_LOGI(GATTS_TAG, "  序列号: %d", pkt->header.seq);
    ESP_LOGI(GATTS_TAG, "  开关组1: 0x%02X", pkt->switch1);
    ESP_LOGI(GATTS_TAG, "    上:%d 下:%d 左:%d 右:%d 中:%d",
             (pkt->switch1 & OWL_SW_UP) ? 1 : 0,
             (pkt->switch1 & OWL_SW_DOWN) ? 1 : 0,
             (pkt->switch1 & OWL_SW_LEFT) ? 1 : 0,
             (pkt->switch1 & OWL_SW_RIGHT) ? 1 : 0,
             (pkt->switch1 & OWL_SW_CENTER) ? 1 : 0);
    ESP_LOGI(GATTS_TAG, "  开关组2: 0x%02X", pkt->switch2);
    ESP_LOGI(GATTS_TAG, "    上:%d 下:%d 左:%d 右:%d 中:%d",
             (pkt->switch2 & OWL_SW_UP) ? 1 : 0,
             (pkt->switch2 & OWL_SW_DOWN) ? 1 : 0,
             (pkt->switch2 & OWL_SW_LEFT) ? 1 : 0,
             (pkt->switch2 & OWL_SW_RIGHT) ? 1 : 0,
             (pkt->switch2 & OWL_SW_CENTER) ? 1 : 0);
    
    // 继电器控制映射
    // 开关1-中 (0x10) -> 灯光 (IO10)
    // 开关1-上 (0x01) -> 声音 (IO7)
    // 开关1-下 (0x02) -> 开炮 (IO8)
    
    bool light_on = (pkt->switch1 & OWL_SW_CENTER) != 0;
    bool sound_on = (pkt->switch1 & OWL_SW_UP) != 0;
    bool cannon_on = (pkt->switch1 & OWL_SW_DOWN) != 0;
    
    // 更新灯光继电器
    if (light_on != relay_light_on) {
        relay_light_on = light_on;
        relay_set(RELAY_LIGHT_GPIO, relay_light_on);
        ESP_LOGI(GATTS_TAG, "  → 灯光继电器: %s", relay_light_on ? "开" : "关");
    }
    
    // 更新声音继电器
    if (sound_on != relay_sound_on) {
        relay_sound_on = sound_on;
        relay_set(RELAY_SOUND_GPIO, relay_sound_on);
        ESP_LOGI(GATTS_TAG, "  → 声音继电器: %s", relay_sound_on ? "开" : "关");
    }
    
    // 更新开炮继电器
    if (cannon_on != relay_cannon_on) {
        relay_cannon_on = cannon_on;
        relay_set(RELAY_CANNON_GPIO, relay_cannon_on);
        ESP_LOGI(GATTS_TAG, "  → 开炮继电器: %s", relay_cannon_on ? "开" : "关");
    }
}

static void parse_heartbeat_packet(owl_heartbeat_pkt_t *pkt) {
    ESP_LOGI(GATTS_TAG, "=== HEARTBEAT 包 ===");
    ESP_LOGI(GATTS_TAG, "  序列号: %d", pkt->header.seq);
    ESP_LOGI(GATTS_TAG, "  状态: 0x%02X", pkt->status);
    ESP_LOGI(GATTS_TAG, "  电池: %d.%dV", pkt->battery / 10, pkt->battery % 10);
    
    // 更新心跳时间
    last_heartbeat_time = esp_timer_get_time() / 1000;
}

static void parse_command_packet(owl_command_pkt_t *pkt) {
    ESP_LOGI(GATTS_TAG, "=== COMMAND 包 ===");
    ESP_LOGI(GATTS_TAG, "  序列号: %d", pkt->header.seq);
    ESP_LOGI(GATTS_TAG, "  命令码: 0x%02X", pkt->cmd);
    ESP_LOGI(GATTS_TAG, "  参数: 0x%02X", pkt->param);
    ESP_LOGI(GATTS_TAG, "  需确认: %s", pkt->need_ack ? "是" : "否");
    
    switch (pkt->cmd) {
        // 工作模式命令
        case OWL_CMD_MODE_SWITCH:
            if (pkt->param <= OWL_MODE_PRESET) {
                mode_switch((owl_mode_t)pkt->param);
                send_ack(OWL_CMD_MODE_SWITCH, OWL_ERR_NONE);
            } else {
                send_ack(OWL_CMD_MODE_SWITCH, OWL_ERR_INVALID_CMD);
            }
            break;
            
        case OWL_CMD_PRESET_START:
            if (g_current_mode == OWL_MODE_PRESET) {
                preset_start(pkt->param);
            } else {
                // 自动切换到预设模式再执行
                mode_switch(OWL_MODE_PRESET);
                preset_start(pkt->param);
            }
            break;
            
        case OWL_CMD_PRESET_STOP:
            preset_stop();
            break;
            
        case OWL_CMD_RECORD_START:
            mode_switch(OWL_MODE_RECORD);
            record_enter(pkt->param);
            break;
            
        case OWL_CMD_RECORD_STOP:
            if (g_current_mode == OWL_MODE_RECORD) {
                record_save();
                mode_switch(OWL_MODE_REMOTE);
            }
            break;
            
        case OWL_CMD_RECORD_DELETE:
            record_delete(pkt->param);
            break;
        
        // 原有命令
        case OWL_CMD_EMERGENCY_STOP:
            ESP_LOGW(GATTS_TAG, "  ⚠️ 紧急停止!");
            // 停止预设执行
            if (g_preset_running) preset_stop();
            // 停止录制
            if (g_recording) { record_save(); mode_switch(OWL_MODE_REMOTE); }
            break;
        case OWL_CMD_RESET:
            ESP_LOGI(GATTS_TAG, "  系统复位");
            break;
        case OWL_CMD_CALIBRATE:
            ESP_LOGI(GATTS_TAG, "  校准模式");
            break;
        default:
            ESP_LOGI(GATTS_TAG, "  未知命令");
            send_ack(pkt->cmd, OWL_ERR_INVALID_CMD);
            break;
    }
}

static void parse_control_packet(uint8_t *data, uint16_t len) {
    if (len < 3) {
        ESP_LOGW(GATTS_TAG, "数据包太短: %d bytes", len);
        return;
    }
    
    uint8_t pkt_type = OWL_GET_PKT_TYPE((owl_packet_header_t*)data);
    
    switch (pkt_type) {
        case OWL_PKT_JOYSTICK:
            if (len >= sizeof(owl_joystick_pkt_t)) {
                if (g_current_mode == OWL_MODE_REMOTE || g_current_mode == OWL_MODE_RECORD) {
                    parse_joystick_packet((owl_joystick_pkt_t*)data);
                }
                // 录制模式下同时记录帧
                if (g_current_mode == OWL_MODE_RECORD) {
                    record_add_joystick_frame((owl_joystick_pkt_t*)data);
                }
                // 安防/预设模式下忽略摇杆
            }
            break;
        case OWL_PKT_SWITCH:
            if (len >= sizeof(owl_switch_pkt_t)) {
                if (g_current_mode == OWL_MODE_REMOTE || g_current_mode == OWL_MODE_RECORD) {
                    parse_switch_packet((owl_switch_pkt_t*)data);
                }
                // 录制模式下同时记录帧
                if (g_current_mode == OWL_MODE_RECORD) {
                    record_add_switch_frame((owl_switch_pkt_t*)data);
                }
                // 安防/预设模式下忽略开关
            }
            break;
        case OWL_PKT_HEARTBEAT:
            if (len >= sizeof(owl_heartbeat_pkt_t)) {
                parse_heartbeat_packet((owl_heartbeat_pkt_t*)data);
            }
            break;
        case OWL_PKT_COMMAND:
            if (len >= sizeof(owl_command_pkt_t)) {
                parse_command_packet((owl_command_pkt_t*)data);
            }
            break;
        default:
            ESP_LOGW(GATTS_TAG, "未知包类型: 0x%02X", pkt_type);
            break;
    }
}

/*============================================================================
 *                              GAP 事件处理
 *============================================================================*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "广播启动失败, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "广播启动成功");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "连接参数更新, conn_int=%d, latency=%d, timeout=%d",
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

/*============================================================================
 *                              GATT 事件处理
 *============================================================================*/

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                                          esp_gatt_if_t gatts_if, 
                                          esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT 注册成功, app_id=%d", param->reg.app_id);
        
        // 设置设备名
        esp_ble_gap_set_device_name(owl_device_name);
        
        // 配置广播数据
        esp_ble_gap_config_adv_data(&adv_data);
        adv_config_done |= adv_config_flag;
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        adv_config_done |= scan_rsp_config_flag;
        
        // 创建服务
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile.service_id.id.uuid.uuid.uuid16 = OWL_SERVICE_UUID;
        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
        break;
        
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "服务创建成功, handle=%d", param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(gl_profile.service_handle);
        
        // 添加控制通道特征值 (Write Without Response)
        {
            esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = OWL_CHAR_CONTROL_UUID};
            esp_ble_gatts_add_char(gl_profile.service_handle, &char_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                   NULL, NULL);
        }
        break;
        
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "特征值添加成功, attr_handle=%d", param->add_char.attr_handle);
        
        // 根据UUID判断是哪个特征值
        if (param->add_char.char_uuid.uuid.uuid16 == OWL_CHAR_CONTROL_UUID) {
            gl_profile.char_control_handle = param->add_char.attr_handle;
            
            // 添加反馈通道特征值 (Notify)
            esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = OWL_CHAR_FEEDBACK_UUID};
            esp_ble_gatts_add_char(gl_profile.service_handle, &char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
        }
        else if (param->add_char.char_uuid.uuid.uuid16 == OWL_CHAR_FEEDBACK_UUID) {
            gl_profile.char_feedback_handle = param->add_char.attr_handle;
            
            // 添加 CCC 描述符
            esp_bt_uuid_t descr_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG};
            esp_ble_gatts_add_char_descr(gl_profile.service_handle, &descr_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        }
        break;
        
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "描述符添加成功, attr_handle=%d", param->add_char_descr.attr_handle);
        gl_profile.descr_feedback_handle = param->add_char_descr.attr_handle;
        
        // 添加系统命令特征值 (Write + Indicate)
        {
            esp_bt_uuid_t char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = OWL_CHAR_COMMAND_UUID};
            esp_ble_gatts_add_char(gl_profile.service_handle, &char_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_INDICATE,
                                   NULL, NULL);
        }
        break;
        
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "服务启动成功");
        break;
        
    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(GATTS_TAG, "客户端连接, conn_id=%d", param->connect.conn_id);
        ESP_LOGI(GATTS_TAG, "客户端 MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        
        // MAC 地址验证
        if (!verify_mac(param->connect.remote_bda)) {
            ESP_LOGW(GATTS_TAG, "⚠️ 未授权设备，断开连接!");
            ESP_LOGW(GATTS_TAG, "期望 MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(g_bound_mac));
            esp_ble_gap_disconnect(param->connect.remote_bda);
            break;
        }
        
        // 首次连接，自动绑定
        if (!g_is_bound) {
            ESP_LOGI(GATTS_TAG, "首次连接，自动绑定设备");
            save_bound_mac(param->connect.remote_bda);
        } else {
            ESP_LOGI(GATTS_TAG, "✅ 已绑定设备连接成功");
        }
        
        gl_profile.conn_id = param->connect.conn_id;
        gatts_if_global = gatts_if;
        conn_id_global = param->connect.conn_id;
        
        // 初始化心跳时间
        last_heartbeat_time = esp_timer_get_time() / 1000;
        
        // 更新连接参数
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x18;    // 30ms
        conn_params.min_int = 0x10;    // 20ms
        conn_params.timeout = 400;     // 4000ms
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
        
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "客户端断开, reason=0x%02x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        local_mtu = 23;
        last_seq = 0;
        last_heartbeat_time = 0;
        break;
        
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "收到写入请求, handle=%d, len=%d", 
                 param->write.handle, param->write.len);
        
        // 控制通道写入
        if (param->write.handle == gl_profile.char_control_handle) {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            parse_control_packet(param->write.value, param->write.len);
        }
        
        // CCC 写入（启用 Notify）
        if (param->write.handle == gl_profile.descr_feedback_handle && param->write.len == 2) {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001) {
                ESP_LOGI(GATTS_TAG, "Notify 已启用");
            } else if (descr_value == 0x0000) {
                ESP_LOGI(GATTS_TAG, "Notify 已禁用");
            }
        }
        
        // 发送响应
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;
        
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU 交换完成, MTU=%d", param->mtu.mtu);
        local_mtu = param->mtu.mtu;
        break;
        
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, 
                                  esp_gatt_if_t gatts_if, 
                                  esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "注册失败, status=%d", param->reg.status);
            return;
        }
    }
    
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile.gatts_if) {
        gatts_profile_event_handler(event, gatts_if, param);
    }
}

/*============================================================================
 *                              主函数
 *============================================================================*/

void app_main(void) {
    esp_err_t ret;

    // 初始化 NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 加载绑定信息
    load_bound_mac();

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "启用蓝牙控制器失败: %s", esp_err_to_name(ret));
        return;
    }

    // 初始化 Bluedroid
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Bluedroid 初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "启用 Bluedroid 失败: %s", esp_err_to_name(ret));
        return;
    }

    // 注册回调
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS 回调注册失败: %x", ret);
        return;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GAP 回调注册失败: %x", ret);
        return;
    }

    // 注册应用
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "应用注册失败: %x", ret);
        return;
    }

    // 设置 MTU
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "设置 MTU 失败: %x", ret);
    }

    // 初始化 LD6004 雷达
    radar_config_t radar_cfg = RADAR_CONFIG_DEFAULT();
    radar_handle = RADAR_INIT(&radar_cfg);
    if (radar_handle != NULL) {
        RADAR_ADD_HANDLER(radar_handle, radar_event_handler, NULL);
        radar_enabled = true;
        ESP_LOGI(GATTS_TAG, "LD6004 Radar initialized (TX=GPIO%d, RX=GPIO%d)",
                 CONFIG_RADAR_UART_TX_PIN, CONFIG_RADAR_UART_RX_PIN);
    } else {
        ESP_LOGW(GATTS_TAG, "LD6004 Radar init failed, continuing without radar");
    }

    // 初始化舵机
    servo_init();

    // 初始化继电器
    relay_init();

    // 启动心跳检测任务
    xTaskCreate(heartbeat_monitor_task, "heartbeat_monitor", 2048, NULL, 5, NULL);

    // 启动安防模式定时更新任务
    xTaskCreate(security_task, "security", 4096, NULL, 4, NULL);

    ESP_LOGI(GATTS_TAG, "猫头鹰 BLE 服务端初始化完成");
    ESP_LOGI(GATTS_TAG, "设备名: %s", OWL_DEVICE_NAME);
    ESP_LOGI(GATTS_TAG, "服务 UUID: 0x%04X", OWL_SERVICE_UUID);
}
