/*
 * 猫头鹰玩具 BLE GATT Client (遥控器端)
 * 基于 ESP-IDF 官方示例修改
 * 
 * 功能：
 * - 扫描并连接猫头鹰设备
 * - 发送测试数据包（摇杆、开关、心跳）
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
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"

#include "owl_protocol.h"

#define GATTC_TAG "OWL_CLIENT"

/*============================================================================
 *                              GATT 定义
 *============================================================================*/

#define PROFILE_NUM             1
#define PROFILE_APP_ID          0
#define INVALID_HANDLE          0

// 目标设备名
static char target_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = OWL_DEVICE_NAME;

// 目标服务和特征值 UUID
#define REMOTE_SERVICE_UUID         0xFF00
#define REMOTE_CHAR_CONTROL_UUID    0xFF01
#define REMOTE_CHAR_FEEDBACK_UUID   0xFF02
#define REMOTE_CHAR_COMMAND_UUID    0xFF03

/*============================================================================
 *                              全局变量
 *============================================================================*/

static bool connect = false;
static bool get_server = false;

// 序列号
static uint8_t seq_num = 0;

// 特征值句柄
static uint16_t char_control_handle = INVALID_HANDLE;
static uint16_t char_feedback_handle = INVALID_HANDLE;
static uint16_t char_command_handle = INVALID_HANDLE;

// 测试数据发送定时器
static esp_timer_handle_t test_timer;

/*============================================================================
 *                              扫描参数
 *============================================================================*/

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = ESP_BLE_GAP_SCAN_ITVL_MS(50),
    .scan_window = ESP_BLE_GAP_SCAN_WIN_MS(30),
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

/*============================================================================
 *                              Profile 结构
 *============================================================================*/

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile = {
    .gattc_cb = NULL,
    .gattc_if = ESP_GATT_IF_NONE,
    .app_id = PROFILE_APP_ID,
};

/*============================================================================
 *                              数据包构建函数
 *============================================================================*/

static uint8_t get_next_seq(void) {
    return seq_num++;
}

static void build_joystick_packet(owl_joystick_pkt_t *pkt, uint8_t x, uint8_t y, uint8_t btn) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_JOYSTICK);
    pkt->header.seq = get_next_seq();
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->x_axis = x;
    pkt->y_axis = y;
    pkt->button_flags = btn & 0x01;
}

static void build_switch_packet(owl_switch_pkt_t *pkt, uint8_t sw1, uint8_t sw2) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_SWITCH);
    pkt->header.seq = get_next_seq();
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->switch1 = sw1;
    pkt->switch2 = sw2;
    pkt->change_flags = 0x03;  // 两组都有变化
}

static void build_heartbeat_packet(owl_heartbeat_pkt_t *pkt, uint8_t status, uint8_t battery) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_HEARTBEAT);
    pkt->header.seq = get_next_seq();
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->status = status;
    pkt->battery = battery;
}

static void build_command_packet(owl_command_pkt_t *pkt, uint8_t cmd, uint8_t param, uint8_t need_ack) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_COMMAND);
    pkt->header.seq = get_next_seq();
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->cmd = cmd;
    pkt->param = param;
    pkt->need_ack = need_ack;
}

/*============================================================================
 *                              测试数据发送
 *============================================================================*/

static void send_test_data(void *arg) {
    if (!connect || char_control_handle == INVALID_HANDLE) {
        return;
    }
    
    static int test_phase = 0;
    
    switch (test_phase % 4) {
        case 0: {
            // 发送摇杆数据
            owl_joystick_pkt_t pkt;
            build_joystick_packet(&pkt, 128, 128, 0);  // 中点，按键松开
            ESP_LOGI(GATTC_TAG, ">>> 发送 JOYSTICK 包");
            ESP_LOGI(GATTC_TAG, "    X=%d, Y=%d, 按键=%d", pkt.x_axis, pkt.y_axis, pkt.button_flags);
            esp_ble_gattc_write_char(gl_profile.gattc_if, gl_profile.conn_id,
                                     char_control_handle,
                                     sizeof(pkt), (uint8_t*)&pkt,
                                     ESP_GATT_WRITE_TYPE_NO_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            break;
        }
        case 1: {
            // 发送摇杆数据（移动）
            owl_joystick_pkt_t pkt;
            build_joystick_packet(&pkt, 200, 100, 1);  // 右上，按键按下
            ESP_LOGI(GATTC_TAG, ">>> 发送 JOYSTICK 包");
            ESP_LOGI(GATTC_TAG, "    X=%d, Y=%d, 按键=%d", pkt.x_axis, pkt.y_axis, pkt.button_flags);
            esp_ble_gattc_write_char(gl_profile.gattc_if, gl_profile.conn_id,
                                     char_control_handle,
                                     sizeof(pkt), (uint8_t*)&pkt,
                                     ESP_GATT_WRITE_TYPE_NO_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            break;
        }
        case 2: {
            // 发送开关数据
            owl_switch_pkt_t pkt;
            build_switch_packet(&pkt, OWL_SW_UP | OWL_SW_CENTER, OWL_SW_RIGHT);
            ESP_LOGI(GATTC_TAG, ">>> 发送 SWITCH 包");
            ESP_LOGI(GATTC_TAG, "    开关1=0x%02X, 开关2=0x%02X", pkt.switch1, pkt.switch2);
            esp_ble_gattc_write_char(gl_profile.gattc_if, gl_profile.conn_id,
                                     char_control_handle,
                                     sizeof(pkt), (uint8_t*)&pkt,
                                     ESP_GATT_WRITE_TYPE_NO_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            break;
        }
        case 3: {
            // 发送心跳数据
            owl_heartbeat_pkt_t pkt;
            build_heartbeat_packet(&pkt, 0x00, 37);  // 正常状态，3.7V
            ESP_LOGI(GATTC_TAG, ">>> 发送 HEARTBEAT 包");
            ESP_LOGI(GATTC_TAG, "    状态=0x%02X, 电池=%d.%dV", pkt.status, pkt.battery/10, pkt.battery%10);
            esp_ble_gattc_write_char(gl_profile.gattc_if, gl_profile.conn_id,
                                     char_control_handle,
                                     sizeof(pkt), (uint8_t*)&pkt,
                                     ESP_GATT_WRITE_TYPE_NO_RSP,
                                     ESP_GATT_AUTH_REQ_NONE);
            break;
        }
    }
    
    test_phase++;
}

/*============================================================================
 *                              GAP 事件处理
 *============================================================================*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(30);  // 扫描 30 秒
        break;
        
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "扫描启动失败");
            break;
        }
        ESP_LOGI(GATTC_TAG, "开始扫描...");
        break;
        
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // 检查设备名
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            
            adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                 ESP_BLE_AD_TYPE_NAME_CMPL,
                                                 &adv_name_len);
            
            if (adv_name != NULL && strncmp((char*)adv_name, target_device_name, adv_name_len) == 0) {
                ESP_LOGI(GATTC_TAG, "发现目标设备: %s", target_device_name);
                ESP_LOGI(GATTC_TAG, "地址: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->scan_rst.bda));
                
                // 停止扫描并连接
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(gl_profile.gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
            }
        }
        break;
        
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "扫描停止");
        break;
        
    default:
        break;
    }
}

/*============================================================================
 *                              GATT 事件处理
 *============================================================================*/

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, 
                                          esp_gatt_if_t gattc_if, 
                                          esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT 客户端注册成功");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
        
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "连接失败, status=%d", param->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "连接成功");
        gl_profile.conn_id = param->open.conn_id;
        memcpy(gl_profile.remote_bda, param->open.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_send_mtu_req(gattc_if, param->open.conn_id);
        break;
        
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU 交换完成, MTU=%d", param->cfg_mtu.mtu);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
        
    case ESP_GATTC_SEARCH_RES_EVT:
        if (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "发现服务: 0x%04X", REMOTE_SERVICE_UUID);
            get_server = true;
            gl_profile.service_start_handle = param->search_res.start_handle;
            gl_profile.service_end_handle = param->search_res.end_handle;
        }
        break;
        
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (get_server) {
            // 获取特征值
            esp_gattc_char_elem_t *char_elem_result = NULL;
            uint16_t count = 0;
            esp_bt_uuid_t char_uuid;
            
            // 先获取特征值数量
            esp_ble_gattc_get_attr_count(gl_profile.gattc_if, gl_profile.conn_id,
                                         ESP_GATT_DB_CHARACTERISTIC,
                                         gl_profile.service_start_handle,
                                         gl_profile.service_end_handle,
                                         INVALID_HANDLE, &count);
            
            if (count > 0) {
                char_elem_result = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (char_elem_result) {
                    // 获取控制通道特征值
                    char_uuid.len = ESP_UUID_LEN_16;
                    char_uuid.uuid.uuid16 = REMOTE_CHAR_CONTROL_UUID;
                    
                    esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(
                        gl_profile.gattc_if, gl_profile.conn_id,
                        gl_profile.service_start_handle, gl_profile.service_end_handle,
                        char_uuid, char_elem_result, &count);
                    
                    if (status == ESP_GATT_OK && count > 0) {
                        char_control_handle = char_elem_result[0].char_handle;
                        ESP_LOGI(GATTC_TAG, "控制通道特征值: handle=%d", char_control_handle);
                    }
                    free(char_elem_result);
                }
            }
            
            // 启动测试定时器
            if (char_control_handle != INVALID_HANDLE) {
                connect = true;
                ESP_LOGI(GATTC_TAG, "开始发送测试数据...");
                
                esp_timer_create_args_t timer_args = {
                    .callback = send_test_data,
                    .arg = NULL,
                    .name = "test_timer"
                };
                esp_timer_create(&timer_args, &test_timer);
                esp_timer_start_periodic(test_timer, 500000);  // 每 500ms 发送一次
            }
        }
        break;
        
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "断开连接");
        connect = false;
        get_server = false;
        char_control_handle = INVALID_HANDLE;
        seq_num = 0;
        
        if (test_timer != NULL) {
            esp_timer_stop(test_timer);
            esp_timer_delete(test_timer);
            test_timer = NULL;
        }
        
        // 重新扫描
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
        
    default:
        break;
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, 
                                  esp_gatt_if_t gattc_if, 
                                  esp_ble_gattc_cb_param_t *param) {
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gattc_if = gattc_if;
        } else {
            ESP_LOGE(GATTC_TAG, "注册失败, status=%d", param->reg.status);
            return;
        }
    }
    
    if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile.gattc_if) {
        gattc_profile_event_handler(event, gattc_if, param);
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

    // 释放经典蓝牙内存
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 初始化蓝牙控制器
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "蓝牙控制器初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "启用蓝牙控制器失败: %s", esp_err_to_name(ret));
        return;
    }

    // 初始化 Bluedroid
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "Bluedroid 初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "启用 Bluedroid 失败: %s", esp_err_to_name(ret));
        return;
    }

    // 注册回调
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "GATTC 回调注册失败: %x", ret);
        return;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "GAP 回调注册失败: %x", ret);
        return;
    }

    // 注册应用
    ret = esp_ble_gattc_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "应用注册失败: %x", ret);
        return;
    }

    // 设置 MTU
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "设置 MTU 失败: %x", ret);
    }

    ESP_LOGI(GATTC_TAG, "猫头鹰 BLE 客户端初始化完成");
    ESP_LOGI(GATTC_TAG, "目标设备: %s", target_device_name);
}
