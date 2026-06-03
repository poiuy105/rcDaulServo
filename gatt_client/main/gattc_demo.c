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
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"

#define GATTC_TAG "OWL_CLIENT"

// 心跳超时配置
#define HEARTBEAT_TIMEOUT_MS    3000    // 3秒无接收认为断线
#define RECONNECT_DELAY_MS      2000    // 断线后2秒开始重连

// 心跳检测状态
static volatile bool heartbeat_timeout = false;

/*============================================================================
 *                              工作模式管理
 *============================================================================*/
static volatile owl_mode_t g_current_mode = OWL_MODE_REMOTE;

/*============================================================================
 *                              WS2812 LED 配置
 *============================================================================*/

#define WS2812_GPIO         10
#define WS2812_LED_NUM      4
#define WS2812_BRIGHTNESS   50  // 0-255, 降低亮度避免刺眼

// LED索引
#define LED_CONN            0   // 连接状态
#define LED_BATTERY         1   // 电池电量
#define LED_KEY             2   // 按键反馈
#define LED_COMM            3   // 通信状态

// LED颜色定义 (RGB)
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_color_t;

// 预定义颜色
static const led_color_t COLOR_OFF     = {0, 0, 0};
static const led_color_t COLOR_RED     = {255, 0, 0};
static const led_color_t COLOR_GREEN   = {0, 255, 0};
static const led_color_t COLOR_BLUE    = {0, 0, 255};
static const led_color_t COLOR_YELLOW  = {255, 255, 0};
static const led_color_t COLOR_ORANGE  = {255, 165, 0};
static const led_color_t COLOR_PURPLE  = {255, 0, 255};
static const led_color_t COLOR_CYAN    = {0, 255, 255};
static const led_color_t COLOR_WHITE   = {255, 255, 255};
static const led_color_t COLOR_PINK    = {255, 192, 203};

// LED状态
static led_color_t led_colors[WS2812_LED_NUM];
static led_strip_handle_t led_strip = NULL;

// 初始化WS2812
static void ws2812_init(void) {
    // RMT TX通道配置
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = WS2812_LED_NUM,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    // 初始状态：全部关闭
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_colors[i] = COLOR_OFF;
    }
    led_strip_clear(led_strip);
    
    ESP_LOGI(GATTC_TAG, "WS2812 LED初始化完成");
    ESP_LOGI(GATTC_TAG, "  GPIO: %d, LED数量: %d", WS2812_GPIO, WS2812_LED_NUM);
}

// 设置单个LED颜色
static void ws2812_set_led(int index, led_color_t color) {
    if (index < 0 || index >= WS2812_LED_NUM) return;
    
    led_colors[index] = color;
    led_strip_set_pixel(led_strip, index, 
                        (color.r * WS2812_BRIGHTNESS) / 255,
                        (color.g * WS2812_BRIGHTNESS) / 255,
                        (color.b * WS2812_BRIGHTNESS) / 255);
    led_strip_refresh(led_strip);
}

// 刷新所有LED
static void ws2812_refresh(void) {
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_strip_set_pixel(led_strip, i,
                            (led_colors[i].r * WS2812_BRIGHTNESS) / 255,
                            (led_colors[i].g * WS2812_BRIGHTNESS) / 255,
                            (led_colors[i].b * WS2812_BRIGHTNESS) / 255);
    }
    led_strip_refresh(led_strip);
}

// 关闭所有LED
static void ws2812_all_off(void) {
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_colors[i] = COLOR_OFF;
    }
    led_strip_clear(led_strip);
}

// 启动动画（彩虹渐变）
static void ws2812_startup_animation(void) {
    led_color_t rainbow[] = {
        {255, 0, 0},    // 红
        {255, 127, 0},  // 橙
        {255, 255, 0},  // 黄
        {0, 255, 0},    // 绿
        {0, 0, 255},    // 蓝
        {75, 0, 130},   // 靛
        {148, 0, 211},  // 紫
    };
    
    for (int round = 0; round < 2; round++) {
        for (int i = 0; i < 7; i++) {
            for (int j = 0; j < WS2812_LED_NUM; j++) {
                led_strip_set_pixel(led_strip, j,
                                    (rainbow[i].r * WS2812_BRIGHTNESS) / 255,
                                    (rainbow[i].g * WS2812_BRIGHTNESS) / 255,
                                    (rainbow[i].b * WS2812_BRIGHTNESS) / 255);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    led_strip_clear(led_strip);
}

// LED状态更新函数前置声明
static void ws2812_update_connection(bool connected);
static void ws2812_update_battery(uint8_t battery_percent);

// 配对成功动画
static void ws2812_pairing_animation(void) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < WS2812_LED_NUM; j++) {
            led_strip_set_pixel(led_strip, j,
                                (COLOR_WHITE.r * WS2812_BRIGHTNESS) / 255,
                                (COLOR_WHITE.g * WS2812_BRIGHTNESS) / 255,
                                (COLOR_WHITE.b * WS2812_BRIGHTNESS) / 255);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // 动画结束后恢复连接和电量显示
    ws2812_update_connection(true);
    ws2812_update_battery(70);
}

// LED状态更新函数定义
static void ws2812_update_connection(bool connected) {
    if (!connected) {
        ws2812_set_led(LED_CONN, COLOR_RED);  // 红色=未连接
        return;
    }
    switch (g_current_mode) {
        case OWL_MODE_REMOTE:
            ws2812_set_led(LED_CONN, COLOR_GREEN);    // 绿色=遥控模式
            break;
        case OWL_MODE_SECURITY:
            ws2812_set_led(LED_CONN, COLOR_RED);       // 红色=安防模式
            break;
        case OWL_MODE_PRESET:
            ws2812_set_led(LED_CONN, COLOR_BLUE);      // 蓝色=预设模式
            break;
        case OWL_MODE_RECORD:
            ws2812_set_led(LED_CONN, COLOR_YELLOW);    // 黄色=录制模式
            break;
        default:
            ws2812_set_led(LED_CONN, COLOR_GREEN);
            break;
    }
}

static void ws2812_update_battery(uint8_t battery_percent) {
    led_color_t color;
    if (battery_percent > 80) {
        color = COLOR_GREEN;
    } else if (battery_percent > 50) {
        color = COLOR_BLUE;
    } else if (battery_percent > 20) {
        color = COLOR_YELLOW;
    } else {
        color = COLOR_RED;
    }
    ws2812_set_led(LED_BATTERY, color);
}

static void ws2812_update_key(uint8_t switch1, uint8_t switch2, uint8_t joystick_btn) {
    led_color_t color = COLOR_OFF;
    
    if (joystick_btn) {
        color = COLOR_BLUE;
    } else if (switch1 & OWL_SW_CENTER) {
        color = COLOR_WHITE;
    } else if (switch1 & OWL_SW_UP) {
        color = COLOR_PURPLE;
    } else if (switch1 & OWL_SW_DOWN) {
        color = COLOR_CYAN;
    } else if (switch1 & (OWL_SW_LEFT | OWL_SW_RIGHT)) {
        color = COLOR_ORANGE;
    } else if (switch2) {
        color = COLOR_PINK;
    }
    
    ws2812_set_led(LED_KEY, color);
}

static void ws2812_update_comm(bool sending) {
    ws2812_set_led(LED_COMM, sending ? COLOR_CYAN : COLOR_GREEN);
}

/*============================================================================
 *                              摇杆 ADC 配置
 *============================================================================*/

// GPIO 定义
#define JOYSTICK_X_GPIO     3
#define JOYSTICK_Y_GPIO     4
#define JOYSTICK_BTN_GPIO   9

// ADC 配置
#define JOYSTICK_ADC_UNIT   ADC_UNIT_1
#define JOYSTICK_X_CHANNEL  ADC_CHANNEL_3   // IO3
#define JOYSTICK_Y_CHANNEL  ADC_CHANNEL_4   // IO4

// ADC句柄
static adc_oneshot_unit_handle_t adc_handle = NULL;

// 初始化摇杆ADC
static void joystick_adc_init(void) {
    // 初始化ADC单元
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = JOYSTICK_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // 配置X轴通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_X_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_CHANNEL, &config));

    // 配置按键GPIO
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_conf));

    ESP_LOGI(GATTC_TAG, "摇杆ADC初始化完成");
    ESP_LOGI(GATTC_TAG, "  X轴: GPIO%d (ADC1_CH%d)", JOYSTICK_X_GPIO, JOYSTICK_X_CHANNEL);
    ESP_LOGI(GATTC_TAG, "  Y轴: GPIO%d (ADC1_CH%d)", JOYSTICK_Y_GPIO, JOYSTICK_Y_CHANNEL);
    ESP_LOGI(GATTC_TAG, "  按键: GPIO%d", JOYSTICK_BTN_GPIO);
}

// 读取摇杆值
static void joystick_read(uint8_t *x, uint8_t *y, uint8_t *btn) {
    int raw_x = 2048, raw_y = 2048;  // 默认中点值

    if (adc_oneshot_read(adc_handle, JOYSTICK_X_CHANNEL, &raw_x) != ESP_OK) {
        raw_x = 2048;
    }
    if (adc_oneshot_read(adc_handle, JOYSTICK_Y_CHANNEL, &raw_y) != ESP_OK) {
        raw_y = 2048;
    }

    *x = (uint8_t)((raw_x * 255) / 4095);
    *y = (uint8_t)((raw_y * 255) / 4095);
    *btn = (gpio_get_level(JOYSTICK_BTN_GPIO) == 0) ? 1 : 0;
}

/*============================================================================
 *                              矩阵按键配置
 *============================================================================*/

// 行线（输出）
#define MATRIX_ROW1_GPIO    2
#define MATRIX_ROW2_GPIO    8
static const int matrix_rows[] = {MATRIX_ROW1_GPIO, MATRIX_ROW2_GPIO};
#define MATRIX_ROWS         2

// 列线（输入，上拉）
#define MATRIX_COL1_GPIO    6   // 上
#define MATRIX_COL2_GPIO    5   // 下
#define MATRIX_COL3_GPIO    7   // 左
#define MATRIX_COL4_GPIO    1   // 右
#define MATRIX_COL5_GPIO    0   // 中
static const int matrix_cols[] = {MATRIX_COL1_GPIO, MATRIX_COL2_GPIO, MATRIX_COL3_GPIO, MATRIX_COL4_GPIO, MATRIX_COL5_GPIO};
#define MATRIX_COLS         5

// 按键状态
static uint8_t matrix_key_state[MATRIX_ROWS][MATRIX_COLS] = {0};
static uint8_t matrix_key_last_state[MATRIX_ROWS][MATRIX_COLS] = {0};
// 消抖计数器（连续读到相同值N次才认为有效）
static uint8_t matrix_debounce_count[MATRIX_ROWS][MATRIX_COLS] = {0};
#define DEBOUNCE_THRESHOLD  3  // 连续3次(30ms)相同才确认

// 初始化矩阵按键
static void matrix_key_init(void) {
    // 配置行线为输出
    gpio_config_t row_conf = {
        .pin_bit_mask = (1ULL << MATRIX_ROW1_GPIO) | (1ULL << MATRIX_ROW2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&row_conf));
    
    // 配置列线为输入（上拉）
    gpio_config_t col_conf = {
        .pin_bit_mask = (1ULL << MATRIX_COL1_GPIO) | (1ULL << MATRIX_COL2_GPIO) |
                        (1ULL << MATRIX_COL3_GPIO) | (1ULL << MATRIX_COL4_GPIO) |
                        (1ULL << MATRIX_COL5_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&col_conf));
    
    // 初始化行线为高电平
    gpio_set_level(MATRIX_ROW1_GPIO, 1);
    gpio_set_level(MATRIX_ROW2_GPIO, 1);
    
    ESP_LOGI(GATTC_TAG, "矩阵按键初始化完成");
    ESP_LOGI(GATTC_TAG, "  行线: GPIO%d, GPIO%d", MATRIX_ROW1_GPIO, MATRIX_ROW2_GPIO);
    ESP_LOGI(GATTC_TAG, "  列线: GPIO%d, GPIO%d, GPIO%d, GPIO%d, GPIO%d",
             MATRIX_COL1_GPIO, MATRIX_COL2_GPIO, MATRIX_COL3_GPIO, MATRIX_COL4_GPIO, MATRIX_COL5_GPIO);
}

// 扫描矩阵按键，返回是否有变化（含软件消抖）
static bool matrix_key_scan(uint8_t *switch1, uint8_t *switch2) {
    bool changed = false;
    uint8_t raw_state[MATRIX_ROWS][MATRIX_COLS] = {0};
    
    // 逐行扫描原始值
    for (int row = 0; row < MATRIX_ROWS; row++) {
        gpio_set_level(matrix_rows[row], 0);
        esp_rom_delay_us(10);
        for (int col = 0; col < MATRIX_COLS; col++) {
            raw_state[row][col] = (gpio_get_level(matrix_cols[col]) == 0) ? 1 : 0;
        }
        gpio_set_level(matrix_rows[row], 1);
    }
    
    // 软件消抖：连续读到相同值DEBOUNCE_THRESHOLD次才更新
    for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int col = 0; col < MATRIX_COLS; col++) {
            if (raw_state[row][col] == matrix_key_state[row][col]) {
                // 值未变，重置计数器
                matrix_debounce_count[row][col] = 0;
            } else {
                // 值变化，累加计数器
                matrix_debounce_count[row][col]++;
                if (matrix_debounce_count[row][col] >= DEBOUNCE_THRESHOLD) {
                    // 连续N次读到新值，确认变化
                    matrix_key_last_state[row][col] = matrix_key_state[row][col];
                    matrix_key_state[row][col] = raw_state[row][col];
                    matrix_debounce_count[row][col] = 0;
                    changed = true;
                }
            }
        }
    }
    
    // 转换为switch1/switch2格式
    *switch1 = 0;
    *switch2 = 0;
    
    if (matrix_key_state[0][0]) *switch1 |= OWL_SW_UP;
    if (matrix_key_state[0][1]) *switch1 |= OWL_SW_DOWN;
    if (matrix_key_state[0][2]) *switch1 |= OWL_SW_LEFT;
    if (matrix_key_state[0][3]) *switch1 |= OWL_SW_RIGHT;
    if (matrix_key_state[0][4]) *switch1 |= OWL_SW_CENTER;
    
    if (matrix_key_state[1][0]) *switch2 |= OWL_SW_UP;
    if (matrix_key_state[1][1]) *switch2 |= OWL_SW_DOWN;
    if (matrix_key_state[1][2]) *switch2 |= OWL_SW_LEFT;
    if (matrix_key_state[1][3]) *switch2 |= OWL_SW_RIGHT;
    if (matrix_key_state[1][4]) *switch2 |= OWL_SW_CENTER;
    
    return changed;
}

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

static volatile bool connect = false;
static volatile bool get_server = false;

// 序列号（各包类型独立，避免服务器端丢包误报）
static volatile uint8_t seq_joystick = 0;
static volatile uint8_t seq_heartbeat = 0;
static volatile uint8_t seq_switch = 0;
static volatile uint8_t seq_command = 0;

// 特征值句柄
static volatile uint16_t char_control_handle = INVALID_HANDLE;
static volatile uint16_t char_feedback_handle = INVALID_HANDLE;
static volatile uint16_t char_command_handle = INVALID_HANDLE;
static volatile uint16_t char_feedback_ccc_handle = INVALID_HANDLE;

// 测试数据发送定时器
static esp_timer_handle_t test_timer;

// 接收心跳时间（用于检测服务端存活）
static volatile int64_t last_recv_time = 0;

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
        ESP_LOGI(GATTC_TAG, "NVS 未初始化，无绑定信息");
        g_is_bound = false;
        return;
    }
    
    err = nvs_get_blob(nvs_handle, OWL_KEY_REMOTE_MAC, g_bound_mac, &length);
    if (err == ESP_OK && length == 6) {
        g_is_bound = true;
        ESP_LOGI(GATTC_TAG, "已绑定猫头鹰 MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(g_bound_mac));
    } else {
        g_is_bound = false;
        ESP_LOGI(GATTC_TAG, "无绑定信息，将自动绑定首个连接的设备");
    }
    
    nvs_close(nvs_handle);
}

// 保存绑定的 MAC 地址
static void save_bound_mac(uint8_t *mac) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "NVS 打开失败");
        return;
    }
    
    err = nvs_set_blob(nvs_handle, OWL_KEY_REMOTE_MAC, mac, 6);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            memcpy(g_bound_mac, mac, 6);
            g_is_bound = true;
            ESP_LOGI(GATTC_TAG, "猫头鹰 MAC 已保存: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(mac));
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
    ESP_LOGI(GATTC_TAG, "绑定信息已清除");
}

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

static void build_joystick_packet(owl_joystick_pkt_t *pkt, uint8_t x, uint8_t y, uint8_t btn) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_JOYSTICK);
    pkt->header.seq = seq_joystick++;
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->x_axis = x;
    pkt->y_axis = y;
    pkt->button_flags = btn & 0x01;
}

static void build_switch_packet(owl_switch_pkt_t *pkt, uint8_t sw1, uint8_t sw2) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_SWITCH);
    pkt->header.seq = seq_switch++;
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->switch1 = sw1;
    pkt->switch2 = sw2;
    pkt->change_flags = 0x03;  // 两组都有变化
}

static void build_heartbeat_packet(owl_heartbeat_pkt_t *pkt, uint8_t status, uint8_t battery) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_HEARTBEAT);
    pkt->header.seq = seq_heartbeat++;
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->status = status;
    pkt->battery = battery;
}

static void build_command_packet(owl_command_pkt_t *pkt, uint8_t cmd, uint8_t param, uint8_t need_ack) {
    pkt->header.version_type = OWL_MAKE_HEADER(OWL_PKT_COMMAND);
    pkt->header.seq = seq_command++;
    pkt->header.timestamp = (uint8_t)(esp_timer_get_time() / 1000);
    pkt->cmd = cmd;
    pkt->param = param;
    pkt->need_ack = need_ack;
}

/*============================================================================
 *                              数据发送函数
 *============================================================================*/

// BLE写操作辅助宏（检查返回值）
#define BLE_WRITE_CHAR(if_, cid_, handle_, data_, len_, write_type_) \
    do { \
        esp_gatt_status_t _rc = esp_ble_gattc_write_char((if_), (cid_), (handle_), \
            (len_), (uint8_t*)(data_), (write_type_), ESP_GATT_AUTH_REQ_NONE); \
        if (_rc != ESP_GATT_OK) { \
            ESP_LOGW(GATTC_TAG, "BLE写失败: handle=%d rc=0x%x", (handle_), _rc); \
        } \
    } while(0)

static void send_joystick_data(void) {
    // 读取真实摇杆值
    uint8_t x, y, btn;
    joystick_read(&x, &y, &btn);
    
    // 发送摇杆数据
    owl_joystick_pkt_t pkt;
    build_joystick_packet(&pkt, x, y, btn);
    ESP_LOGD(GATTC_TAG, ">>> 发送 JOYSTICK 包 (真实摇杆)");
    ESP_LOGD(GATTC_TAG, "    X=%d, Y=%d, 按键=%s", pkt.x_axis, pkt.y_axis, pkt.button_flags ? "按下" : "松开");
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

static void send_switch_data(uint8_t sw1, uint8_t sw2) {
    owl_switch_pkt_t pkt;
    build_switch_packet(&pkt, sw1, sw2);
    ESP_LOGD(GATTC_TAG, ">>> 发送 SWITCH 包 (矩阵按键)");
    ESP_LOGD(GATTC_TAG, "    开关1=0x%02X, 开关2=0x%02X", pkt.switch1, pkt.switch2);
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

/*============================================================================
 *                              模式切换命令发送
 *============================================================================*/

static void send_mode_command(uint8_t cmd, uint8_t param) {
    if (!connect) return;
    owl_command_pkt_t pkt;
    build_command_packet(&pkt, cmd, param, 1);
    BLE_WRITE_CHAR(gl_profile.gattc_if,
                   gl_profile.conn_id,
                   char_control_handle,
                   &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
    ESP_LOGI(GATTC_TAG, "[模式] 发送命令: 0x%02X param=%d", cmd, param);
}

// 组合键检测与处理
static void check_combo_keys(uint8_t switch1, uint8_t switch2) {
    // 检测组合键：同一帧中两个或以上按键按下
    uint8_t pressed_count = 0;
    uint8_t combo = 0;
    
    if (switch1 & OWL_SW_UP)    { pressed_count++; combo |= 0x01; }
    if (switch1 & OWL_SW_DOWN)  { pressed_count++; combo |= 0x02; }
    if (switch1 & OWL_SW_LEFT)  { pressed_count++; combo |= 0x04; }
    if (switch1 & OWL_SW_RIGHT) { pressed_count++; combo |= 0x08; }
    if (switch1 & OWL_SW_CENTER){ pressed_count++; combo |= 0x10; }
    
    if (pressed_count < 2) return;  // 不是组合键
    
    ESP_LOGI(GATTC_TAG, "[组合键] 检测到: UP=%d DOWN=%d LEFT=%d RIGHT=%d CENTER=%d",
             (switch1 & OWL_SW_UP) ? 1 : 0,
             (switch1 & OWL_SW_DOWN) ? 1 : 0,
             (switch1 & OWL_SW_LEFT) ? 1 : 0,
             (switch1 & OWL_SW_RIGHT) ? 1 : 0,
             (switch1 & OWL_SW_CENTER) ? 1 : 0);
    
    // 上+下 = 安防模式
    if ((switch1 & OWL_SW_UP) && (switch1 & OWL_SW_DOWN)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_SECURITY);
        g_current_mode = OWL_MODE_SECURITY;
        ws2812_update_connection(true);
    }
    // 左+右 = 遥控模式
    else if ((switch1 & OWL_SW_LEFT) && (switch1 & OWL_SW_RIGHT)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_REMOTE);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
    }
    // 上+中 = 预设模式
    else if ((switch1 & OWL_SW_UP) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_PRESET);
        g_current_mode = OWL_MODE_PRESET;
        ws2812_update_connection(true);
    }
    // 下+中 = 开始/停止录制
    else if ((switch1 & OWL_SW_DOWN) && (switch1 & OWL_SW_CENTER)) {
        if (g_current_mode == OWL_MODE_RECORD) {
            send_mode_command(OWL_CMD_RECORD_STOP, 0);
            g_current_mode = OWL_MODE_REMOTE;
        } else {
            send_mode_command(OWL_CMD_RECORD_START, 0);
            g_current_mode = OWL_MODE_RECORD;
        }
        ws2812_update_connection(true);
    }
    // 左+中 = 执行预设1
    else if ((switch1 & OWL_SW_LEFT) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_PRESET_START, 0);
    }
    // 右+中 = 执行预设2
    else if ((switch1 & OWL_SW_RIGHT) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_PRESET_START, 1);
    }
}

// 处理猫头鹰发来的事件包（非阻塞，仅更新状态）
static void handle_event_packet(uint8_t *data, uint16_t len) {
    if (len < 7) return;
    
    uint8_t event_type = data[3];  // 跳过3字节header
    uint8_t p1 = data[4];
    uint8_t p2 = data[5];
    uint8_t p3 = data[6];
    
    switch (event_type) {
    case OWL_EVENT_INTRUSION_DETECTED:
        ESP_LOGW(GATTC_TAG, "[事件] 检测到入侵! 目标数=%d", p3);
        // LED2闪红色（非阻塞：仅设置颜色，由主循环管理闪烁时序）
        ws2812_set_led(LED_KEY, COLOR_RED);
        break;
        
    case OWL_EVENT_INTRUSION_LOST:
        ESP_LOGI(GATTC_TAG, "[事件] 入侵者消失");
        ws2812_set_led(LED_KEY, COLOR_OFF);
        break;
        
    case OWL_EVENT_PRESET_COMPLETED:
        ESP_LOGI(GATTC_TAG, "[事件] 预设%d执行完成", p1);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_RECORD_STARTED:
        ESP_LOGI(GATTC_TAG, "[事件] 录制开始，槽位%d", p1);
        g_current_mode = OWL_MODE_RECORD;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_RECORD_STOPPED:
        ESP_LOGI(GATTC_TAG, "[事件] 录制停止，保存%d帧到槽位%d", (p2 << 8) | p3, p1);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_MODE_CHANGED:
        ESP_LOGI(GATTC_TAG, "[事件] 模式已切换为: %d", p1);
        g_current_mode = (owl_mode_t)p1;
        ws2812_update_connection(true);
        break;
        
    default:
        ESP_LOGI(GATTC_TAG, "[事件] 未知事件: 0x%02X", event_type);
        break;
    }
}

static void send_heartbeat_data(void) {
    owl_heartbeat_pkt_t pkt;
    build_heartbeat_packet(&pkt, 0x00, 37);  // 正常状态，3.7V
    ESP_LOGD(GATTC_TAG, ">>> 发送 HEARTBEAT 包");
    ESP_LOGD(GATTC_TAG, "    状态=0x%02X, 电池=%d.%dV", pkt.status, pkt.battery/10, pkt.battery%10);
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

// 定时器回调：仅发送心跳数据（esp_timer回调在ISR上下文，不能做ADC读取）
static void send_test_data(void *arg) {
    if (!connect || char_control_handle == INVALID_HANDLE) {
        return;
    }
    
    send_heartbeat_data();
}

// 按键扫描任务（同时负责周期性发送摇杆数据）
static void key_scan_task(void *arg) {
    uint8_t last_sw1 = 0, last_sw2 = 0;
    uint8_t last_btn = 0;
    uint8_t combo_cooldown = 0;  // 组合键冷却计数器（防止连续触发）
    uint8_t joystick_send_counter = 0;  // 摇杆发送计数器（每50次=500ms发一次）
    static uint8_t comm_led_counter = 0;
    
    while (1) {
        if (connect && char_control_handle != INVALID_HANDLE) {
            uint8_t sw1, sw2;
            bool changed = matrix_key_scan(&sw1, &sw2);
            
            // 读取摇杆按键状态
            uint8_t x, y, btn;
            joystick_read(&x, &y, &btn);
            
            // 周期性发送摇杆数据（每500ms，替代定时器中的摇杆发送）
            joystick_send_counter++;
            if (joystick_send_counter >= 50) {
                joystick_send_counter = 0;
                send_joystick_data();
            }
            
            // 更新按键LED状态
            ws2812_update_key(sw1, sw2, btn);
            
            // 按键变化时立即发送
            if (changed && (sw1 != last_sw1 || sw2 != last_sw2)) {
                // 检测组合键（先于普通发送）
                uint8_t pressed_count = 0;
                if (sw1 & OWL_SW_UP) pressed_count++;
                if (sw1 & OWL_SW_DOWN) pressed_count++;
                if (sw1 & OWL_SW_LEFT) pressed_count++;
                if (sw1 & OWL_SW_RIGHT) pressed_count++;
                if (sw1 & OWL_SW_CENTER) pressed_count++;
                
                if (pressed_count >= 2 && combo_cooldown == 0) {
                    check_combo_keys(sw1, sw2);
                    combo_cooldown = 30;  // 300ms冷却（30次×10ms扫描周期）
                    last_sw1 = sw1;
                    last_sw2 = sw2;
                } else if (pressed_count < 2) {
                    // 发送开关状态
                    ESP_LOGI(GATTC_TAG, "  发送SWITCH包: switch1=0x%02X switch2=0x%02X", sw1, sw2);
                    send_switch_data(sw1, sw2);
                    last_sw1 = sw1;
                    last_sw2 = sw2;
                    
                    // 通信LED闪烁
                    ws2812_update_comm(true);
                    comm_led_counter = 5;  // 50ms后自动关闭 (5次×10ms)
                }
            }
            
            // 摇杆按键变化
            if (btn != last_btn) {
                last_btn = btn;
            }
            
            // 冷却计数器递减
            if (combo_cooldown > 0) combo_cooldown--;
        }
        
        if (comm_led_counter > 0) {
            comm_led_counter--;
            if (comm_led_counter == 0) ws2812_update_comm(false);
        }
        
        // 10ms扫描一次
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 心跳检测任务（检测最后一次接收任何数据的时间）
static void heartbeat_monitor_task(void *arg) {
    while (1) {
        if (connect) {
            int64_t now = esp_timer_get_time() / 1000;
            int64_t last_recv = last_recv_time;
            
            // 检查接收超时（任何Notify/Indicate都更新last_recv_time）
            if (last_recv > 0 && (now - last_recv) > HEARTBEAT_TIMEOUT_MS) {
                if (!heartbeat_timeout) {
                    heartbeat_timeout = true;
                    ESP_LOGW(GATTC_TAG, "接收超时！服务端可能已断线");
                    ws2812_update_connection(false);  // LED变红色
                }
            } else {
                heartbeat_timeout = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒检查一次
    }
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
            
            if (adv_name != NULL) {
                size_t target_len = strlen(target_device_name);
                if (adv_name_len == target_len &&
                    strncmp((char*)adv_name, target_device_name, target_len) == 0) {
                ESP_LOGI(GATTC_TAG, "发现目标设备: %s", target_device_name);
                ESP_LOGI(GATTC_TAG, "地址: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->scan_rst.bda));
                
                // 检查是否已绑定该设备
                if (g_is_bound) {
                    if (memcmp(param->scan_rst.bda, g_bound_mac, 6) == 0) {
                        ESP_LOGI(GATTC_TAG, "✅ 发现已绑定的猫头鹰，准备连接");
                    } else {
                        ESP_LOGW(GATTC_TAG, "⚠️ 发现其他猫头鹰，跳过 (已绑定其他设备)");
                        return;  // 跳过未绑定的设备
                    }
                } else {
                    ESP_LOGI(GATTC_TAG, "未绑定状态，将绑定此设备");
                }
                
                // 停止扫描并连接
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(gl_profile.gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
                }
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
        ESP_LOGI(GATTC_TAG, "猫头鹰 MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->open.remote_bda));
        gl_profile.conn_id = param->open.conn_id;
        memcpy(gl_profile.remote_bda, param->open.remote_bda, sizeof(esp_bd_addr_t));
        
        // 首次连接，保存绑定信息
        if (!g_is_bound) {
            save_bound_mac(param->open.remote_bda);
        }
        
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
                    
                    // 获取反馈通道特征值 (Notify)
                    char_uuid.uuid.uuid16 = REMOTE_CHAR_FEEDBACK_UUID;
                    uint16_t fb_count = count;
                    status = esp_ble_gattc_get_char_by_uuid(
                        gl_profile.gattc_if, gl_profile.conn_id,
                        gl_profile.service_start_handle, gl_profile.service_end_handle,
                        char_uuid, char_elem_result, &fb_count);
                    
                    if (status == ESP_GATT_OK && fb_count > 0) {
                        char_feedback_handle = char_elem_result[0].char_handle;
                        ESP_LOGI(GATTC_TAG, "反馈通道特征值: handle=%d", char_feedback_handle);
                        
                        // 获取CCC描述符句柄并注册Notify
                        esp_gattc_descr_elem_t *descr_result = 
                            (esp_gattc_descr_elem_t*)malloc(sizeof(esp_gattc_descr_elem_t) * 2);
                        if (descr_result) {
                            uint16_t descr_count = 2;
                            esp_bt_uuid_t ccc_uuid = {
                                .len = ESP_UUID_LEN_16,
                                .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
                            };
                            status = esp_ble_gattc_get_descr_by_char_handle(
                                gl_profile.gattc_if, gl_profile.conn_id,
                                char_feedback_handle, ccc_uuid,
                                descr_result, &descr_count);
                            if (status == ESP_GATT_OK && descr_count > 0) {
                                char_feedback_ccc_handle = descr_result[0].handle;
                                // 注册Notify
                                uint8_t notify_val[2] = {0x01, 0x00};  // Enable notification
                                esp_ble_gattc_write_char_descr(
                                    gl_profile.gattc_if, gl_profile.conn_id,
                                    char_feedback_ccc_handle,
                                    sizeof(notify_val), notify_val,
                                    ESP_GATT_WRITE_TYPE_NO_RSP,
                                    ESP_GATT_AUTH_REQ_NONE);
                                ESP_LOGI(GATTC_TAG, "已注册Notify (CCC handle=%d)", char_feedback_ccc_handle);
                            }
                            free(descr_result);
                        }
                    }
                    
                    // 获取系统命令特征值
                    char_uuid.uuid.uuid16 = REMOTE_CHAR_COMMAND_UUID;
                    uint16_t cmd_count = count;
                    status = esp_ble_gattc_get_char_by_uuid(
                        gl_profile.gattc_if, gl_profile.conn_id,
                        gl_profile.service_start_handle, gl_profile.service_end_handle,
                        char_uuid, char_elem_result, &cmd_count);
                    
                    if (status == ESP_GATT_OK && cmd_count > 0) {
                        char_command_handle = char_elem_result[0].char_handle;
                        ESP_LOGI(GATTC_TAG, "系统命令特征值: handle=%d", char_command_handle);
                    }
                    
                    free(char_elem_result);
                }
            }
            
            // 启动测试定时器和按键扫描任务
            if (char_control_handle != INVALID_HANDLE) {
                connect = true;
                ESP_LOGI(GATTC_TAG, "开始发送测试数据...");
                
                // 初始化接收时间
                last_recv_time = esp_timer_get_time() / 1000;
                heartbeat_timeout = false;
                
                // 更新LED状态：已连接
                ws2812_update_connection(true);
                ws2812_update_battery(70);  // 模拟电量70%
                ws2812_pairing_animation();
                
                esp_timer_create_args_t timer_args = {
                    .callback = send_test_data,
                    .arg = NULL,
                    .name = "test_timer"
                };
                esp_err_t timer_ret = esp_timer_create(&timer_args, &test_timer);
                if (timer_ret != ESP_OK) {
                    ESP_LOGE(GATTC_TAG, "定时器创建失败: %s", esp_err_to_name(timer_ret));
                    test_timer = NULL;
                } else {
                    timer_ret = esp_timer_start_periodic(test_timer, 500000);
                    if (timer_ret != ESP_OK) {
                        ESP_LOGE(GATTC_TAG, "定时器启动失败: %s", esp_err_to_name(timer_ret));
                        esp_timer_delete(test_timer);
                        test_timer = NULL;
                    }
                }
                
                // 启动按键扫描任务（只启动一次）
                static bool key_task_started = false;
                if (!key_task_started) {
                    xTaskCreate(key_scan_task, "key_scan", 2048, NULL, 5, NULL);
                    key_task_started = true;
                }
                
                // 启动心跳检测任务（只启动一次）
                static bool heartbeat_task_started = false;
                if (!heartbeat_task_started) {
                    xTaskCreate(heartbeat_monitor_task, "heartbeat_monitor", 2048, NULL, 5, NULL);
                    heartbeat_task_started = true;
                }
            }
        }
        break;
        
    case ESP_GATTC_NOTIFY_EVT:
        // 更新接收时间（用于心跳超时检测）
        last_recv_time = esp_timer_get_time() / 1000;
        
        if (param->notify.handle == char_control_handle || param->notify.handle == char_feedback_handle) {
            uint8_t *p_data = param->notify.value;
            uint16_t p_data_len = param->notify.value_len;
            
            if (p_data_len < 3) break;
            
            uint8_t pkt_type = OWL_GET_PKT_TYPE((owl_packet_header_t*)p_data);
            
            switch (pkt_type) {
            case OWL_PKT_RADAR_STATUS:
                if (p_data_len >= 7) {
                    ESP_LOGI(GATTC_TAG, "  [雷达状态] 目标数=%d x=%d y=%d z=%d",
                             p_data[3], p_data[4], p_data[5], p_data[6]);
                }
                break;
            case OWL_PKT_EVENT:
                handle_event_packet(p_data, p_data_len);
                break;
            case OWL_PKT_ACK:
                if (p_data_len >= sizeof(owl_ack_pkt_t)) {
                    ESP_LOGI(GATTC_TAG, "  [ACK] cmd=0x%02X result=%d", p_data[3], p_data[4]);
                }
                break;
            default:
                break;
            }
        }
        break;
        
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "断开连接");
        connect = false;
        get_server = false;
        char_control_handle = INVALID_HANDLE;
        char_feedback_handle = INVALID_HANDLE;
        char_command_handle = INVALID_HANDLE;
        char_feedback_ccc_handle = INVALID_HANDLE;
        last_recv_time = 0;
        // 重置各序列号
        seq_joystick = 0;
        seq_heartbeat = 0;
        seq_switch = 0;
        seq_command = 0;
        
        if (test_timer != NULL) {
            esp_timer_stop(test_timer);
            esp_timer_delete(test_timer);
            test_timer = NULL;
        }
        
        // 更新LED状态：未连接
        ws2812_update_connection(false);
        
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

    // 加载绑定信息
    load_bound_mac();

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

    // 初始化摇杆ADC
    joystick_adc_init();

    // 初始化矩阵按键
    matrix_key_init();

    // 初始化WS2812 LED
    ws2812_init();
    ws2812_startup_animation();

    ESP_LOGI(GATTC_TAG, "猫头鹰 BLE 客户端初始化完成");
    ESP_LOGI(GATTC_TAG, "目标设备: %s", target_device_name);
}
