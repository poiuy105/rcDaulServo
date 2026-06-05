/*
 * ????? BLE GATT Client (????)
 * ?? ESP-IDF ??????
 * 
 * ???
 * - ??????????
 * - ?????????????????
 * - ?? BLE ????
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
#include "esp_task_wdt.h"

#define GATTC_TAG "OWL_CLIENT"

// ??????
#define HEARTBEAT_TIMEOUT_MS    3000    // 3????????
#define RECONNECT_DELAY_MS      2000    // ???2?????

/*=== ????? ===*/
#define WDT_TIMEOUT_S           5

// ??????
static volatile bool heartbeat_timeout = false;

/*============================================================================
 *                              ??????
 *============================================================================*/
static volatile owl_mode_t g_current_mode = OWL_MODE_REMOTE;

/*============================================================================
 *                              WS2812 LED ??
 *============================================================================*/

#define WS2812_GPIO         10
#define WS2812_LED_NUM      4
#define WS2812_BRIGHTNESS   50  // 0-255, ????????

// LED??
#define LED_CONN            0   // ????
#define LED_BATTERY         1   // ????
#define LED_KEY             2   // ????
#define LED_COMM            3   // ????

// LED???? (RGB)
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_color_t;

// ?????
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

// LED??
static volatile led_color_t led_colors[WS2812_LED_NUM];
static led_strip_handle_t led_strip = NULL;
static volatile bool led_dirty = false;  // ??LED???????????

// LED?????????????????????
typedef enum {
    LED_ANIM_NONE = 0,
    LED_ANIM_PAIRING,
} led_anim_type_t;
static volatile led_anim_type_t led_anim_type = LED_ANIM_NONE;
static volatile uint8_t led_anim_step = 0;
static volatile bool led_anim_request = false;

// ???WS2812
static void ws2812_init(void) {
    // RMT TX????
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
    
    // ?????????
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_colors[i] = COLOR_OFF;
    }
    led_strip_clear(led_strip);
    
    ESP_LOGI(GATTC_TAG, "WS2812 LED?????");
    ESP_LOGI(GATTC_TAG, "  GPIO: %d, LED??: %d", WS2812_GPIO, WS2812_LED_NUM);
}

// ????LED???????????dirty?led_task???
static void ws2812_set_led(int index, led_color_t color) {
    if (index < 0 || index >= WS2812_LED_NUM) return;
    led_colors[index] = color;
    led_dirty = true;
}

// ????LED???????????????
static void ws2812_refresh_hw(void) {
    if (led_strip == NULL) return;
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_strip_set_pixel(led_strip, i,
                            (led_colors[i].r * WS2812_BRIGHTNESS) / 255,
                            (led_colors[i].g * WS2812_BRIGHTNESS) / 255,
                            (led_colors[i].b * WS2812_BRIGHTNESS) / 255);
    }
    led_strip_refresh(led_strip);
}

// ????LED???????
static void ws2812_all_off(void) {
    for (int i = 0; i < WS2812_LED_NUM; i++) {
        led_colors[i] = COLOR_OFF;
    }
    led_dirty = true;
}

// ??????????
static void ws2812_startup_animation(void) {
    led_color_t rainbow[] = {
        {255, 0, 0},    // ?
        {255, 127, 0},  // ?
        {255, 255, 0},  // ?
        {0, 255, 0},    // ?
        {0, 0, 255},    // ?
        {75, 0, 130},   // ?
        {148, 0, 211},  // ?
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

// LED??????????
static void ws2812_update_connection(bool connected);
static void ws2812_update_battery(uint8_t battery_percent);

// ??????????????led_task??????????
static void ws2812_request_pairing_animation(void) {
    led_anim_type = LED_ANIM_PAIRING;
    led_anim_step = 0;
    led_anim_request = true;
}

// LED????????
static void ws2812_update_connection(bool connected) {
    if (!connected) {
        ws2812_set_led(LED_CONN, COLOR_RED);  // ??=???
        return;
    }
    switch (g_current_mode) {
        case OWL_MODE_REMOTE:
            ws2812_set_led(LED_CONN, COLOR_GREEN);    // ??=????
            break;
        case OWL_MODE_SECURITY:
            ws2812_set_led(LED_CONN, COLOR_RED);       // ??=????
            break;
        case OWL_MODE_PRESET:
            ws2812_set_led(LED_CONN, COLOR_BLUE);      // ??=????
            break;
        case OWL_MODE_RECORD:
            ws2812_set_led(LED_CONN, COLOR_YELLOW);    // ??=????
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

// ??????????????????
static void wdt_subscribe(void);
static inline void wdt_feed(void);

// LED????????????????RMT??????BLE?????RMT
static void led_update_task(void *arg) {
    ESP_LOGI(GATTC_TAG, "LED??????");
    wdt_subscribe();

    while (1) {
        // ????
        if (led_anim_request && led_strip != NULL) {
            if (led_anim_type == LED_ANIM_PAIRING) {
                // ?????3?????
                // ??200ms?????? = 6? ? 200ms = 1.2s
                if (led_anim_step < 6) {
                    if (led_anim_step % 2 == 0) {
                        // ???
                        for (int j = 0; j < WS2812_LED_NUM; j++) {
                            led_strip_set_pixel(led_strip, j,
                                (COLOR_WHITE.r * WS2812_BRIGHTNESS) / 255,
                                (COLOR_WHITE.g * WS2812_BRIGHTNESS) / 255,
                                (COLOR_WHITE.b * WS2812_BRIGHTNESS) / 255);
                        }
                        led_strip_refresh(led_strip);
                    } else {
                        // ?
                        led_strip_clear(led_strip);
                    }
                    led_anim_step++;
                } else {
                    // ??????????????
                    led_anim_request = false;
                    led_anim_type = LED_ANIM_NONE;
                    ws2812_update_connection(true);
                    ws2812_update_battery(70);
                    led_dirty = true;
                }
            } else {
                led_anim_request = false;
                led_anim_type = LED_ANIM_NONE;
            }
        }

        // ??dirty?LED?????
        if (led_dirty && led_strip != NULL) {
            ws2812_refresh_hw();
            led_dirty = false;
        }

        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(200));  // 200ms????
    }
}

/*============================================================================
 *                              ?? ADC ??
 *============================================================================*/

// GPIO ??
#define JOYSTICK_X_GPIO     3
#define JOYSTICK_Y_GPIO     4
#define JOYSTICK_BTN_GPIO   9

// ADC ??
#define JOYSTICK_ADC_UNIT   ADC_UNIT_1
#define JOYSTICK_X_CHANNEL  ADC_CHANNEL_3   // IO3
#define JOYSTICK_Y_CHANNEL  ADC_CHANNEL_4   // IO4

// ADC??
static adc_oneshot_unit_handle_t adc_handle = NULL;

// ?????ADC
static void joystick_adc_init(void) {
    // ???ADC??
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = JOYSTICK_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // ??X???
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_X_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_CHANNEL, &config));

    // ????GPIO
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << JOYSTICK_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_conf));

    ESP_LOGI(GATTC_TAG, "??ADC?????");
    ESP_LOGI(GATTC_TAG, "  X?: GPIO%d (ADC1_CH%d)", JOYSTICK_X_GPIO, JOYSTICK_X_CHANNEL);
    ESP_LOGI(GATTC_TAG, "  Y?: GPIO%d (ADC1_CH%d)", JOYSTICK_Y_GPIO, JOYSTICK_Y_CHANNEL);
    ESP_LOGI(GATTC_TAG, "  ??: GPIO%d", JOYSTICK_BTN_GPIO);
}

// ?????
static void joystick_read(uint8_t *x, uint8_t *y, uint8_t *btn) {
    int raw_x = 2048, raw_y = 2048;  // ?????

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
 *                              ??????
 *============================================================================*/

// ??????
#define MATRIX_ROW1_GPIO    2
#define MATRIX_ROW2_GPIO    8
static const int matrix_rows[] = {MATRIX_ROW1_GPIO, MATRIX_ROW2_GPIO};
#define MATRIX_ROWS         2

// ?????????
#define MATRIX_COL1_GPIO    6   // ?
#define MATRIX_COL2_GPIO    5   // ?
#define MATRIX_COL3_GPIO    7   // ?
#define MATRIX_COL4_GPIO    1   // ?
#define MATRIX_COL5_GPIO    0   // ?
static const int matrix_cols[] = {MATRIX_COL1_GPIO, MATRIX_COL2_GPIO, MATRIX_COL3_GPIO, MATRIX_COL4_GPIO, MATRIX_COL5_GPIO};
#define MATRIX_COLS         5

// ????
static uint8_t matrix_key_state[MATRIX_ROWS][MATRIX_COLS] = {0};
static uint8_t matrix_key_last_state[MATRIX_ROWS][MATRIX_COLS] = {0};
// ?????????????N???????
static uint8_t matrix_debounce_count[MATRIX_ROWS][MATRIX_COLS] = {0};
#define DEBOUNCE_THRESHOLD  3  // ??3?(30ms)?????

// ???????
static void matrix_key_init(void) {
    // ???????
    gpio_config_t row_conf = {
        .pin_bit_mask = (1ULL << MATRIX_ROW1_GPIO) | (1ULL << MATRIX_ROW2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&row_conf));
    
    // ???????????
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
    
    // ?????????
    gpio_set_level(MATRIX_ROW1_GPIO, 1);
    gpio_set_level(MATRIX_ROW2_GPIO, 1);
    
    ESP_LOGI(GATTC_TAG, "?????????");
    ESP_LOGI(GATTC_TAG, "  ??: GPIO%d, GPIO%d", MATRIX_ROW1_GPIO, MATRIX_ROW2_GPIO);
    ESP_LOGI(GATTC_TAG, "  ??: GPIO%d, GPIO%d, GPIO%d, GPIO%d, GPIO%d",
             MATRIX_COL1_GPIO, MATRIX_COL2_GPIO, MATRIX_COL3_GPIO, MATRIX_COL4_GPIO, MATRIX_COL5_GPIO);
}

// ?????????????????????
static bool matrix_key_scan(uint8_t *switch1, uint8_t *switch2) {
    bool changed = false;
    uint8_t raw_state[MATRIX_ROWS][MATRIX_COLS] = {0};
    
    // ???????
    for (int row = 0; row < MATRIX_ROWS; row++) {
        gpio_set_level(matrix_rows[row], 0);
        esp_rom_delay_us(10);
        for (int col = 0; col < MATRIX_COLS; col++) {
            raw_state[row][col] = (gpio_get_level(matrix_cols[col]) == 0) ? 1 : 0;
        }
        gpio_set_level(matrix_rows[row], 1);
    }
    
    // ????????????DEBOUNCE_THRESHOLD????
    for (int row = 0; row < MATRIX_ROWS; row++) {
        for (int col = 0; col < MATRIX_COLS; col++) {
            if (raw_state[row][col] == matrix_key_state[row][col]) {
                // ?????????
                matrix_debounce_count[row][col] = 0;
            } else {
                // ?????????
                matrix_debounce_count[row][col]++;
                if (matrix_debounce_count[row][col] >= DEBOUNCE_THRESHOLD) {
                    // ??N??????????
                    matrix_key_last_state[row][col] = matrix_key_state[row][col];
                    matrix_key_state[row][col] = raw_state[row][col];
                    matrix_debounce_count[row][col] = 0;
                    changed = true;
                }
            }
        }
    }
    
    // ???switch1/switch2??
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
 *                              GATT ??
 *============================================================================*/

#define PROFILE_NUM             1
#define PROFILE_APP_ID          0
#define INVALID_HANDLE          0

// ?????
static char target_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = OWL_DEVICE_NAME;

// ???????? UUID
#define REMOTE_SERVICE_UUID         0xFF00
#define REMOTE_CHAR_CONTROL_UUID    0xFF01
#define REMOTE_CHAR_FEEDBACK_UUID   0xFF02
#define REMOTE_CHAR_COMMAND_UUID    0xFF03

/*============================================================================
 *                              ????
 *============================================================================*/

static volatile bool connect = false;
static volatile bool get_server = false;

// ??????????????????????
static volatile uint8_t seq_joystick = 0;
static volatile uint8_t seq_heartbeat = 0;
static volatile uint8_t seq_switch = 0;
static volatile uint8_t seq_command = 0;

// ?????
static volatile uint16_t char_control_handle = INVALID_HANDLE;
static volatile uint16_t char_feedback_handle = INVALID_HANDLE;
static volatile uint16_t char_command_handle = INVALID_HANDLE;
static volatile uint16_t char_feedback_ccc_handle = INVALID_HANDLE;

// ?????????
static esp_timer_handle_t test_timer;

// ?????????????????
static volatile int64_t last_recv_time = 0;

/*============================================================================
 *                              MAC ????
 *============================================================================*/

// ????
static volatile bool g_is_bound = false;
static volatile uint8_t g_bound_mac[6] = {0};

// ????? MAC ??
static void load_bound_mac(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    size_t length = 6;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(GATTC_TAG, "NVS ??????????");
        g_is_bound = false;
        return;
    }
    
    uint8_t tmp_mac[6] = {0};
    err = nvs_get_blob(nvs_handle, OWL_KEY_REMOTE_MAC, tmp_mac, &length);
    if (err == ESP_OK && length == 6) {
        memcpy((void*)g_bound_mac, tmp_mac, 6);
        g_is_bound = true;
        ESP_LOGI(GATTC_TAG, "?????? MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(tmp_mac));
    } else {
        g_is_bound = false;
        ESP_LOGI(GATTC_TAG, "??????????????????");
    }
    
    nvs_close(nvs_handle);
}

// ????? MAC ??
static void save_bound_mac(uint8_t *mac) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "NVS ????");
        return;
    }
    
    err = nvs_set_blob(nvs_handle, OWL_KEY_REMOTE_MAC, mac, 6);
    if (err != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "MAC????: %s", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "MAC commit??: %s", esp_err_to_name(err));
        } else {
            memcpy((void*)g_bound_mac, mac, 6);  // volatile??
            g_is_bound = true;
            ESP_LOGI(GATTC_TAG, "??? MAC ???: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(mac));
        }
    }
    
    nvs_close(nvs_handle);
}

// ????
static void clear_bound_mac(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open(OWL_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return;
    
    err = nvs_erase_key(nvs_handle, OWL_KEY_REMOTE_MAC);
    if (err == ESP_OK) {
        esp_err_t commit_err = nvs_commit(nvs_handle);
        if (commit_err != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "NVS commit??: %s", esp_err_to_name(commit_err));
        }
    }
    
    nvs_close(nvs_handle);
    g_is_bound = false;
    memset((void*)g_bound_mac, 0, 6);
    ESP_LOGI(GATTC_TAG, "???????");
}

/*============================================================================
 *                              ????
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
 *                              Profile ??
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
 *                              ???????
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
    pkt->change_flags = 0x03;  // ??????
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
 *                              ??????
 *============================================================================*/

// BLE?????????????
#define BLE_WRITE_CHAR(if_, cid_, handle_, data_, len_, write_type_) \
    do { \
        esp_gatt_status_t _rc = esp_ble_gattc_write_char((if_), (cid_), (handle_), \
            (len_), (uint8_t*)(data_), (write_type_), ESP_GATT_AUTH_REQ_NONE); \
        if (_rc != ESP_GATT_OK) { \
            ESP_LOGW(GATTC_TAG, "BLE???: handle=%d rc=0x%x", (handle_), _rc); \
        } \
    } while(0)

static void send_joystick_data(void) {
    // ???????
    uint8_t x, y, btn;
    joystick_read(&x, &y, &btn);
    
    // ??????
    owl_joystick_pkt_t pkt;
    build_joystick_packet(&pkt, x, y, btn);
    ESP_LOGD(GATTC_TAG, ">>> ?? JOYSTICK ? (????)");
    ESP_LOGD(GATTC_TAG, "    X=%d, Y=%d, ??=%s", pkt.x_axis, pkt.y_axis, pkt.button_flags ? "??" : "??");
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

static void send_switch_data(uint8_t sw1, uint8_t sw2) {
    owl_switch_pkt_t pkt;
    build_switch_packet(&pkt, sw1, sw2);
    ESP_LOGD(GATTC_TAG, ">>> ?? SWITCH ? (????)");
    ESP_LOGD(GATTC_TAG, "    ??1=0x%02X, ??2=0x%02X", pkt.switch1, pkt.switch2);
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

/*============================================================================
 *                              ????????
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
    ESP_LOGI(GATTC_TAG, "[??] ????: 0x%02X param=%d", cmd, param);
}

// ????????
static void check_combo_keys(uint8_t switch1, uint8_t switch2) {
    // ???????????????????
    uint8_t pressed_count = 0;
    uint8_t combo = 0;
    
    if (switch1 & OWL_SW_UP)    { pressed_count++; combo |= 0x01; }
    if (switch1 & OWL_SW_DOWN)  { pressed_count++; combo |= 0x02; }
    if (switch1 & OWL_SW_LEFT)  { pressed_count++; combo |= 0x04; }
    if (switch1 & OWL_SW_RIGHT) { pressed_count++; combo |= 0x08; }
    if (switch1 & OWL_SW_CENTER){ pressed_count++; combo |= 0x10; }
    
    if (pressed_count < 2) return;  // ?????
    
    ESP_LOGI(GATTC_TAG, "[???] ???: UP=%d DOWN=%d LEFT=%d RIGHT=%d CENTER=%d",
             (switch1 & OWL_SW_UP) ? 1 : 0,
             (switch1 & OWL_SW_DOWN) ? 1 : 0,
             (switch1 & OWL_SW_LEFT) ? 1 : 0,
             (switch1 & OWL_SW_RIGHT) ? 1 : 0,
             (switch1 & OWL_SW_CENTER) ? 1 : 0);
    
    // ?+? = ????
    if ((switch1 & OWL_SW_UP) && (switch1 & OWL_SW_DOWN)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_SECURITY);
        g_current_mode = OWL_MODE_SECURITY;
        ws2812_update_connection(true);
    }
    // ?+? = ????
    else if ((switch1 & OWL_SW_LEFT) && (switch1 & OWL_SW_RIGHT)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_REMOTE);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
    }
    // ?+? = ????
    else if ((switch1 & OWL_SW_UP) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_MODE_SWITCH, OWL_MODE_PRESET);
        g_current_mode = OWL_MODE_PRESET;
        ws2812_update_connection(true);
    }
    // ?+? = ??/????
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
    // ?+? = ????1
    else if ((switch1 & OWL_SW_LEFT) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_PRESET_START, 0);
    }
    // ?+? = ????2
    else if ((switch1 & OWL_SW_RIGHT) && (switch1 & OWL_SW_CENTER)) {
        send_mode_command(OWL_CMD_PRESET_START, 1);
    }
}

// ??????????????????????
static void handle_event_packet(uint8_t *data, uint16_t len) {
    if (len < sizeof(owl_event_pkt_t)) return;
    
    owl_event_pkt_t *evt = (owl_event_pkt_t*)data;
    
    switch (evt->event_type) {
    case OWL_EVENT_INTRUSION_DETECTED:
        ESP_LOGW(GATTC_TAG, "[??] ?????! ???=%d", evt->p3);
        // LED2?????????????????????????
        ws2812_set_led(LED_KEY, COLOR_RED);
        break;
        
    case OWL_EVENT_INTRUSION_LOST:
        ESP_LOGI(GATTC_TAG, "[??] ?????");
        ws2812_set_led(LED_KEY, COLOR_OFF);
        break;
        
    case OWL_EVENT_PRESET_COMPLETED:
        ESP_LOGI(GATTC_TAG, "[??] ??%d????", evt->p1);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_RECORD_STARTED:
        ESP_LOGI(GATTC_TAG, "[??] ???????%d", evt->p1);
        g_current_mode = OWL_MODE_RECORD;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_RECORD_STOPPED:
        ESP_LOGI(GATTC_TAG, "[??] ???????%d????%d", (evt->p2 << 8) | evt->p3, evt->p1);
        g_current_mode = OWL_MODE_REMOTE;
        ws2812_update_connection(true);
        break;
        
    case OWL_EVENT_MODE_CHANGED:
        if (evt->p1 <= OWL_MODE_RECORD) {
            ESP_LOGI(GATTC_TAG, "[??] ??????: %d", evt->p1);
            g_current_mode = (owl_mode_t)evt->p1;
        } else {
            ESP_LOGW(GATTC_TAG, "[??] ?????: %d", evt->p1);
        }
        ws2812_update_connection(true);
        break;
        
    default:
        ESP_LOGI(GATTC_TAG, "[??] ????: 0x%02X", evt->event_type);
        break;
    }
}

static void send_heartbeat_data(void) {
    owl_heartbeat_pkt_t pkt;
    build_heartbeat_packet(&pkt, 0x00, 37);  // ?????3.7V
    ESP_LOGD(GATTC_TAG, ">>> ?? HEARTBEAT ?");
    ESP_LOGD(GATTC_TAG, "    ??=0x%02X, ??=%d.%dV", pkt.status, pkt.battery/10, pkt.battery%10);
    BLE_WRITE_CHAR(gl_profile.gattc_if, gl_profile.conn_id,
                   char_control_handle, &pkt, sizeof(pkt),
                   ESP_GATT_WRITE_TYPE_NO_RSP);
}

/**
 * @brief ?????????????TWDT???????
 */
static void wdt_init(void) {
    // ESP-IDF v5.4 sdkconfig?CONFIG_ESP_TASK_WDT=y??????TWDT
    ESP_LOGI(GATTC_TAG, "TWDT??=%ds??sdkconfig??????", WDT_TIMEOUT_S);
}

/**
 * @brief ??????TWDT???????????
 */
static void wdt_subscribe(void) {
    esp_err_t ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "TWDT????: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief ???????????????????
 */
static inline void wdt_feed(void) {
    esp_task_wdt_reset();
}

// ??????????????esp_timer???ISR???????ADC???
static void send_test_data(void *arg) {
    if (!connect || char_control_handle == INVALID_HANDLE) {
        return;
    }
    
    send_heartbeat_data();
}

// ?????????????????????
static void key_scan_task(void *arg) {
    uint8_t last_sw1 = 0, last_sw2 = 0;
    uint8_t last_btn = 0;
    uint8_t combo_cooldown = 0;  // ????????????????
    uint8_t joystick_send_counter = 0;  // ?????????50?=500ms????
    static uint8_t comm_led_counter = 0;
    
    wdt_subscribe();
    
    while (1) {
        if (connect && char_control_handle != INVALID_HANDLE) {
            uint8_t sw1, sw2;
            bool changed = matrix_key_scan(&sw1, &sw2);
            
            // ????????
            uint8_t x, y, btn;
            joystick_read(&x, &y, &btn);
            
            // ???????????500ms?????????????
            joystick_send_counter++;
            if (joystick_send_counter >= 50) {
                joystick_send_counter = 0;
                send_joystick_data();
            }
            
            // ????LED??
            ws2812_update_key(sw1, sw2, btn);
            
            // ?????????
            if (changed && (sw1 != last_sw1 || sw2 != last_sw2)) {
                // ?????????????
                uint8_t pressed_count = 0;
                if (sw1 & OWL_SW_UP) pressed_count++;
                if (sw1 & OWL_SW_DOWN) pressed_count++;
                if (sw1 & OWL_SW_LEFT) pressed_count++;
                if (sw1 & OWL_SW_RIGHT) pressed_count++;
                if (sw1 & OWL_SW_CENTER) pressed_count++;
                
                if (pressed_count >= 2 && combo_cooldown == 0) {
                    check_combo_keys(sw1, sw2);
                    combo_cooldown = 30;  // 300ms???30??10ms?????
                    last_sw1 = sw1;
                    last_sw2 = sw2;
                } else if (pressed_count < 2) {
                    // ??????
                    ESP_LOGI(GATTC_TAG, "  ??SWITCH?: switch1=0x%02X switch2=0x%02X", sw1, sw2);
                    send_switch_data(sw1, sw2);
                    last_sw1 = sw1;
                    last_sw2 = sw2;
                    
                    // ??LED??
                    ws2812_update_comm(true);
                    comm_led_counter = 5;  // 50ms????? (5??10ms)
                }
            }
            
            // ??????
            if (btn != last_btn) {
                last_btn = btn;
            }
            
            // ???????
            if (combo_cooldown > 0) combo_cooldown--;
        }
        
        if (comm_led_counter > 0) {
            comm_led_counter--;
            if (comm_led_counter == 0) ws2812_update_comm(false);
        }
        
        // 10ms????
        wdt_feed();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ???????????????????????
static void heartbeat_monitor_task(void *arg) {
    wdt_subscribe();
    
    while (1) {
        if (connect) {
            int64_t now = esp_timer_get_time() / 1000;
            int64_t last_recv = last_recv_time;
            
            // ?????????Notify/Indicate???last_recv_time?
            if (last_recv > 0 && (now - last_recv) > HEARTBEAT_TIMEOUT_MS) {
                if (!heartbeat_timeout) {
                    heartbeat_timeout = true;
                    ESP_LOGW(GATTC_TAG, "?????????????");
                    ws2812_update_connection(false);  // LED???
                }
            } else {
                heartbeat_timeout = false;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // ??????
        wdt_feed();
    }
}

/*============================================================================
 *                              GAP ????
 *============================================================================*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(30);  // ?? 30 ?
        break;
        
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "??????");
            break;
        }
        ESP_LOGI(GATTC_TAG, "????...");
        break;
        
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // ?????
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            
            adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv,
                                                 ESP_BLE_AD_TYPE_NAME_CMPL,
                                                 &adv_name_len);
            
            if (adv_name != NULL) {
                size_t target_len = strlen(target_device_name);
                if (adv_name_len == target_len &&
                    strncmp((char*)adv_name, target_device_name, target_len) == 0) {
                ESP_LOGI(GATTC_TAG, "??????: %s", target_device_name);
                ESP_LOGI(GATTC_TAG, "??: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->scan_rst.bda));
                
                // ??????????
                if (g_is_bound) {
                    if (memcmp(param->scan_rst.bda, (const void*)g_bound_mac, 6) == 0) {
                        ESP_LOGI(GATTC_TAG, "? ??????????????");
                    } else {
                        ESP_LOGW(GATTC_TAG, "?? ?????????? (???????)");
                        return;  // ????????
                    }
                } else {
                    ESP_LOGI(GATTC_TAG, "????????????");
                }
                
                // ???????
                esp_ble_gap_stop_scanning();
                esp_gatt_status_t open_ret = esp_ble_gattc_open(gl_profile.gattc_if, param->scan_rst.bda, BLE_ADDR_TYPE_PUBLIC, true);
                if (open_ret != ESP_GATT_OK) {
                    ESP_LOGE(GATTC_TAG, "??????: 0x%x", open_ret);
                }
                }
            }
        }
        break;
        
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "????");
        break;
        
    default:
        break;
    }
}

/*============================================================================
 *                              GATT ??????????
 *============================================================================*/

static void discover_all_characteristics(void) {
    esp_gattc_char_elem_t *char_elem = NULL;
    uint16_t count = 0;
    esp_bt_uuid_t char_uuid;

    esp_ble_gattc_get_attr_count(gl_profile.gattc_if, gl_profile.conn_id,
                                 ESP_GATT_DB_CHARACTERISTIC,
                                 gl_profile.service_start_handle,
                                 gl_profile.service_end_handle,
                                 INVALID_HANDLE, &count);

    if (count == 0) return;

    char_elem = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * count);
    if (!char_elem) return;

    char_uuid.len = ESP_UUID_LEN_16;

    // ????
    char_uuid.uuid.uuid16 = REMOTE_CHAR_CONTROL_UUID;
    uint16_t cnt = count;
    if (esp_ble_gattc_get_char_by_uuid(gl_profile.gattc_if, gl_profile.conn_id,
            gl_profile.service_start_handle, gl_profile.service_end_handle,
            char_uuid, char_elem, &cnt) == ESP_GATT_OK && cnt > 0) {
        char_control_handle = char_elem[0].char_handle;
        ESP_LOGI(GATTC_TAG, "???????: handle=%d", char_control_handle);
    }

    // ????
    char_uuid.uuid.uuid16 = REMOTE_CHAR_FEEDBACK_UUID;
    cnt = count;
    if (esp_ble_gattc_get_char_by_uuid(gl_profile.gattc_if, gl_profile.conn_id,
            gl_profile.service_start_handle, gl_profile.service_end_handle,
            char_uuid, char_elem, &cnt) == ESP_GATT_OK && cnt > 0) {
        char_feedback_handle = char_elem[0].char_handle;
        ESP_LOGI(GATTC_TAG, "???????: handle=%d", char_feedback_handle);

        // ??CCC???Notify
        esp_gattc_descr_elem_t *descr = (esp_gattc_descr_elem_t*)malloc(sizeof(esp_gattc_descr_elem_t) * 2);
        if (descr) {
            uint16_t d_cnt = 2;
            esp_bt_uuid_t ccc_uuid = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
            if (esp_ble_gattc_get_descr_by_char_handle(gl_profile.gattc_if, gl_profile.conn_id,
                    char_feedback_handle, ccc_uuid, descr, &d_cnt) == ESP_GATT_OK && d_cnt > 0) {
                char_feedback_ccc_handle = descr[0].handle;
                uint8_t notify_val[2] = {0x01, 0x00};
                esp_ble_gattc_write_char_descr(gl_profile.gattc_if, gl_profile.conn_id,
                    char_feedback_ccc_handle, sizeof(notify_val), notify_val,
                    ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
                ESP_LOGI(GATTC_TAG, "???Notify (CCC handle=%d)", char_feedback_ccc_handle);
            }
            free(descr);
        }
    }

    // ????
    char_uuid.uuid.uuid16 = REMOTE_CHAR_COMMAND_UUID;
    cnt = count;
    if (esp_ble_gattc_get_char_by_uuid(gl_profile.gattc_if, gl_profile.conn_id,
            gl_profile.service_start_handle, gl_profile.service_end_handle,
            char_uuid, char_elem, &cnt) == ESP_GATT_OK && cnt > 0) {
        char_command_handle = char_elem[0].char_handle;
        ESP_LOGI(GATTC_TAG, "???????: handle=%d", char_command_handle);
    }

    free(char_elem);
}

static void start_connection_tasks(void) {
    // ???????
    last_recv_time = esp_timer_get_time() / 1000;
    heartbeat_timeout = false;

    // ??LED??????????led_task??????
    ws2812_update_connection(true);
    ws2812_update_battery(70);
    ws2812_request_pairing_animation();

    // ?????
    esp_timer_create_args_t timer_args = {
        .callback = send_test_data,
        .arg = NULL,
        .name = "test_timer"
    };
    esp_err_t timer_ret = esp_timer_create(&timer_args, &test_timer);
    if (timer_ret != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "???????: %s", esp_err_to_name(timer_ret));
        test_timer = NULL;
    } else {
        timer_ret = esp_timer_start_periodic(test_timer, 500000);
        if (timer_ret != ESP_OK) {
            ESP_LOGE(GATTC_TAG, "???????: %s", esp_err_to_name(timer_ret));
            esp_timer_delete(test_timer);
            test_timer = NULL;
        }
    }

    // ???????????????
    static bool key_task_started = false;
    if (!key_task_started) {
        xTaskCreate(key_scan_task, "key_scan", 2048, NULL, 5, NULL);
        key_task_started = true;
    }

    // ???????????????
    static bool heartbeat_task_started = false;
    if (!heartbeat_task_started) {
        xTaskCreate(heartbeat_monitor_task, "heartbeat_monitor", 2048, NULL, 5, NULL);
        heartbeat_task_started = true;
    }

    // ??LED?????????????RMT???
    static bool led_task_started = false;
    if (!led_task_started) {
        xTaskCreate(led_update_task, "led_update", 2048, NULL, 5, NULL);
        led_task_started = true;
    }
}

/*============================================================================
 *                              GATT ????
 *============================================================================*/

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, 
                                          esp_gatt_if_t gattc_if, 
                                          esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT ???????");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
        
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "????, status=%d", param->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "????");
        ESP_LOGI(GATTC_TAG, "??? MAC: " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->open.remote_bda));
        gl_profile.conn_id = param->open.conn_id;
        memcpy(gl_profile.remote_bda, param->open.remote_bda, sizeof(esp_bd_addr_t));
        
        // ???????????
        if (!g_is_bound) {
            save_bound_mac(param->open.remote_bda);
        }
        
        esp_ble_gattc_send_mtu_req(gattc_if, param->open.conn_id);
        break;
        
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU ????, MTU=%d", param->cfg_mtu.mtu);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
        
    case ESP_GATTC_SEARCH_RES_EVT:
        if (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "????: 0x%04X", REMOTE_SERVICE_UUID);
            get_server = true;
            gl_profile.service_start_handle = param->search_res.start_handle;
            gl_profile.service_end_handle = param->search_res.end_handle;
        }
        break;
        
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (get_server) {
            discover_all_characteristics();
            if (char_control_handle != INVALID_HANDLE) {
                connect = true;
                ESP_LOGI(GATTC_TAG, "????????...");
                start_connection_tasks();
            }
        }
        break;
        
    case ESP_GATTC_NOTIFY_EVT:
        // ????????????????
        last_recv_time = esp_timer_get_time() / 1000;
        
        if (param->notify.handle == char_control_handle || param->notify.handle == char_feedback_handle) {
            uint8_t *p_data = param->notify.value;
            uint16_t p_data_len = param->notify.value_len;
            
            if (p_data_len < 3) break;
            
            uint8_t pkt_type = OWL_GET_PKT_TYPE((owl_packet_header_t*)p_data);
            
            switch (pkt_type) {
            case OWL_PKT_RADAR_STATUS:
                if (p_data_len >= 7) {
                    ESP_LOGI(GATTC_TAG, "  [????] ???=%d x=%d y=%d z=%d",
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
        ESP_LOGI(GATTC_TAG, "????");
        connect = false;
        get_server = false;
        char_control_handle = INVALID_HANDLE;
        char_feedback_handle = INVALID_HANDLE;
        char_command_handle = INVALID_HANDLE;
        char_feedback_ccc_handle = INVALID_HANDLE;
        last_recv_time = 0;
        // ??????
        seq_joystick = 0;
        seq_heartbeat = 0;
        seq_switch = 0;
        seq_command = 0;
        
        if (test_timer != NULL) {
            esp_timer_stop(test_timer);
            esp_timer_delete(test_timer);
            test_timer = NULL;
        }
        
        // ??LED??????
        ws2812_update_connection(false);
        
        // ????
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
            ESP_LOGE(GATTC_TAG, "????, status=%d", param->reg.status);
            return;
        }
    }
    
    if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile.gattc_if) {
        gattc_profile_event_handler(event, gattc_if, param);
    }
}

/*============================================================================
 *                              ???
 *============================================================================*/

void app_main(void) {
    esp_err_t ret;

    // ??? NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ??????
    load_bound_mac();

    // ????????
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // ????????
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "??????????: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "?????????: %s", esp_err_to_name(ret));
        return;
    }

    // ??? Bluedroid
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "Bluedroid ?????: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "?? Bluedroid ??: %s", esp_err_to_name(ret));
        return;
    }

    // ????
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "GATTC ??????: %x", ret);
        return;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "GAP ??????: %x", ret);
        return;
    }

    // ????
    ret = esp_ble_gattc_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "??????: %x", ret);
        return;
    }

    // ?? MTU
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "?? MTU ??: %x", ret);
    }

    // ????????
    wdt_init();

    // ?????ADC
    joystick_adc_init();

    // ???????
    matrix_key_init();

    // ???WS2812 LED
    ws2812_init();
    ws2812_startup_animation();

    ESP_LOGI(GATTC_TAG, "??? BLE ????????");
    ESP_LOGI(GATTC_TAG, "????: %s", target_device_name);
}
