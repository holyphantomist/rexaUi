#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt_tx.h"
#include "esp_pm.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "world_cities.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_vfs_fat.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "sdmmc_cmd.h"
#include "rom/tjpgd.h"
#define LCD_WIDTH 320
#define LCD_HEIGHT 170
#define LCD_BYTES (LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t))
#define LCD_TRANSFER_CHUNK 4092
#define SD_MOUNT_POINT "/sdcard"
#define SD_APPS_DIRECTORY SD_MOUNT_POINT "/REXA/APPS"
#define SDMMC_CMD_PIN 39
#define SDMMC_D0_PIN 40
#define SDMMC_CLK_PIN 41
#define SD_APP_LIMIT 8
#define SD_APP_NAME_LENGTH 24
#define SD_APP_TEXT_LENGTH 72
#define SD_APP_FILENAME_LENGTH 32
#define SD_APP_FILE_LIMIT 2048
#define SD_LUA_FILE_LIMIT 16384
#define SD_LUA_MEMORY_LIMIT (128 * 1024)
#define SD_LUA_INSTRUCTION_LIMIT 50000
#define SD_LUA_HTTP_RESPONSE_LIMIT 2048

#define LCD_PIN_MOSI 13
#define LCD_PIN_SCLK 10
#define LCD_PIN_DC 11
#define LCD_PIN_CS 12
#define LCD_PIN_RST 9
#define LCD_PIN_BL 14

#define JOY_PIN_X 1
#define JOY_PIN_Y 2
#define JOY_PIN_SW 3
#define POWER_ADC_CHANNEL ADC_CHANNEL_3

#define FRAME_INTERVAL_MS 33
#define JOYSTICK_DEADZONE 850
#define BRIGHTNESS_STEP 5
#define WIFI_NETWORK_LIMIT 10
#define BLE_DEVICE_LIMIT 8
#define KEYBOARD_COLUMNS 12
#define KEYBOARD_ROWS 4
#define KEYBOARD_KEY_COUNT (KEYBOARD_COLUMNS * KEYBOARD_ROWS)
#define PASSWORD_CAPACITY 64
#define HOME_ITEM_COUNT 11
#define HOME_COLUMN_COUNT 4
#define SETTINGS_ITEM_COUNT 10
#define SETTINGS_COLUMN_COUNT 2
#define SLEEP_TIMEOUT_MAX_MINUTES 120

static const char *TAG = "rexaUi";

static spi_device_handle_t lcd_spi;
static adc_oneshot_unit_handle_t adc1;
static adc_cali_handle_t power_adc_calibration = NULL;
static nvs_handle_t settings;
static uint16_t *framebuffer;
static uint8_t brightness = 40;
static float system_voltage = 0.0f;
static bool game_over = false;
static unsigned game_score = 0;
static unsigned battery_level = 0;

typedef enum {
    WIFI_STATE_OFF,
    WIFI_STATE_STARTING,
    WIFI_STATE_SCANNING,
    WIFI_STATE_READY,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_FAILED
} wifi_state_t;

typedef enum {
    BLE_STATE_OFF,
    BLE_STATE_STARTING,
    BLE_STATE_SCANNING,
    BLE_STATE_READY,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_FAILED
} ble_state_t;

typedef struct {
    char name[33];
    esp_bd_addr_t address;
    esp_ble_addr_type_t address_type;
    int rssi;
} ble_device_t;

typedef struct {
    char filename[SD_APP_FILENAME_LENGTH];
    char title[SD_APP_NAME_LENGTH];
    bool lua_script;
} sd_app_entry_t;

typedef struct {
    char title[SD_APP_NAME_LENGTH];
    char type[12];
    char text[SD_APP_TEXT_LENGTH];
    uint16_t accent;
} sd_app_package_t;

static volatile wifi_state_t wifi_state = WIFI_STATE_STARTING;
static volatile ble_state_t ble_state = BLE_STATE_OFF;
static volatile bool wifi_enabled = true;
static volatile bool bluetooth_enabled = true;
static bool screen_flipped = false;
static bool joystick_flipped = false;
static bool bluetooth_initialized = false;
static bool bluetooth_host_initialized = false;
static bool bluetooth_controller_enabled = false;
static bool bluetooth_host_enabled = false;
static bool language_turkish = false;
static wifi_ap_record_t wifi_networks[WIFI_NETWORK_LIMIT];
static uint16_t wifi_network_count = 0;
static unsigned wifi_selected_network = 0;
static char wifi_connected_ssid[33];
static bool wifi_switching_network = false;
static unsigned wifi_retry_count = 0;
static char wifi_password[PASSWORD_CAPACITY];
static size_t wifi_password_length = 0;
static bool keyboard_shift = true;
static bool keyboard_symbols = false;
static unsigned keyboard_selected_key = 0;
static unsigned language_selected_item = 0;
static unsigned clock_city_selected = 0;
static unsigned clock_city = 0;
static unsigned settings_selected_item = 0;
static uint8_t sleep_timeout_minutes = 0;
static bool sleep_mode_active = false;
static bool power_saver = false;
static uint8_t battery_capacity_index = 1;
static unsigned power_selected_row = 0;
static unsigned led_selected_row = 0;
static bool led_from_home = false;
static uint8_t led_mode = 0;
static uint8_t led_color = 0;
static uint8_t led_speed = 2;
static uint8_t led_level = 30;
static float led_load_ma = 0.0f;
static uint8_t led_preview[3];
static float power_est_percent = -1.0f;
static float power_est_ma = 0.0f;
static int power_est_minutes = -1;
static int64_t power_est_last_us = 0;
static const uint16_t battery_capacity_presets[] = {500, 600, 800, 1000, 1500, 2000, 3000, 4000, 5000};
#define BATTERY_CAPACITY_COUNT (sizeof(battery_capacity_presets) / sizeof(battery_capacity_presets[0]))
#define POWER_SAVER_BRIGHTNESS 20
#define POWER_SAVER_SLEEP_MINUTES 1
#define LED_PIN 15
#define LED_COUNT 2
#define LED_MODE_COUNT 6
#define LED_COLOR_COUNT 8
#define LED_SPEED_COUNT 5
static int64_t last_user_input_us = 0;
static bool sntp_started = false;
static sdmmc_card_t *sd_card = NULL;
static bool sd_card_mounted = false;
static esp_err_t sd_last_mount_error = ESP_OK;
static int64_t sd_next_check_us = 0;
static unsigned sd_app_count = 0;
static unsigned sd_selected_app = 0;
static bool sd_app_running = false;
static lua_State *sd_lua_state = NULL;
static size_t sd_lua_memory_used = 0;
static unsigned sd_lua_instructions_left = 0;
static bool sd_lua_button_pressed = false;
static char sd_lua_error[SD_APP_TEXT_LENGTH];
static i2c_master_bus_handle_t sd_imu_bus;
static i2c_master_dev_handle_t sd_imu_device;
static bool sd_imu_initialized = false;
static int32_t sd_app_counter = 0;
static uint16_t sd_app_accent = 0;
static char sd_app_title[SD_APP_NAME_LENGTH];
static char sd_app_text[SD_APP_TEXT_LENGTH];
static char sd_app_type[12];
static sd_app_entry_t sd_apps[SD_APP_LIMIT];
static char sd_status[24] = "NO CARD";
static bool sd_status_is_error = true;
static const char wifi_symbol_keys[] = "!#$%&'()*+,-./:;<=>?@[\\]^_";
static const char wifi_extra_symbol_keys[] = "{|}~";
static const char wifi_password_punctuation[] = "@.-_";
static ble_device_t ble_devices[BLE_DEVICE_LIMIT];
static unsigned ble_device_count = 0;
static unsigned ble_selected_device = 0;
static int ble_connected_device = -1;
static bool ble_scan_parameters_ready = false;
static esp_gatt_if_t ble_gatt_interface = ESP_GATT_IF_NONE;
static bool ble_gatt_app_registered = false;

typedef enum {
    VIEW_HOME,
    VIEW_CLOCK,
    VIEW_GAME,
    VIEW_SETTINGS,
    VIEW_BRIGHTNESS,
    VIEW_SLEEP_SETTINGS,
    VIEW_SYSTEM,
    VIEW_WIFI,
    VIEW_WIFI_KEYBOARD,
    VIEW_BLUETOOTH,
    VIEW_LANGUAGE,
    VIEW_CLOCK_SETTINGS,
    VIEW_WEATHER,
    VIEW_SD_APPS,
    VIEW_SD_APP,
    VIEW_GALLERY,
    VIEW_MEDIA,
    VIEW_POWER,
    VIEW_LED
} view_t;

typedef enum {
    DIR_NONE,
    DIR_UP,
    DIR_DOWN,
    DIR_LEFT,
    DIR_RIGHT
} direction_t;

static direction_t sd_lua_input_direction = DIR_NONE;
static view_t current_view = VIEW_HOME;
static view_t previous_view = VIEW_HOME;
static unsigned selected_item = 0;
static int64_t transition_started_us = 0;
static bool transition_active = false;
static bool button_candidate_down = false;
static bool button_stable_down = false;
static int64_t button_change_us = 0;
static direction_t last_direction = DIR_NONE;
static int64_t last_direction_us = 0;
static int64_t last_voltage_us = 0;
static int64_t last_game_step_us = 0;
static int64_t app_started_us = 0;
static int64_t brightness_last_change_us = 0;
static bool brightness_dirty = false;
static bool keyboard_too_short = false;

#define WEATHER_HOURS 6

typedef struct {
    float temperature;
    float apparent;
    int humidity;
    float wind;
    int code;
    int hour_count;
    int hours[WEATHER_HOURS];
    float hour_temperature[WEATHER_HOURS];
    int hour_code[WEATHER_HOURS];
} weather_data_t;

static weather_data_t weather_data;
static bool weather_valid = false;
static volatile bool weather_loading = false;
static volatile bool weather_failed = false;
static volatile unsigned weather_location_gen = 1;
static volatile int weather_data_province = -1;
static int64_t weather_last_fetch_us = 0;
static int64_t weather_last_attempt_us = -100000000LL;

#define WEATHER_NAME_CAPACITY 28

typedef struct {
    float latitude;
    float longitude;
    char name[WEATHER_NAME_CAPACITY];
} weather_location_t;

static weather_location_t weather_location = {41.0082f, 28.9784f, "ISTANBUL, TR"};
static volatile uint8_t geo_state = 0; // 0 bos, 1 araniyor, 2 bulunamadi, 3 ag hatasi
static char geo_query[PASSWORD_CAPACITY];
static bool keyboard_city_mode = false;

typedef struct {
    const char *name;
    const char *timezone;
} clock_city_t;

static const clock_city_t clock_cities[] = {
    {"ISTANBUL", "TRT-3"},
    {"LONDON", "GMT0BST,M3.5.0/1,M10.5.0/2"},
    {"NEW YORK", "EST5EDT,M3.2.0/2,M11.1.0/2"},
    {"LOS ANGELES", "PST8PDT,M3.2.0/2,M11.1.0/2"},
    {"TOKYO", "JST-9"},
    {"DUBAI", "GST-4"},
    {"UTC", "UTC0"},
#define WORLD_CLOCK(name, lat, lon, tz) {name, tz},
    WORLD_CITY_LIST(WORLD_CLOCK)
#undef WORLD_CLOCK
};

static unsigned battery_percentage(float voltage);
static void start_wifi_scan(void);
static void start_ble_scan(void);
static void connect_selected_ble(void);
static const char *wifi_state_label(wifi_state_t state);
static void transition_to(view_t next_view);
static esp_err_t set_wifi_enabled(bool enabled);
static esp_err_t set_bluetooth_enabled(bool enabled);
static esp_err_t bluetooth_start(void);
static void apply_clock_city(void);
static void start_time_sync(void);
static void sd_release(void);
static void sd_scan_apps(void);
static void sd_handle_removal(void);
static void enter_sleep_mode(void);
static void exit_sleep_mode(void);

static const int8_t digit_font[10][5] = {
    {0x3e, 0x51, 0x49, 0x45, 0x3e},
    {0x00, 0x42, 0x7f, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4b, 0x31},
    {0x18, 0x14, 0x12, 0x7f, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3c, 0x4a, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1e}
};

static const int8_t letter_font[26][5] = {
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36},
    {0x3e, 0x41, 0x41, 0x41, 0x22}, {0x7f, 0x41, 0x41, 0x22, 0x1c},
    {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 0x09, 0x09, 0x09, 0x01},
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 0x08, 0x08, 0x08, 0x7f},
    {0x00, 0x41, 0x7f, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3f, 0x01},
    {0x7f, 0x08, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40},
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, {0x7f, 0x04, 0x08, 0x10, 0x7f},
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, {0x7f, 0x09, 0x09, 0x09, 0x06},
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7f, 0x01, 0x01},
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, {0x1f, 0x20, 0x40, 0x20, 0x1f},
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

static uint16_t color565(uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t color = (uint16_t)(((red & 0xf8) << 8) |
                                ((green & 0xfc) << 3) |
                                (blue >> 3));
    return (uint16_t)((color << 8) | (color >> 8));
}

static uint16_t media_color565(uint8_t red, uint8_t green, uint8_t blue)
{
    return color565(blue, green, red);
}

#define C_BG_TOP color565(10, 18, 36)
#define C_BG_BOTTOM color565(4, 8, 18)
#define C_BAR color565(13, 22, 40)
#define C_CARD color565(20, 31, 52)
#define C_CARD_HI color565(31, 46, 76)
#define C_LINE color565(42, 58, 90)
#define C_TEXT color565(241, 246, 252)
#define C_MUTED color565(130, 147, 174)
#define C_BLUE color565(94, 148, 255)
#define C_CYAN color565(72, 212, 236)
#define C_GREEN color565(74, 217, 156)
#define C_ORANGE color565(255, 170, 92)
#define C_RED color565(255, 102, 118)
#define C_PURPLE color565(175, 136, 255)

static esp_err_t lcd_transfer(const void *data, size_t length)
{
    spi_transaction_t transaction = {
        .length = length * 8,
        .tx_buffer = data
    };
    return spi_device_polling_transmit(lcd_spi, &transaction);
}

static esp_err_t lcd_command(uint8_t command)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(LCD_PIN_DC, 0), TAG, "LCD D/C komut seviyesi ayarlanamadi");
    return lcd_transfer(&command, sizeof(command));
}

static esp_err_t lcd_data(const void *data, size_t length)
{
    ESP_RETURN_ON_ERROR(gpio_set_level(LCD_PIN_DC, 1), TAG, "LCD D/C veri seviyesi ayarlanamadi");
    esp_err_t result = spi_device_acquire_bus(lcd_spi, portMAX_DELAY);
    if (result != ESP_OK) {
        return result;
    }

    const uint8_t *bytes = data;
    size_t remaining = length;
    while (remaining > 0) {
        size_t chunk_size = remaining > LCD_TRANSFER_CHUNK ? LCD_TRANSFER_CHUNK : remaining;
        spi_transaction_t transaction = {
            .length = chunk_size * 8,
            .tx_buffer = bytes,
            .flags = remaining > chunk_size ? SPI_TRANS_CS_KEEP_ACTIVE : 0
        };
        result = spi_device_polling_transmit(lcd_spi, &transaction);
        if (result != ESP_OK) {
            break;
        }
        bytes += chunk_size;
        remaining -= chunk_size;
    }

    spi_device_release_bus(lcd_spi);
    return result;
}

static esp_err_t lcd_command_data(uint8_t command, const uint8_t *data, size_t length)
{
    ESP_RETURN_ON_ERROR(lcd_command(command), TAG, "LCD komutu gonderilemedi");
    if (length > 0) {
        ESP_RETURN_ON_ERROR(lcd_data(data, length), TAG, "LCD komut verisi gonderilemedi");
    }
    return ESP_OK;
}

static esp_err_t display_init(void)
{
    gpio_config_t control_pins = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&control_pins), TAG, "LCD GPIO ayarlanamadi");

    spi_bus_config_t bus_config = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_BYTES
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO),
                        TAG, "LCD SPI veri yolu baslatilamadi");

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 1
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &device_config, &lcd_spi),
                        TAG, "LCD SPI aygiti eklenemedi");

    ESP_RETURN_ON_ERROR(gpio_set_level(LCD_PIN_RST, 0), TAG, "LCD reset ayarlanamadi");
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(gpio_set_level(LCD_PIN_RST, 1), TAG, "LCD reset ayarlanamadi");
    vTaskDelay(pdMS_TO_TICKS(120));

    ESP_RETURN_ON_ERROR(lcd_command(0x01), TAG, "LCD yazilim reseti gonderilemedi");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_command(0x11), TAG, "LCD uyku modu kapatilamadi");
    vTaskDelay(pdMS_TO_TICKS(120));

    const uint8_t color_mode[] = {0x55};
    const uint8_t memory_access[] = {0x68};
    const uint8_t porch[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    const uint8_t gate[] = {0x35};
    const uint8_t vcom[] = {0x19};
    const uint8_t lcm[] = {0x2c};
    const uint8_t voltage[] = {0x01};
    const uint8_t vdv_vrh_enable[] = {0x12};
    const uint8_t vrh_set[] = {0x20};
    const uint8_t frame_rate[] = {0x0f};
    const uint8_t power[] = {0xa4, 0xa1};
    const uint8_t positive_gamma[] = {0xd0, 0x04, 0x0d, 0x11, 0x13, 0x2b, 0x3f, 0x54, 0x4c, 0x18, 0x0d, 0x0b, 0x1f, 0x23};
    const uint8_t negative_gamma[] = {0xd0, 0x04, 0x0c, 0x11, 0x13, 0x2c, 0x3f, 0x44, 0x51, 0x2f, 0x1f, 0x1f, 0x20, 0x23};
    ESP_RETURN_ON_ERROR(lcd_command_data(0x3a, color_mode, sizeof(color_mode)), TAG, "LCD renk modu ayarlanamadi");
    ESP_RETURN_ON_ERROR(lcd_command_data(0x36, memory_access, sizeof(memory_access)), TAG, "LCD yonelimi ayarlanamadi");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xb2, porch, sizeof(porch)), TAG, "LCD porch ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xb7, gate, sizeof(gate)), TAG, "LCD gate ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xbb, vcom, sizeof(vcom)), TAG, "LCD VCOM ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xc0, lcm, sizeof(lcm)), TAG, "LCD LCM ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xc2, voltage, sizeof(voltage)), TAG, "LCD voltaj ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xc3, vdv_vrh_enable, sizeof(vdv_vrh_enable)), TAG, "LCD VDV ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xc4, vrh_set, sizeof(vrh_set)), TAG, "LCD VRH ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xc6, frame_rate, sizeof(frame_rate)), TAG, "LCD kare hizi ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xd0, power, sizeof(power)), TAG, "LCD guc ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xe0, positive_gamma, sizeof(positive_gamma)), TAG, "LCD gama ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0xe1, negative_gamma, sizeof(negative_gamma)), TAG, "LCD gama ayari basarisiz");
    ESP_RETURN_ON_ERROR(lcd_command_data(0x21, NULL, 0), TAG, "LCD ters cevirme ayari basarisiz");

    ESP_RETURN_ON_ERROR(lcd_command(0x13), TAG, "LCD normal goruntu modu acilamadi");
    ESP_RETURN_ON_ERROR(lcd_command(0x29), TAG, "LCD goruntu acilamadi");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t display_present(void)
{
    const uint8_t memory_access[] = {screen_flipped ? 0xa8 : 0x68};
    static const uint8_t column_address[] = {0x00, 0x00, 0x01, 0x3f};
    static const uint8_t row_address[] = {0x00, 0x23, 0x00, 0xcc};

    ESP_RETURN_ON_ERROR(lcd_command_data(0x36, memory_access, sizeof(memory_access)),
                        TAG, "LCD yonelimi guncellenemedi");
    ESP_RETURN_ON_ERROR(lcd_command_data(0x2a, column_address, sizeof(column_address)),
                        TAG, "LCD sutun araligi ayarlanamadi");
    ESP_RETURN_ON_ERROR(lcd_command_data(0x2b, row_address, sizeof(row_address)),
                        TAG, "LCD satir araligi ayarlanamadi");
    ESP_RETURN_ON_ERROR(lcd_command(0x2c), TAG, "LCD piksel aktarimi baslatilamadi");
    return lcd_data(framebuffer, LCD_BYTES);
}

static esp_err_t backlight_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "Arka isik PWM zamanlayicisi baslatilamadi");

    ledc_channel_config_t channel_config = {
        .gpio_num = LCD_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG, "Arka isik PWM kanali baslatilamadi");
    return ESP_OK;
}

static esp_err_t set_brightness(uint8_t value)
{
    if (value > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    brightness = value;
    uint8_t shown = power_saver && value > POWER_SAVER_BRIGHTNESS ? POWER_SAVER_BRIGHTNESS : value;
    uint32_t duty = ((uint32_t)shown * 255U) / 100U;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty),
                        TAG, "Parlaklik PWM degeri ayarlanamadi");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &adc1), TAG, "ADC birimi baslatilamadi");

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc1, ADC_CHANNEL_0, &channel_config),
                        TAG, "Joystick X ADC kanali ayarlanamadi");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc1, ADC_CHANNEL_1, &channel_config),
                        TAG, "Joystick Y ADC kanali ayarlanamadi");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc1, POWER_ADC_CHANNEL, &channel_config),
                        TAG, "Guc ADC kanali ayarlanamadi");

    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = POWER_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    if (adc_cali_create_scheme_curve_fitting(&calibration_config,
                                             &power_adc_calibration) != ESP_OK) {
        power_adc_calibration = NULL;
        ESP_LOGW(TAG, "ADC kalibrasyonu yok; ham deger kullanilacak");
    }
    return ESP_OK;
}

static int month_number(const char *month)
{
    static const char *months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    for (int i = 0; i < 12; ++i) {
        if (strncmp(month, months[i], 3) == 0) {
            return i;
        }
    }
    return 0;
}

static void set_clock_to_build_time(void)
{
    setenv("TZ", clock_cities[0].timezone, 1);
    tzset();
    char month[4] = {0};
    int day = 1;
    int year = 2024;
    int hour = 0;
    int minute = 0;
    int second = 0;
    if (sscanf(__DATE__, "%3s %d %d", month, &day, &year) != 3 ||
        sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second) != 3) {
        ESP_LOGE(TAG, "Derleme zamani okunamadi; saat sifirdan baslayacak");
        return;
    }

    struct tm build_time = {
        .tm_sec = second,
        .tm_min = minute,
        .tm_hour = hour,
        .tm_mday = day,
        .tm_mon = month_number(month),
        .tm_year = year - 1900
    };
    time_t timestamp = mktime(&build_time);
    if (timestamp == (time_t)-1) {
        ESP_LOGE(TAG, "Derleme zamani sistem saatine donusturulemedi");
        return;
    }
    struct timeval now = {.tv_sec = timestamp, .tv_usec = 0};
    if (settimeofday(&now, NULL) != 0) {
        ESP_LOGE(TAG, "Sistem saati ayarlanamadi");
    }
}

static void apply_clock_city(void)
{
    if (clock_city >= sizeof(clock_cities) / sizeof(clock_cities[0])) {
        ESP_LOGW(TAG, "Kayitli sehir secenegi gecersiz; Istanbul seciliyor");
        clock_city = 0;
    }
    if (setenv("TZ", clock_cities[clock_city].timezone, 1) != 0) {
        ESP_LOGE(TAG, "Sehir zaman dilimi ayarlanamadi");
        return;
    }
    tzset();
}

static void start_time_sync(void)
{
    if (!wifi_enabled || sntp_started) {
        return;
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    sntp_started = true;
    ESP_LOGI(TAG, "Saat NTP ile esitleme icin baslatildi");
}

static uint8_t load_u8_setting(const char *key, uint8_t fallback, uint8_t maximum)
{
    uint8_t value = fallback;
    if (nvs_get_u8(settings, key, &value) != ESP_OK || value > maximum) {
        return fallback;
    }
    return value;
}

static void settings_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("nova", NVS_READWRITE, &settings));

    uint8_t stored_brightness = 40;
    esp_err_t result = nvs_get_u8(settings, "brightness", &stored_brightness);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "brightness", 40));
        ESP_ERROR_CHECK(nvs_commit(settings));
        stored_brightness = 40;
    } else {
        ESP_ERROR_CHECK(result);
    }

    if (stored_brightness > 100) {
        ESP_LOGW(TAG, "Kayitli parlaklik degeri gecersiz; varsayilan %%40 kullaniliyor");
        stored_brightness = 40;
    }
    ESP_ERROR_CHECK(set_brightness(stored_brightness));

    uint8_t stored_language = 0;
    result = nvs_get_u8(settings, "language", &stored_language);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "language", 0));
        ESP_ERROR_CHECK(nvs_commit(settings));
    } else {
        ESP_ERROR_CHECK(result);
        if (stored_language > 1) {
            ESP_LOGW(TAG, "Kayitli dil secenegi gecersiz; English varsayilacak");
            stored_language = 0;
        }
    }
    language_turkish = stored_language == 1;

    uint8_t stored_city = 0;
    result = nvs_get_u8(settings, "clock_city", &stored_city);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "clock_city", 0));
        ESP_ERROR_CHECK(nvs_commit(settings));
    } else {
        ESP_ERROR_CHECK(result);
        if (stored_city >= sizeof(clock_cities) / sizeof(clock_cities[0])) {
            ESP_LOGW(TAG, "Kayitli sehir secenegi gecersiz; Istanbul varsayilacak");
            stored_city = 0;
        }
    }
    clock_city = stored_city;
    clock_city_selected = stored_city;

    weather_location_t stored_location;
    size_t stored_size = sizeof(stored_location);
    result = nvs_get_blob(settings, "wx_loc", &stored_location, &stored_size);
    if (result == ESP_OK && stored_size == sizeof(stored_location) &&
        isfinite(stored_location.latitude) && isfinite(stored_location.longitude)) {
        stored_location.name[WEATHER_NAME_CAPACITY - 1] = '\0';
        weather_location = stored_location;
    }
    apply_clock_city();

    uint8_t stored_wifi_enabled = 1;
    result = nvs_get_u8(settings, "wifi_on", &stored_wifi_enabled);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "wifi_on", 1));
        ESP_ERROR_CHECK(nvs_commit(settings));
    } else {
        ESP_ERROR_CHECK(result);
        if (stored_wifi_enabled > 1) {
            ESP_LOGW(TAG, "Kayitli Wi-Fi anahtari gecersiz; acik varsayilacak");
            stored_wifi_enabled = 1;
        }
    }
    wifi_enabled = stored_wifi_enabled == 1;

    uint8_t stored_bluetooth_enabled = 1;
    result = nvs_get_u8(settings, "bluetooth_on", &stored_bluetooth_enabled);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "bluetooth_on", 1));
        ESP_ERROR_CHECK(nvs_commit(settings));
    } else {
        ESP_ERROR_CHECK(result);
        if (stored_bluetooth_enabled > 1) {
            ESP_LOGW(TAG, "Kayitli Bluetooth anahtari gecersiz; acik varsayilacak");
            stored_bluetooth_enabled = 1;
        }
    }
    bluetooth_enabled = stored_bluetooth_enabled == 1;

    uint8_t stored_flip = 0;
    if (nvs_get_u8(settings, "flip", &stored_flip) != ESP_OK || stored_flip > 1) {
        stored_flip = 0;
    }
    screen_flipped = stored_flip == 1;

    uint8_t stored_joy_flip = 0;
    if (nvs_get_u8(settings, "joy_flip", &stored_joy_flip) != ESP_OK || stored_joy_flip > 1) {
        stored_joy_flip = 0;
    }
    joystick_flipped = stored_joy_flip == 1;

    power_saver = load_u8_setting("pwr_save", 0, 1) == 1;
    battery_capacity_index = load_u8_setting("bat_cap2", 1, BATTERY_CAPACITY_COUNT - 1);
    led_mode = load_u8_setting("led_mode", 0, LED_MODE_COUNT - 1);
    led_color = load_u8_setting("led_color", 0, LED_COLOR_COUNT - 1);
    led_speed = load_u8_setting("led_speed", 2, LED_SPEED_COUNT - 1);
    led_level = load_u8_setting("led_level", 30, 100);
    if (led_level < 10) {
        led_level = 10;
    }

    uint8_t stored_sleep_minutes = 0;
    result = nvs_get_u8(settings, "sleep_min", &stored_sleep_minutes);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_ERROR_CHECK(nvs_set_u8(settings, "sleep_min", 0));
        ESP_ERROR_CHECK(nvs_commit(settings));
    } else {
        ESP_ERROR_CHECK(result);
    }
    if (stored_sleep_minutes > SLEEP_TIMEOUT_MAX_MINUTES) {
        ESP_LOGW(TAG, "Kayitli uyku suresi gecersiz; kapali varsayilacak");
        stored_sleep_minutes = 0;
    }
    sleep_timeout_minutes = stored_sleep_minutes;
}

static void save_boolean_setting(const char *key, bool enabled)
{
    ESP_ERROR_CHECK(nvs_set_u8(settings, key, enabled ? 1 : 0));
    ESP_ERROR_CHECK(nvs_commit(settings));
}

static void save_brightness(void)
{
    ESP_ERROR_CHECK(nvs_set_u8(settings, "brightness", brightness));
    ESP_ERROR_CHECK(nvs_commit(settings));
    ESP_ERROR_CHECK(set_brightness(brightness));
    brightness_dirty = false;
}

static void save_u8_setting(const char *key, uint8_t value)
{
    ESP_ERROR_CHECK(nvs_set_u8(settings, key, value));
    ESP_ERROR_CHECK(nvs_commit(settings));
}

static wifi_ps_type_t wifi_power_mode(void)
{
    return power_saver ? WIFI_PS_MAX_MODEM : WIFI_PS_MIN_MODEM;
}

static void power_apply_profile(void)
{
    esp_pm_config_t config = {
        .max_freq_mhz = power_saver ? 80 : 160,
        .min_freq_mhz = power_saver ? 80 : 160,
        .light_sleep_enable = false
    };
    esp_err_t result = esp_pm_configure(&config);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "CPU frekansi ayarlanamadi: %s", esp_err_to_name(result));
    }
    if (!sleep_mode_active) {
        set_brightness(brightness);
    }
    if (wifi_enabled) {
        esp_wifi_set_ps(wifi_power_mode());
    }
    power_est_last_us = 0;
}

static float power_model_ma(void)
{
    unsigned mhz = (unsigned)(esp_clk_cpu_freq() / 1000000U);
    float ma = mhz >= 240 ? 50.0f : (mhz >= 160 ? 38.0f : 25.0f);
    if (wifi_enabled) {
        ma += wifi_state == WIFI_STATE_CONNECTED ? (power_saver ? 14.0f : 26.0f) : 20.0f;
    }
    if (bluetooth_enabled) {
        ma += 12.0f;
    }
    unsigned shown = power_saver && brightness > POWER_SAVER_BRIGHTNESS ? POWER_SAVER_BRIGHTNESS : brightness;
    ma += 6.0f + 0.30f * (float)shown;
    if (sd_card_mounted) {
        ma += 2.0f;
    }
    return ma + led_load_ma;
}

// Tahmin 10 saniyede bir yenilenir: yuzde filtrelenir, akim modelden hesaplanir.
static void power_estimate_service(int64_t now_us)
{
    if (power_est_last_us != 0 && now_us - power_est_last_us < 10000000) {
        return;
    }
    power_est_last_us = now_us;
    if (wifi_state == WIFI_STATE_CONNECTED && !sleep_mode_active) {
        esp_wifi_set_ps(wifi_power_mode());
    }
    float percent = (float)battery_level;
    if (power_est_percent < 0.0f) {
        power_est_percent = percent;
    } else {
        power_est_percent += (percent - power_est_percent) * 0.3f;
    }
    float ma = power_model_ma();
    power_est_ma = power_est_ma <= 0.0f ? ma : power_est_ma + (ma - power_est_ma) * 0.5f;
    float remaining_mah = (float)battery_capacity_presets[battery_capacity_index] *
                          power_est_percent / 100.0f;
    power_est_minutes = (int)(remaining_mah / power_est_ma * 60.0f + 0.5f);
}

static void format_battery_time(char *out, size_t size)
{
    if (power_est_minutes == -2) {
        snprintf(out, size, "USB");
    } else if (power_est_minutes < 0) {
        snprintf(out, size, "--");
    } else if (power_est_minutes >= 99 * 60) {
        snprintf(out, size, ">99%s", language_turkish ? "S" : "H");
    } else if (power_est_minutes >= 60) {
        snprintf(out, size, language_turkish ? "%uS %02uDK" : "%uH %02uM",
                 (unsigned)(power_est_minutes / 60), (unsigned)(power_est_minutes % 60));
    } else {
        snprintf(out, size, language_turkish ? "%uDK" : "%uM", (unsigned)power_est_minutes);
    }
}

static rmt_channel_handle_t led_channel = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static uint8_t led_last_frame[LED_COUNT * 3];
static bool led_frame_valid = false;

static const uint8_t led_palette[LED_COLOR_COUNT][3] = {
    {255, 0, 0}, {255, 90, 0}, {255, 200, 0}, {0, 255, 0},
    {0, 220, 255}, {0, 40, 255}, {150, 0, 255}, {255, 255, 255}
};
static const uint16_t led_slow_period_ms[LED_SPEED_COUNT] = {4000, 2800, 1800, 1100, 700};
static const uint16_t led_fast_period_ms[LED_SPEED_COUNT] = {1600, 1100, 700, 450, 300};
static const uint16_t led_rainbow_period_ms[LED_SPEED_COUNT] = {9000, 6000, 4000, 2500, 1500};

static esp_err_t led_init(void)
{
    rmt_tx_channel_config_t channel_config = {
        .gpio_num = LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel_config, &led_channel), TAG, "LED kanali acilamadi");
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&encoder_config, &led_encoder), TAG, "LED kodlayici acilamadi");
    return rmt_enable(led_channel);
}

static void led_hue_color(unsigned hue, uint8_t rgb[3])
{
    unsigned rise = (hue % 60) * 255 / 60;
    switch ((hue / 60) % 6) {
    case 0: rgb[0] = 255; rgb[1] = (uint8_t)rise; rgb[2] = 0; break;
    case 1: rgb[0] = (uint8_t)(255 - rise); rgb[1] = 255; rgb[2] = 0; break;
    case 2: rgb[0] = 0; rgb[1] = 255; rgb[2] = (uint8_t)rise; break;
    case 3: rgb[0] = 0; rgb[1] = (uint8_t)(255 - rise); rgb[2] = 255; break;
    case 4: rgb[0] = (uint8_t)rise; rgb[1] = 0; rgb[2] = 255; break;
    default: rgb[0] = 255; rgb[1] = 0; rgb[2] = (uint8_t)(255 - rise); break;
    }
}

static void led_service(int64_t now_us)
{
    if (led_channel == NULL) {
        return;
    }
    uint8_t frame[LED_COUNT * 3] = {0};
    memset(led_preview, 0, sizeof(led_preview));
    if (led_mode != 0 && !power_saver && !sleep_mode_active) {
        uint32_t now_ms = (uint32_t)(now_us / 1000);
        for (unsigned i = 0; i < LED_COUNT; ++i) {
            uint8_t base[3] = {led_palette[led_color][0], led_palette[led_color][1],
                               led_palette[led_color][2]};
            float level = 1.0f;
            if (led_mode == 2) {
                uint32_t period = led_slow_period_ms[led_speed];
                float phase = (float)(now_ms % period) / (float)period;
                float wave = phase < 0.5f ? phase * 2.0f : 2.0f - phase * 2.0f;
                level = wave * wave;
            } else if (led_mode == 3) {
                uint32_t period = led_fast_period_ms[led_speed];
                level = (now_ms % period) < period / 2 ? 1.0f : 0.0f;
            } else if (led_mode == 4) {
                uint32_t period = led_rainbow_period_ms[led_speed];
                unsigned hue = (unsigned)((now_ms % period) * 360U / period) + i * 180U;
                led_hue_color(hue % 360U, base);
            } else if (led_mode == 5) {
                uint32_t period = led_fast_period_ms[led_speed];
                bool first_half = (now_ms % period) < period / 2;
                level = (i == 0) == first_half ? 1.0f : 0.0f;
            }
            for (unsigned c = 0; c < 3; ++c) {
                uint8_t shown = (uint8_t)((float)base[c] * level);
                if (i == 0) {
                    led_preview[c] = shown;
                }
                frame[i * 3 + (c == 0 ? 1 : (c == 1 ? 0 : 2))] = (uint8_t)((unsigned)shown * led_level / 100U);
            }
        }
    }
    unsigned total = 0;
    for (unsigned i = 0; i < sizeof(frame); ++i) {
        total += frame[i];
    }
    led_load_ma += ((float)total / 255.0f * 20.0f - led_load_ma) * 0.05f;
    if (led_frame_valid && memcmp(led_last_frame, frame, sizeof(frame)) == 0) {
        return;
    }
    rmt_transmit_config_t transmit_config = {.loop_count = 0};
    if (rmt_transmit(led_channel, led_encoder, frame, sizeof(frame), &transmit_config) == ESP_OK) {
        rmt_tx_wait_all_done(led_channel, 20);
        memcpy(led_last_frame, frame, sizeof(frame));
        led_frame_valid = true;
    }
}

static bool sleep_bluetooth_suspended = false;
static uint8_t sleep_saved_brightness = 40;
static int sleep_drawn_minute = -1;
#define SLEEP_CLOCK_BRIGHTNESS 8

static void enter_sleep_mode(void)
{
    if (sleep_mode_active) {
        return;
    }
    sleep_mode_active = true;
    sleep_drawn_minute = -1;
    sleep_saved_brightness = brightness;
    sd_app_running = false;
    if (bluetooth_enabled) {
        if (set_bluetooth_enabled(false) == ESP_OK) {
            sleep_bluetooth_suspended = true;
            save_boolean_setting("bluetooth_on", true);
        }
    }
    if (wifi_enabled) {
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    }
    if (set_brightness(SLEEP_CLOCK_BRIGHTNESS) != ESP_OK) {
        ESP_LOGE(TAG, "Uyku saati parlakligi ayarlanamadi");
    }
}

static void exit_sleep_mode(void)
{
    if (!sleep_mode_active) {
        return;
    }
    sleep_mode_active = false;
    if (wifi_enabled) {
        esp_wifi_set_ps(wifi_power_mode());
    }
    if (sleep_bluetooth_suspended) {
        sleep_bluetooth_suspended = false;
        if (set_bluetooth_enabled(true) != ESP_OK) {
            ESP_LOGE(TAG, "Bluetooth uyku sonrasi acilamadi");
        }
    }
    ESP_ERROR_CHECK(set_brightness(sleep_saved_brightness));
}

static void set_pixel(int x, int y, uint16_t color)
{
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT) {
        framebuffer[y * LCD_WIDTH + x] = color;
    }
}

static void fill_screen(uint16_t color)
{
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        framebuffer[i] = color;
    }
}

static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0 || x >= LCD_WIDTH || y >= LCD_HEIGHT ||
        x + width <= 0 || y + height <= 0) {
        return;
    }
    int left = x < 0 ? 0 : x;
    int top = y < 0 ? 0 : y;
    int right = x + width > LCD_WIDTH ? LCD_WIDTH : x + width;
    int bottom = y + height > LCD_HEIGHT ? LCD_HEIGHT : y + height;
    for (int row = top; row < bottom; ++row) {
        for (int column = left; column < right; ++column) {
            framebuffer[row * LCD_WIDTH + column] = color;
        }
    }
}

static void draw_rect(int x, int y, int width, int height, uint16_t color)
{
    fill_rect(x, y, width, 1, color);
    fill_rect(x, y + height - 1, width, 1, color);
    fill_rect(x, y, 1, height, color);
    fill_rect(x + width - 1, y, 1, height, color);
}

static void blend_pixel(int x, int y, uint16_t color, unsigned alpha)
{
    if (alpha == 0 || x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) {
        return;
    }
    uint16_t *target = &framebuffer[y * LCD_WIDTH + x];
    if (alpha >= 255) {
        *target = color;
        return;
    }
    uint16_t source = (uint16_t)((color << 8) | (color >> 8));
    uint16_t destination = (uint16_t)((*target << 8) | (*target >> 8));
    unsigned red = (((source >> 11) & 0x1f) * alpha +
                    ((destination >> 11) & 0x1f) * (255 - alpha)) / 255;
    unsigned green = (((source >> 5) & 0x3f) * alpha +
                      ((destination >> 5) & 0x3f) * (255 - alpha)) / 255;
    unsigned blue = ((source & 0x1f) * alpha + (destination & 0x1f) * (255 - alpha)) / 255;
    uint16_t mixed = (uint16_t)((red << 11) | (green << 5) | blue);
    *target = (uint16_t)((mixed << 8) | (mixed >> 8));
}

static unsigned edge_alpha(float distance, float radius)
{
    float coverage = radius - distance + 0.5f;
    if (coverage <= 0.0f) {
        return 0;
    }
    if (coverage >= 1.0f) {
        return 255;
    }
    return (unsigned)(coverage * 255.0f);
}

static void fill_round_rect(int x, int y, int width, int height, int radius, uint16_t color)
{
    if (width <= 0 || height <= 0) {
        return;
    }
    if (radius * 2 > width) {
        radius = width / 2;
    }
    if (radius * 2 > height) {
        radius = height / 2;
    }
    fill_rect(x + radius, y, width - 2 * radius, height, color);
    fill_rect(x, y + radius, radius, height - 2 * radius, color);
    fill_rect(x + width - radius, y + radius, radius, height - 2 * radius, color);
    for (int dy = 0; dy < radius; ++dy) {
        for (int dx = 0; dx < radius; ++dx) {
            float offset_x = (float)radius - 0.5f - (float)dx;
            float offset_y = (float)radius - 0.5f - (float)dy;
            unsigned alpha = edge_alpha(sqrtf(offset_x * offset_x + offset_y * offset_y),
                                        (float)radius);
            blend_pixel(x + dx, y + dy, color, alpha);
            blend_pixel(x + width - 1 - dx, y + dy, color, alpha);
            blend_pixel(x + dx, y + height - 1 - dy, color, alpha);
            blend_pixel(x + width - 1 - dx, y + height - 1 - dy, color, alpha);
        }
    }
}

static void fill_circle(int center_x, int center_y, int radius, uint16_t color)
{
    int inner = radius > 0 ? (radius - 1) * (radius - 1) : -1;
    int outer = (radius + 1) * (radius + 1);
    for (int y = -radius - 1; y <= radius + 1; ++y) {
        for (int x = -radius - 1; x <= radius + 1; ++x) {
            int squared = x * x + y * y;
            if (squared <= inner) {
                set_pixel(center_x + x, center_y + y, color);
            } else if (squared < outer) {
                blend_pixel(center_x + x, center_y + y, color,
                            edge_alpha(sqrtf((float)squared), (float)radius));
            }
        }
    }
}

static void fill_vertical_gradient(int top, int bottom, uint16_t top_color, uint16_t bottom_color)
{
    unsigned top_red = (unsigned)(((top_color << 8) | (top_color >> 8)) >> 11) & 0x1f;
    unsigned top_green = (unsigned)(((top_color << 8) | (top_color >> 8)) >> 5) & 0x3f;
    unsigned top_blue = (unsigned)((top_color << 8) | (top_color >> 8)) & 0x1f;
    unsigned bottom_red = (unsigned)(((bottom_color << 8) | (bottom_color >> 8)) >> 11) & 0x1f;
    unsigned bottom_green = (unsigned)(((bottom_color << 8) | (bottom_color >> 8)) >> 5) & 0x3f;
    unsigned bottom_blue = (unsigned)((bottom_color << 8) | (bottom_color >> 8)) & 0x1f;
    int span = bottom - top - 1 > 0 ? bottom - top - 1 : 1;
    for (int y = top; y < bottom; ++y) {
        int step = y - top;
        unsigned red = (top_red * (unsigned)(span - step) + bottom_red * (unsigned)step) / (unsigned)span;
        unsigned green = (top_green * (unsigned)(span - step) + bottom_green * (unsigned)step) / (unsigned)span;
        unsigned blue = (top_blue * (unsigned)(span - step) + bottom_blue * (unsigned)step) / (unsigned)span;
        uint16_t mixed = (uint16_t)((red << 11) | (green << 5) | blue);
        fill_rect(0, y, LCD_WIDTH, 1, (uint16_t)((mixed << 8) | (mixed >> 8)));
    }
}

static const int8_t *glyph_for(char character)
{
    static const int8_t punctuation[][5] = {
        {0x00, 0x00, 0x5f, 0x00, 0x00}, {0x14, 0x7f, 0x14, 0x7f, 0x14},
        {0x24, 0x2a, 0x7f, 0x2a, 0x12}, {0x36, 0x49, 0x55, 0x22, 0x50},
        {0x00, 0x05, 0x03, 0x00, 0x00}, {0x00, 0x1c, 0x22, 0x41, 0x00},
        {0x00, 0x41, 0x22, 0x1c, 0x00}, {0x14, 0x08, 0x3e, 0x08, 0x14},
        {0x08, 0x08, 0x3e, 0x08, 0x08}, {0x00, 0x50, 0x30, 0x00, 0x00},
        {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x14, 0x22, 0x41, 0x00},
        {0x14, 0x14, 0x14, 0x14, 0x14}, {0x41, 0x22, 0x14, 0x08, 0x00},
        {0x02, 0x01, 0x51, 0x09, 0x06}, {0x00, 0x7f, 0x41, 0x41, 0x00},
        {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7f, 0x00},
        {0x04, 0x02, 0x01, 0x02, 0x04}, {0x40, 0x40, 0x40, 0x40, 0x40},
        {0x00, 0x08, 0x36, 0x41, 0x00}, {0x00, 0x00, 0x7f, 0x00, 0x00},
        {0x00, 0x41, 0x36, 0x08, 0x00}, {0x08, 0x04, 0x08, 0x10, 0x08},
        {0x3e, 0x41, 0x5d, 0x55, 0x1e}
    };
    if (character >= '0' && character <= '9') {
        return digit_font[character - '0'];
    }
    if (character >= 'A' && character <= 'Z') {
        return letter_font[character - 'A'];
    }
    if (character >= 'a' && character <= 'z') {
        return letter_font[character - 'a'];
    }
    switch (character) {
    case '!': return punctuation[0];
    case '#': return punctuation[1];
    case '$': return punctuation[2];
    case '&': return punctuation[3];
    case '\'': return punctuation[4];
    case '(': return punctuation[5];
    case ')': return punctuation[6];
    case '*': return punctuation[7];
    case '+': return punctuation[8];
    case ',': return punctuation[9];
    case ';': return punctuation[10];
    case '<': return punctuation[11];
    case '=': return punctuation[12];
    case '>': return punctuation[13];
    case '?': return punctuation[14];
    case '[': return punctuation[15];
    case '\\': return punctuation[16];
    case ']': return punctuation[17];
    case '^': return punctuation[18];
    case '_': return punctuation[19];
    case '{': return punctuation[20];
    case '|': return punctuation[21];
    case '}': return punctuation[22];
    case '~': return punctuation[23];
    case '@': return punctuation[24];
    default: return NULL;
    }
}

typedef struct {
    const char *english;
    const char *turkish;
} text_translation_t;

static const char *localized_text(const char *text)
{
    static const text_translation_t translations[] = {
        {"SMART CONSOLE", "AKILLI KONSOL"}, {"MOVE SELECT", "SECMEK ICIN IT"},
        {"CLOCK", "SAAT"}, {"GAME", "OYUN"}, {"LIGHT", "ISIK"},
        {"SYSTEM", "SISTEM"}, {"LANGUAGE", "DIL"}, {"SETTINGS", "AYARLAR"},
        {"CITY / TIME", "SEHIR/SAAT"}, {"WEATHER", "HAVA DURUMU"}, {"CLEAR", "ACIK"},
        {"PARTLY CLOUDY", "PARCALI BULUTLU"}, {"CLOUDY", "BULUTLU"}, {"FOG", "SISLI"},
        {"DRIZZLE", "CISELEME"}, {"RAIN", "YAGMUR"}, {"SNOW", "KAR"}, {"SHOWERS", "SAGANAK"},
        {"STORM", "FIRTINA"}, {"GALLERY", "GALERI"}, {"PHOTO", "FOTO"}, {"PAUSED", "DURDU"},
        {"NO MEDIA", "MEDYA YOK"}, {"LOADING", "YUKLENIYOR"}, {"WIFI NEEDED", "WIFI GEREKLI"},
        {"NO DATA", "VERI YOK"}, {"SELECT CITY", "SEHIR SECIN"},
        {"TIME ZONE", "SAAT DILIMI"}, {"SYNC OVER WIFI", "WIFI ILE ESITLE"},
        {"TIME SYNC NEEDS WIFI", "SAAT ICIN WIFI GEREKIR"},
        {"SW SELECT / WIFI SYNC", "SW SEC / WIFI ESITLE"},
        {"TIME ZONE SAVED", "SAAT DILIMI KAYITLI"},
        {"LIVE TIME", "CANLI SAAT"}, {"CLICK BACK", "GERI ICIN BAS"},
        {"SCORE", "SKOR"}, {"GAME OVER", "OYUN BITTI"},
        {"CLICK RESTART", "YENIDEN BASLAT"}, {"BRIGHTNESS", "PARLAKLIK"},
        {"SAVED AUTOMATIC", "OTOMATIK KAYIT"}, {"DEFAULT 40%", "VARSAYILAN 40%"},
        {"DEVICE STATUS", "CIHAZ DURUMU"}, {"REXAUI DEVICE STATUS", "REXAUI CIHAZ DURUMU"},
        {"USB POWER", "USB GUC"}, {"BATTERY MODE", "PIL MODU"},
        {"USB POWER ACTIVE", "USB GUC AKTIF"}, {"ESTIMATED LEVEL", "TAHMINI SEVIYE"},
        {"BATTERY", "PIL"}, {"BUILD TIME", "DERLEME ZAMANI"},
        {"FREE RAM", "BOS RAM"}, {"UPTIME", "CALISMA SURESI"}, {"BRIGHT", "ISIK"},
        {"ROTATION", "DONUS"}, {"SD APPS", "SD UYGULAMA"},
        {"SD RAM APPS", "SD RAM UYGULAMALARI"}, {"NO CARD", "KART YOK"},
        {"NO APPS", "UYGULAMA YOK"}, {"CARD REMOVED", "KART CIKARILDI"},
        {"CARD ERROR", "KART HATASI"}, {"MOVE LEFT OR RIGHT", "SOLA VEYA SAGA IT"},
        {"SD RAM APP", "SD RAM UYGULAMASI"}, {"COUNTER", "SAYAC"},
        {"CLOCK APP", "SAAT UYGULAMASI"}, {"MESSAGE APP", "MESAJ UYGULAMASI"},
        {"ADD APPS TO /REXA/APPS", "/REXA/APPS KLASORUNE EKLE"},
        {"JOYSTICK TO PLAY", "JOYSTICK ILE OYNA"}, {"MOVE RESTART", "IT: YENIDEN BASLAT"},
        {"ADJUST", "AYARLA"}, {"LEFT", "SOL"}, {"RIGHT", "SAG"},
        {"ENABLE IN SYSTEM", "SISTEMDEN ACIN"}, {"WIFI OFF", "WIFI KAPALI"},
        {"SW TOGGLE / LEFT BACK", "SW DEGISTIR / SOL GERI"},
        {"BS ERASE  CAP LOCK", "BS SIL  CAP BUYUK HARF"},
        {"WIFI STARTING", "WIFI BASLIYOR"}, {"SCANNING NETWORKS", "AGLAR ARANIYOR"},
        {"SELECT NETWORK", "AG SECIN"}, {"CONNECTING", "BAGLANIYOR"},
        {"CONNECTED", "BAGLANDI"}, {"CONNECTION FAILED", "BAGLANTI HATASI"},
        {"SEARCHING", "ARANIYOR"}, {"NO NETWORKS", "AG BULUNAMADI"},
        {"MOVE RIGHT RESCAN", "SAGA IT: TEKRAR TARA"},
        {"HIDDEN NETWORK", "GIZLI AG"}, {"OPEN", "ACIK"}, {"LOCKED", "SIFRELI"},
        {"CLICK CONNECT", "BAGLANMAK ICIN BAS"}, {"CLICK SELECT", "SECMEK ICIN BAS"},
        {"LEFT BACK RIGHT SCAN", "SOL GERI SAG TARA"},
        {"BLUETOOTH OFF", "BLUETOOTH KAPALI"},
        {"BLUETOOTH STARTING", "BLUETOOTH BASLIYOR"},
        {"SCANNING DEVICES", "CIHAZLAR ARANIYOR"},
        {"SELECT BLE DEVICE", "BLE CIHAZI SECIN"},
        {"BLUETOOTH ERROR", "BLUETOOTH HATASI"},
        {"NO BLE DEVICES", "BLE CIHAZI YOK"},
        {"WIFI PASSWORD", "WIFI SIFRESI"}, {"UPPERCASE", "BUYUK HARF"},
        {"LOWERCASE", "KUCUK HARF"}, {"SYMBOLS", "SEMBOLLER"},
        {"PASSWORD MIN 8", "SIFRE EN AZ 8"},
        {"SW TYPE / OK JOIN", "SW YAZ / OK BAGLAN"},
        {"CAP", "CAP"}, {"SYM", "SYM"}, {"BS", "BS"}, {"CLR", "SIL"},
        {"SP", "BOS"}, {"OK", "OK"}, {"BK", "GERI"},
        {"SELECT LANGUAGE", "DIL SECIN"}, {"ENGLISH", "INGILIZCE"},
        {"TURKCE", "TURKCE"}, {"USE SW TO SELECT", "SECMEK ICIN SW"},
        {"REXAUI", "REXAUI"}, {"WIFI", "WIFI"}, {"BLUETOOTH", "BLUETOOTH"},
        {"ON", "ACIK"}, {"OFF", "KAPALI"}, {"NETWORKS", "AGLAR"},
        {"DEVICES", "CIHAZLAR"}, {"PASSWORD", "SIFRE"}, {"PASS", "SIFRE"},
        {"BAT", "PIL"}, {"MOVE", "IT"}, {"SELECT", "SEC"},
        {"PAUSED", "DURAKLATILDI"},
        {"SLEEP TIMER", "UYKU ZAMANI"}, {"WIFI SWITCH", "WIFI ANAHTARI"},
        {"BT SWITCH", "BT ANAHTARI"}, {"POWER SAVER", "PIL TASARRUFU"}, {"POWER", "GUC"},
        {"BATTERY SIZE", "PIL KAPASITESI"}, {"TIME LEFT", "KALAN SURE"}, {"DRAW", "AKIM"},
        {"SCREEN", "EKRAN"}, {"MODE", "MOD"}, {"COLOR", "RENK"}, {"SPEED", "HIZ"},
        {"BRIGHT", "PARLAK"}, {"SOLID", "SABIT"}, {"BREATHE", "NEFES"}, {"BLINK", "YANIP SON"},
        {"RAINBOW", "GOKKUSAGI"}, {"ALTERNATE", "SIRALI"}, {"RED", "KIRMIZI"},
        {"ORANGE", "TURUNCU"}, {"YELLOW", "SARI"}, {"GREEN", "YESIL"}, {"BLUE", "MAVI"},
        {"PURPLE", "MOR"}, {"WHITE", "BEYAZ"}, {"SAVER ON", "TASARRUF ACIK"},
        {"SW CHANGE / LEFT BACK", "SW DEGISTIR / SOL GERI"}, {"FLIP SCREEN", "EKRANI CEVIR"}, {"FLIP JOYSTICK", "JOYSTICK TERSLE"},
        {"SW SELECT / LEFT BACK", "SW SEC / SOL GERI"},
        {"ENABLE IN SETTINGS", "AYARLARDAN ACIN"}, {"CLICK / LEFT BACK", "BAS / SOL GERI"},
        {"UP/DOWN ADJUST", "YUKARI/ASAGI AYARLA"}, {"0 = DISABLED", "0 = KAPALI"},
        {"TIMEOUT", "SURE"}, {"CPU", "CPU"}, {"LUA", "LUA"},
        {"SW RUN / LEFT BACK / RIGHT RESCAN", "SW CALISTIR / SOL GERI / SAG TARA"},
        {"LEFT BACK", "SOL GERI"}, {"APP STOPPED", "UYGULAMA DURDU"},
        {"APP FAILED", "UYGULAMA HATASI"},
        {"USE SW TO SELECT / LEFT BACK", "SECMEK ICIN SW / SOL GERI"},
        {"CLICK CONNECT / LEFT BACK", "BAGLAN: BAS / SOL GERI"},
        {"CLICK SELECT / LEFT BACK", "SEC: BAS / SOL GERI"},
        {"NO CARD", "KART YOK"}, {"SNAKE", "YILAN"}, {"TIME", "SAAT"}
    };
    if (!language_turkish) {
        return text;
    }
    for (size_t i = 0; i < sizeof(translations) / sizeof(translations[0]); ++i) {
        if (strcmp(text, translations[i].english) == 0) {
            return translations[i].turkish;
        }
    }
    return text;
}

static void draw_text(int x, int y, const char *text, int scale, uint16_t color)
{
    text = localized_text(text);
    int origin_x = x;
    for (const char *cursor = text; *cursor != '\0'; ++cursor) {
        char character = *cursor;
        if (character == '\n') {
            y += 9 * scale;
            x = origin_x;
            continue;
        }
        const int8_t *glyph = glyph_for(character);
        if (glyph != NULL) {
            for (int column = 0; column < 5; ++column) {
                for (int row = 0; row < 7; ++row) {
                    if ((glyph[column] & (1 << row)) != 0) {
                        fill_rect(x + column * scale, y + row * scale,
                                  scale, scale, color);
                    }
                }
            }
        } else if (character == ':' || character == '.') {
            fill_rect(x + 2 * scale, y + 2 * scale, scale, scale, color);
            fill_rect(x + 2 * scale, y + 5 * scale, scale, scale, color);
        } else if (character == '-') {
            fill_rect(x + scale, y + 3 * scale, 3 * scale, scale, color);
        } else if (character == '%') {
            set_pixel(x + scale, y + scale, color);
            set_pixel(x + 3 * scale, y + 5 * scale, color);
            draw_rect(x, y, 2 * scale, 2 * scale, color);
            draw_rect(x + 3 * scale, y + 4 * scale, 2 * scale, 2 * scale, color);
        } else if (character == '/') {
            for (int i = 0; i < 5; ++i) {
                fill_rect(x + (4 - i) * scale, y + i * scale, scale, scale, color);
            }
        }
        x += 6 * scale;
    }
}

static void draw_centered_text(int center_x, int y, const char *text, int scale, uint16_t color)
{
    text = localized_text(text);
    int width = (int)strlen(text) * 6 * scale - scale;
    draw_text(center_x - width / 2, y, text, scale, color);
}

static void draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        set_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static void draw_circle(int center_x, int center_y, int radius, uint16_t color);

static uint16_t icon_cut_color;

static void draw_icon(unsigned item, int center_x, int center_y, uint16_t color)
{
    const uint16_t cut = icon_cut_color;
    if (item == 12) {
        draw_line(center_x + 3, center_y - 10, center_x - 5, center_y + 1, color);
        draw_line(center_x + 4, center_y - 10, center_x - 4, center_y + 1, color);
        draw_line(center_x - 5, center_y + 1, center_x + 4, center_y + 1, color);
        draw_line(center_x + 4, center_y + 1, center_x - 3, center_y + 10, color);
        draw_line(center_x + 5, center_y + 1, center_x - 2, center_y + 10, color);
        return;
    }
    if (item == 11) {
        fill_circle(center_x, center_y - 2, 8, color);
        fill_round_rect(center_x - 4, center_y + 6, 8, 5, 2, color);
        fill_circle(center_x - 3, center_y - 4, 2, cut);
        return;
    }
    if (item == 0) {
        draw_circle(center_x, center_y, 10, color);
        draw_circle(center_x, center_y, 9, color);
        draw_line(center_x, center_y, center_x, center_y - 6, color);
        draw_line(center_x, center_y, center_x + 5, center_y + 3, color);
        fill_circle(center_x, center_y, 1, color);
    } else if (item == 1) {
        fill_round_rect(center_x - 11, center_y - 8, 22, 16, 6, color);
        fill_round_rect(center_x - 2, center_y - 5, 4, 10, 2, cut);
        fill_round_rect(center_x - 5, center_y - 2, 10, 4, 2, cut);
    } else if (item == 2) {
        fill_round_rect(center_x - 4, center_y - 9, 8, 18, 4, color);
        fill_circle(center_x, center_y - 7, 2, cut);
    } else if (item == 3) {
        fill_round_rect(center_x - 11, center_y - 8, 22, 16, 4, color);
        draw_line(center_x - 6, center_y - 3, center_x + 6, center_y - 3, cut);
        draw_line(center_x - 6, center_y + 3, center_x + 2, center_y + 3, cut);
        fill_circle(center_x + 6, center_y + 3, 1, cut);
    } else if (item == 5) {
        draw_line(center_x, center_y - 10, center_x, center_y + 10, color);
        draw_line(center_x, center_y - 10, center_x + 7, center_y - 4, color);
        draw_line(center_x + 7, center_y - 4, center_x, center_y + 1, color);
        draw_line(center_x, center_y + 1, center_x + 7, center_y + 7, color);
        draw_line(center_x + 7, center_y + 7, center_x, center_y + 10, color);
        draw_line(center_x, center_y + 1, center_x - 4, center_y - 3, color);
        draw_line(center_x - 4, center_y - 3, center_x, center_y - 7, color);
        draw_line(center_x, center_y + 1, center_x - 4, center_y + 5, color);
        draw_line(center_x - 4, center_y + 5, center_x, center_y + 9, color);
    } else if (item == 8) {
        fill_round_rect(center_x - 8, center_y - 10, 16, 20, 3, color);
        fill_rect(center_x - 4, center_y - 6, 8, 1, cut);
        fill_rect(center_x - 4, center_y - 2, 8, 1, cut);
        fill_rect(center_x - 4, center_y + 2, 8, 1, cut);
        fill_rect(center_x - 4, center_y + 6, 8, 1, cut);
    } else if (item == 9) {
        for (int tooth = 0; tooth < 8; ++tooth) {
            float angle = (float)tooth * 0.785398f;
            fill_circle(center_x + (int)(cosf(angle) * 9.0f),
                        center_y + (int)(sinf(angle) * 9.0f), 2, color);
        }
        fill_circle(center_x, center_y, 7, color);
        fill_circle(center_x, center_y, 3, cut);
    } else if (item == 10) {
        fill_round_rect(center_x - 11, center_y - 9, 22, 18, 3, color);
        fill_round_rect(center_x - 8, center_y - 6, 16, 12, 1, cut);
        fill_circle(center_x + 3, center_y - 2, 2, color);
        fill_circle(center_x - 3, center_y + 3, 3, color);
        fill_circle(center_x + 3, center_y + 3, 3, color);
        fill_rect(center_x - 6, center_y + 3, 12, 3, color);
    } else {
        draw_circle(center_x, center_y, 9, color);
        draw_circle(center_x, center_y, 10, color);
        draw_line(center_x - 9, center_y, center_x + 9, center_y, color);
        draw_line(center_x, center_y - 9, center_x, center_y + 9, color);
        draw_line(center_x - 6, center_y - 6, center_x + 6, center_y - 6, color);
        draw_line(center_x - 6, center_y + 6, center_x + 6, center_y + 6, color);
    }
}

static void draw_circle(int center_x, int center_y, int radius, uint16_t color)
{
    int x = radius;
    int y = 0;
    int error = 1 - radius;
    while (x >= y) {
        set_pixel(center_x + x, center_y + y, color);
        set_pixel(center_x + y, center_y + x, color);
        set_pixel(center_x - y, center_y + x, color);
        set_pixel(center_x - x, center_y + y, color);
        set_pixel(center_x - x, center_y - y, color);
        set_pixel(center_x - y, center_y - x, color);
        set_pixel(center_x + y, center_y - x, color);
        set_pixel(center_x + x, center_y - y, color);
        ++y;
        if (error < 0) {
            error += 2 * y + 1;
        } else {
            --x;
            error += 2 * (y - x) + 1;
        }
    }
}

static int text_width(const char *text, int scale)
{
    text = localized_text(text);
    int length = (int)strlen(text);
    return length > 0 ? length * 6 * scale - scale : 0;
}

static void draw_text_right(int right, int y, const char *text, int scale, uint16_t color)
{
    draw_text(right - text_width(text, scale), y, text, scale, color);
}

static void draw_background(void)
{
    fill_vertical_gradient(0, LCD_HEIGHT, C_BG_TOP, C_BG_BOTTOM);
}

static void draw_card(int x, int y, int width, int height, int radius, bool selected,
                      uint16_t accent)
{
    if (selected) {
        fill_round_rect(x, y, width, height, radius, accent);
        fill_round_rect(x + 1, y + 1, width - 2, height - 2, radius - 1, C_CARD_HI);
    } else {
        fill_round_rect(x, y, width, height, radius, C_CARD);
    }
}

static void draw_switch(int x, int y, bool on)
{
    fill_round_rect(x, y, 30, 16, 8, on ? C_GREEN : C_LINE);
    fill_circle(on ? x + 22 : x + 8, y + 8, 6, C_TEXT);
}

static void draw_progress(int x, int y, int width, int height, unsigned percent,
                          uint16_t accent)
{
    fill_round_rect(x, y, width, height, height / 2, C_LINE);
    int filled = (width * (int)percent) / 100;
    if (filled > 0) {
        fill_round_rect(x, y, filled < height ? height : filled, height, height / 2, accent);
    }
}

static void draw_chip(int x, int y, const char *label, bool active, uint16_t accent)
{
    fill_round_rect(x, y, 24, 12, 6, active ? accent : C_CARD);
    draw_centered_text(x + 12, y + 2, label, 1, active ? C_BAR : C_MUTED);
}

static unsigned list_first(unsigned selected, unsigned count, unsigned visible)
{
    if (count <= visible || selected < visible / 2) {
        return 0;
    }
    unsigned first = selected - visible / 2;
    return first + visible > count ? count - visible : first;
}

static void draw_scroll_bar(unsigned first, unsigned count, unsigned visible, int top,
                            int height)
{
    if (count <= visible) {
        return;
    }
    int thumb = height * (int)visible / (int)count;
    if (thumb < 8) {
        thumb = 8;
    }
    int offset = (height - thumb) * (int)first / (int)(count - visible);
    fill_round_rect(315, top, 2, height, 1, C_LINE);
    fill_round_rect(315, top + offset, 2, thumb, 1, C_BLUE);
}

static void draw_battery_glyph(int x, int y, unsigned percentage)
{
    fill_round_rect(x, y, 20, 10, 3, C_MUTED);
    fill_round_rect(x + 1, y + 1, 18, 8, 2, C_BAR);
    fill_round_rect(x + 2, y + 2, (16 * (int)percentage) / 100, 6, 1,
                    percentage > 20 ? C_GREEN : C_RED);
    fill_rect(x + 20, y + 3, 2, 4, C_MUTED);
}

static void draw_status_bar(void)
{
    fill_rect(0, 0, LCD_WIDTH, 20, C_BAR);
    fill_rect(0, 20, LCD_WIDTH, 1, C_LINE);
    draw_battery_glyph(8, 5, battery_level);
    char battery_text[8];
    snprintf(battery_text, sizeof(battery_text), "%u%%", battery_level);
    draw_text(36, 6, battery_text, 1, battery_level <= 15 ? C_RED : C_TEXT);
    int bars = 0;
    wifi_ap_record_t access_point;
    if (wifi_enabled && wifi_state == WIFI_STATE_CONNECTED &&
        esp_wifi_sta_get_ap_info(&access_point) == ESP_OK) {
        bars = access_point.rssi >= -55 ? 4 : access_point.rssi >= -65 ? 3 :
               access_point.rssi >= -75 ? 2 : 1;
    }
    for (int bar = 0; bar < 4; ++bar) {
        int height = 4 + bar * 3;
        fill_rect(238 + bar * 5, 16 - height, 3, height, bar < bars ? C_GREEN : C_LINE);
    }
    draw_chip(262, 4, "WF", wifi_enabled, C_GREEN);
    draw_chip(290, 4, "BT", bluetooth_enabled, C_BLUE);
}

static void draw_header(const char *title, const char *subtitle)
{
    draw_status_bar();
    const int gap = 10;
    int title_width = text_width(title, 1);
    int subtitle_width = subtitle != NULL ? text_width(subtitle, 1) : 0;
    if (subtitle_width > 0 && title_width + gap + subtitle_width > 190) {
        subtitle = NULL;
        subtitle_width = 0;
    }
    int total = title_width + (subtitle_width > 0 ? gap + subtitle_width : 0);
    int x = 160 - total / 2;
    draw_text(x, 6, title, 1, C_TEXT);
    if (subtitle_width > 0) {
        draw_text(x + title_width + gap, 6, subtitle, 1, C_MUTED);
    }
}

static void draw_footer(const char *hint)
{
    draw_centered_text(160, 158, hint, 1, C_MUTED);
}

static void draw_segment_digit(int x, int y, int width, int height, int thickness,
                               int digit, uint16_t color)
{
    static const uint8_t masks[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
    uint8_t mask = masks[digit < 0 || digit > 9 ? 0 : digit];
    int radius = thickness / 2;
    int middle = y + (height - thickness) / 2;
    if (mask & 0x01) fill_round_rect(x, y, width, thickness, radius, color);
    if (mask & 0x02) fill_round_rect(x + width - thickness, y, thickness, height / 2 + radius, radius, color);
    if (mask & 0x04) fill_round_rect(x + width - thickness, middle, thickness, height / 2 + radius, radius, color);
    if (mask & 0x08) fill_round_rect(x, y + height - thickness, width, thickness, radius, color);
    if (mask & 0x10) fill_round_rect(x, middle, thickness, height / 2 + radius, radius, color);
    if (mask & 0x20) fill_round_rect(x, y, thickness, height / 2 + radius, radius, color);
    if (mask & 0x40) fill_round_rect(x, middle, width, thickness, radius, color);
}

static int draw_big_time(int x, int y, int width, int height, int thickness,
                         int hour, int minute, uint16_t color)
{
    const int gap = thickness + 2;
    draw_segment_digit(x, y, width, height, thickness, hour / 10, color);
    x += width + gap;
    draw_segment_digit(x, y, width, height, thickness, hour % 10, color);
    x += width + gap;
    fill_round_rect(x, y + height / 3 - thickness / 2, thickness, thickness, thickness / 2, color);
    fill_round_rect(x, y + (2 * height) / 3 - thickness / 2, thickness, thickness, thickness / 2, color);
    x += thickness + gap;
    draw_segment_digit(x, y, width, height, thickness, minute / 10, color);
    x += width + gap;
    draw_segment_digit(x, y, width, height, thickness, minute % 10, color);
    return x + width;
}

static const char *weekday_name(int weekday)
{
    static const char *weekdays_en[] = {
        "SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY",
        "THURSDAY", "FRIDAY", "SATURDAY"
    };
    static const char *weekdays_tr[] = {
        "PAZAR", "PAZARTESI", "SALI", "CARSAMBA",
        "PERSEMBE", "CUMA", "CUMARTESI"
    };
    if (weekday < 0 || weekday > 6) {
        weekday = 0;
    }
    return language_turkish ? weekdays_tr[weekday] : weekdays_en[weekday];
}

static uint16_t mix_color(uint16_t from, uint16_t to, unsigned amount)
{
    uint16_t a = (uint16_t)((from << 8) | (from >> 8));
    uint16_t b = (uint16_t)((to << 8) | (to >> 8));
    unsigned red = (((a >> 11) & 0x1f) * (255 - amount) + ((b >> 11) & 0x1f) * amount) / 255;
    unsigned green = (((a >> 5) & 0x3f) * (255 - amount) + ((b >> 5) & 0x3f) * amount) / 255;
    unsigned blue = ((a & 0x1f) * (255 - amount) + (b & 0x1f) * amount) / 255;
    uint16_t mixed = (uint16_t)((red << 11) | (green << 5) | blue);
    return (uint16_t)((mixed << 8) | (mixed >> 8));
}

static void fill_gradient_card(int x, int y, int width, int height, int radius,
                               uint16_t top, uint16_t bottom)
{
    if (radius * 2 > width) {
        radius = width / 2;
    }
    for (int row = 0; row < height; ++row) {
        uint16_t color = mix_color(top, bottom, (unsigned)(row * 255 / (height - 1)));
        int edge = row < radius ? radius - 1 - row :
                   (row >= height - radius ? row - (height - radius) : -1);
        int inset = 0;
        unsigned alpha = 255;
        if (edge >= 0) {
            float dy = (float)edge + 0.5f;
            float dx = sqrtf((float)(radius * radius) - dy * dy);
            float start = (float)radius - dx;
            inset = (int)ceilf(start);
            alpha = (unsigned)((float)inset - start > 0.0f ? ((float)inset - start) * 255.0f : 255.0f);
        }
        fill_rect(x + inset, y + row, width - 2 * inset, 1, color);
        if (inset > 0) {
            blend_pixel(x + inset - 1, y + row, color, alpha);
            blend_pixel(x + width - inset, y + row, color, alpha);
        }
    }
}

static void draw_cloud(int cx, int cy, int r, uint16_t color)
{
    int side = r * 2 / 5 + 1;
    fill_circle(cx - r / 2, cy + r / 6, side, color);
    fill_circle(cx + r / 2, cy + r / 6, side, color);
    fill_circle(cx, cy - r / 8, r / 2 + 1, color);
    int height = r / 6 + side;
    fill_round_rect(cx - r * 9 / 10, cy, r * 9 / 5, height, height / 2, color);
}

static void draw_weather_icon(int cx, int cy, int code, int r)
{
    const uint16_t sun = color565(255, 200, 60);
    const uint16_t light = color565(232, 240, 250);
    const uint16_t dark = color565(160, 176, 200);
    const uint16_t drop = color565(90, 170, 255);
    int drop_width = r / 8 + 1;
    if (code <= 1) {
        if (r >= 9) {
            for (int ray = 0; ray < 8; ++ray) {
                float angle = (float)ray * 0.785398f;
                fill_circle(cx + (int)(cosf(angle) * (float)r * 0.9f),
                            cy + (int)(sinf(angle) * (float)r * 0.9f), r >= 14 ? 2 : 1, sun);
            }
        }
        fill_circle(cx, cy, r * 6 / 10 + 1, sun);
    } else if (code == 2) {
        fill_circle(cx + r / 3, cy - r / 3, r * 4 / 10 + 1, sun);
        draw_cloud(cx - r / 6, cy + r / 6, r * 8 / 10, light);
    } else if (code == 3 || code == 45 || code == 48) {
        draw_cloud(cx, cy, r, code == 3 ? light : dark);
    } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
        draw_cloud(cx, cy - r / 4, r * 9 / 10, dark);
        for (int i = 0; i < 3; ++i) {
            fill_round_rect(cx - r / 2 + i * r / 2, cy + r / 2, drop_width, r / 3 + 1, 1, drop);
        }
    } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
        draw_cloud(cx, cy - r / 4, r * 9 / 10, dark);
        for (int i = 0; i < 3; ++i) {
            fill_circle(cx - r / 2 + i * r / 2, cy + r * 6 / 10, r / 9 + 1, C_TEXT);
        }
    } else {
        draw_cloud(cx, cy - r / 4, r * 9 / 10, dark);
        draw_line(cx + r / 6, cy + r / 4, cx - r / 5, cy + r * 3 / 5, sun);
        draw_line(cx - r / 5, cy + r * 3 / 5, cx + r / 6, cy + r * 3 / 5, sun);
        draw_line(cx + r / 6, cy + r * 3 / 5, cx - r / 6, cy + r, sun);
    }
}

static const char *weather_text(int code)
{
    if (code <= 1) return "CLEAR";
    if (code == 2) return "PARTLY CLOUDY";
    if (code == 3) return "CLOUDY";
    if (code == 45 || code == 48) return "FOG";
    if (code >= 51 && code <= 57) return "DRIZZLE";
    if (code >= 61 && code <= 67) return "RAIN";
    if (code >= 71 && code <= 77) return "SNOW";
    if (code >= 80 && code <= 82) return "SHOWERS";
    if (code == 85 || code == 86) return "SNOW";
    return "STORM";
}

static bool weather_ready(void)
{
    return weather_valid && weather_data_province == (int)weather_location_gen;
}

static void draw_weather_hours(int x, int top, int column)
{
    for (int i = 0; i < weather_data.hour_count; ++i) {
        int center = x + i * column + column / 2;
        char text[8];
        snprintf(text, sizeof(text), "%02d", weather_data.hours[i]);
        draw_centered_text(center, top, text, 1, C_MUTED);
        draw_weather_icon(center, top + 19, weather_data.hour_code[i], 5);
        snprintf(text, sizeof(text), "%d", (int)lroundf(weather_data.hour_temperature[i]));
        draw_centered_text(center, top + 32, text, 1, C_TEXT);
    }
}

static void draw_home_widget(unsigned item, int left, int width, struct tm *local_time,
                             bool valid_time)
{
    const int center_x = left + width / 2;
    if (item == 0) {
        draw_big_time(left + (width - 142) / 2, 38, 26, 46, 6,
                      valid_time ? local_time->tm_hour : 0,
                      valid_time ? local_time->tm_min : 0, C_TEXT);
        char text[24] = "--";
        if (valid_time) {
            snprintf(text, sizeof(text), "%s %02d.%02d", weekday_name(local_time->tm_wday),
                     local_time->tm_mday, local_time->tm_mon + 1);
        }
        draw_centered_text(center_x, 100, text, 1, C_TEXT);
        draw_centered_text(center_x, 114, clock_cities[clock_city].name, 1, C_MUTED);
        draw_progress(left + 30, 134, width - 60, 4,
                      valid_time ? (unsigned)(local_time->tm_sec * 100 / 59) : 0, C_TEXT);
        return;
    }
    if (weather_ready()) {
        draw_weather_icon(left + 28, 56, weather_data.code, 17);
        char text[12];
        snprintf(text, sizeof(text), "%dC", (int)lroundf(weather_data.temperature));
        draw_text(left + 52, 40, text, 3, C_TEXT);
        draw_text(left + 52, 70, weather_text(weather_data.code), 1, C_TEXT);
        draw_text(left + 52, 82, weather_location.name, 1, C_MUTED);
        fill_rect(left + 6, 94, width - 12, 1, C_LINE);
        draw_weather_hours(left + 6, 100, (width - 12) / WEATHER_HOURS);
        return;
    }
    draw_weather_icon(center_x, 62, 2, 20);
    draw_centered_text(center_x, 100, weather_location.name, 1, C_TEXT);
    draw_centered_text(center_x, 118,
                       weather_loading ? "LOADING" :
                       (wifi_state != WIFI_STATE_CONNECTED ? "WIFI NEEDED" : "NO DATA"),
                       1, C_MUTED);
}

static void draw_home(void)
{
    draw_background();
    draw_status_bar();

    time_t now = time(NULL);
    struct tm local_time = {0};
    bool valid_time = localtime_r(&now, &local_time) != NULL;
    char time_text[8] = "--:--";
    if (valid_time) {
        strftime(time_text, sizeof(time_text), "%H:%M", &local_time);
    }
    draw_centered_text(160, 6, time_text, 1, C_TEXT);

    const char *labels[HOME_ITEM_COUNT] = {"CLOCK", "WEATHER", "GAME", "BLUETOOTH",
                                           "LANGUAGE", "CITY / TIME", "SD APPS", "GALLERY",
                                                                                      "LED", "POWER", "SETTINGS"};
                                               const unsigned icon_ids[HOME_ITEM_COUNT] = {0, 0, 1, 5, 3, 7, 8, 10, 11, 12, 9};
    const uint16_t tops[HOME_ITEM_COUNT] = {
        color565(52, 211, 153), color565(56, 189, 248), color565(255, 190, 70),
        color565(176, 148, 255), color565(90, 230, 140), color565(110, 170, 255),
        color565(236, 130, 250), color565(251, 146, 160), color565(250, 120, 220),
                color565(120, 230, 90), color565(255, 214, 40)
            };
    const uint16_t bottoms[HOME_ITEM_COUNT] = {
        color565(4, 120, 87), color565(37, 70, 200), color565(236, 72, 120),
        color565(91, 33, 182), color565(13, 148, 136), color565(30, 64, 175),
        color565(126, 34, 206), color565(190, 24, 93), color565(109, 40, 217),
                color565(21, 128, 61), color565(234, 88, 12)
            };
    static const int offsets[] = {-2, 2, -1, 1, 0};
    static const int widths[] = {8, 8, 52, 52, 180};
    static const int lefts[] = {2, 310, 14, 254, 70};
    for (unsigned slot = 0; slot < 5; ++slot) {
        unsigned item = (selected_item + HOME_ITEM_COUNT + (unsigned)(offsets[slot] + (int)HOME_ITEM_COUNT)) %
                        HOME_ITEM_COUNT;
        bool center = offsets[slot] == 0;
        unsigned fade = center ? 0 : (offsets[slot] == -1 || offsets[slot] == 1 ? 90 : 150);
        uint16_t top = mix_color(tops[item], C_BG_BOTTOM, fade);
        uint16_t bottom = mix_color(bottoms[item], C_BG_BOTTOM, fade);
        fill_gradient_card(lefts[slot], 28, widths[slot], 122, 14, top, bottom);
        icon_cut_color = mix_color(top, bottom, 100);
        int center_x = lefts[slot] + widths[slot] / 2;
        if (center && (item == 0 || item == 1)) {
            draw_home_widget(item, lefts[slot], widths[slot], &local_time, valid_time);
        } else if (center) {
            draw_icon(icon_ids[item], center_x, 66, C_TEXT);
            draw_centered_text(center_x, 124, labels[item], 1, C_TEXT);
        } else if (widths[slot] >= 52) {
            uint16_t faded = mix_color(C_TEXT, bottom, 90);
            if (item == 1) {
                draw_weather_icon(center_x, 86, weather_ready() ? weather_data.code : 2, 13);
            } else {
                draw_icon(icon_ids[item], center_x, 86, faded);
            }
        }
    }
    for (unsigned dot = 0; dot < HOME_ITEM_COUNT; ++dot) {
        fill_circle(160 - ((int)HOME_ITEM_COUNT - 1) * 5 + (int)dot * 10, 160,
                    dot == selected_item ? 2 : 1, dot == selected_item ? C_TEXT : C_LINE);
    }
}

static void draw_settings(void)
{
    const char *labels[SETTINGS_ITEM_COUNT] = {"LIGHT", "SYSTEM", "WIFI", "SLEEP TIMER",
                                               "WIFI SWITCH", "BT SWITCH", "FLIP SCREEN", "FLIP JOYSTICK",
                                               "POWER SAVER", "LED"};
    const uint16_t accents[SETTINGS_ITEM_COUNT] = {C_ORANGE, C_CYAN, C_GREEN, C_PURPLE,
                                                   C_GREEN, C_BLUE, C_ORANGE, C_CYAN,
                                                   C_GREEN, C_PURPLE};
    draw_background();
    draw_header("SETTINGS", NULL);
    unsigned selected_row = settings_selected_item / SETTINGS_COLUMN_COUNT;
    unsigned first_row = selected_row > 3 ? selected_row - 3 : 0;
    for (unsigned item = 0; item < SETTINGS_ITEM_COUNT; ++item) {
        unsigned row = item / SETTINGS_COLUMN_COUNT;
        if (row < first_row || row > first_row + 3) {
            continue;
        }
        int x = 8 + (int)(item % SETTINGS_COLUMN_COUNT) * 156;
        int y = 24 + (int)(row - first_row) * 33;
        bool selected = settings_selected_item == item;
        draw_card(x, y, 148, 30, 9, selected, accents[item]);
        fill_round_rect(x + 9, y + 6, 3, 18, 1, accents[item]);
        draw_text(x + 20, y + 6, labels[item], 1, selected ? C_TEXT : C_MUTED);

        char value[24];
        uint16_t value_color = C_MUTED;
        if (item == 0) {
            snprintf(value, sizeof(value), "%u%%", (unsigned)brightness);
        } else if (item == 1) {
            bool usb_power = system_voltage >= 4.5f;
            snprintf(value, sizeof(value), "%s", usb_power ? "USB POWER" : "BATTERY");
        } else if (item == 2) {
            if (wifi_enabled && wifi_connected_ssid[0] != '\0') {
                snprintf(value, sizeof(value), "%.14s", wifi_connected_ssid);
                value_color = C_GREEN;
            } else {
                snprintf(value, sizeof(value), "%s", wifi_enabled ? "ON" : "OFF");
            }
        } else if (item == 3) {
            if (sleep_timeout_minutes == 0) {
                snprintf(value, sizeof(value), "OFF");
            } else {
                snprintf(value, sizeof(value), language_turkish ? "%u DK" : "%u MIN",
                         (unsigned)sleep_timeout_minutes);
            }
        } else if (item == 8) {
            snprintf(value, sizeof(value), "%s", power_saver ? "ON" : "OFF");
            value_color = power_saver ? C_GREEN : C_MUTED;
        } else if (item == 9) {
            static const char *const led_mode_names[LED_MODE_COUNT] = {
                "OFF", "SOLID", "BREATHE", "BLINK", "RAINBOW", "ALTERNATE"};
            snprintf(value, sizeof(value), "%s", led_mode_names[led_mode]);
            value_color = led_mode == 0 ? C_MUTED : C_PURPLE;
        } else {
            bool on = item == 4 ? wifi_enabled : (item == 5 ? bluetooth_enabled :
                      (item == 6 ? screen_flipped : joystick_flipped));
            snprintf(value, sizeof(value), "%s", on ? "ON" : "OFF");
            value_color = on ? C_GREEN : C_ORANGE;
            draw_switch(x + 106, y + 7, on);
        }
        draw_text(x + 20, y + 17, value, 1, value_color);
    }
    draw_footer("SW SELECT / LEFT BACK");
}

static void draw_hand(int center_x, int center_y, float angle, int length, int radius,
                      uint16_t color)
{
    for (int step = 0; step <= length; ++step) {
        fill_circle(center_x + (int)(cosf(angle) * (float)step),
                    center_y + (int)(sinf(angle) * (float)step), radius, color);
    }
}

static void draw_clock(void)
{
    char clock_subtitle[24];
    snprintf(clock_subtitle, sizeof(clock_subtitle), "%s", clock_cities[clock_city].name);
    draw_background();
    draw_header("CLOCK", clock_subtitle);

    time_t now = time(NULL);
    struct tm local_time = {0};
    char date_text[16] = "--";
    if (localtime_r(&now, &local_time) != NULL) {
        strftime(date_text, sizeof(date_text), "%d.%m.%Y", &local_time);
    }

    const int center_x = 84;
    const int center_y = 90;
    fill_circle(center_x, center_y, 62, C_LINE);
    fill_circle(center_x, center_y, 60, C_CARD);
    for (int tick = 0; tick < 12; ++tick) {
        float angle = (float)tick * 0.523599f - 1.570796f;
        int inner = tick % 3 == 0 ? 49 : 53;
        for (int step = inner; step <= 56; ++step) {
            fill_circle(center_x + (int)(cosf(angle) * (float)step),
                        center_y + (int)(sinf(angle) * (float)step),
                        tick % 3 == 0 ? 1 : 0, tick % 3 == 0 ? C_CYAN : C_MUTED);
        }
    }
    float hour_angle = ((float)(local_time.tm_hour % 12) * 30.0f +
                        (float)local_time.tm_min * 0.5f - 90.0f) * 0.0174533f;
    float minute_angle = ((float)local_time.tm_min * 6.0f +
                          (float)local_time.tm_sec * 0.1f - 90.0f) * 0.0174533f;
    float second_angle = ((float)local_time.tm_sec * 6.0f - 90.0f) * 0.0174533f;
    draw_hand(center_x, center_y, hour_angle, 28, 2, C_TEXT);
    draw_hand(center_x, center_y, minute_angle, 42, 1, C_BLUE);
    draw_hand(center_x, center_y, second_angle, 48, 0, C_ORANGE);
    fill_circle(center_x, center_y, 4, C_ORANGE);
    fill_circle(center_x, center_y, 2, C_CARD);

    draw_card(160, 28, 152, 124, 14, false, C_BLUE);
    draw_big_time(172, 40, 24, 42, 6, local_time.tm_hour, local_time.tm_min, C_TEXT);
    draw_progress(172, 96, 128, 5, (unsigned)(local_time.tm_sec * 100 / 59), C_BLUE);
    draw_text(172, 110, weekday_name(local_time.tm_wday), 1, C_CYAN);
    draw_text(172, 124, date_text, 2, C_TEXT);
    draw_footer("CLICK BACK");
}

#define GAME_COLUMNS 18
#define GAME_ROWS 9
#define GAME_CELL 14
#define GAME_ORIGIN_X 8
#define GAME_ORIGIN_Y 26
#define SNAKE_CAPACITY (GAME_COLUMNS * GAME_ROWS)

typedef struct {
    int8_t x;
    int8_t y;
} point_t;

static point_t snake[SNAKE_CAPACITY];
static unsigned snake_length = 4;
static point_t apple = {7, 6};
static direction_t snake_direction = DIR_RIGHT;

static void spawn_apple(void)
{
    do {
        apple.x = (int8_t)(esp_random() % GAME_COLUMNS);
        apple.y = (int8_t)(esp_random() % GAME_ROWS);
        bool occupied = false;
        for (unsigned i = 0; i < snake_length; ++i) {
            if (snake[i].x == apple.x && snake[i].y == apple.y) {
                occupied = true;
                break;
            }
        }
        if (!occupied) {
            return;
        }
    } while (true);
}

static void reset_game(void)
{
    snake_length = 4;
    snake[0] = (point_t){5, 6};
    snake[1] = (point_t){4, 6};
    snake[2] = (point_t){3, 6};
    snake[3] = (point_t){2, 6};
    snake_direction = DIR_RIGHT;
    game_score = 0;
    game_over = false;
    spawn_apple();
    last_game_step_us = esp_timer_get_time();
}

static void step_game(void)
{
    if (game_over) {
        return;
    }
    point_t next = snake[0];
    if (snake_direction == DIR_UP) {
        --next.y;
    } else if (snake_direction == DIR_DOWN) {
        ++next.y;
    } else if (snake_direction == DIR_LEFT) {
        --next.x;
    } else if (snake_direction == DIR_RIGHT) {
        ++next.x;
    }
    if (next.x < 0 || next.x >= GAME_COLUMNS || next.y < 0 || next.y >= GAME_ROWS) {
        game_over = true;
        return;
    }

    bool ate_apple = next.x == apple.x && next.y == apple.y;
    unsigned collision_length = ate_apple ? snake_length : snake_length - 1;
    for (unsigned i = 0; i < collision_length; ++i) {
        if (snake[i].x == next.x && snake[i].y == next.y) {
            game_over = true;
            return;
        }
    }
    if (ate_apple && snake_length < SNAKE_CAPACITY) {
        ++snake_length;
        ++game_score;
    }
    for (unsigned i = snake_length - 1; i > 0; --i) {
        snake[i] = snake[i - 1];
    }
    snake[0] = next;
    if (ate_apple) {
        if (snake_length == SNAKE_CAPACITY) {
            game_over = true;
        } else {
            spawn_apple();
        }
    }
}

static void draw_game(void)
{
    draw_background();
    draw_header("SNAKE", NULL);

    fill_round_rect(GAME_ORIGIN_X - 2, GAME_ORIGIN_Y - 2, GAME_COLUMNS * GAME_CELL + 4,
                    GAME_ROWS * GAME_CELL + 4, 8, C_LINE);
    fill_round_rect(GAME_ORIGIN_X - 1, GAME_ORIGIN_Y - 1, GAME_COLUMNS * GAME_CELL + 2,
                    GAME_ROWS * GAME_CELL + 2, 7, C_BAR);
    fill_circle(GAME_ORIGIN_X + apple.x * GAME_CELL + GAME_CELL / 2,
                GAME_ORIGIN_Y + apple.y * GAME_CELL + GAME_CELL / 2,
                GAME_CELL / 3, C_RED);
    for (unsigned i = 0; i < snake_length; ++i) {
        int x = GAME_ORIGIN_X + snake[i].x * GAME_CELL + 1;
        int y = GAME_ORIGIN_Y + snake[i].y * GAME_CELL + 1;
        fill_round_rect(x, y, GAME_CELL - 2, GAME_CELL - 2, 5, i == 0 ? C_CYAN : C_GREEN);
    }

    draw_card(268, 26, 44, 60, 10, false, C_ORANGE);
    draw_centered_text(290, 36, "SCORE", 1, C_MUTED);
    char score_text[8];
    snprintf(score_text, sizeof(score_text), "%03u", game_score);
    draw_centered_text(290, 56, score_text, 2, C_ORANGE);

    if (game_over) {
        draw_card(66, 62, 140, 54, 12, true, C_ORANGE);
        draw_centered_text(136, 74, "GAME OVER", 1, C_ORANGE);
        draw_centered_text(136, 92, "MOVE RESTART", 1, C_TEXT);
    } else {
        draw_footer("CLICK BACK");
    }
}

static void draw_brightness(void)
{
    draw_background();
    draw_header("BRIGHTNESS", NULL);

    draw_card(12, 28, 296, 122, 14, false, C_ORANGE);
    for (int ray = 0; ray < 8; ++ray) {
        float angle = (float)ray * 0.785398f;
        fill_circle(60 + (int)(cosf(angle) * 30.0f), 78 + (int)(sinf(angle) * 30.0f), 3,
                    C_ORANGE);
    }
    fill_circle(60, 78, 18, C_ORANGE);
    fill_circle(60, 78, 10, C_CARD);

    char value_text[8];
    snprintf(value_text, sizeof(value_text), "%u%%", (unsigned)brightness);
    draw_text_right(296, 48, value_text, 4, C_TEXT);
    draw_progress(112, 100, 184, 8, brightness, C_ORANGE);
    draw_text(112, 122, "LEFT", 1, C_MUTED);
    draw_text_right(296, 122, "RIGHT", 1, C_MUTED);
    draw_footer("CLICK BACK");
}

static unsigned battery_percentage(float voltage)
{
    static const float curve_voltage[] = {3.30f, 3.50f, 3.62f, 3.72f, 3.80f, 3.88f, 3.97f, 4.08f, 4.20f};
    static const float curve_percent[] = {0.0f, 5.0f, 15.0f, 30.0f, 45.0f, 60.0f, 75.0f, 90.0f, 100.0f};
    const unsigned points = sizeof(curve_voltage) / sizeof(curve_voltage[0]);
    if (voltage >= curve_voltage[points - 1]) {
        return 100;
    }
    if (voltage <= curve_voltage[0]) {
        return 0;
    }
    for (unsigned i = 1; i < points; ++i) {
        if (voltage <= curve_voltage[i]) {
            float span = curve_voltage[i] - curve_voltage[i - 1];
            float ratio = (voltage - curve_voltage[i - 1]) / span;
            return (unsigned)(curve_percent[i - 1] + ratio * (curve_percent[i] - curve_percent[i - 1]) + 0.5f);
        }
    }
    return 100;
}

static void draw_battery_icon(int x, int y, unsigned percentage, uint16_t color)
{
    fill_round_rect(x, y, 56, 28, 6, color);
    fill_round_rect(x + 2, y + 2, 52, 24, 5, C_BAR);
    fill_round_rect(x + 5, y + 5, (46 * (int)percentage) / 100, 18, 3,
                    percentage > 20 ? C_GREEN : C_RED);
    fill_round_rect(x + 56, y + 8, 4, 12, 2, color);
}

static void draw_system_row(int y, const char *label, const char *value, uint16_t color)
{
    draw_text(160, y, label, 1, C_MUTED);
    draw_text_right(300, y, value, 1, color);
}

static void draw_system(void)
{
    draw_background();
    draw_header("SYSTEM", NULL);

    bool usb_power = system_voltage >= 4.5f;
    unsigned battery = battery_percentage(system_voltage);
    draw_card(10, 28, 140, 122, 14, false, C_GREEN);
    draw_battery_icon(30, 44, battery, C_TEXT);
    char battery_text[8];
    snprintf(battery_text, sizeof(battery_text), "%u%%", battery);
    draw_centered_text(80, 82, battery_text, 3, C_TEXT);
    char voltage_text[16];
    snprintf(voltage_text, sizeof(voltage_text), "%.2f V", system_voltage);
    draw_centered_text(80, 108, voltage_text, 1, C_CYAN);
    draw_centered_text(80, 121, usb_power ? "USB POWER" : "BATTERY", 1,
                       usb_power ? C_GREEN : C_ORANGE);
    char estimate_text[20];
    format_battery_time(estimate_text, sizeof(estimate_text));
    char estimate_line[40];
    snprintf(estimate_line, sizeof(estimate_line), "%s %s%s", language_turkish ? "KALAN" : "LEFT",
             usb_power ? "USB " : "", estimate_text);
    draw_centered_text(80, 135, estimate_line, 1, C_GREEN);

    draw_card(156, 28, 154, 122, 14, false, C_BLUE);
    draw_system_row(36, "SYSTEM", "REXAUI", C_CYAN);
    char text[20];
    snprintf(text, sizeof(text), "%u MHZ", (unsigned)(esp_clk_cpu_freq() / 1000000U));
    draw_system_row(50, "CPU", text, C_TEXT);
    snprintf(text, sizeof(text), "%u KB", (unsigned)(esp_get_free_heap_size() / 1024U));
    draw_system_row(64, "FREE RAM", text, C_TEXT);
    uint32_t uptime = (uint32_t)((esp_timer_get_time() - app_started_us) / 1000000LL);
    snprintf(text, sizeof(text), "%02u:%02u:%02u", (unsigned)(uptime / 3600U),
             (unsigned)((uptime / 60U) % 60U), (unsigned)(uptime % 60U));
    draw_system_row(78, "UPTIME", text, C_TEXT);
    snprintf(text, sizeof(text), "%u%%", (unsigned)brightness);
    draw_system_row(92, "BRIGHT", text, C_TEXT);
    draw_system_row(106, "WIFI", wifi_enabled ? "ON" : "OFF", wifi_enabled ? C_GREEN : C_ORANGE);
    draw_system_row(120, "BLUETOOTH", bluetooth_enabled ? "ON" : "OFF",
                    bluetooth_enabled ? C_GREEN : C_ORANGE);
    draw_footer("CLICK / LEFT BACK");
}

static void draw_power(void)
{
    draw_background();
    draw_header("POWER SAVER", NULL);
    for (unsigned row = 0; row < 2; ++row) {
        int y = 26 + (int)row * 34;
        bool selected = power_selected_row == row;
        uint16_t accent = row == 0 ? C_GREEN : C_CYAN;
        draw_card(10, y, 300, 30, 9, selected, accent);
        fill_round_rect(19, y + 6, 3, 18, 1, accent);
        draw_text(30, y + 11, row == 0 ? "POWER SAVER" : "BATTERY SIZE", 1,
                  selected ? C_TEXT : C_MUTED);
        if (row == 0) {
            draw_switch(266, y + 7, power_saver);
            draw_text_right(256, y + 11, power_saver ? "ON" : "OFF", 1,
                            power_saver ? C_GREEN : C_ORANGE);
        } else {
            char text[16];
            snprintf(text, sizeof(text), "%u MAH",
                     (unsigned)battery_capacity_presets[battery_capacity_index]);
            draw_text_right(298, y + 11, text, 1, C_TEXT);
        }
    }

    draw_card(10, 96, 300, 54, 12, false, C_BLUE);
    char text[24];
    draw_text(22, 104, "TIME LEFT", 1, C_MUTED);
    format_battery_time(text, sizeof(text));
    draw_text(22, 121, text, 2, power_est_minutes == -2 ? C_CYAN : C_GREEN);
    snprintf(text, sizeof(text), "%u MA", (unsigned)(power_est_ma + 0.5f));
    draw_text(176, 104, "DRAW", 1, C_MUTED);
    draw_text_right(298, 104, text, 1, C_TEXT);
    snprintf(text, sizeof(text), "%u%%", battery_level);
    draw_text(176, 118, "BATTERY", 1, C_MUTED);
    draw_text_right(298, 118, text, 1, battery_level <= 15 ? C_RED : C_TEXT);
    unsigned shown = power_saver && brightness > POWER_SAVER_BRIGHTNESS ? POWER_SAVER_BRIGHTNESS : brightness;
    snprintf(text, sizeof(text), "%u%%", shown);
    draw_text(176, 132, "SCREEN", 1, C_MUTED);
    draw_text_right(298, 132, text, 1, C_TEXT);
    draw_footer("SW CHANGE / LEFT BACK");
}

static void draw_led(void)
{
    static const char *const mode_names[LED_MODE_COUNT] = {
        "OFF", "SOLID", "BREATHE", "BLINK", "RAINBOW", "ALTERNATE"};
    static const char *const color_names[LED_COLOR_COUNT] = {
        "RED", "ORANGE", "YELLOW", "GREEN", "CYAN", "BLUE", "PURPLE", "WHITE"};
    static const char *const row_labels[4] = {"MODE", "COLOR", "SPEED", "BRIGHT"};
    draw_background();
    draw_header("LED", NULL);
    for (unsigned row = 0; row < 4; ++row) {
        int y = 24 + (int)row * 33;
        bool selected = led_selected_row == row;
        draw_card(8, y, 192, 30, 9, selected, C_PURPLE);
        fill_round_rect(17, y + 6, 3, 18, 1, C_PURPLE);
        draw_text(28, y + 11, row_labels[row], 1, selected ? C_TEXT : C_MUTED);
        char text[16];
        if (row == 0) {
            snprintf(text, sizeof(text), "%s", mode_names[led_mode]);
        } else if (row == 1) {
            snprintf(text, sizeof(text), "%s", color_names[led_color]);
            fill_circle(112, y + 15, 6, color565(led_palette[led_color][0],
                                                 led_palette[led_color][1],
                                                 led_palette[led_color][2]));
        } else if (row == 2) {
            snprintf(text, sizeof(text), "%u/%u", (unsigned)led_speed + 1U, (unsigned)LED_SPEED_COUNT);
        } else {
            snprintf(text, sizeof(text), "%u%%", (unsigned)led_level);
        }
        draw_text_right(190, y + 11, text, 1, C_TEXT);
    }

    draw_card(208, 24, 104, 129, 12, false, C_PURPLE);
    bool lit = led_preview[0] != 0 || led_preview[1] != 0 || led_preview[2] != 0;
    fill_circle(260, 70, 30, C_LINE);
    fill_circle(260, 70, 27, lit ? color565(led_preview[0], led_preview[1], led_preview[2]) : C_BAR);
    draw_centered_text(260, 116, mode_names[led_mode], 1, C_TEXT);
    if (power_saver) {
        draw_centered_text(260, 132, "SAVER ON", 1, C_ORANGE);
    }
    draw_footer("SW CHANGE / LEFT BACK");
}

static void draw_sleep_settings(void)
{
    draw_background();
    draw_header("SLEEP TIMER", NULL);
    draw_card(12, 28, 296, 122, 14, false, C_PURPLE);
    char minute_text[24];
    if (sleep_timeout_minutes == 0) {
        snprintf(minute_text, sizeof(minute_text), "OFF");
    } else {
        snprintf(minute_text, sizeof(minute_text), "%u MIN", (unsigned)sleep_timeout_minutes);
    }
    draw_centered_text(160, 44, "TIMEOUT", 1, C_MUTED);
    draw_centered_text(160, 68, minute_text, 4, C_CYAN);
    draw_centered_text(160, 112, "UP/DOWN ADJUST", 1, C_TEXT);
    draw_centered_text(160, 130, "0 = DISABLED", 1, C_MUTED);
    draw_footer("CLICK / LEFT BACK");
}

static void draw_language(void)
{
    draw_background();
    draw_header("LANGUAGE", NULL);

    const char *languages[] = {"ENGLISH", "TURKCE"};
    for (unsigned item = 0; item < 2; ++item) {
        int x = 12 + (int)item * 152;
        bool selected = item == language_selected_item;
        bool active = (item == 1) == language_turkish;
        draw_card(x, 38, 144, 90, 14, selected, C_CYAN);
        draw_centered_text(x + 72, 70, languages[item], 2, active ? C_GREEN : C_TEXT);
        if (active) {
            draw_centered_text(x + 72, 100, "ON", 1, C_GREEN);
        }
    }
    draw_footer("USE SW TO SELECT / LEFT BACK");
}

static void draw_clock_settings(void)
{
    draw_background();
    draw_header("CITY / TIME", NULL);

    const unsigned count = sizeof(clock_cities) / sizeof(clock_cities[0]);
    const unsigned visible = 4;
    unsigned first = list_first(clock_city_selected, count, visible);
    for (unsigned row = 0; row < visible && first + row < count; ++row) {
        unsigned i = first + row;
        int y = 28 + (int)row * 31;
        bool active = i == clock_city;
        draw_card(10, y, 298, 27, 8, i == clock_city_selected, C_CYAN);
        draw_text(22, y + 10, clock_cities[i].name, 1, active ? C_GREEN : C_TEXT);
        if (active) {
            draw_text_right(296, y + 10, "ON", 1, C_GREEN);
        }
    }
    draw_scroll_bar(first, count, visible, 28, 120);
    draw_footer(wifi_enabled ? "SW SELECT / WIFI SYNC" : "TIME SYNC NEEDS WIFI");
}

static void draw_weather(void)
{
    draw_background();
    draw_header("WEATHER", NULL);

    int name_scale = text_width(weather_location.name, 2) <= 300 ? 2 : 1;
    draw_centered_text(160, name_scale == 2 ? 20 : 24, weather_location.name, name_scale, C_CYAN);

    if (weather_ready()) {
        char text[24];
        draw_weather_icon(58, 68, weather_data.code, 22);
        snprintf(text, sizeof(text), "%dC", (int)lroundf(weather_data.temperature));
        draw_text(110, 44, text, 4, C_TEXT);
        draw_text(110, 78, weather_text(weather_data.code), 1, C_TEXT);
        snprintf(text, sizeof(text), "FEELS %dC", (int)lroundf(weather_data.apparent));
        draw_text(218, 48, text, 1, C_MUTED);
        snprintf(text, sizeof(text), "HUMIDITY %d%%", weather_data.humidity);
        draw_text(218, 62, text, 1, C_MUTED);
        snprintf(text, sizeof(text), "WIND %d KMH", (int)lroundf(weather_data.wind));
        draw_text(218, 76, text, 1, C_MUTED);
        fill_rect(10, 100, 300, 1, C_LINE);
        draw_weather_hours(10, 108, 50);
    } else {
        draw_weather_icon(160, 70, 2, 24);
        const char *status = weather_loading ? "LOADING" :
                             (wifi_state != WIFI_STATE_CONNECTED ? "WIFI NEEDED" : "NO DATA");
        draw_centered_text(160, 112, status, 1, C_MUTED);
    }
    const char *hint = "SW SEARCH CITY / LEFT BACK";
    if (geo_state == 1) {
        hint = "SEARCHING...";
    } else if (geo_state == 2) {
        hint = "CITY NOT FOUND";
    } else if (geo_state == 3) {
        hint = "SEARCH NEEDS WIFI";
    }
    draw_footer(hint);
}

static bool sd_ascii_text_is_valid(const char *text, size_t capacity)
{
    size_t length = strlen(text);
    if (length == 0 || length >= capacity) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        unsigned char character = (unsigned char)text[i];
        if (character < 32 || character > 126) {
            return false;
        }
    }
    return true;
}

static char *sd_trim(char *text)
{
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    size_t length = strlen(text);
    while (length > 0 &&
           (text[length - 1] == ' ' || text[length - 1] == '\t' ||
            text[length - 1] == '\r' || text[length - 1] == '\n')) {
        text[--length] = '\0';
    }
    return text;
}

static void *sd_lua_allocator(void *user_data, void *pointer, size_t old_size,
                              size_t new_size)
{
    size_t *used = user_data;
    if (new_size == 0) {
        if (pointer != NULL) {
            *used = old_size <= *used ? *used - old_size : 0;
            free(pointer);
        }
        return NULL;
    }
    size_t previous_size = pointer == NULL ? 0 : old_size;
    if (previous_size > *used ||
        new_size > SD_LUA_MEMORY_LIMIT - (*used - previous_size)) {
        return NULL;
    }
    void *resized = realloc(pointer, new_size);
    if (resized != NULL) {
        *used = *used - previous_size + new_size;
    }
    return resized;
}

static void sd_lua_instruction_hook(lua_State *state, lua_Debug *debug)
{
    (void)debug;
    if (sd_lua_instructions_left < 1000) {
        sd_lua_instructions_left = 0;
        luaL_error(state, "instruction limit exceeded");
    }
    sd_lua_instructions_left -= 1000;
}

static void sd_lua_remove_global(lua_State *state, const char *name)
{
    lua_pushnil(state);
    lua_setglobal(state, name);
}

static uint16_t sd_lua_color(lua_Integer rgb)
{
    uint32_t color = (uint32_t)rgb & 0x00ffffffU;
    return color565((uint8_t)(color >> 16), (uint8_t)(color >> 8),
                    (uint8_t)color);
}

static int sd_lua_clear(lua_State *state)
{
    uint16_t color = lua_isnoneornil(state, 1) ?
                     C_BG_BOTTOM :
                     sd_lua_color(luaL_checkinteger(state, 1));
    fill_screen(color);
    return 0;
}

static int sd_lua_text(lua_State *state)
{
    lua_Integer x_value = luaL_checkinteger(state, 1);
    lua_Integer y_value = luaL_checkinteger(state, 2);
    size_t length = 0;
    const char *text = luaL_checklstring(state, 3, &length);
    lua_Integer rgb = luaL_optinteger(state, 4, 0xffffff);
    lua_Integer scale = luaL_optinteger(state, 5, 1);
    if (length > 64 || scale < 1 || scale > 3) {
        return luaL_error(state, "text length or scale out of range");
    }
    if (x_value < -LCD_WIDTH || x_value > LCD_WIDTH ||
        y_value < -LCD_HEIGHT || y_value > LCD_HEIGHT) {
        return luaL_error(state, "text position out of range");
    }
    int x = (int)x_value;
    int y = (int)y_value;
    char bounded_text[65];
    memcpy(bounded_text, text, length);
    bounded_text[length] = '\0';
    draw_text(x, y, bounded_text, (int)scale, sd_lua_color(rgb));
    return 0;
}

static int sd_lua_fill_rect(lua_State *state)
{
    lua_Integer x_value = luaL_checkinteger(state, 1);
    lua_Integer y_value = luaL_checkinteger(state, 2);
    lua_Integer width_value = luaL_checkinteger(state, 3);
    lua_Integer height_value = luaL_checkinteger(state, 4);
    lua_Integer rgb = luaL_optinteger(state, 5, 0xffffff);
    if (x_value < -LCD_WIDTH || x_value > LCD_WIDTH ||
        y_value < -LCD_HEIGHT || y_value > LCD_HEIGHT ||
        width_value < 0 || width_value > LCD_WIDTH ||
        height_value < 0 || height_value > LCD_HEIGHT) {
        return luaL_error(state, "rectangle out of range");
    }
    int x = (int)x_value;
    int y = (int)y_value;
    int width = (int)width_value;
    int height = (int)height_value;
    fill_rect(x, y, width, height, sd_lua_color(rgb));
    return 0;
}

static int sd_lua_circle(lua_State *state)
{
    lua_Integer x_value = luaL_checkinteger(state, 1);
    lua_Integer y_value = luaL_checkinteger(state, 2);
    lua_Integer radius_value = luaL_checkinteger(state, 3);
    lua_Integer rgb = luaL_optinteger(state, 4, 0xffffff);
    if (x_value < -LCD_WIDTH || x_value > LCD_WIDTH ||
        y_value < -LCD_HEIGHT || y_value > LCD_HEIGHT ||
        radius_value < 0 || radius_value > LCD_WIDTH) {
        return luaL_error(state, "circle out of range");
    }
    int x = (int)x_value;
    int y = (int)y_value;
    int radius = (int)radius_value;
    fill_circle(x, y, radius, sd_lua_color(rgb));
    return 0;
}

static int sd_lua_input(lua_State *state)
{
    const char *direction = "none";
    switch (sd_lua_input_direction) {
    case DIR_UP: direction = "up"; break;
    case DIR_DOWN: direction = "down"; break;
    case DIR_LEFT: direction = "left"; break;
    case DIR_RIGHT: direction = "right"; break;
    case DIR_NONE: break;
    }
    lua_createtable(state, 0, 2);
    lua_pushstring(state, direction);
    lua_setfield(state, -2, "direction");
    lua_pushboolean(state, sd_lua_button_pressed);
    lua_setfield(state, -2, "pressed");
    return 1;
}

static int sd_lua_clock(lua_State *state)
{
    time_t now = time(NULL);
    struct tm local_time = {0};
    if (localtime_r(&now, &local_time) == NULL) {
        lua_pushnil(state);
        return 1;
    }
    lua_createtable(state, 0, 7);
    lua_pushinteger(state, local_time.tm_year + 1900);
    lua_setfield(state, -2, "year");
    lua_pushinteger(state, local_time.tm_mon + 1);
    lua_setfield(state, -2, "month");
    lua_pushinteger(state, local_time.tm_mday);
    lua_setfield(state, -2, "day");
    lua_pushinteger(state, local_time.tm_hour);
    lua_setfield(state, -2, "hour");
    lua_pushinteger(state, local_time.tm_min);
    lua_setfield(state, -2, "minute");
    lua_pushinteger(state, local_time.tm_sec);
    lua_setfield(state, -2, "second");
    lua_pushinteger(state, (lua_Integer)now);
    lua_setfield(state, -2, "unix");
    return 1;
}

static int sd_lua_wifi(lua_State *state)
{
    lua_createtable(state, 0, 4);
    lua_pushboolean(state, wifi_enabled);
    lua_setfield(state, -2, "enabled");
    lua_pushboolean(state, wifi_state == WIFI_STATE_CONNECTED);
    lua_setfield(state, -2, "connected");
    lua_pushstring(state, wifi_state_label(wifi_state));
    lua_setfield(state, -2, "state");
    lua_pushstring(state, wifi_connected_ssid);
    lua_setfield(state, -2, "ssid");
    return 1;
}

typedef struct {
    char data[SD_LUA_HTTP_RESPONSE_LIMIT + 1];
    size_t length;
    bool overflow;
} sd_lua_http_response_t;

static esp_err_t sd_lua_http_event(esp_http_client_event_t *event)
{
    sd_lua_http_response_t *response = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        if ((size_t)event->data_len > SD_LUA_HTTP_RESPONSE_LIMIT - response->length) {
            response->overflow = true;
            return ESP_ERR_NO_MEM;
        }
        memcpy(response->data + response->length, event->data,
               (size_t)event->data_len);
        response->length += (size_t)event->data_len;
        response->data[response->length] = '\0';
    }
    return ESP_OK;
}

static bool sd_lua_json_number(const char *object_start, const char *object_end,
                              const char *key, double *value)
{
    char key_text[40];
    int length = snprintf(key_text, sizeof(key_text), "\"%s\"", key);
    if (length < 0 || (size_t)length >= sizeof(key_text)) {
        return false;
    }
    const char *position = strstr(object_start, key_text);
    if (position == NULL || position >= object_end) {
        return false;
    }
    position += strlen(key_text);
    while (position < object_end &&
           (*position == ' ' || *position == '\t' || *position == '\r' ||
            *position == '\n')) {
        ++position;
    }
    if (position >= object_end || *position++ != ':') {
        return false;
    }
    while (position < object_end &&
           (*position == ' ' || *position == '\t' || *position == '\r' ||
            *position == '\n')) {
        ++position;
    }
    char *end = NULL;
    double parsed = strtod(position, &end);
    if (end == position || end > object_end || !isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

typedef struct {
    char *data;
    size_t length;
    bool overflow;
} weather_response_t;

#define WEATHER_RESPONSE_LIMIT 4096

static esp_err_t weather_http_event(esp_http_client_event_t *event)
{
    weather_response_t *response = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        if ((size_t)event->data_len > WEATHER_RESPONSE_LIMIT - response->length) {
            response->overflow = true;
            return ESP_ERR_NO_MEM;
        }
        memcpy(response->data + response->length, event->data, (size_t)event->data_len);
        response->length += (size_t)event->data_len;
        response->data[response->length] = '\0';
    }
    return ESP_OK;
}

static int weather_parse_numbers(const char *start, const char *key, double *values, int limit)
{
    char pattern[40];
    snprintf(pattern, sizeof(pattern), "\"%s\":[", key);
    const char *position = strstr(start, pattern);
    if (position == NULL) {
        return 0;
    }
    position += strlen(pattern);
    int count = 0;
    while (count < limit) {
        char *end = NULL;
        double value = strtod(position, &end);
        if (end == position) {
            break;
        }
        values[count++] = value;
        position = end;
        if (*position != ',') {
            break;
        }
        ++position;
    }
    return count;
}

static bool weather_parse(const char *json, weather_data_t *out)
{
    const char *current = strstr(json, "\"current\":{");
    const char *current_end = current == NULL ? NULL : strchr(current, '}');
    const char *hourly = strstr(json, "\"hourly\":{");
    double temperature, apparent, humidity, wind, code;
    if (current == NULL || current_end == NULL || hourly == NULL ||
        !sd_lua_json_number(current, current_end, "temperature_2m", &temperature) ||
        !sd_lua_json_number(current, current_end, "apparent_temperature", &apparent) ||
        !sd_lua_json_number(current, current_end, "relative_humidity_2m", &humidity) ||
        !sd_lua_json_number(current, current_end, "wind_speed_10m", &wind) ||
        !sd_lua_json_number(current, current_end, "weather_code", &code)) {
        return false;
    }
    double temperatures[WEATHER_HOURS];
    double codes[WEATHER_HOURS];
    int temperature_count = weather_parse_numbers(hourly, "temperature_2m", temperatures, WEATHER_HOURS);
    int code_count = weather_parse_numbers(hourly, "weather_code", codes, WEATHER_HOURS);
    int count = temperature_count < code_count ? temperature_count : code_count;
    const char *time_position = strstr(hourly, "\"time\":[");
    if (count == 0 || time_position == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    int parsed = 0;
    for (; parsed < count; ++parsed) {
        const char *open = strchr(time_position, '"');
        if (open == NULL || strlen(open) < 14) {
            break;
        }
        out->hours[parsed] = (open[12] - '0') * 10 + (open[13] - '0');
        out->hour_temperature[parsed] = (float)temperatures[parsed];
        out->hour_code[parsed] = (int)codes[parsed];
        time_position = strchr(open + 1, '"');
        if (time_position == NULL) {
            ++parsed;
            break;
        }
        ++time_position;
    }
    if (parsed == 0) {
        return false;
    }
    out->hour_count = parsed;
    out->temperature = (float)temperature;
    out->apparent = (float)apparent;
    out->humidity = (int)humidity;
    out->wind = (float)wind;
    out->code = (int)code;
    return true;
}

static void weather_fetch_task(void *argument)
{
    unsigned province = (unsigned)(uintptr_t)argument;
    weather_response_t response = {.data = calloc(1, WEATHER_RESPONSE_LIMIT + 1)};
    weather_data_t parsed;
    bool ok = false;
    if (response.data != NULL) {
        char url[360];
        snprintf(url, sizeof(url),
                 "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
                 "&current=temperature_2m,relative_humidity_2m,apparent_temperature,weather_code,wind_speed_10m"
                 "&hourly=temperature_2m,weather_code&forecast_hours=%d&timezone=auto",
                 (double)weather_location.latitude, (double)weather_location.longitude,
                 WEATHER_HOURS);
        esp_http_client_config_t config = {
            .url = url,
            .event_handler = weather_http_event,
            .user_data = &response,
            .timeout_ms = 8000,
            .buffer_size = 1024,
            .buffer_size_tx = 512,
            .crt_bundle_attach = esp_crt_bundle_attach
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client != NULL) {
            esp_err_t result = esp_http_client_perform(client);
            int status = esp_http_client_get_status_code(client);
            esp_http_client_cleanup(client);
            ok = result == ESP_OK && !response.overflow && status == 200 &&
                 weather_parse(response.data, &parsed);
        }
    }
    free(response.data);
    if (ok) {
        weather_data = parsed;
        weather_valid = true;
        weather_data_province = (int)province;
        weather_last_fetch_us = esp_timer_get_time();
        weather_failed = false;
    } else {
        weather_failed = true;
        ESP_LOGW(TAG, "Hava durumu alinamadi");
    }
    weather_loading = false;
    vTaskDelete(NULL);
}

static void weather_request(unsigned generation)
{
    if (weather_loading || !wifi_enabled || wifi_state != WIFI_STATE_CONNECTED) {
        return;
    }
    weather_loading = true;
    weather_last_attempt_us = esp_timer_get_time();
    if (xTaskCreate(weather_fetch_task, "weather", 12288, (void *)(uintptr_t)generation, 3,
                    NULL) != pdPASS) {
        weather_loading = false;
    }
}

static void geocode_task(void *argument)
{
    (void)argument;
    weather_response_t response = {.data = calloc(1, WEATHER_RESPONSE_LIMIT + 1)};
    uint8_t state = 3;
    if (response.data != NULL) {
        char url[220];
        int length = snprintf(url, sizeof(url),
                              "https://geocoding-api.open-meteo.com/v1/search?count=1&language=en&format=json&name=");
        for (const char *c = geo_query; *c != '\0' && length < (int)sizeof(url) - 4; ++c) {
            if (isalnum((unsigned char)*c)) {
                url[length++] = *c;
            } else {
                length += snprintf(url + length, sizeof(url) - (size_t)length, "%%%02X",
                                   (unsigned)(unsigned char)*c);
            }
        }
        url[length] = '\0';
        esp_http_client_config_t config = {
            .url = url,
            .event_handler = weather_http_event,
            .user_data = &response,
            .timeout_ms = 8000,
            .buffer_size = 1024,
            .buffer_size_tx = 512,
            .crt_bundle_attach = esp_crt_bundle_attach
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (client != NULL) {
            esp_err_t result = esp_http_client_perform(client);
            int status = esp_http_client_get_status_code(client);
            esp_http_client_cleanup(client);
            if (result == ESP_OK && !response.overflow && status == 200) {
                state = 2;
                const char *results = strstr(response.data, "\"results\":[{");
                const char *end = results == NULL ? NULL : strchr(results, '}');
                double latitude, longitude;
                const char *name = results == NULL ? NULL : strstr(results, "\"name\":\"");
                if (end != NULL && name != NULL && name < end &&
                    sd_lua_json_number(results, end, "latitude", &latitude) &&
                    sd_lua_json_number(results, end, "longitude", &longitude)) {
                    weather_location_t found = {.latitude = (float)latitude,
                                                .longitude = (float)longitude};
                    size_t used = 0;
                    for (name += 8; *name != '"' && *name != '\0' && used < 18; ++name) {
                        unsigned char c = (unsigned char)*name;
                        if (c >= 32 && c < 127) {
                            found.name[used++] = (char)toupper(c);
                        }
                    }
                    const char *country = strstr(results, "\"country_code\":\"");
                    if (country != NULL && country < end && used > 0) {
                        country += 16;
                        if (isalpha((unsigned char)country[0]) && isalpha((unsigned char)country[1])) {
                            found.name[used++] = ',';
                            found.name[used++] = ' ';
                            found.name[used++] = (char)toupper((unsigned char)country[0]);
                            found.name[used++] = (char)toupper((unsigned char)country[1]);
                        }
                    }
                    found.name[used] = '\0';
                    if (used > 0) {
                        weather_location = found;
                        nvs_set_blob(settings, "wx_loc", &found, sizeof(found));
                        nvs_commit(settings);
                        weather_valid = false;
                        weather_location_gen = weather_location_gen + 1;
                        weather_last_attempt_us = -100000000LL;
                        state = 0;
                    }
                }
            }
        }
    }
    free(response.data);
    geo_state = state;
    vTaskDelete(NULL);
}

static void geocode_request(void)
{
    if (geo_state == 1) {
        return;
    }
    if (!wifi_enabled || wifi_state != WIFI_STATE_CONNECTED) {
        geo_state = 3;
        return;
    }
    geo_state = 1;
    if (xTaskCreate(geocode_task, "geocode", 8192, NULL, 3, NULL) != pdPASS) {
        geo_state = 3;
    }
}

static void weather_service(int64_t now_us)
{
    if (weather_loading || geo_state == 1 || !wifi_enabled || wifi_state != WIFI_STATE_CONNECTED) {
        return;
    }
    bool stale = !weather_valid || weather_data_province != (int)weather_location_gen ||
                 now_us - weather_last_fetch_us >= 900000000LL;
    int64_t retry_us = weather_failed ? 60000000LL : 3000000LL;
    if (stale && now_us - weather_last_attempt_us >= retry_us) {
        weather_request(weather_location_gen);
    }
}

static int sd_lua_weather(lua_State *state)
{
    lua_Number latitude = luaL_checknumber(state, 1);
    lua_Number longitude = luaL_checknumber(state, 2);
    if (!isfinite(latitude) || latitude < -90.0 || latitude > 90.0 ||
        !isfinite(longitude) || longitude < -180.0 || longitude > 180.0) {
        return luaL_error(state, "latitude/longitude out of range");
    }
    if (!wifi_enabled || wifi_state != WIFI_STATE_CONNECTED) {
        lua_pushnil(state);
        lua_pushstring(state, "Wi-Fi is not connected");
        return 2;
    }

    char url[256];
    int url_length = snprintf(
        url, sizeof(url),
        "https://api.open-meteo.com/v1/forecast?latitude=%.5f&longitude=%.5f&current=temperature_2m,relative_humidity_2m,apparent_temperature,precipitation,weather_code,wind_speed_10m&timezone=auto",
        (double)latitude, (double)longitude);
    if (url_length < 0 || (size_t)url_length >= sizeof(url)) {
        lua_pushnil(state);
        lua_pushstring(state, "weather query is too long");
        return 2;
    }
    sd_lua_http_response_t response = {0};
    esp_http_client_config_t config = {
        .url = url,
        .event_handler = sd_lua_http_event,
        .user_data = &response,
        .timeout_ms = 4000,
        .buffer_size = 512,
        .buffer_size_tx = 512,
        .crt_bundle_attach = esp_crt_bundle_attach
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        lua_pushnil(state);
        lua_pushstring(state, "weather client initialization failed");
        return 2;
    }
    esp_err_t request_result = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (request_result != ESP_OK || response.overflow || status_code != 200) {
        lua_pushnil(state);
        lua_pushfstring(state, "weather request failed (status %d)",
                        status_code);
        return 2;
    }

    const char *current = strstr(response.data, "\"current\"");
    const char *object_start = current == NULL ? NULL : strchr(current, '{');
    const char *object_end = object_start == NULL ? NULL : strchr(object_start, '}');
    double temperature = 0.0;
    double humidity = 0.0;
    double apparent = 0.0;
    double precipitation = 0.0;
    double weather_code = 0.0;
    double wind_speed = 0.0;
    if (object_start == NULL || object_end == NULL ||
        !sd_lua_json_number(object_start, object_end, "temperature_2m", &temperature) ||
        !sd_lua_json_number(object_start, object_end, "relative_humidity_2m", &humidity) ||
        !sd_lua_json_number(object_start, object_end, "apparent_temperature", &apparent) ||
        !sd_lua_json_number(object_start, object_end, "precipitation", &precipitation) ||
        !sd_lua_json_number(object_start, object_end, "weather_code", &weather_code) ||
        !sd_lua_json_number(object_start, object_end, "wind_speed_10m", &wind_speed)) {
        lua_pushnil(state);
        lua_pushstring(state, "invalid weather response");
        return 2;
    }
    lua_createtable(state, 0, 6);
    lua_pushnumber(state, temperature);
    lua_setfield(state, -2, "temperature_c");
    lua_pushnumber(state, humidity);
    lua_setfield(state, -2, "relative_humidity_percent");
    lua_pushnumber(state, apparent);
    lua_setfield(state, -2, "apparent_temperature_c");
    lua_pushnumber(state, precipitation);
    lua_setfield(state, -2, "precipitation_mm");
    lua_pushinteger(state, (lua_Integer)weather_code);
    lua_setfield(state, -2, "weather_code");
    lua_pushnumber(state, wind_speed);
    lua_setfield(state, -2, "wind_speed_kmh");
    return 1;
}

static int sd_lua_ble_scan(lua_State *state)
{
    if (!bluetooth_enabled) {
        lua_pushboolean(state, false);
        lua_pushstring(state, "Bluetooth is disabled");
        return 2;
    }
    start_ble_scan();
    lua_pushboolean(state, ble_state == BLE_STATE_SCANNING ||
                           ble_state == BLE_STATE_READY);
    return 1;
}

static int sd_lua_ble_devices(lua_State *state)
{
    lua_createtable(state, (int)ble_device_count, 0);
    for (unsigned i = 0; i < ble_device_count; ++i) {
        lua_createtable(state, 0, 2);
        lua_pushstring(state, ble_devices[i].name);
        lua_setfield(state, -2, "name");
        lua_pushinteger(state, ble_devices[i].rssi);
        lua_setfield(state, -2, "rssi");
        lua_rawseti(state, -2, (lua_Integer)i + 1);
    }
    return 1;
}

static int sd_lua_ble_connect(lua_State *state)
{
    lua_Integer index = luaL_checkinteger(state, 1);
    if (!bluetooth_enabled || index < 1 ||
        (uint64_t)index > ble_device_count || !ble_gatt_app_registered) {
        lua_pushboolean(state, false);
        return 1;
    }
    ble_selected_device = (unsigned)index - 1;
    connect_selected_ble();
    lua_pushboolean(state, ble_state == BLE_STATE_CONNECTING ||
                           ble_state == BLE_STATE_CONNECTED);
    return 1;
}

static esp_err_t sd_imu_write(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};
    return i2c_master_transmit(sd_imu_device, packet, sizeof(packet), 100);
}

static esp_err_t sd_imu_read(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(sd_imu_device, &reg, 1, data, length, 100);
}

static esp_err_t sd_imu_init(void)
{
    if (sd_imu_initialized) {
        return ESP_OK;
    }
    if (sd_imu_bus == NULL) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = GPIO_NUM_47,
            .scl_io_num = GPIO_NUM_48,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true
        };
        esp_err_t result = i2c_new_master_bus(&bus_config, &sd_imu_bus);
        if (result != ESP_OK) {
            return result;
        }
    }
    if (sd_imu_device == NULL) {
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x6b,
            .scl_speed_hz = 300000
        };
        esp_err_t result = i2c_master_bus_add_device(sd_imu_bus, &device_config,
                                                      &sd_imu_device);
        if (result != ESP_OK) {
            return result;
        }
    }
    uint8_t chip_id = 0;
    esp_err_t result = sd_imu_read(0x00, &chip_id, 1);
    if (result != ESP_OK) {
        return result;
    }
    if (chip_id != 0x05) {
        return ESP_ERR_NOT_FOUND;
    }
    result = sd_imu_write(0x02, 0x60);
    if (result == ESP_OK) result = sd_imu_write(0x03, 0x25);
    if (result == ESP_OK) result = sd_imu_write(0x04, 0x65);
    if (result == ESP_OK) result = sd_imu_write(0x06, 0x00);
    if (result == ESP_OK) result = sd_imu_write(0x08, 0x03);
    if (result == ESP_OK) {
        sd_imu_initialized = true;
    }
    return result;
}

static int sd_lua_imu(lua_State *state)
{
    esp_err_t result = sd_imu_init();
    if (result != ESP_OK) {
        lua_pushnil(state);
        lua_pushfstring(state, "QMI8658 unavailable: %s", esp_err_to_name(result));
        return 2;
    }
    uint8_t data[14];
    result = sd_imu_read(0x33, data, sizeof(data));
    if (result != ESP_OK) {
        lua_pushnil(state);
        lua_pushstring(state, "QMI8658 read failed");
        return 2;
    }
    int16_t raw_temperature = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    int16_t acceleration[3];
    int16_t gyro[3];
    for (unsigned i = 0; i < 3; ++i) {
        acceleration[i] = (int16_t)((uint16_t)data[3 + i * 2] << 8 |
                                    data[2 + i * 2]);
        gyro[i] = (int16_t)((uint16_t)data[9 + i * 2] << 8 |
                            data[8 + i * 2]);
    }
    lua_createtable(state, 0, 3);
    lua_createtable(state, 0, 3);
    lua_pushnumber(state, (double)acceleration[0] * 9.80665 / 4096.0);
    lua_setfield(state, -2, "x");
    lua_pushnumber(state, (double)acceleration[1] * 9.80665 / 4096.0);
    lua_setfield(state, -2, "y");
    lua_pushnumber(state, (double)acceleration[2] * 9.80665 / 4096.0);
    lua_setfield(state, -2, "z");
    lua_setfield(state, -2, "acceleration");
    lua_createtable(state, 0, 3);
    lua_pushnumber(state, gyro[0] / 32.0);
    lua_setfield(state, -2, "x");
    lua_pushnumber(state, gyro[1] / 32.0);
    lua_setfield(state, -2, "y");
    lua_pushnumber(state, gyro[2] / 32.0);
    lua_setfield(state, -2, "z");
    lua_setfield(state, -2, "gyro_dps");
    lua_pushnumber(state, raw_temperature / 256.0);
    lua_setfield(state, -2, "temperature_c");
    return 1;
}

static void sd_lua_open_library(lua_State *state, const char *name,
                                lua_CFunction open_function)
{
    luaL_requiref(state, name, open_function, 1);
    lua_pop(state, 1);
}

static void sd_lua_open_api(lua_State *state)
{
    static const luaL_Reg functions[] = {
        {"clear", sd_lua_clear},
        {"text", sd_lua_text},
        {"fill_rect", sd_lua_fill_rect},
        {"circle", sd_lua_circle},
        {"input", sd_lua_input},
        {"clock", sd_lua_clock},
        {"wifi", sd_lua_wifi},
        {"weather", sd_lua_weather},
        {"ble_scan", sd_lua_ble_scan},
        {"ble_devices", sd_lua_ble_devices},
        {"ble_connect", sd_lua_ble_connect},
        {"imu", sd_lua_imu},
        {NULL, NULL}
    };
    lua_newtable(state);
    luaL_setfuncs(state, functions, 0);
    lua_setglobal(state, "rexa");
}

static int sd_lua_setup_sandbox(lua_State *state)
{
    sd_lua_open_library(state, LUA_GNAME, luaopen_base);
    sd_lua_open_library(state, LUA_MATHLIBNAME, luaopen_math);
    sd_lua_open_library(state, LUA_STRLIBNAME, luaopen_string);
    sd_lua_open_library(state, LUA_TABLIBNAME, luaopen_table);
    sd_lua_open_library(state, LUA_UTF8LIBNAME, luaopen_utf8);
    sd_lua_remove_global(state, "dofile");
    sd_lua_remove_global(state, "loadfile");
    sd_lua_remove_global(state, "load");
    sd_lua_remove_global(state, "collectgarbage");
    sd_lua_remove_global(state, "print");
    sd_lua_remove_global(state, "warn");
    sd_lua_remove_global(state, "require");
    sd_lua_remove_global(state, "pcall");
    sd_lua_remove_global(state, "xpcall");
    sd_lua_open_api(state);
    return 0;
}

static void sd_lua_close(void)
{
    if (sd_lua_state != NULL) {
        lua_close(sd_lua_state);
        sd_lua_state = NULL;
    }
    sd_lua_memory_used = 0;
}

static bool sd_lua_call(const char *function_name, bool pass_input)
{
    if (sd_lua_state == NULL) {
        return false;
    }
    lua_getglobal(sd_lua_state, function_name);
    if (!lua_isfunction(sd_lua_state, -1)) {
        lua_pop(sd_lua_state, 1);
        return true;
    }
    int argument_count = 0;
    if (pass_input) {
        sd_lua_input(sd_lua_state);
        argument_count = 1;
        lua_pushnumber(sd_lua_state, FRAME_INTERVAL_MS / 1000.0);
        ++argument_count;
    }
    sd_lua_instructions_left = SD_LUA_INSTRUCTION_LIMIT;
    lua_sethook(sd_lua_state, sd_lua_instruction_hook, LUA_MASKCOUNT, 1000);
    int result = lua_pcall(sd_lua_state, argument_count, 0, 0);
    lua_sethook(sd_lua_state, NULL, 0, 0);
    if (result == LUA_OK) {
        return true;
    }
    const char *message = lua_tostring(sd_lua_state, -1);
    snprintf(sd_lua_error, sizeof(sd_lua_error), "%s",
             message == NULL ? "Lua runtime error" : message);
    ESP_LOGE(TAG, "SD Lua app %s failed: %s", function_name, sd_lua_error);
    lua_pop(sd_lua_state, 1);
    sd_lua_close();
    sd_app_running = false;
    snprintf(sd_app_type, sizeof(sd_app_type), "%s", "error");
    return false;
}

static bool sd_lua_run_file(const char *path, const char *title)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Lua app could not be opened: %s", path);
        return false;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return false;
    }
    long file_length = ftell(file);
    if (file_length <= 0 || file_length > SD_LUA_FILE_LIMIT ||
        fseek(file, 0, SEEK_SET) != 0) {
        fclose(file);
        ESP_LOGE(TAG, "Lua app size is invalid: %s", path);
        return false;
    }
    char *source = malloc((size_t)file_length);
    if (source == NULL) {
        fclose(file);
        return false;
    }
    size_t bytes_read = fread(source, 1, (size_t)file_length, file);
    bool read_failed = ferror(file) != 0;
    fclose(file);
    if (read_failed || bytes_read != (size_t)file_length) {
        free(source);
        ESP_LOGE(TAG, "Lua app read was incomplete: %s", path);
        return false;
    }

    sd_lua_memory_used = 0;
    sd_lua_state = lua_newstate(sd_lua_allocator, &sd_lua_memory_used,
                                luaL_makeseed(NULL));
    if (sd_lua_state == NULL) {
        free(source);
        ESP_LOGE(TAG, "Lua state creation failed (memory limit)");
        return false;
    }
    lua_pushcfunction(sd_lua_state, sd_lua_setup_sandbox);
    int setup_result = lua_pcall(sd_lua_state, 0, 0, 0);
    if (setup_result != LUA_OK) {
        const char *message = lua_tostring(sd_lua_state, -1);
        snprintf(sd_lua_error, sizeof(sd_lua_error), "%s",
                 message == NULL ? "Lua sandbox setup failed" : message);
        lua_pop(sd_lua_state, 1);
        free(source);
        sd_lua_close();
        return false;
    }

    int load_result = luaL_loadbufferx(sd_lua_state, source, bytes_read, title, "t");
    free(source);
    if (load_result != LUA_OK) {
        const char *message = lua_tostring(sd_lua_state, -1);
        snprintf(sd_lua_error, sizeof(sd_lua_error), "%s",
                 message == NULL ? "Lua syntax error" : message);
        ESP_LOGE(TAG, "Lua app load failed: %s", sd_lua_error);
        sd_lua_close();
        return false;
    }
    sd_lua_instructions_left = SD_LUA_INSTRUCTION_LIMIT;
    lua_sethook(sd_lua_state, sd_lua_instruction_hook, LUA_MASKCOUNT, 1000);
    int run_result = lua_pcall(sd_lua_state, 0, 0, 0);
    lua_sethook(sd_lua_state, NULL, 0, 0);
    if (run_result != LUA_OK) {
        const char *message = lua_tostring(sd_lua_state, -1);
        snprintf(sd_lua_error, sizeof(sd_lua_error), "%s",
                 message == NULL ? "Lua startup failed" : message);
        ESP_LOGE(TAG, "Lua app startup failed: %s", sd_lua_error);
        lua_pop(sd_lua_state, 1);
        sd_lua_close();
        return false;
    }
    return true;
}

static bool sd_parse_package(const char *path, sd_app_package_t *package)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return false;
    }

    memset(package, 0, sizeof(*package));
    package->accent = color565(55, 223, 220);
    bool header_found = false;
    bool name_found = false;
    bool type_found = false;
    char line[128];
    bool valid = true;

    while (fgets(line, sizeof(line), file) != NULL) {
        size_t length = strlen(line);
        if (length == sizeof(line) - 1 && line[length - 1] != '\n' && !feof(file)) {
            valid = false;
            break;
        }

        char *content = sd_trim(line);
        if (*content == '\0' || *content == '#') {
            continue;
        }
        if (!header_found) {
            header_found = strcmp(content, "REXAUI/1") == 0;
            if (!header_found) {
                valid = false;
                break;
            }
            continue;
        }

        char *separator = strchr(content, '=');
        if (separator == NULL) {
            valid = false;
            break;
        }
        *separator = '\0';
        char *key = sd_trim(content);
        char *value = sd_trim(separator + 1);
        if (strcmp(key, "NAME") == 0) {
            if (!sd_ascii_text_is_valid(value, sizeof(package->title))) {
                valid = false;
                break;
            }
            snprintf(package->title, sizeof(package->title), "%s", value);
            name_found = true;
        } else if (strcmp(key, "TYPE") == 0) {
            if (strcmp(value, "counter") != 0 &&
                strcmp(value, "clock") != 0 &&
                strcmp(value, "message") != 0) {
                valid = false;
                break;
            }
            snprintf(package->type, sizeof(package->type), "%s", value);
            type_found = true;
        } else if (strcmp(key, "TEXT") == 0) {
            if (strlen(value) >= sizeof(package->text)) {
                valid = false;
                break;
            }
            for (size_t i = 0; value[i] != '\0'; ++i) {
                unsigned char character = (unsigned char)value[i];
                if (character < 32 || character > 126) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                break;
            }
            snprintf(package->text, sizeof(package->text), "%s", value);
        } else if (strcmp(key, "COLOR") == 0) {
            if (strlen(value) != 6) {
                valid = false;
                break;
            }
            for (size_t i = 0; i < 6; ++i) {
                char character = value[i];
                if (!((character >= '0' && character <= '9') ||
                      (character >= 'A' && character <= 'F') ||
                      (character >= 'a' && character <= 'f'))) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                break;
            }
            unsigned long rgb = strtoul(value, NULL, 16);
            package->accent = color565((uint8_t)(rgb >> 16),
                                       (uint8_t)(rgb >> 8),
                                       (uint8_t)rgb);
        } else {
            valid = false;
            break;
        }
    }

    if (ferror(file) != 0) {
        valid = false;
    }
    fclose(file);
    return valid && header_found && name_found && type_found;
}

static bool sd_has_package_extension(const char *filename)
{
    size_t length = strlen(filename);
    if (length < 4) {
        return false;
    }
    const char *extension = filename + length - 4;
    return extension[0] == '.' &&
           (extension[1] == 'r' || extension[1] == 'R') &&
           (extension[2] == 'x' || extension[2] == 'X') &&
           (extension[3] == 'a' || extension[3] == 'A');
}

static bool sd_has_lua_extension(const char *filename)
{
    size_t length = strlen(filename);
    return length > 4 && filename[length - 4] == '.' &&
           (filename[length - 3] == 'l' || filename[length - 3] == 'L') &&
           (filename[length - 2] == 'u' || filename[length - 2] == 'U') &&
           (filename[length - 1] == 'a' || filename[length - 1] == 'A');
}

static esp_err_t sd_mount_card(void)
{
    if (sd_card_mounted) {
        return ESP_OK;
    }
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = SDMMC_CLK_PIN;
    slot_config.cmd = SDMMC_CMD_PIN;
    slot_config.d0 = SDMMC_D0_PIN;

    esp_err_t result = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config,
                                               &mount_config, &sd_card);
    if (result != ESP_OK) {
        sd_card = NULL;
        sd_card_mounted = false;
        snprintf(sd_status, sizeof(sd_status), "CARD ERROR");
        sd_status_is_error = true;
        if (result != sd_last_mount_error) {
            ESP_LOGW(TAG, "SD kart baglanamadi: %s", esp_err_to_name(result));
            sd_last_mount_error = result;
        }
        return result;
    }

    sd_card_mounted = true;
    sd_last_mount_error = ESP_OK;
    sd_status_is_error = false;
    sd_next_check_us = esp_timer_get_time() + 1000000;
    ESP_LOGI(TAG, "SD kart baglandi; uygulamalar salt okunur yuklenecek");
    return ESP_OK;
}

static void sd_scan_apps(void)
{
    sd_app_count = 0;
    sd_selected_app = 0;
    memset(sd_apps, 0, sizeof(sd_apps));
    if (sd_card_mounted && sd_card != NULL) {
        esp_err_t status = sdmmc_get_status(sd_card);
        if (status != ESP_OK) {
            sd_handle_removal();
            return;
        }
    }
    if (sd_mount_card() != ESP_OK) {
        return;
    }

    DIR *directory = opendir(SD_APPS_DIRECTORY);
    if (directory == NULL) {
        snprintf(sd_status, sizeof(sd_status), "NO APPS");
        sd_status_is_error = true;
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL && sd_app_count < SD_APP_LIMIT) {
        bool lua_script = sd_has_lua_extension(entry->d_name);
        if ((!lua_script && !sd_has_package_extension(entry->d_name)) ||
            strlen(entry->d_name) >= SD_APP_FILENAME_LENGTH) {
            continue;
        }

        char path[128];
        int path_length = snprintf(path, sizeof(path), "%s/%s",
                                  SD_APPS_DIRECTORY, entry->d_name);
        if (path_length < 0 || (size_t)path_length >= sizeof(path)) {
            continue;
        }
        struct stat file_status;
        if (stat(path, &file_status) != 0 || !S_ISREG(file_status.st_mode) ||
            file_status.st_size <= 0 ||
            file_status.st_size > (lua_script ? SD_LUA_FILE_LIMIT :
                                                  SD_APP_FILE_LIMIT)) {
            continue;
        }

        char app_title[SD_APP_NAME_LENGTH] = {0};
        if (lua_script) {
            size_t title_length = strlen(entry->d_name) - 4;
            if (title_length == 0) {
                continue;
            }
            if (title_length >= sizeof(app_title)) {
                title_length = sizeof(app_title) - 1;
            }
            memcpy(app_title, entry->d_name, title_length);
            if (!sd_ascii_text_is_valid(app_title, sizeof(app_title))) {
                ESP_LOGW(TAG, "Lua app filename is not printable ASCII: %s",
                         entry->d_name);
                continue;
            }
        } else {
            sd_app_package_t package;
            if (!sd_parse_package(path, &package)) {
                ESP_LOGW(TAG, "Gecersiz rexaUi paketi atlandi: %s",
                         entry->d_name);
                continue;
            }
            snprintf(app_title, sizeof(app_title), "%s", package.title);
        }
        size_t filename_length = strlen(entry->d_name);
        memcpy(sd_apps[sd_app_count].filename, entry->d_name, filename_length + 1);
        snprintf(sd_apps[sd_app_count].title,
                 sizeof(sd_apps[sd_app_count].title), "%s", app_title);
        sd_apps[sd_app_count].lua_script = lua_script;
        ++sd_app_count;
    }
    closedir(directory);

    if (sd_app_count == 0) {
        snprintf(sd_status, sizeof(sd_status), "NO APPS");
        sd_status_is_error = true;
    } else {
        snprintf(sd_status, sizeof(sd_status), "READY");
        sd_status_is_error = false;
    }
}

#define MEDIA_LIMIT 48
#define MEDIA_PATH_LENGTH 96
#define MEDIA_SCAN_DEPTH 2
#define JPEG_POOL_SIZE 6144
#define JPEG_WINDOW 20
#define GIF_MAX_CODES 4096

typedef struct {
    char path[MEDIA_PATH_LENGTH];
    char title[SD_APP_NAME_LENGTH];
    bool gif;
} media_entry_t;

typedef enum {
    MEDIA_IDLE,
    MEDIA_PENDING,
    MEDIA_STILL,
    MEDIA_ANIMATED,
    MEDIA_ERROR
} media_state_t;

static media_entry_t media_files[MEDIA_LIMIT];
static unsigned media_count = 0;
static unsigned media_selected = 0;
static media_state_t media_state = MEDIA_IDLE;
static const char *media_error_text = "BAD FILE";
static FILE *media_file = NULL;

typedef struct {
    FILE *file;
    unsigned consumed;
    int dest_width, dest_height;
    int origin_x, origin_y;
    int scaled_width, scaled_height;
    int slow_length, fast_length;
    bool swap, reverse_fast, reverse_slow;
    uint16_t *xmap;
    uint16_t (*sum)[3];
    uint8_t *count;
    int flushed;
    int failure;
} jpeg_context_t;

typedef struct {
    uint16_t palette[256];
    uint16_t global_palette[256];
    uint16_t *prefix;
    uint8_t *suffix;
    uint8_t *stack;
    uint16_t *xs, *ys;
    uint16_t canvas_width, canvas_height;
    int dest_width, dest_height, origin_x, origin_y;
    long data_start;
    bool has_global_palette;
    bool interlaced;
    bool transparent;
    uint8_t transparent_index;
    uint8_t disposal;
    uint16_t delay_cs;
    uint16_t frame_x, frame_y, frame_width, frame_height;
    uint16_t last_x, last_y, last_width, last_height;
    uint8_t last_disposal;
    unsigned frames_in_pass;
    uint8_t block[256];
    unsigned block_size, block_position;
    uint32_t bit_buffer;
    unsigned bit_count;
} gif_context_t;

static gif_context_t *gif = NULL;
static int64_t media_next_us = 0;

static void media_free_gif(void)
{
    if (gif != NULL) {
        free(gif->prefix);
        free(gif->suffix);
        free(gif->stack);
        free(gif->xs);
        free(gif->ys);
        free(gif);
        gif = NULL;
    }
}

static void media_close(void)
{
    if (media_file != NULL) {
        fclose(media_file);
        media_file = NULL;
    }
    media_free_gif();
    media_state = MEDIA_IDLE;
}

static bool media_has_suffix(const char *name, const char *const *suffixes, unsigned count)
{
    size_t length = strlen(name);
    for (unsigned i = 0; i < count; ++i) {
        size_t suffix_length = strlen(suffixes[i]);
        if (length > suffix_length &&
            strcasecmp(name + length - suffix_length, suffixes[i]) == 0) {
            return true;
        }
    }
    return false;
}

static void media_scan_directory(const char *directory_path, unsigned depth)
{
    static const char *const jpeg_suffixes[] = {".jpg", ".jpeg"};
    static const char *const gif_suffixes[] = {".gif"};
    DIR *directory = opendir(directory_path);
    if (directory == NULL) {
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL && media_count < MEDIA_LIMIT) {
        const char *name = entry->d_name;
        if (name[0] == '.' || name[0] == '_' || strcmp(name, "System Volume Information") == 0) {
            continue;
        }
        char path[MEDIA_PATH_LENGTH];
        int length = snprintf(path, sizeof(path), "%s/%s", directory_path, name);
        if (length < 0 || (size_t)length >= sizeof(path)) {
            continue;
        }
        if (entry->d_type == DT_DIR) {
            if (depth < MEDIA_SCAN_DEPTH) {
                media_scan_directory(path, depth + 1);
            }
            continue;
        }
        bool is_gif = media_has_suffix(name, gif_suffixes, 1);
        if (!is_gif && !media_has_suffix(name, jpeg_suffixes, 2)) {
            continue;
        }
        media_entry_t *item = &media_files[media_count];
        memcpy(item->path, path, (size_t)length + 1);
        const char *dot = strrchr(name, '.');
        size_t title_length = (size_t)(dot - name);
        if (title_length >= sizeof(item->title)) {
            title_length = sizeof(item->title) - 1;
        }
        memcpy(item->title, name, title_length);
        item->title[title_length] = '\0';
        item->gif = is_gif;
        ++media_count;
    }
    closedir(directory);
}

static void media_scan(void)
{
    media_close();
    media_count = 0;
    media_selected = 0;
    memset(media_files, 0, sizeof(media_files));
    if (sd_card_mounted && sd_card != NULL && sdmmc_get_status(sd_card) != ESP_OK) {
        sd_handle_removal();
        return;
    }
    if (sd_mount_card() != ESP_OK) {
        return;
    }
    media_scan_directory(SD_MOUNT_POINT, 0);
    snprintf(sd_status, sizeof(sd_status), media_count == 0 ? "NO MEDIA" : "READY");
    sd_status_is_error = media_count == 0;
}

static void media_fit(int width, int height, int *dest_width, int *dest_height)
{
    if (width <= LCD_WIDTH && height <= LCD_HEIGHT) {
        *dest_width = width;
        *dest_height = height;
    } else if ((long)width * LCD_HEIGHT > (long)height * LCD_WIDTH) {
        *dest_width = LCD_WIDTH;
        *dest_height = (int)((long)height * LCD_WIDTH / width);
    } else {
        *dest_height = LCD_HEIGHT;
        *dest_width = (int)((long)width * LCD_HEIGHT / height);
    }
    if (*dest_width < 1) *dest_width = 1;
    if (*dest_height < 1) *dest_height = 1;
}

static uint16_t media_pixel(unsigned red, unsigned green, unsigned blue, int x, int y)
{
    static const int8_t bayer[16] = {-8, 0, -6, 2, 2, -4, 4, -2, -5, 3, -7, 1, 5, -1, 3, -3};
    int noise = bayer[(y & 3) * 4 + (x & 3)];
    int r = (int)red + noise / 2;
    int g = (int)green + noise / 4;
    int b = (int)blue + noise / 2;
    r = r < 0 ? 0 : (r > 255 ? 255 : r);
    g = g < 0 ? 0 : (g > 255 ? 255 : g);
    b = b < 0 ? 0 : (b > 255 ? 255 : b);
    return media_color565((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

static int media_exif_orientation(FILE *file)
{
    uint8_t marker[4];
    if (fread(marker, 1, 2, file) != 2 || marker[0] != 0xff || marker[1] != 0xd8) {
        return 1;
    }
    for (int segment = 0; segment < 16; ++segment) {
        if (fread(marker, 1, 4, file) != 4 || marker[0] != 0xff) {
            break;
        }
        unsigned length = ((unsigned)marker[2] << 8) | marker[3];
        if (length < 2 || marker[1] == 0xda) {
            break;
        }
        if (marker[1] != 0xe1 || length < 16) {
            if (fseek(file, (long)length - 2, SEEK_CUR) != 0) {
                break;
            }
            continue;
        }
        uint8_t tiff[160];
        size_t want = length - 2 > sizeof(tiff) ? sizeof(tiff) : length - 2;
        if (fread(tiff, 1, want, file) != want || memcmp(tiff, "Exif\0\0", 6) != 0 ||
            want < 22) {
            break;
        }
        const uint8_t *base = tiff + 6;
        size_t available = want - 6;
        bool little = base[0] == 'I';
#define EXIF16(p) (little ? (unsigned)((p)[0] | ((p)[1] << 8)) : (unsigned)(((p)[0] << 8) | (p)[1]))
        unsigned offset = little ? (unsigned)(base[4] | (base[5] << 8)) :
                                   (unsigned)((base[6] << 8) | base[7]);
        if (offset + 2 > available) {
            break;
        }
        unsigned entries = EXIF16(base + offset);
        for (unsigned i = 0; i < entries && offset + 2 + (i + 1) * 12 <= available; ++i) {
            const uint8_t *entry = base + offset + 2 + i * 12;
            if (EXIF16(entry) == 0x0112) {
                unsigned value = EXIF16(entry + 8);
                return value >= 1 && value <= 8 ? (int)value : 1;
            }
        }
#undef EXIF16
        break;
    }
    return 1;
}

static UINT jpeg_input(JDEC *decoder, BYTE *buffer, UINT size)
{
    jpeg_context_t *context = decoder->device;
    context->consumed += size;
    if (context->consumed >= 32768) {
        context->consumed = 0;
        vTaskDelay(1);
    }
    if (buffer == NULL) {
        return fseek(context->file, (long)size, SEEK_CUR) == 0 ? size : 0;
    }
    return (UINT)fread(buffer, 1, size, context->file);
}

static void jpeg_flush(jpeg_context_t *context, int until)
{
    for (; context->flushed < until; ++context->flushed) {
        int slot = context->flushed % JPEG_WINDOW;
        int slow = context->reverse_slow ? context->slow_length - 1 - context->flushed :
                                           context->flushed;
        for (int i = 0; i < context->fast_length; ++i) {
            unsigned samples = context->count[slot * LCD_WIDTH + i];
            if (samples == 0) {
                continue;
            }
            uint16_t *sum = context->sum[slot * LCD_WIDTH + i];
            int x = context->swap ? slow : i;
            int y = context->swap ? i : slow;
            framebuffer[(context->origin_y + y) * LCD_WIDTH + context->origin_x + x] =
                media_pixel(sum[0] / samples, sum[1] / samples, sum[2] / samples, x, y);
            sum[0] = sum[1] = sum[2] = 0;
            context->count[slot * LCD_WIDTH + i] = 0;
        }
    }
}

static UINT jpeg_output(JDEC *decoder, void *bitmap, JRECT *rect)
{
    jpeg_context_t *context = decoder->device;
    const uint8_t *source = bitmap;
    int width = rect->right - rect->left + 1;
    for (int row = rect->top; row <= rect->bottom; ++row) {
        if (row >= context->scaled_height) {
            source += width * 3;
            continue;
        }
        int slow = (int)((long)row * context->slow_length / context->scaled_height);
        int slot = slow % JPEG_WINDOW;
        for (int column = 0; column < width; ++column, source += 3) {
            int sx = rect->left + column;
            if (sx >= context->scaled_width) {
                continue;
            }
            int index = slot * LCD_WIDTH + context->xmap[sx];
            context->sum[index][0] = (uint16_t)(context->sum[index][0] + source[0]);
            context->sum[index][1] = (uint16_t)(context->sum[index][1] + source[1]);
            context->sum[index][2] = (uint16_t)(context->sum[index][2] + source[2]);
            ++context->count[index];
        }
    }
    if (rect->right + 1 >= context->scaled_width) {
        int done = rect->bottom + 1;
        int until = done >= context->scaled_height ? context->slow_length :
                    (int)((long)done * context->slow_length / context->scaled_height);
        jpeg_flush(context, until);
    }
    return 1;
}

static bool media_show_jpeg(const char *path)
{
    media_error_text = "BAD FILE";
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return false;
    }
    int orientation = media_exif_orientation(file);
    rewind(file);

    static const uint8_t swap_table[9] = {0, 0, 0, 0, 0, 1, 1, 1, 1};
    static const uint8_t flip_x_table[9] = {0, 0, 1, 1, 0, 0, 1, 1, 0};
    static const uint8_t flip_y_table[9] = {0, 0, 0, 1, 1, 0, 0, 1, 1};
    bool swap = swap_table[orientation];
    bool flip_x = flip_x_table[orientation];
    bool flip_y = flip_y_table[orientation];

    jpeg_context_t *context = calloc(1, sizeof(*context));
    JDEC *decoder = malloc(sizeof(*decoder));
    void *pool = malloc(JPEG_POOL_SIZE);
    bool ok = false;
    if (context == NULL || decoder == NULL || pool == NULL) {
        media_error_text = "NO MEMORY";
        goto finish;
    }
    context->file = file;
    JRESULT result = jd_prepare(decoder, jpeg_input, pool, JPEG_POOL_SIZE, context);
    if (result != JDR_OK) {
        media_error_text = result == JDR_FMT3 ? "UNSUPPORTED JPEG" : "BAD FILE";
        goto finish;
    }

    int oriented_width = swap ? (int)decoder->height : (int)decoder->width;
    int oriented_height = swap ? (int)decoder->width : (int)decoder->height;
    media_fit(oriented_width, oriented_height, &context->dest_width, &context->dest_height);
    unsigned scale = 0;
    while (scale < 3 && (oriented_width >> (scale + 1)) >= context->dest_width &&
           (oriented_height >> (scale + 1)) >= context->dest_height) {
        ++scale;
    }
    context->scaled_width = (int)decoder->width >> scale;
    context->scaled_height = (int)decoder->height >> scale;
    if (context->scaled_width < 1 || context->scaled_height < 1) {
        goto finish;
    }
    context->swap = swap;
    context->fast_length = swap ? context->dest_height : context->dest_width;
    context->slow_length = swap ? context->dest_width : context->dest_height;
    context->reverse_fast = swap ? flip_y : flip_x;
    context->reverse_slow = swap ? flip_x : flip_y;
    context->origin_x = (LCD_WIDTH - context->dest_width) / 2;
    context->origin_y = (LCD_HEIGHT - context->dest_height) / 2;

    context->xmap = malloc((size_t)context->scaled_width * sizeof(uint16_t));
    context->sum = calloc((size_t)JPEG_WINDOW * LCD_WIDTH, sizeof(*context->sum));
    context->count = calloc((size_t)JPEG_WINDOW * LCD_WIDTH, 1);
    if (context->xmap == NULL || context->sum == NULL || context->count == NULL) {
        media_error_text = "NO MEMORY";
        goto finish;
    }
    for (int x = 0; x < context->scaled_width; ++x) {
        int mapped = (int)((long)x * context->fast_length / context->scaled_width);
        context->xmap[x] = (uint16_t)(context->reverse_fast ? context->fast_length - 1 - mapped :
                                                              mapped);
    }

    memset(framebuffer, 0, LCD_BYTES);
    result = jd_decomp(decoder, jpeg_output, (BYTE)scale);
    if (result == JDR_OK) {
        jpeg_flush(context, context->slow_length);
        ok = true;
    }

finish:
    if (context != NULL) {
        free(context->xmap);
        free(context->sum);
        free(context->count);
    }
    free(context);
    free(decoder);
    free(pool);
    fclose(file);
    return ok;
}

static int gif_read_byte(gif_context_t *context)
{
    if (context->block_position >= context->block_size) {
        int size = fgetc(media_file);
        if (size <= 0) {
            context->block_size = context->block_position = 0;
            return -1;
        }
        if (fread(context->block, 1, (size_t)size, media_file) != (size_t)size) {
            return -1;
        }
        context->block_size = (unsigned)size;
        context->block_position = 0;
    }
    return context->block[context->block_position++];
}

static void gif_skip_blocks(void)
{
    int size;
    while ((size = fgetc(media_file)) > 0) {
        if (fseek(media_file, size, SEEK_CUR) != 0) {
            break;
        }
    }
}

static bool gif_read_palette(uint16_t *palette, unsigned entries)
{
    uint8_t rgb[3];
    for (unsigned i = 0; i < entries; ++i) {
        if (fread(rgb, 1, 3, media_file) != 3) {
            return false;
        }
        palette[i] = media_color565(rgb[0], rgb[1], rgb[2]);
    }
    return true;
}

static void gif_paint(gif_context_t *context, int x, int y, uint16_t color)
{
    int x0 = context->xs[x];
    int x1 = context->xs[x + 1];
    int y0 = context->ys[y];
    int y1 = context->ys[y + 1];
    for (int row = y0; row < y1; ++row) {
        uint16_t *target = &framebuffer[(context->origin_y + row) * LCD_WIDTH + context->origin_x];
        for (int column = x0; column < x1; ++column) {
            target[column] = color;
        }
    }
}

static void gif_clear_rect(gif_context_t *context, int x, int y, int width, int height)
{
    if (x + width > context->canvas_width) width = context->canvas_width - x;
    if (y + height > context->canvas_height) height = context->canvas_height - y;
    if (width <= 0 || height <= 0) {
        return;
    }
    for (int row = context->ys[y]; row < context->ys[y + height]; ++row) {
        uint16_t *target = &framebuffer[(context->origin_y + row) * LCD_WIDTH + context->origin_x];
        for (int column = context->xs[x]; column < context->xs[x + width]; ++column) {
            target[column] = 0;
        }
    }
}

static bool gif_decode_image(gif_context_t *context)
{
    int minimum = fgetc(media_file);
    if (minimum < 1 || minimum > 11) {
        return false;
    }
    const unsigned clear_code = 1u << minimum;
    const unsigned end_code = clear_code + 1;
    unsigned next_code = clear_code + 2;
    unsigned code_size = (unsigned)minimum + 1;
    int previous = -1;
    unsigned first = 0;
    int x = 0;
    int y = 0;
    int pass = 0;
    int stack_top = 0;
    context->block_size = context->block_position = 0;
    context->bit_buffer = 0;
    context->bit_count = 0;

    while (y < context->frame_height) {
        while (context->bit_count < code_size) {
            int byte = gif_read_byte(context);
            if (byte < 0) {
                goto done;
            }
            context->bit_buffer |= (uint32_t)byte << context->bit_count;
            context->bit_count += 8;
        }
        unsigned code = context->bit_buffer & ((1u << code_size) - 1);
        context->bit_buffer >>= code_size;
        context->bit_count -= code_size;

        if (code == clear_code) {
            next_code = clear_code + 2;
            code_size = (unsigned)minimum + 1;
            previous = -1;
            continue;
        }
        if (code == end_code) {
            break;
        }
        unsigned incoming = code;
        if (previous < 0) {
            if (code >= clear_code) {
                break;
            }
            context->stack[stack_top++] = (uint8_t)code;
            first = code;
        } else {
            if (code >= next_code) {
                if (code > next_code) {
                    break;
                }
                context->stack[stack_top++] = (uint8_t)first;
                code = (unsigned)previous;
            }
            while (code >= clear_code) {
                if (stack_top >= GIF_MAX_CODES - 1) {
                    goto done;
                }
                context->stack[stack_top++] = context->suffix[code];
                code = context->prefix[code];
            }
            first = code;
            context->stack[stack_top++] = (uint8_t)code;
            if (next_code < GIF_MAX_CODES) {
                context->prefix[next_code] = (uint16_t)previous;
                context->suffix[next_code] = (uint8_t)first;
                ++next_code;
                if (next_code == (1u << code_size) && code_size < 12) {
                    ++code_size;
                }
            }
        }
        previous = (int)incoming;

        while (stack_top > 0 && y < context->frame_height) {
            uint8_t index = context->stack[--stack_top];
            int canvas_x = context->frame_x + x;
            int canvas_y = context->frame_y + y;
            if ((!context->transparent || index != context->transparent_index) &&
                canvas_x < context->canvas_width && canvas_y < context->canvas_height) {
                gif_paint(context, canvas_x, canvas_y, context->palette[index]);
            }
            if (++x >= context->frame_width) {
                x = 0;
                if (!context->interlaced) {
                    ++y;
                } else {
                    static const int step[4] = {8, 8, 4, 2};
                    static const int start[4] = {0, 4, 2, 1};
                    y += step[pass];
                    while (y >= context->frame_height && pass < 3) {
                        ++pass;
                        y = start[pass];
                    }
                    if (y >= context->frame_height) {
                        y = context->frame_height;
                    }
                }
            }
        }
        stack_top = 0;
    }
done:
    gif_skip_blocks();
    return true;
}

// Bir sonraki kareyi cizer; kare gecikmesini (1/100 sn) dondurur, hata olursa -1.
static int gif_next_frame(void)
{
    gif_context_t *context = gif;
    bool restarted = false;
    for (;;) {
        int block = fgetc(media_file);
        if (block == 0x21) {
            int label = fgetc(media_file);
            if (label == 0xf9) {
                uint8_t data[6];
                if (fread(data, 1, 6, media_file) != 6) {
                    return -1;
                }
                context->disposal = (data[1] >> 2) & 7;
                context->transparent = data[1] & 1;
                context->delay_cs = (uint16_t)(data[2] | (data[3] << 8));
                context->transparent_index = data[4];
            } else {
                gif_skip_blocks();
            }
        } else if (block == 0x2c) {
            uint8_t descriptor[9];
            if (fread(descriptor, 1, 9, media_file) != 9) {
                return -1;
            }
            if (context->last_disposal == 2) {
                gif_clear_rect(context, context->last_x, context->last_y,
                               context->last_width, context->last_height);
            }
            context->frame_x = (uint16_t)(descriptor[0] | (descriptor[1] << 8));
            context->frame_y = (uint16_t)(descriptor[2] | (descriptor[3] << 8));
            context->frame_width = (uint16_t)(descriptor[4] | (descriptor[5] << 8));
            context->frame_height = (uint16_t)(descriptor[6] | (descriptor[7] << 8));
            context->interlaced = descriptor[8] & 0x40;
            if (descriptor[8] & 0x80) {
                if (!gif_read_palette(context->palette, 1u << ((descriptor[8] & 7) + 1))) {
                    return -1;
                }
            } else {
                memcpy(context->palette, context->global_palette, sizeof(context->palette));
            }
            if (!gif_decode_image(context)) {
                return -1;
            }
            context->last_x = context->frame_x;
            context->last_y = context->frame_y;
            context->last_width = context->frame_width;
            context->last_height = context->frame_height;
            context->last_disposal = context->disposal;
            ++context->frames_in_pass;
            int delay = context->delay_cs < 2 ? 10 : context->delay_cs;
            context->delay_cs = 0;
            context->transparent = false;
            context->disposal = 0;
            return delay;
        } else {
            if (restarted || context->frames_in_pass == 0) {
                return -1;
            }
            if (context->frames_in_pass == 1) {
                return -2;
            }
            restarted = true;
            context->frames_in_pass = 0;
            context->last_disposal = 0;
            memset(framebuffer, 0, LCD_BYTES);
            if (fseek(media_file, context->data_start, SEEK_SET) != 0) {
                return -1;
            }
        }
    }
}

static bool media_show_gif(const char *path)
{
    media_error_text = "BAD FILE";
    media_file = fopen(path, "rb");
    if (media_file == NULL) {
        return false;
    }
    uint8_t header[13];
    if (fread(header, 1, sizeof(header), media_file) != sizeof(header) ||
        memcmp(header, "GIF8", 4) != 0) {
        return false;
    }
    gif = calloc(1, sizeof(*gif));
    if (gif == NULL) {
        media_error_text = "NO MEMORY";
        return false;
    }
    gif->canvas_width = (uint16_t)(header[6] | (header[7] << 8));
    gif->canvas_height = (uint16_t)(header[8] | (header[9] << 8));
    if (gif->canvas_width == 0 || gif->canvas_height == 0) {
        return false;
    }
    gif->prefix = malloc(GIF_MAX_CODES * sizeof(uint16_t));
    gif->suffix = malloc(GIF_MAX_CODES);
    gif->stack = malloc(GIF_MAX_CODES);
    gif->xs = malloc(((size_t)gif->canvas_width + 1) * sizeof(uint16_t));
    gif->ys = malloc(((size_t)gif->canvas_height + 1) * sizeof(uint16_t));
    if (gif->prefix == NULL || gif->suffix == NULL || gif->stack == NULL ||
        gif->xs == NULL || gif->ys == NULL) {
        media_error_text = "NO MEMORY";
        return false;
    }
    if (header[10] & 0x80) {
        if (!gif_read_palette(gif->global_palette, 1u << ((header[10] & 7) + 1))) {
            return false;
        }
    }
    gif->data_start = ftell(media_file);

    media_fit(gif->canvas_width, gif->canvas_height, &gif->dest_width, &gif->dest_height);
    // Kucuk GIF'ler tam sayi katiyla buyutulur; kenarlar keskin kalir.
    unsigned zoom = (unsigned)(LCD_WIDTH / gif->canvas_width);
    unsigned zoom_y = (unsigned)(LCD_HEIGHT / gif->canvas_height);
    if (zoom_y < zoom) zoom = zoom_y;
    if (zoom > 1 && gif->canvas_width <= LCD_WIDTH && gif->canvas_height <= LCD_HEIGHT) {
        gif->dest_width = (int)(gif->canvas_width * zoom);
        gif->dest_height = (int)(gif->canvas_height * zoom);
    }
    gif->origin_x = (LCD_WIDTH - gif->dest_width) / 2;
    gif->origin_y = (LCD_HEIGHT - gif->dest_height) / 2;
    for (int i = 0; i <= gif->canvas_width; ++i) {
        gif->xs[i] = (uint16_t)((long)i * gif->dest_width / gif->canvas_width);
    }
    for (int i = 0; i <= gif->canvas_height; ++i) {
        gif->ys[i] = (uint16_t)((long)i * gif->dest_height / gif->canvas_height);
    }

    memset(framebuffer, 0, LCD_BYTES);
    int delay = gif_next_frame();
    if (delay == -2) {
        media_state = MEDIA_STILL;
        return true;
    }
    if (delay < 0) {
        return false;
    }
    media_state = MEDIA_ANIMATED;
    media_next_us = esp_timer_get_time() + (int64_t)delay * 10000;
    return true;
}

static void media_start(void)
{
    const media_entry_t *item = &media_files[media_selected];
    bool ok = item->gif ? media_show_gif(item->path) : media_show_jpeg(item->path);
    if (!ok) {
        media_close();
        media_state = MEDIA_ERROR;
        return;
    }
    if (media_state == MEDIA_PENDING) {
        media_state = MEDIA_STILL;
    }
    if (media_state == MEDIA_STILL && media_file != NULL) {
        fclose(media_file);
        media_file = NULL;
        media_free_gif();
    }
}

static void media_open(unsigned index)
{
    if (index >= media_count) {
        return;
    }
    media_close();
    media_selected = index;
    media_state = MEDIA_PENDING;
}

static void draw_media(void)
{
    if (media_state == MEDIA_PENDING) {
        draw_background();
        draw_centered_text(160, 78, "LOADING", 1, C_MUTED);
        draw_centered_text(160, 94, media_files[media_selected].title, 1, C_TEXT);
        display_present();
        media_start();
        return;
    }
    if (media_state == MEDIA_ERROR) {
        draw_background();
        draw_card(30, 50, 260, 70, 14, false, C_ORANGE);
        draw_centered_text(160, 76, media_error_text, 1, C_ORANGE);
        draw_centered_text(160, 96, media_files[media_selected].title, 1, C_MUTED);
        return;
    }
    if (media_state == MEDIA_ANIMATED && esp_timer_get_time() >= media_next_us) {
        int delay = gif_next_frame();
        if (delay < 0) {
            sd_handle_removal();
            return;
        }
        int64_t now = esp_timer_get_time();
        media_next_us += (int64_t)delay * 10000;
        if (media_next_us < now) {
            media_next_us = now;
        }
    }
}
static void draw_gallery(void)
{
    draw_background();
    draw_header("GALLERY", NULL);

    if (!sd_card_mounted) {
        draw_card(10, 40, 300, 80, 14, false, C_ORANGE);
        draw_centered_text(160, 66, sd_status, 1, sd_status_is_error ? C_ORANGE : C_TEXT);
        draw_centered_text(160, 90, "NO CARD", 1, C_MUTED);
    } else if (media_count == 0) {
        draw_card(10, 34, 300, 100, 14, false, C_ORANGE);
        draw_centered_text(160, 56, "NO MEDIA", 1, C_ORANGE);
        draw_centered_text(160, 80, "COPY JPG / JPEG / GIF TO THE CARD", 1, C_MUTED);
        draw_centered_text(160, 100, "ANY FOLDER WORKS", 1, C_MUTED);
    } else {
        const unsigned visible = 4;
        unsigned first = list_first(media_selected, media_count, visible);
        for (unsigned row = 0; row < visible && first + row < media_count; ++row) {
            unsigned i = first + row;
            int y = 28 + (int)row * 31;
            draw_card(10, y, 298, 27, 8, i == media_selected, C_CYAN);
            draw_text(22, y + 10, media_files[i].title, 1, C_TEXT);
            draw_text_right(296, y + 10, media_files[i].gif ? "GIF" : "PHOTO", 1, C_MUTED);
        }
        draw_scroll_bar(first, media_count, visible, 28, 120);
    }
    draw_footer(sd_card_mounted ? "SW OPEN / LEFT BACK / RIGHT RESCAN" : "LEFT BACK");
}

static void sd_handle_removal(void)
{
    media_close();
    media_count = 0;
    media_selected = 0;
    memset(media_files, 0, sizeof(media_files));
    sd_lua_close();
    if (sd_card_mounted && sd_card != NULL) {
        esp_err_t result = esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "SD kart guvenli kaldirilamadi: %s", esp_err_to_name(result));
        }
    }
    sd_card = NULL;
    sd_card_mounted = false;
    sd_app_count = 0;
    sd_selected_app = 0;
    sd_app_running = false;
    sd_app_counter = 0;
    sd_app_accent = 0;
    memset(sd_lua_error, 0, sizeof(sd_lua_error));
    memset(sd_apps, 0, sizeof(sd_apps));
    memset(sd_app_title, 0, sizeof(sd_app_title));
    memset(sd_app_text, 0, sizeof(sd_app_text));
    memset(sd_app_type, 0, sizeof(sd_app_type));
    snprintf(sd_status, sizeof(sd_status), "CARD REMOVED");
    sd_status_is_error = true;
    sd_next_check_us = esp_timer_get_time() + 2000000;
    if (current_view == VIEW_SD_APP) {
        transition_to(VIEW_SD_APPS);
    } else if (current_view == VIEW_MEDIA) {
        transition_to(VIEW_GALLERY);
    }
    ESP_LOGW(TAG, "SD kart cikarildi; gecici uygulama bellekten silindi");
}

static void sd_service(int64_t now_us)
{
    if (!sd_card_mounted) {
        if ((current_view == VIEW_SD_APPS || current_view == VIEW_GALLERY) &&
            now_us >= sd_next_check_us) {
            sd_next_check_us = now_us + 2000000;
            if (current_view == VIEW_GALLERY) {
                media_scan();
            } else {
                sd_scan_apps();
            }
        }
        return;
    }
    if (now_us < sd_next_check_us) {
        return;
    }
    sd_next_check_us = now_us + 1000000;
    esp_err_t result = sdmmc_get_status(sd_card);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "SD kart erisimi kesildi: %s", esp_err_to_name(result));
        sd_handle_removal();
    }
}

static void sd_release(void)
{
    media_close();
    media_count = 0;
    memset(media_files, 0, sizeof(media_files));
    sd_lua_close();
    if (sd_card_mounted && sd_card != NULL) {
        esp_err_t result = esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "SD kart baglantisi kapatilamadi: %s", esp_err_to_name(result));
        }
    }
    sd_card = NULL;
    sd_card_mounted = false;
    sd_app_count = 0;
    sd_app_running = false;
    sd_app_counter = 0;
    sd_app_accent = 0;
    memset(sd_lua_error, 0, sizeof(sd_lua_error));
    memset(sd_apps, 0, sizeof(sd_apps));
    memset(sd_app_title, 0, sizeof(sd_app_title));
    memset(sd_app_text, 0, sizeof(sd_app_text));
    memset(sd_app_type, 0, sizeof(sd_app_type));
}

static bool sd_run_selected_app(void)
{
    if (!sd_card_mounted || sd_selected_app >= sd_app_count) {
        sd_scan_apps();
        return false;
    }

    bool lua_script = sd_apps[sd_selected_app].lua_script;
    char path[128];
    int path_length = snprintf(path, sizeof(path), "%s/%s",
                               SD_APPS_DIRECTORY, sd_apps[sd_selected_app].filename);
    if (path_length < 0 || (size_t)path_length >= sizeof(path)) {
        snprintf(sd_status, sizeof(sd_status), "CARD ERROR");
        sd_status_is_error = true;
        return false;
    }

    struct stat file_status;
    if (stat(path, &file_status) != 0 || !S_ISREG(file_status.st_mode) ||
        file_status.st_size <= 0 ||
        file_status.st_size > (lua_script ? SD_LUA_FILE_LIMIT :
                                             SD_APP_FILE_LIMIT)) {
        snprintf(sd_status, sizeof(sd_status), "CARD ERROR");
        sd_status_is_error = true;
        return false;
    }
    if (lua_script) {
        snprintf(sd_app_title, sizeof(sd_app_title), "%s",
                 sd_apps[sd_selected_app].title);
        snprintf(sd_app_type, sizeof(sd_app_type), "%s", "lua");
        memset(sd_lua_error, 0, sizeof(sd_lua_error));
        sd_app_running = sd_lua_run_file(path, sd_app_title);
        if (!sd_app_running) {
            if (sd_lua_error[0] == '\0') {
                snprintf(sd_lua_error, sizeof(sd_lua_error), "%s",
                         "Unable to load Lua app");
            }
            snprintf(sd_app_type, sizeof(sd_app_type), "%s", "error");
        }
        return true;
    }
    sd_app_package_t package;
    if (!sd_parse_package(path, &package)) {
        snprintf(sd_status, sizeof(sd_status), "CARD ERROR");
        sd_status_is_error = true;
        ESP_LOGE(TAG, "Secilen rexaUi paketi okunamadi: %s",
                 sd_apps[sd_selected_app].filename);
        return false;
    }
    snprintf(sd_app_title, sizeof(sd_app_title), "%s", package.title);
    snprintf(sd_app_type, sizeof(sd_app_type), "%s", package.type);
    snprintf(sd_app_text, sizeof(sd_app_text), "%s", package.text);
    sd_app_accent = package.accent;
    sd_app_counter = 0;
    sd_app_running = true;
    memset(&package, 0, sizeof(package));
    return true;
}

static void draw_sd_app_text(int y, const char *text, uint16_t color)
{
    const size_t characters_per_line = 44;
    size_t length = strlen(text);
    size_t offset = 0;
    unsigned line_number = 0;
    while (offset < length && line_number < 3) {
        size_t line_length = length - offset;
        if (line_length > characters_per_line) {
            line_length = characters_per_line;
            size_t break_at = line_length;
            while (break_at > 0 && text[offset + break_at] != ' ') {
                --break_at;
            }
            if (break_at > 0) {
                line_length = break_at;
            }
        }
        char line[characters_per_line + 1];
        memcpy(line, text + offset, line_length);
        line[line_length] = '\0';
        draw_centered_text(160, y + (int)line_number * 12, line, 1, color);
        offset += line_length;
        while (text[offset] == ' ') {
            ++offset;
        }
        ++line_number;
    }
}

static void draw_sd_apps(void)
{
    draw_background();
    draw_header("SD APPS", NULL);

    if (!sd_card_mounted) {
        draw_card(10, 40, 300, 80, 14, false, C_ORANGE);
        draw_centered_text(160, 66, sd_status, 1, sd_status_is_error ? C_ORANGE : C_TEXT);
        draw_centered_text(160, 90, "NO CARD", 1, C_MUTED);
    } else if (sd_app_count == 0) {
        draw_card(10, 34, 300, 100, 14, false, C_ORANGE);
        draw_centered_text(160, 56, "NO APPS", 1, C_ORANGE);
        draw_centered_text(160, 80, "ADD APPS TO /REXA/APPS", 1, C_MUTED);
        draw_centered_text(160, 100, "REXAUI/1 PACKAGE", 1, C_MUTED);
    } else {
        const unsigned visible = 4;
        unsigned first = list_first(sd_selected_app, sd_app_count, visible);
        for (unsigned row = 0; row < visible && first + row < sd_app_count; ++row) {
            unsigned i = first + row;
            int y = 28 + (int)row * 31;
            draw_card(10, y, 298, 27, 8, i == sd_selected_app, C_CYAN);
            draw_text(22, y + 10, sd_apps[i].title, 1, C_TEXT);
            draw_text_right(296, y + 10, sd_apps[i].lua_script ? "LUA" : "RXA", 1, C_MUTED);
        }
        draw_scroll_bar(first, sd_app_count, visible, 28, 120);
    }

    draw_footer(sd_card_mounted ? "SW RUN / LEFT BACK / RIGHT RESCAN" : "LEFT BACK");
}

static void draw_sd_app(void)
{
    if (strcmp(sd_app_type, "lua") == 0 && sd_app_running) {
        draw_background();
        sd_lua_call("update", true);
        if (sd_app_running) {
            sd_lua_call("draw", false);
        }
        sd_lua_input_direction = DIR_NONE;
        sd_lua_button_pressed = false;
        if (sd_app_running) {
            draw_header(sd_app_title, NULL);
            return;
        }
    }
    if (strcmp(sd_app_type, "lua") == 0) {
        draw_background();
        draw_header(sd_app_title, NULL);
        draw_centered_text(160, 80, "APP STOPPED", 1, C_MUTED);
        draw_footer("CLICK BACK");
        return;
    }
    if (strcmp(sd_app_type, "error") == 0) {
        draw_background();
        draw_header(sd_app_title, NULL);
        draw_card(10, 30, 300, 110, 14, false, C_ORANGE);
        draw_centered_text(160, 46, "APP FAILED", 1, C_ORANGE);
        draw_sd_app_text(72, sd_lua_error, C_TEXT);
        draw_footer("CLICK BACK");
        return;
    }
    draw_background();
    draw_header(sd_app_title, NULL);
    draw_card(10, 30, 300, 110, 14, false, sd_app_accent);

    if (strcmp(sd_app_type, "counter") == 0) {
        char value[16];
        snprintf(value, sizeof(value), "%ld", (long)sd_app_counter);
        draw_centered_text(160, 48, value, 3, sd_app_accent);
        draw_sd_app_text(90, sd_app_text, C_TEXT);
        draw_centered_text(160, 124, "MOVE LEFT OR RIGHT", 1, C_MUTED);
    } else if (strcmp(sd_app_type, "clock") == 0) {
        time_t now = time(NULL);
        struct tm local_time = {0};
        char time_text[9] = "--:--:--";
        if (localtime_r(&now, &local_time) != NULL) {
            strftime(time_text, sizeof(time_text), "%H:%M:%S", &local_time);
        }
        draw_centered_text(160, 56, time_text, 3, sd_app_accent);
        draw_sd_app_text(100, sd_app_text, C_TEXT);
    } else {
        draw_sd_app_text(70, sd_app_text, C_TEXT);
    }
    draw_footer("CLICK BACK");
}

static const char *wifi_state_label(wifi_state_t state)
{
    switch (state) {
    case WIFI_STATE_OFF: return "WIFI OFF";
    case WIFI_STATE_STARTING: return "WIFI STARTING";
    case WIFI_STATE_SCANNING: return "SCANNING NETWORKS";
    case WIFI_STATE_READY: return "SELECT NETWORK";
    case WIFI_STATE_CONNECTING: return "CONNECTING";
    case WIFI_STATE_CONNECTED: return "CONNECTED";
    case WIFI_STATE_FAILED: return "CONNECTION FAILED";
    }
    return "WIFI";
}

static void draw_wifi(void)
{
    draw_background();
    draw_header("WIFI", wifi_state_label(wifi_state));

    if (!wifi_enabled) {
        draw_centered_text(160, 70, "WIFI OFF", 1, C_MUTED);
        draw_centered_text(160, 90, "ENABLE IN SETTINGS", 1, C_TEXT);
    } else if (wifi_network_count == 0 && wifi_state == WIFI_STATE_SCANNING) {
        draw_centered_text(160, 80, "SEARCHING", 1, C_CYAN);
    } else if (wifi_network_count == 0 && wifi_state != WIFI_STATE_CONNECTED) {
        draw_centered_text(160, 70, "NO NETWORKS", 1, C_TEXT);
        draw_centered_text(160, 90, "MOVE RIGHT RESCAN", 1, C_MUTED);
    }

    const unsigned visible = 4;
    unsigned first = list_first(wifi_selected_network, wifi_network_count, visible);
    for (unsigned row = 0; row < visible && first + row < wifi_network_count; ++row) {
        unsigned i = first + row;
        int y = 28 + (int)row * 31;
        bool selected = i == wifi_selected_network;
        draw_card(10, y, 298, 27, 8, selected, C_CYAN);

        char ssid[33];
        memcpy(ssid, wifi_networks[i].ssid, sizeof(wifi_networks[i].ssid));
        ssid[sizeof(wifi_networks[i].ssid)] = '\0';
        if (ssid[0] == '\0') {
            snprintf(ssid, sizeof(ssid), "HIDDEN NETWORK");
        }
        draw_text(20, y + 5, ssid, 1, C_TEXT);
        draw_text(20, y + 16, wifi_networks[i].authmode == WIFI_AUTH_OPEN ? "OPEN" : "LOCKED",
                  1, selected ? C_CYAN : C_MUTED);
        char signal[8];
        snprintf(signal, sizeof(signal), "%d", (int)wifi_networks[i].rssi);
        draw_text_right(296, y + 10, signal, 1, C_MUTED);
    }
    draw_scroll_bar(first, wifi_network_count, visible, 28, 120);

    draw_footer(wifi_state == WIFI_STATE_CONNECTED ? "CLICK CONNECT / LEFT BACK" :
                "CLICK SELECT / LEFT BACK");
}

static const char *ble_state_label(ble_state_t state)
{
    switch (state) {
    case BLE_STATE_OFF: return "BLUETOOTH OFF";
    case BLE_STATE_STARTING: return "BLUETOOTH STARTING";
    case BLE_STATE_SCANNING: return "SCANNING DEVICES";
    case BLE_STATE_READY: return "SELECT BLE DEVICE";
    case BLE_STATE_CONNECTING: return "CONNECTING";
    case BLE_STATE_CONNECTED: return "CONNECTED";
    case BLE_STATE_FAILED: return "BLUETOOTH ERROR";
    }
    return "BLUETOOTH";
}

static void draw_bluetooth(void)
{
    draw_background();
    draw_header("BLUETOOTH", ble_state_label(ble_state));

    if (!bluetooth_enabled) {
        draw_centered_text(160, 70, "BLUETOOTH OFF", 1, C_MUTED);
        draw_centered_text(160, 90, "ENABLE IN SETTINGS", 1, C_TEXT);
    } else if (ble_device_count == 0 && ble_state == BLE_STATE_SCANNING) {
        draw_centered_text(160, 80, "SEARCHING", 1, C_PURPLE);
    } else if (ble_device_count == 0 && ble_state != BLE_STATE_CONNECTED) {
        draw_centered_text(160, 70, "NO BLE DEVICES", 1, C_TEXT);
        draw_centered_text(160, 90, "MOVE RIGHT RESCAN", 1, C_MUTED);
    }

    const unsigned visible = 4;
    unsigned first = list_first(ble_selected_device, ble_device_count, visible);
    for (unsigned row = 0; row < visible && first + row < ble_device_count; ++row) {
        unsigned i = first + row;
        int y = 28 + (int)row * 31;
        draw_card(10, y, 298, 27, 8, i == ble_selected_device, C_PURPLE);
        draw_text(22, y + 10, ble_devices[i].name, 1, C_TEXT);
        char signal[12];
        snprintf(signal, sizeof(signal), "R%d", ble_devices[i].rssi);
        draw_text_right(296, y + 10, signal, 1, C_MUTED);
    }
    draw_scroll_bar(first, ble_device_count, visible, 28, 120);

    draw_footer(bluetooth_enabled ? "CLICK CONNECT / LEFT BACK" : "LEFT BACK");
}

static void draw_wifi_keyboard(void)
{
    static const char keyboard_keys[KEYBOARD_KEY_COUNT] = {
        'A','B','C','D','E','F','G','H',
        'I','J','K','L','M','N','O','P',
        'Q','R','S','T','U','V','W','X',
        'Y','Z','0','1','2','3','4','5',
        '6','7','8','9','@','.','-','_',
        'B','C','Y','P','C','O','B',' '
    };
    draw_background();
    draw_header(keyboard_city_mode ? "SEARCH CITY" : "WIFI PASSWORD",
                keyboard_symbols ? "SYMBOLS" : (keyboard_shift ? "UPPERCASE" : "LOWERCASE"));

    draw_card(10, 26, 298, 21, 8, false, C_CYAN);
    draw_text(18, 33, keyboard_city_mode ? "CITY" : "PASS", 1, C_MUTED);
    const char *shown = wifi_password;
    if (wifi_password_length > 20) {
        shown += wifi_password_length - 20;
    }
    draw_text(60, 33, shown, 1, C_TEXT);
    if (wifi_password_length > 20) {
        draw_text(50, 33, "<", 1, C_MUTED);
    }
    char length_text[8];
    snprintf(length_text, sizeof(length_text), "%u/63", (unsigned)wifi_password_length);
    draw_text_right(298, 33, length_text, 1, C_CYAN);

    for (unsigned key = 0; key < KEYBOARD_KEY_COUNT; ++key) {
        unsigned row = key / KEYBOARD_COLUMNS;
        unsigned column = key % KEYBOARD_COLUMNS;
        int x = 10 + (int)column * 25;
        int y = 52 + (int)row * 26;
        bool selected = key == keyboard_selected_key;
        draw_card(x, y, 23, 23, 6, selected, C_CYAN);

        char label[4] = {keyboard_keys[key], '\0', '\0', '\0'};
        if (key < 26 && keyboard_symbols) {
            label[0] = wifi_symbol_keys[key];
        } else if (key < 26 && !keyboard_shift) {
            label[0] = (char)(label[0] + ('a' - 'A'));
        }
        if (key >= 36 && key < 40 && keyboard_symbols) {
            label[0] = wifi_extra_symbol_keys[key - 36];
        }
        if (key == 40) snprintf(label, sizeof(label), "BS");
        if (key == 41) snprintf(label, sizeof(label), "CAP");
        if (key == 42) snprintf(label, sizeof(label), "SYM");
        if (key == 43) snprintf(label, sizeof(label), "SP");
        if (key == 44) snprintf(label, sizeof(label), "CLR");
        if (key == 45) snprintf(label, sizeof(label), "OK");
        if (key == 46) snprintf(label, sizeof(label), "BK");
        if (key == 47) label[0] = ' ';
        draw_centered_text(x + 11, y + 8, label, 1,
                           selected ? C_CYAN : (key >= 40 ? C_BLUE : C_TEXT));
    }

    draw_footer(keyboard_too_short ? (keyboard_city_mode ? "TYPE A CITY NAME" : "PASSWORD MIN 8")
                                   : (keyboard_city_mode ? "SW TYPE / OK SEARCH" : "SW TYPE / OK JOIN"));
}

static void wifi_save_credentials(const char *ssid, const char *password)
{
    char old_ssid[33] = "";
    char old_password[PASSWORD_CAPACITY] = "";
    size_t length = sizeof(old_ssid);
    if (nvs_get_str(settings, "wf_ssid", old_ssid, &length) != ESP_OK) {
        old_ssid[0] = '\0';
    }
    length = sizeof(old_password);
    if (nvs_get_str(settings, "wf_pass", old_password, &length) != ESP_OK) {
        old_password[0] = '\0';
    }
    if (strcmp(old_ssid, ssid) == 0 && strcmp(old_password, password) == 0) {
        return;
    }
    if (nvs_set_str(settings, "wf_ssid", ssid) == ESP_OK &&
        nvs_set_str(settings, "wf_pass", password) == ESP_OK) {
        nvs_commit(settings);
    }
}

static void wifi_auto_connect(void)
{
    wifi_config_t config = {0};
    char ssid[33] = "";
    char password[PASSWORD_CAPACITY] = "";
    size_t length = sizeof(ssid);
    if (nvs_get_str(settings, "wf_ssid", ssid, &length) != ESP_OK || ssid[0] == '\0') {
        return;
    }
    length = sizeof(password);
    if (nvs_get_str(settings, "wf_pass", password, &length) != ESP_OK) {
        password[0] = '\0';
    }
    memcpy(config.sta.ssid, ssid, strnlen(ssid, sizeof(config.sta.ssid)));
    memcpy(config.sta.password, password, strnlen(password, sizeof(config.sta.password)));
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;
    if (esp_wifi_set_config(WIFI_IF_STA, &config) == ESP_OK) {
        wifi_retry_count = 0;
        wifi_state = WIFI_STATE_CONNECTING;
        esp_wifi_connect();
    }
}

static void start_wifi_scan(void)
{
    if (!wifi_enabled) {
        wifi_state = WIFI_STATE_OFF;
        return;
    }
    if (wifi_state == WIFI_STATE_SCANNING || wifi_state == WIFI_STATE_CONNECTING) {
        return;
    }

    wifi_network_count = 0;
    wifi_selected_network = 0;
    wifi_state = WIFI_STATE_SCANNING;
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 250
    };
    esp_err_t result = esp_wifi_scan_start(&scan_config, false);
    if (result != ESP_OK) {
        wifi_state = WIFI_STATE_FAILED;
        ESP_LOGE(TAG, "Wi-Fi taramasi baslatilamadi: %s", esp_err_to_name(result));
    }
}

static void wifi_event_handler(void *argument, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)argument;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        if (!wifi_enabled) {
            wifi_network_count = 0;
            wifi_state = WIFI_STATE_OFF;
            return;
        }
        uint16_t count = WIFI_NETWORK_LIMIT;
        esp_err_t result = esp_wifi_scan_get_ap_records(&count, wifi_networks);
        if (result != ESP_OK) {
            wifi_network_count = 0;
            wifi_state = WIFI_STATE_FAILED;
            ESP_LOGE(TAG, "Wi-Fi ag listesi alinamadi: %s", esp_err_to_name(result));
            return;
        }
        wifi_network_count = count;
        wifi_state = wifi_connected_ssid[0] != '\0' ? WIFI_STATE_CONNECTED :
                     (count > 0 ? WIFI_STATE_READY : WIFI_STATE_FAILED);
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (!wifi_enabled) {
            wifi_state = WIFI_STATE_OFF;
            return;
        }
        if (wifi_switching_network) {
            wifi_switching_network = false;
            wifi_connected_ssid[0] = '\0';
            return;
        }
        if (wifi_state == WIFI_STATE_CONNECTING || wifi_state == WIFI_STATE_CONNECTED) {
            wifi_connected_ssid[0] = '\0';
            if (wifi_retry_count < 5) {
                ++wifi_retry_count;
                wifi_state = WIFI_STATE_CONNECTING;
                esp_wifi_connect();
            } else {
                wifi_state = WIFI_STATE_FAILED;
                ESP_LOGW(TAG, "Wi-Fi baglantisi kesildi");
            }
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        if (!wifi_enabled) {
            return;
        }
        (void)event_data;
        wifi_config_t active = {0};
        if (esp_wifi_get_config(WIFI_IF_STA, &active) != ESP_OK) {
            return;
        }
        wifi_state = WIFI_STATE_CONNECTED;
        snprintf(wifi_connected_ssid, sizeof(wifi_connected_ssid), "%s", (char *)active.sta.ssid);
        wifi_switching_network = false;
        wifi_retry_count = 0;
        wifi_save_credentials((char *)active.sta.ssid, (char *)active.sta.password);
        ESP_LOGI(TAG, "Wi-Fi baglandi; IP alindi");
        start_time_sync();
    }
}

static void connect_selected_wifi(void)
{
    if (wifi_selected_network >= wifi_network_count) {
        return;
    }

    wifi_ap_record_t *network = &wifi_networks[wifi_selected_network];
    wifi_config_t config = {0};
    memcpy(config.sta.ssid, network->ssid, sizeof(network->ssid));
    memcpy(config.sta.password, wifi_password, wifi_password_length);
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;

    if (wifi_connected_ssid[0] != '\0') {
        wifi_state = WIFI_STATE_CONNECTING;
        wifi_switching_network = true;
        esp_err_t disconnect_result = esp_wifi_disconnect();
        if (disconnect_result != ESP_OK) {
            wifi_switching_network = false;
            wifi_state = WIFI_STATE_FAILED;
            ESP_LOGE(TAG, "Mevcut Wi-Fi baglantisi kesilemedi: %s",
                     esp_err_to_name(disconnect_result));
            return;
        }
    }

    esp_err_t result = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (result != ESP_OK) {
        wifi_switching_network = false;
        wifi_connected_ssid[0] = '\0';
        wifi_state = WIFI_STATE_FAILED;
        ESP_LOGE(TAG, "Wi-Fi ag ayarlari uygulanamadi: %s", esp_err_to_name(result));
        return;
    }

    wifi_state = WIFI_STATE_CONNECTING;
    wifi_retry_count = 0;
    result = esp_wifi_connect();
    if (result != ESP_OK) {
        wifi_state = WIFI_STATE_FAILED;
        ESP_LOGE(TAG, "Wi-Fi baglantisi baslatilamadi: %s", esp_err_to_name(result));
    }
}

static void begin_selected_wifi(void)
{
    if (wifi_selected_network >= wifi_network_count) {
        start_wifi_scan();
        return;
    }

    if (wifi_state == WIFI_STATE_CONNECTED &&
        memcmp(wifi_connected_ssid, wifi_networks[wifi_selected_network].ssid,
               sizeof(wifi_networks[wifi_selected_network].ssid)) == 0) {
        return;
    }

    wifi_password_length = 0;
    memset(wifi_password, 0, sizeof(wifi_password));
    if (wifi_networks[wifi_selected_network].authmode == WIFI_AUTH_OPEN) {
        connect_selected_wifi();
        return;
    }

    keyboard_shift = true;
    keyboard_symbols = false;
    keyboard_selected_key = 0;
    transition_to(VIEW_WIFI_KEYBOARD);
}

static void ble_gap_event_handler(esp_gap_ble_cb_event_t event,
                                  esp_ble_gap_cb_param_t *parameter)
{
    if (!bluetooth_enabled) {
        return;
    }
    if (event == ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT) {
        ble_scan_parameters_ready = parameter->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS;
        if (!ble_scan_parameters_ready) {
            ble_state = BLE_STATE_FAILED;
            ESP_LOGE(TAG, "BLE tarama parametreleri ayarlanamadi");
        } else if (current_view == VIEW_BLUETOOTH) {
            start_ble_scan();
        } else {
            ble_state = BLE_STATE_READY;
        }
        return;
    }

    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) {
        return;
    }

    if (parameter->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
        if (ble_state == BLE_STATE_SCANNING) {
            ble_state = BLE_STATE_READY;
        }
        return;
    }

    if (parameter->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
        return;
    }

    if (ble_device_count >= BLE_DEVICE_LIMIT) {
        return;
    }

    for (unsigned i = 0; i < ble_device_count; ++i) {
        if (memcmp(ble_devices[i].address, parameter->scan_rst.bda, sizeof(esp_bd_addr_t)) == 0) {
            ble_devices[i].rssi = parameter->scan_rst.rssi;
            return;
        }
    }

    ble_device_t *device = &ble_devices[ble_device_count];
    memset(device, 0, sizeof(*device));
    memcpy(device->address, parameter->scan_rst.bda, sizeof(esp_bd_addr_t));
    device->address_type = parameter->scan_rst.ble_addr_type;
    device->rssi = parameter->scan_rst.rssi;

    uint8_t name_length = 0;
    uint8_t *name = esp_ble_resolve_adv_data(parameter->scan_rst.ble_adv,
                                             ESP_BLE_AD_TYPE_NAME_CMPL,
                                             &name_length);
    if (name == NULL) {
        name = esp_ble_resolve_adv_data(parameter->scan_rst.ble_adv,
                                        ESP_BLE_AD_TYPE_NAME_SHORT,
                                        &name_length);
    }
    if (name != NULL && name_length > 0) {
        size_t copy_length = name_length < sizeof(device->name) - 1 ?
                             name_length : sizeof(device->name) - 1;
        memcpy(device->name, name, copy_length);
        device->name[copy_length] = '\0';
    } else {
        snprintf(device->name, sizeof(device->name), "%02X:%02X:%02X:%02X",
                 parameter->scan_rst.bda[2], parameter->scan_rst.bda[3],
                 parameter->scan_rst.bda[4], parameter->scan_rst.bda[5]);
    }
    ++ble_device_count;
}

static void ble_gatt_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gatt_interface,
                                   esp_ble_gattc_cb_param_t *parameter)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (parameter->reg.status != ESP_GATT_OK) {
            ble_state = BLE_STATE_FAILED;
            ESP_LOGE(TAG, "BLE GATT uygulamasi kaydedilemedi");
            return;
        }
        ble_gatt_interface = gatt_interface;
        ble_gatt_app_registered = true;
        if (ble_scan_parameters_ready) {
            ble_state = BLE_STATE_READY;
        }
        return;
    }

    if (event == ESP_GATTC_OPEN_EVT) {
        if (parameter->open.status == ESP_GATT_OK) {
            ble_state = BLE_STATE_CONNECTED;
            ble_connected_device = (int)ble_selected_device;
            ESP_LOGI(TAG, "BLE GATT baglantisi kuruldu");
        } else {
            ble_state = BLE_STATE_FAILED;
            ESP_LOGW(TAG, "BLE GATT baglantisi basarisiz: 0x%x", parameter->open.status);
        }
    }
}

static void start_ble_scan(void)
{
    if (!bluetooth_enabled) {
        ble_state = BLE_STATE_OFF;
        return;
    }
    if (!ble_scan_parameters_ready || ble_state == BLE_STATE_SCANNING ||
        ble_state == BLE_STATE_CONNECTING) {
        return;
    }

    ble_device_count = 0;
    ble_selected_device = 0;
    ble_state = BLE_STATE_SCANNING;
    esp_err_t result = esp_ble_gap_start_scanning(8);
    if (result != ESP_OK) {
        ble_state = BLE_STATE_FAILED;
        ESP_LOGE(TAG, "BLE taramasi baslatilamadi: %s", esp_err_to_name(result));
    }
}

static void connect_selected_ble(void)
{
    if (ble_selected_device >= ble_device_count || !ble_gatt_app_registered) {
        return;
    }
    ble_device_t *device = &ble_devices[ble_selected_device];
    esp_err_t result = esp_ble_gattc_open(ble_gatt_interface, device->address,
                                          device->address_type, true);
    if (result != ESP_OK) {
        ble_state = BLE_STATE_FAILED;
        ESP_LOGE(TAG, "BLE baglantisi baslatilamadi: %s", esp_err_to_name(result));
        return;
    }
    ble_state = BLE_STATE_CONNECTING;
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_sta() == NULL ? ESP_FAIL : ESP_OK);

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (wifi_enabled) {
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_state = WIFI_STATE_READY;
        wifi_auto_connect();
    } else {
        wifi_state = WIFI_STATE_OFF;
    }
}

static esp_err_t bluetooth_start(void)
{
    ble_state = BLE_STATE_STARTING;
    if (!bluetooth_initialized) {
        esp_err_t result = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
            return result;
        }

        esp_bt_controller_config_t controller_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ESP_RETURN_ON_ERROR(esp_bt_controller_init(&controller_config), TAG,
                            "Bluetooth denetleyicisi baslatilamadi");
        bluetooth_initialized = true;
    }

    if (!bluetooth_controller_enabled) {
        ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_BLE), TAG,
                            "Bluetooth denetleyicisi acilamadi");
        bluetooth_controller_enabled = true;
    }
    if (!bluetooth_host_initialized) {
        ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "Bluetooth host baslatilamadi");
        bluetooth_host_initialized = true;
    }
    if (!bluetooth_host_enabled) {
        ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "Bluetooth host acilamadi");
        bluetooth_host_enabled = true;
    }
    ESP_RETURN_ON_ERROR(esp_ble_gap_register_callback(ble_gap_event_handler), TAG,
                        "Bluetooth GAP callback kaydedilemedi");
    ESP_RETURN_ON_ERROR(esp_ble_gattc_register_callback(ble_gatt_event_handler), TAG,
                        "Bluetooth GATT callback kaydedilemedi");
    ESP_RETURN_ON_ERROR(esp_ble_gattc_app_register(0x55), TAG,
                        "Bluetooth GATT uygulamasi kaydedilemedi");

    esp_ble_scan_params_t scan_parameters = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    return esp_ble_gap_set_scan_params(&scan_parameters);
}

static esp_err_t set_wifi_enabled(bool enabled)
{
    if (wifi_enabled == enabled) {
        return ESP_OK;
    }
    if (enabled) {
        esp_err_t result = esp_wifi_start();
        if (result != ESP_OK) {
            return result;
        }
        wifi_enabled = true;
        wifi_state = WIFI_STATE_READY;
        wifi_auto_connect();
        save_boolean_setting("wifi_on", true);
        return ESP_OK;
    }

    esp_err_t result = esp_wifi_stop();
    if (result != ESP_OK) {
        return result;
    }
    wifi_enabled = false;
    wifi_connected_ssid[0] = '\0';
    wifi_network_count = 0;
    wifi_state = WIFI_STATE_OFF;
    save_boolean_setting("wifi_on", false);
    return ESP_OK;
}

static esp_err_t set_bluetooth_enabled(bool enabled)
{
    if (bluetooth_enabled == enabled) {
        return ESP_OK;
    }
    if (enabled) {
        bluetooth_enabled = true;
        esp_err_t result = bluetooth_start();
        if (result != ESP_OK) {
            bluetooth_enabled = false;
            ble_state = BLE_STATE_OFF;
            return result;
        }
        save_boolean_setting("bluetooth_on", true);
        return ESP_OK;
    }

    if (ble_state == BLE_STATE_SCANNING) {
        esp_err_t result = esp_ble_gap_stop_scanning();
        if (result != ESP_OK) {
            return result;
        }
    }
    esp_err_t result = esp_bluedroid_disable();
    if (result != ESP_OK) {
        return result;
    }
    bluetooth_host_enabled = false;
    result = esp_bt_controller_disable();
    if (result != ESP_OK) {
        return result;
    }
    bluetooth_controller_enabled = false;
    bluetooth_enabled = false;
    ble_scan_parameters_ready = false;
    ble_gatt_app_registered = false;
    ble_state = BLE_STATE_OFF;
    save_boolean_setting("bluetooth_on", false);
    return ESP_OK;
}

static void bluetooth_init(void)
{
    if (bluetooth_enabled) {
        ESP_ERROR_CHECK(bluetooth_start());
    } else {
        ble_state = BLE_STATE_OFF;
    }
}

static void render_view(view_t view)
{
    switch (view) {
    case VIEW_HOME:
        draw_home();
        break;
    case VIEW_CLOCK:
        draw_clock();
        break;
    case VIEW_GAME:
        draw_game();
        break;
    case VIEW_SETTINGS:
        draw_settings();
        break;
    case VIEW_BRIGHTNESS:
        draw_brightness();
        break;
    case VIEW_SLEEP_SETTINGS:
        draw_sleep_settings();
        break;
    case VIEW_POWER:
        draw_power();
        break;
    case VIEW_LED:
        draw_led();
        break;
    case VIEW_SYSTEM:
        draw_system();
        break;
    case VIEW_WIFI:
        draw_wifi();
        break;
    case VIEW_WIFI_KEYBOARD:
        draw_wifi_keyboard();
        break;
    case VIEW_BLUETOOTH:
        draw_bluetooth();
        break;
    case VIEW_LANGUAGE:
        draw_language();
        break;
    case VIEW_CLOCK_SETTINGS:
        draw_clock_settings();
        break;
    case VIEW_WEATHER:
        draw_weather();
        break;
    case VIEW_SD_APPS:
        draw_sd_apps();
        break;
    case VIEW_SD_APP:
        draw_sd_app();
        break;
    case VIEW_GALLERY:
        draw_gallery();
        break;
    case VIEW_MEDIA:
        draw_media();
        break;
    }
}

static bool sd_view(view_t view)
{
    return view == VIEW_SD_APPS || view == VIEW_SD_APP || view == VIEW_GALLERY ||
           view == VIEW_MEDIA;
}

static void transition_to(view_t next_view)
{
    if (next_view == current_view) {
        return;
    }
    bool leaving_sd = sd_view(current_view) && !sd_view(next_view);
    if (leaving_sd) {
        sd_release();
    }
    previous_view = current_view;
    current_view = next_view;
    transition_started_us = esp_timer_get_time();
    transition_active = next_view != VIEW_MEDIA && previous_view != VIEW_MEDIA;
}

static void apply_transition_fade(float amount)
{
    if (amount >= 0.999f) {
        return;
    }
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        uint16_t color = framebuffer[i];
        color = (uint16_t)((color << 8) | (color >> 8));
        unsigned red = (color >> 11) & 0x1f;
        unsigned green = (color >> 5) & 0x3f;
        unsigned blue = color & 0x1f;
        red = (unsigned)(red * amount);
        green = (unsigned)(green * amount);
        blue = (unsigned)(blue * amount);
        uint16_t scaled = (uint16_t)((red << 11) | (green << 5) | blue);
        framebuffer[i] = (uint16_t)((scaled << 8) | (scaled >> 8));
    }
}

static direction_t read_direction(void)
{
    int x = 2048;
    int y = 2048;
    esp_err_t x_result = adc_oneshot_read(adc1, ADC_CHANNEL_0, &x);
    esp_err_t y_result = adc_oneshot_read(adc1, ADC_CHANNEL_1, &y);
    if (x_result != ESP_OK || y_result != ESP_OK) {
        ESP_LOGE(TAG, "Joystick ADC okunamadi: X=%s, Y=%s",
                 esp_err_to_name(x_result), esp_err_to_name(y_result));
        return DIR_NONE;
    }
    if (y < 2048 - JOYSTICK_DEADZONE) {
        return DIR_UP;
    }
    if (y > 2048 + JOYSTICK_DEADZONE) {
        return DIR_DOWN;
    }
    if (x < 2048 - JOYSTICK_DEADZONE) {
        return DIR_LEFT;
    }
    if (x > 2048 + JOYSTICK_DEADZONE) {
        return DIR_RIGHT;
    }
    return DIR_NONE;
}

static bool read_button_pressed(void)
{
    bool down = gpio_get_level(JOY_PIN_SW) == 0;
    int64_t now = esp_timer_get_time();
    if (down != button_candidate_down) {
        button_candidate_down = down;
        button_change_us = now;
    }
    if (button_candidate_down != button_stable_down &&
        now - button_change_us >= 35000) {
        button_stable_down = button_candidate_down;
        return button_stable_down;
    }
    return false;
}

static void move_selection(direction_t direction)
{
    if (direction == DIR_LEFT || direction == DIR_UP) {
        selected_item = (selected_item + HOME_ITEM_COUNT - 1) % HOME_ITEM_COUNT;
    } else if (direction == DIR_RIGHT || direction == DIR_DOWN) {
        selected_item = (selected_item + 1) % HOME_ITEM_COUNT;
    }
}

static void handle_input(direction_t direction, bool button_pressed)
{
    switch (direction) {
    case DIR_UP: direction = DIR_LEFT; break;
    case DIR_DOWN: direction = DIR_RIGHT; break;
    case DIR_LEFT: direction = DIR_DOWN; break;
    case DIR_RIGHT: direction = DIR_UP; break;
    case DIR_NONE: break;
    }
    if (joystick_flipped) {
        switch (direction) {
        case DIR_UP: direction = DIR_DOWN; break;
        case DIR_DOWN: direction = DIR_UP; break;
        case DIR_LEFT: direction = DIR_RIGHT; break;
        case DIR_RIGHT: direction = DIR_LEFT; break;
        case DIR_NONE: break;
        }
    }

    if (current_view == VIEW_HOME) {
        if (direction != DIR_NONE) {
            move_selection(direction);
        }
        if (button_pressed) {
            if (selected_item == 0) {
                transition_to(VIEW_CLOCK);
            } else if (selected_item == 1) {
                transition_to(VIEW_WEATHER);
            } else if (selected_item == 2) {
                reset_game();
                transition_to(VIEW_GAME);
            } else if (selected_item == 3) {
                transition_to(VIEW_BLUETOOTH);
                start_ble_scan();
            } else if (selected_item == 4) {
                language_selected_item = language_turkish ? 1 : 0;
                transition_to(VIEW_LANGUAGE);
            } else if (selected_item == 5) {
                clock_city_selected = clock_city;
                transition_to(VIEW_CLOCK_SETTINGS);
            } else if (selected_item == 6) {
                sd_selected_app = 0;
                sd_status_is_error = true;
                snprintf(sd_status, sizeof(sd_status), "NO CARD");
                transition_to(VIEW_SD_APPS);
                sd_scan_apps();
            } else if (selected_item == 7) {
                media_selected = 0;
                sd_status_is_error = true;
                snprintf(sd_status, sizeof(sd_status), "NO CARD");
                transition_to(VIEW_GALLERY);
                media_scan();
            } else if (selected_item == 8) {
                led_selected_row = 0;
                led_from_home = true;
                transition_to(VIEW_LED);
            } else if (selected_item == 9) {
                power_selected_row = 0;
                led_from_home = true;
                transition_to(VIEW_POWER);
            } else {
                settings_selected_item = 0;
                transition_to(VIEW_SETTINGS);
            }
        }
        return;
    }

    if (current_view == VIEW_SETTINGS) {
        if (direction == DIR_UP && settings_selected_item >= SETTINGS_COLUMN_COUNT) {
            settings_selected_item -= SETTINGS_COLUMN_COUNT;
        } else if (direction == DIR_DOWN &&
                   settings_selected_item + SETTINGS_COLUMN_COUNT < SETTINGS_ITEM_COUNT) {
            settings_selected_item += SETTINGS_COLUMN_COUNT;
        } else if (direction == DIR_RIGHT &&
                   settings_selected_item % SETTINGS_COLUMN_COUNT == 0) {
            ++settings_selected_item;
        } else if (direction == DIR_LEFT) {
            if (settings_selected_item % SETTINGS_COLUMN_COUNT == 0) {
                transition_to(VIEW_HOME);
                return;
            }
            --settings_selected_item;
        }
        if (button_pressed) {
            esp_err_t result = ESP_OK;
            if (settings_selected_item == 0) {
                transition_to(VIEW_BRIGHTNESS);
            } else if (settings_selected_item == 1) {
                transition_to(VIEW_SYSTEM);
            } else if (settings_selected_item == 2) {
                transition_to(VIEW_WIFI);
                start_wifi_scan();
            } else if (settings_selected_item == 3) {
                transition_to(VIEW_SLEEP_SETTINGS);
            } else if (settings_selected_item == 4) {
                result = set_wifi_enabled(!wifi_enabled);
            } else if (settings_selected_item == 5) {
                result = set_bluetooth_enabled(!bluetooth_enabled);
            } else if (settings_selected_item == 6) {
                screen_flipped = !screen_flipped;
                result = nvs_set_u8(settings, "flip", screen_flipped ? 1 : 0);
                if (result == ESP_OK) {
                    result = nvs_commit(settings);
                }
            } else if (settings_selected_item == 7) {
                joystick_flipped = !joystick_flipped;
                result = nvs_set_u8(settings, "joy_flip", joystick_flipped ? 1 : 0);
                if (result == ESP_OK) {
                    result = nvs_commit(settings);
                }
            } else if (settings_selected_item == 8) {
                power_selected_row = 0;
                led_from_home = false;
                transition_to(VIEW_POWER);
            } else if (settings_selected_item == 9) {
                led_selected_row = 0;
                led_from_home = false;
                transition_to(VIEW_LED);
            }
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Baglanti anahtari degistirilemedi: %s",
                         esp_err_to_name(result));
            }
        }
        return;
    }

    if (current_view == VIEW_GALLERY) {
        if (direction == DIR_LEFT) {
            transition_to(VIEW_HOME);
            return;
        }
        if (direction == DIR_RIGHT) {
            media_scan();
        } else if (media_count > 0 && direction == DIR_UP) {
            media_selected = media_selected == 0 ? media_count - 1 : media_selected - 1;
        } else if (media_count > 0 && direction == DIR_DOWN) {
            media_selected = (media_selected + 1) % media_count;
        }
        if (button_pressed && media_count > 0) {
            media_open(media_selected);
            transition_to(VIEW_MEDIA);
        }
        return;
    }

    if (current_view == VIEW_MEDIA) {
        if (direction == DIR_LEFT || button_pressed) {
            media_close();
            transition_to(VIEW_GALLERY);
            return;
        }
        if (media_count > 1 && (direction == DIR_UP || direction == DIR_DOWN)) {
            media_open(direction == DIR_UP ? (media_selected + media_count - 1) % media_count :
                                             (media_selected + 1) % media_count);
        }
        return;
    }

    if (current_view == VIEW_SD_APPS) {
        if (direction == DIR_LEFT) {
            transition_to(VIEW_HOME);
            return;
        }
        if (direction == DIR_RIGHT) {
            sd_scan_apps();
        } else if (sd_app_count > 0 && direction == DIR_UP) {
            sd_selected_app = sd_selected_app == 0 ? sd_app_count - 1 :
                              sd_selected_app - 1;
        } else if (sd_app_count > 0 && direction == DIR_DOWN) {
            sd_selected_app = (sd_selected_app + 1) % sd_app_count;
        }
        if (button_pressed) {
            if (sd_run_selected_app()) {
                transition_to(VIEW_SD_APP);
            }
        }
        return;
    }

    if (current_view == VIEW_SD_APP) {
        if (strcmp(sd_app_type, "lua") == 0) {
            sd_lua_input_direction = direction;
            sd_lua_button_pressed = button_pressed;
            if (button_pressed) {
                sd_lua_close();
                sd_app_running = false;
                transition_to(VIEW_SD_APPS);
            }
            return;
        }
        if (button_pressed) {
            transition_to(VIEW_SD_APPS);
            return;
        }
        if (strcmp(sd_app_type, "counter") == 0) {
            if (direction == DIR_RIGHT && sd_app_counter < INT32_MAX) {
                ++sd_app_counter;
            } else if (direction == DIR_LEFT && sd_app_counter > INT32_MIN) {
                --sd_app_counter;
            } else if (direction == DIR_UP && sd_app_counter < INT32_MAX) {
                ++sd_app_counter;
            } else if (direction == DIR_DOWN && sd_app_counter > INT32_MIN) {
                --sd_app_counter;
            }
        } else if (direction == DIR_LEFT) {
            transition_to(VIEW_SD_APPS);
        }
        return;
    }

    if (current_view == VIEW_WIFI) {
        if (direction == DIR_LEFT) {
            transition_to(VIEW_SETTINGS);
            return;
        }
        if (direction == DIR_RIGHT) {
            start_wifi_scan();
        } else if (direction == DIR_UP && wifi_network_count > 0) {
            if (wifi_selected_network == 0) {
                wifi_selected_network = wifi_network_count - 1;
            } else {
                --wifi_selected_network;
            }
        } else if (direction == DIR_DOWN && wifi_network_count > 0) {
            wifi_selected_network = (wifi_selected_network + 1) % wifi_network_count;
        }
        if (button_pressed && wifi_state != WIFI_STATE_CONNECTING &&
            wifi_state != WIFI_STATE_SCANNING) {
            bool already_connected = wifi_state == WIFI_STATE_CONNECTED &&
                wifi_selected_network < wifi_network_count &&
                memcmp(wifi_connected_ssid, wifi_networks[wifi_selected_network].ssid,
                       sizeof(wifi_networks[wifi_selected_network].ssid)) == 0;
            if (!already_connected) {
                begin_selected_wifi();
            }
        }
        return;
    }

    if (current_view == VIEW_LANGUAGE) {
        if (direction == DIR_UP || direction == DIR_DOWN ||
            direction == DIR_LEFT || direction == DIR_RIGHT) {
            language_selected_item = language_selected_item == 0 ? 1 : 0;
        }
        if (direction == DIR_LEFT && !button_pressed) {
            transition_to(VIEW_HOME);
            return;
        }
        if (button_pressed) {
            language_turkish = language_selected_item == 1;
            save_boolean_setting("language", language_turkish);
            transition_to(VIEW_HOME);
        }
        return;
    }

    if (current_view == VIEW_CLOCK_SETTINGS) {
        const unsigned city_count = sizeof(clock_cities) / sizeof(clock_cities[0]);
        if (direction == DIR_UP && clock_city_selected > 0) {
            --clock_city_selected;
        } else if (direction == DIR_DOWN && clock_city_selected + 1 < city_count) {
            ++clock_city_selected;
        } else if (direction == DIR_RIGHT) {
            clock_city_selected = clock_city_selected + 8 < city_count ? clock_city_selected + 8
                                                                       : city_count - 1;
        } else if (direction == DIR_LEFT) {
            transition_to(VIEW_HOME);
            return;
        }
        if (button_pressed) {
            clock_city = clock_city_selected;
            apply_clock_city();
            ESP_ERROR_CHECK(nvs_set_u8(settings, "clock_city", (uint8_t)clock_city));
            ESP_ERROR_CHECK(nvs_commit(settings));
            if (wifi_enabled && wifi_state == WIFI_STATE_CONNECTED) {
                start_time_sync();
            }
            transition_to(VIEW_HOME);
        }
        return;
    }

    if (current_view == VIEW_WEATHER) {
        if (direction == DIR_LEFT) {
            transition_to(VIEW_HOME);
            return;
        }
        if (button_pressed && geo_state != 1) {
            geo_state = 0;
            keyboard_city_mode = true;
            wifi_password_length = 0;
            memset(wifi_password, 0, sizeof(wifi_password));
            keyboard_shift = true;
            keyboard_symbols = false;
            keyboard_selected_key = 0;
            transition_to(VIEW_WIFI_KEYBOARD);
        }
        return;
    }

    if (current_view == VIEW_SYSTEM) {
        if (direction == DIR_LEFT || button_pressed) {
            transition_to(VIEW_SETTINGS);
            return;
        }
        return;
    }

    if (current_view == VIEW_POWER) {
        if (direction == DIR_LEFT) {
            transition_to(led_from_home ? VIEW_HOME : VIEW_SETTINGS);
            return;
        }
        if (direction == DIR_UP && power_selected_row > 0) {
            --power_selected_row;
        } else if (direction == DIR_DOWN && power_selected_row < 1) {
            ++power_selected_row;
        }
        if (button_pressed || direction == DIR_RIGHT) {
            if (power_selected_row == 0) {
                power_saver = !power_saver;
                save_u8_setting("pwr_save", power_saver ? 1 : 0);
                power_apply_profile();
            } else {
                battery_capacity_index = (uint8_t)((battery_capacity_index + 1) % BATTERY_CAPACITY_COUNT);
                save_u8_setting("bat_cap2", battery_capacity_index);
                power_est_last_us = 0;
            }
        }
        return;
    }

    if (current_view == VIEW_LED) {
        if (direction == DIR_LEFT) {
            transition_to(led_from_home ? VIEW_HOME : VIEW_SETTINGS);
            return;
        }
        if (direction == DIR_UP && led_selected_row > 0) {
            --led_selected_row;
        } else if (direction == DIR_DOWN && led_selected_row < 3) {
            ++led_selected_row;
        }
        if (button_pressed || direction == DIR_RIGHT) {
            if (led_selected_row == 0) {
                led_mode = (uint8_t)((led_mode + 1) % LED_MODE_COUNT);
                save_u8_setting("led_mode", led_mode);
            } else if (led_selected_row == 1) {
                led_color = (uint8_t)((led_color + 1) % LED_COLOR_COUNT);
                save_u8_setting("led_color", led_color);
            } else if (led_selected_row == 2) {
                led_speed = (uint8_t)((led_speed + 1) % LED_SPEED_COUNT);
                save_u8_setting("led_speed", led_speed);
            } else {
                led_level = led_level >= 100 ? 10 : (uint8_t)(led_level + 10);
                save_u8_setting("led_level", led_level);
            }
        }
        return;
    }

    if (current_view == VIEW_SLEEP_SETTINGS) {
        bool changed = false;
        if (direction == DIR_UP && sleep_timeout_minutes < SLEEP_TIMEOUT_MAX_MINUTES) {
            ++sleep_timeout_minutes;
            changed = true;
        } else if (direction == DIR_DOWN && sleep_timeout_minutes > 0) {
            --sleep_timeout_minutes;
            changed = true;
        } else if (direction == DIR_LEFT || button_pressed) {
            transition_to(VIEW_SETTINGS);
            return;
        }
        if (changed) {
            ESP_ERROR_CHECK(nvs_set_u8(settings, "sleep_min", sleep_timeout_minutes));
            ESP_ERROR_CHECK(nvs_commit(settings));
        }
        return;
    }

    if (current_view == VIEW_WIFI_KEYBOARD) {
        if (direction == DIR_UP && keyboard_selected_key >= KEYBOARD_COLUMNS) {
            keyboard_selected_key -= KEYBOARD_COLUMNS;
        } else if (direction == DIR_DOWN &&
                   keyboard_selected_key + KEYBOARD_COLUMNS < KEYBOARD_KEY_COUNT) {
            keyboard_selected_key += KEYBOARD_COLUMNS;
        } else if (direction == DIR_LEFT && keyboard_selected_key % KEYBOARD_COLUMNS > 0) {
            --keyboard_selected_key;
        } else if (direction == DIR_RIGHT &&
                   keyboard_selected_key % KEYBOARD_COLUMNS < KEYBOARD_COLUMNS - 1) {
            ++keyboard_selected_key;
        }
        if (button_pressed) {
            if (keyboard_selected_key < 26) {
                if (wifi_password_length < PASSWORD_CAPACITY - 1) {
                    char character;
                    if (keyboard_symbols) {
                        character = wifi_symbol_keys[keyboard_selected_key];
                    } else {
                        character = (char)('A' + keyboard_selected_key);
                    }
                    if (!keyboard_symbols && !keyboard_shift) {
                        character = (char)(character + ('a' - 'A'));
                    }
                    wifi_password[wifi_password_length++] = character;
                    wifi_password[wifi_password_length] = '\0';
                    keyboard_too_short = false;
                }
            } else if (keyboard_selected_key < 36) {
                if (wifi_password_length < PASSWORD_CAPACITY - 1) {
                    wifi_password[wifi_password_length++] =
                        (char)('0' + keyboard_selected_key - 26);
                    wifi_password[wifi_password_length] = '\0';
                    keyboard_too_short = false;
                }
            } else if (keyboard_selected_key < 40) {
                if (wifi_password_length < PASSWORD_CAPACITY - 1) {
                    unsigned symbol_index = keyboard_selected_key - 36;
                    wifi_password[wifi_password_length++] = keyboard_symbols ?
                        wifi_extra_symbol_keys[symbol_index] :
                        wifi_password_punctuation[symbol_index];
                    wifi_password[wifi_password_length] = '\0';
                    keyboard_too_short = false;
                }
            } else if (keyboard_selected_key == 40) {
                if (wifi_password_length > 0) {
                    wifi_password[--wifi_password_length] = '\0';
                    keyboard_too_short = false;
                }
            } else if (keyboard_selected_key == 41) {
                keyboard_shift = !keyboard_shift;
            } else if (keyboard_selected_key == 42) {
                keyboard_symbols = !keyboard_symbols;
            } else if (keyboard_selected_key == 43) {
                if (wifi_password_length < PASSWORD_CAPACITY - 1) {
                    wifi_password[wifi_password_length++] = ' ';
                    wifi_password[wifi_password_length] = '\0';
                }
            } else if (keyboard_selected_key == 44) {
                wifi_password_length = 0;
                wifi_password[0] = '\0';
                keyboard_too_short = false;
            } else if (keyboard_selected_key == 45) {
                if (keyboard_city_mode) {
                    if (wifi_password_length >= 2) {
                        memcpy(geo_query, wifi_password, sizeof(geo_query));
                        keyboard_city_mode = false;
                        geocode_request();
                        wifi_password_length = 0;
                        memset(wifi_password, 0, sizeof(wifi_password));
                        transition_to(VIEW_WEATHER);
                    } else {
                        keyboard_too_short = true;
                    }
                } else if (wifi_password_length < 8) {
                    keyboard_too_short = true;
                } else {
                    connect_selected_wifi();
                    transition_to(VIEW_WIFI);
                }
            } else if (keyboard_selected_key == 46) {
                if (keyboard_city_mode) {
                    keyboard_city_mode = false;
                    wifi_password_length = 0;
                    memset(wifi_password, 0, sizeof(wifi_password));
                    transition_to(VIEW_WEATHER);
                } else {
                    transition_to(VIEW_WIFI);
                }
            }
        }
        return;
    }

    if (current_view == VIEW_BLUETOOTH) {
        if (direction == DIR_LEFT) {
            transition_to(VIEW_HOME);
            return;
        }
        if (direction == DIR_RIGHT) {
            start_ble_scan();
        } else if (direction == DIR_UP && ble_device_count > 0) {
            if (ble_selected_device == 0) {
                ble_selected_device = ble_device_count - 1;
            } else {
                --ble_selected_device;
            }
        } else if (direction == DIR_DOWN && ble_device_count > 0) {
            ble_selected_device = (ble_selected_device + 1) % ble_device_count;
        }
        if (button_pressed && ble_state != BLE_STATE_CONNECTING &&
            ble_state != BLE_STATE_SCANNING && ble_state != BLE_STATE_CONNECTED) {
            if (ble_device_count > 0) {
                connect_selected_ble();
            } else {
                start_ble_scan();
            }
        }
        return;
    }

    if (current_view == VIEW_BRIGHTNESS) {
        bool changed = false;
        if (direction == DIR_LEFT || direction == DIR_DOWN) {
            if (brightness >= BRIGHTNESS_STEP) {
                brightness -= BRIGHTNESS_STEP;
                changed = true;
            }
        } else if (direction == DIR_RIGHT || direction == DIR_UP) {
            if (brightness <= 100 - BRIGHTNESS_STEP) {
                brightness += BRIGHTNESS_STEP;
                changed = true;
            }
        }
        if (changed) {
            ESP_ERROR_CHECK(set_brightness(brightness));
            brightness_dirty = true;
            brightness_last_change_us = esp_timer_get_time();
        }
        if (button_pressed) {
            if (brightness_dirty) {
                save_brightness();
            }
            transition_to(VIEW_SETTINGS);
        }
        return;
    }

    if (current_view == VIEW_GAME) {
        if (button_pressed) {
            transition_to(VIEW_HOME);
            return;
        }
        if (game_over && direction != DIR_NONE) {
            reset_game();
            return;
        }
        if (direction != DIR_NONE) {
            bool reverses = (snake_direction == DIR_UP && direction == DIR_DOWN) ||
                            (snake_direction == DIR_DOWN && direction == DIR_UP) ||
                            (snake_direction == DIR_LEFT && direction == DIR_RIGHT) ||
                            (snake_direction == DIR_RIGHT && direction == DIR_LEFT);
            if (!reverses) {
                snake_direction = direction;
            }
        }
        return;
    }

    if (button_pressed) {
        transition_to(VIEW_HOME);
    }
}

static void update_voltage(void)
{
    const int sample_count = 16;
    int samples[16];
    for (int i = 0; i < sample_count; ++i) {
        samples[i] = 0;
        esp_err_t result = adc_oneshot_read(adc1, POWER_ADC_CHANNEL, &samples[i]);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Sistem gerilimi ADC okunamadi: %s", esp_err_to_name(result));
            return;
        }
    }
    for (int i = 1; i < sample_count; ++i) {
        int value = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > value) {
            samples[j + 1] = samples[j];
            --j;
        }
        samples[j + 1] = value;
    }
    int sum = 0;
    for (int i = 4; i < sample_count - 4; ++i) {
        sum += samples[i];
    }
    int raw = sum / (sample_count - 8);
    int millivolts = 0;
    if (power_adc_calibration == NULL ||
        adc_cali_raw_to_voltage(power_adc_calibration, raw, &millivolts) != ESP_OK) {
        millivolts = raw * 3300 / 4095;
    }
    float measured = (float)millivolts * 3.0f / 1000.0f;
    static bool filter_ready = false;
    if (!filter_ready) {
        system_voltage = measured;
        filter_ready = true;
    } else {
        system_voltage += (measured - system_voltage) * 0.1f;
    }
    battery_level = battery_percentage(system_voltage);
}

static void draw_sleep_clock(void)
{
    time_t now = time(NULL);
    struct tm local_time = {0};
    if (localtime_r(&now, &local_time) == NULL ||
        local_time.tm_min == sleep_drawn_minute) {
        return;
    }
    sleep_drawn_minute = local_time.tm_min;
    fill_screen(0);
    draw_big_time(48, 45, 44, 80, 8, local_time.tm_hour, local_time.tm_min, C_TEXT);
    ESP_ERROR_CHECK(display_present());
}

void app_main(void)
{
    app_started_us = esp_timer_get_time();
    set_clock_to_build_time();

    framebuffer = heap_caps_malloc(LCD_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_ERROR_CHECK(framebuffer == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(backlight_init());
    ESP_ERROR_CHECK(adc_init());

    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << JOY_PIN_SW,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&button_config));

    settings_init();
    power_apply_profile();
    if (led_init() != ESP_OK) {
        ESP_LOGW(TAG, "LED surucusu baslatilamadi");
    }
    wifi_init();
    bluetooth_init();
    reset_game();
    update_voltage();
    last_voltage_us = esp_timer_get_time();
    last_user_input_us = app_started_us;

    ESP_LOGI(TAG, "rexaUi arayuzu baslatildi; parlaklik %% %u", (unsigned)brightness);
    while (true) {
        int64_t now_us = esp_timer_get_time();
        direction_t direction = read_direction();
        bool button_pressed = read_button_pressed();
        if (sleep_mode_active) {
            if (direction != DIR_NONE || button_pressed) {
                exit_sleep_mode();
                last_user_input_us = now_us;
                last_direction = DIR_NONE;
                transition_active = false;
            } else {
                draw_sleep_clock();
            }
            led_service(now_us);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (direction != DIR_NONE || button_pressed) {
            last_user_input_us = now_us;
        }
        if (current_view == VIEW_MEDIA) {
            last_user_input_us = now_us;
        }
        unsigned effective_sleep_minutes = sleep_timeout_minutes;
        if (power_saver && (effective_sleep_minutes == 0 ||
                            effective_sleep_minutes > POWER_SAVER_SLEEP_MINUTES)) {
            effective_sleep_minutes = POWER_SAVER_SLEEP_MINUTES;
        }
        if (effective_sleep_minutes > 0) {
            int64_t timeout_us = (int64_t)effective_sleep_minutes * 60LL * 1000000LL;
            if (now_us - last_user_input_us >= timeout_us) {
                enter_sleep_mode();
                vTaskDelay(pdMS_TO_TICKS(FRAME_INTERVAL_MS));
                continue;
            }
        }
        sd_service(now_us);
        weather_service(now_us);
        led_service(now_us);
        power_estimate_service(now_us);
        bool direction_event = false;
        if (direction == DIR_NONE) {
            last_direction = DIR_NONE;
        } else if (direction != last_direction ||
                   now_us - last_direction_us >= 220000) {
            direction_event = true;
            last_direction = direction;
            last_direction_us = now_us;
        }
        handle_input(direction_event ? direction : DIR_NONE, button_pressed);

        if (brightness_dirty && now_us - brightness_last_change_us >= 900000) {
            save_brightness();
        }
        if (current_view == VIEW_GAME && now_us - last_game_step_us >= 190000) {
            step_game();
            last_game_step_us = now_us;
        }
        if (now_us - last_voltage_us >= 1000000) {
            update_voltage();
            last_voltage_us = now_us;
        }

        float fade = 1.0f;
        view_t view_to_draw = current_view;
        if (transition_active) {
            float progress = (float)(now_us - transition_started_us) / 300000.0f;
            if (progress >= 1.0f) {
                transition_active = false;
            } else if (progress < 0.5f) {
                view_to_draw = previous_view;
                fade = 1.0f - progress * 2.0f;
            } else {
                fade = (progress - 0.5f) * 2.0f;
            }
            fade = fade * fade * (3.0f - 2.0f * fade);
        }
        render_view(view_to_draw);
        apply_transition_fade(fade);
        ESP_ERROR_CHECK(display_present());
        bool fast_media = current_view == VIEW_MEDIA && media_state == MEDIA_ANIMATED;
        vTaskDelay(fast_media ? 1 : pdMS_TO_TICKS(power_saver ? 50 : FRAME_INTERVAL_MS));
    }
}
