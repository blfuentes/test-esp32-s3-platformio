#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>

#include "esp_log.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#include <led_strip.h>

#include "wificontroller.h"

#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define TAG "main"

extern "C" void app_main();

// OLED display
SSD1306_t dev;
int center, top, bottom;
char lineChar[20];

// RGB LED
#define TAGLED "led"
#define BLINK_GPIO GPIO_NUM_48
#define CONFIG_BLINK_LED_STRIP 1
#define CONFIG_BLINK_LED_STRIP_BACKEND_SPI 1

static uint8_t s_led_state = 0;

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAGLED, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST};
    spi_config.flags.with_dma = true;
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));

    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

// Define task handle
TaskHandle_t Task1;

// Task function for LED blink
void Task1code(void *parameter)
{
    while (1)
    {
        printf("Hello from the LED task!\n");
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// WIFI
#define EXAMPLE_ESP_WIFI_SSID "SSID_NAME"
#define EXAMPLE_ESP_WIFI_PASS "SSID_PASSWORD"

// IR Sensor
constexpr adc_channel_t adcChannel = ADC_CHANNEL_6;  // ADC channel for the sensor (GPIO 7)
constexpr adc_atten_t adcAtten = ADC_ATTEN_DB_12;    // Attenuation for the ADC
constexpr adc_bitwidth_t adcWidth = ADC_BITWIDTH_12; // ADC resolution

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set("wifi station", ESP_LOG_VERBOSE);

    ESP_LOGI(TAG, "Hello World!");

    // OLED
    ESP_LOGI(TAG, "INTERFACE is i2c");
    ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ESP_LOGI(TAG, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
    ssd1306_clear_screen(&dev, false);

    ssd1306_contrast(&dev, 0xff);

    ssd1306_display_text(&dev, 0, "STARTING", 9, false);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // LED
    configure_led();
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 5, &Task1, 1);

    // Initialize ADC
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &adc_handle));
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = adcWidth,
    };
    adc_chan_cfg.atten = adcAtten;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adcChannel, &adc_chan_cfg));

    // WIFI
    ESP_LOGI(TAG, "Connecting to AP");
    WifiDefinition wificontroller = WifiDefinition();
    wificontroller.setSSID(EXAMPLE_ESP_WIFI_SSID);
    wificontroller.setPassword(EXAMPLE_ESP_WIFI_PASS);
    bool connected = false;

    wificontroller.connect(&connected);
    if (connected)
    {
        ESP_LOGI(TAG, "Connected to AP");
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "CONNECTED", 9, false);
    }
    else
    {
        ESP_LOGI(TAG, "Failed to connect to AP");
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, "FAILED", 6, false);
    }

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    int sensorValue = 0;
    while (true)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adcChannel, &sensorValue));
        ESP_LOGI(TAG, "ADC CH: %d", sensorValue);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}