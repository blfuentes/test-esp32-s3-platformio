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

#include <ultrasonic.h>

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
#define EXAMPLE_ESP_WIFI_PASS "WIFI_PASSWORD"

// IR Sensor
constexpr adc_channel_t adcChannel = ADC_CHANNEL_5;  // ADC channel for the sensor (GPIO 7)
constexpr adc_atten_t adcAtten = ADC_ATTEN_DB_12;    // Attenuation for the ADC
constexpr adc_bitwidth_t adcWidth = ADC_BITWIDTH_12; // ADC resolution

// HC SR04
#define TRIGGER_GPIO GPIO_NUM_5
#define ECHO_GPIO GPIO_NUM_4
#define MAX_DISTANCE_CM 500 // 5m max

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %0.04f cm\n", distance*100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// SHARP GP2Y0A51SK0F
constexpr adc_channel_t adcChannelSharp = ADC_CHANNEL_6;  // ADC channel for the sensor (GPIO 6)
constexpr float referenceVoltage = 5.0;                 // Reference voltage of the ADC
constexpr int adcMaxValue = 4096;                       // Maximum ADC value for a 12-bit ADC

float read_adc(adc_oneshot_unit_handle_t handle, adc_channel_t channel)
{
    int adcValue = 0;
    for (uint16_t c = 0; c < 10; c++)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(handle, adcChannelSharp, &adcValue));
        adcValue += adcValue;
    }
    return adcValue / 10.0;
}

float get_voltage(float adcValue)
{
    return (adcValue / adcMaxValue) * referenceVoltage;
}

float get_adc_value(float voltage)
{
    return (voltage / referenceVoltage) * adcMaxValue;
}

float get_distance(float voltage)
{
    // The equation of the line is y = a/x + b
    // and two (x,y) points on the graph:
    // (30mm, 1.68V) and (150mm, 0.39V)
    const float a = 48.375;
    const float b = 0.0675;
    float dist = 0;
    if (voltage > b)
    {
        dist = a / (voltage - b);
    }
    // alternative formula: https://robojax.com/learn/arduino/?vid=robojax_SHARP_0A51SK_IR
    // float dist = 33.9 + -69.5 * (voltage) + 62.3 * pow(voltage, 2) + -25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);

    return dist;
}

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

    // HC SR04
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

    // Initialize ADC Config
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = adcWidth,
    };
    adc_chan_cfg.atten = adcAtten;

    // Initialize ADC IR Sensor
    // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adcChannel, &adc_chan_cfg));

    // Initialize ADC SHARP Sensor
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adcChannelSharp, &adc_chan_cfg));

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

    // IR Sensor
    int sensorValue = 0;

    // SHARP Sensor
    float sensorValueSharp = 0.0;
    while (true)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adcChannel, &sensorValue));
        ESP_LOGI(TAG, "IR value: %d", sensorValue);
        sensorValueSharp = read_adc(adc_handle, adcChannelSharp);
        float voltage = get_voltage(sensorValueSharp);
        float distance = get_distance(voltage);
        ESP_LOGI(TAG, "SHARP value: %.2f - voltage: %.2f - distance: %.2f", sensorValueSharp, voltage, distance);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}