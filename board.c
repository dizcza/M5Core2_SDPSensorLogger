/*
 * board.c
 *
 *  Created on: Dec 8, 2021
 *      Author: Danylo Ulianych
 */

#ifdef __cplusplus
 extern "C" {
#endif


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp32-hal-log.h"

#include "board.h"

#define M5CORE2_SDP810_MAC     0x3C87C409F0B8
#define TTGO_T8_SDP32_MAC      0x509546F23A08
#define TTGO_T8_SDP806_MAC     0xC098631C5210

#define M5CORE2_GPIO_LED_PIN   25
#define TTGO_T8_GPIO_LED_PIN   21

#define LED_TASK_PRIORITY      0

static SemaphoreHandle_t xSemaphoreLed;
static led_strip_t m_led_strip;

static Board_t m_board;

static const char *TAG = "board";


/**
 * Clears errors other threads raised by switching the LED off.
 */
static void task_led_off() {
	vTaskDelay(pdMS_TO_TICKS(3000));

	while (1) {
		m_board.led_clear_error();
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}


static void ttgo_t8_led_init() {
	const int gpio_led_pin = TTGO_T8_GPIO_LED_PIN;
    gpio_config_t led_io_conf = {
    		.pin_bit_mask = 1ULL << gpio_led_pin,  // select LED pin
			.mode = GPIO_MODE_OUTPUT,                     // set as output mode
			.pull_up_en = 0,                              // disable pull-up
			.pull_down_en = 0,                            // disable pull-down
			.intr_type = GPIO_INTR_DISABLE                // disable interrupt
    };
    gpio_config(&led_io_conf);

    /**
     * Set the LED error flag ON to check whether it will be cleared
     * in the sdptask_led_off thread. If it's not cleared or makes
     * constant flashes, then either a hardware fault occurred or
     * the board is constantly aborting.
     */
	gpio_set_level(gpio_led_pin, 1);
	ESP_LOGI(TAG, "Initialized a simple LED at GPIO %d", gpio_led_pin);
}


static void ttgo_t8_led_set_error() {
	gpio_set_level(TTGO_T8_GPIO_LED_PIN, 1);
}


static void ttgo_t8_led_clear_error() {
	gpio_set_level(TTGO_T8_GPIO_LED_PIN, 0);
}


static void ledstrip_set_color(const rgb_t color) {
	if (xSemaphoreTake(xSemaphoreLed, 0) != pdTRUE) {
		return;
	}
    led_strip_fill(&m_led_strip, 0, m_led_strip.length, color);
    led_strip_flush(&m_led_strip);
	xSemaphoreGive(xSemaphoreLed);
}


static void m5core2_led_set_error() {
	static const rgb_t color_red = { .r = 0x2f, .g = 0x00, .b = 0x00 };
	ledstrip_set_color(color_red);
}


static void m5core2_led_clear_error() {
	static const rgb_t color_inactive = { .r = 0x00, .g = 0x00, .b = 0x00 };
	ledstrip_set_color(color_inactive);
}


static void m5core2_led_init() {
	size_t led_num = 10;
	gpio_num_t led_gpio = M5CORE2_GPIO_LED_PIN;
	m_led_strip.type = LED_STRIP_SK6812;
	m_led_strip.length = led_num;
	m_led_strip.gpio = led_gpio;
	m_led_strip.buf = NULL;
	m_led_strip.brightness = 255;

    led_strip_install();
	if (led_strip_init(&m_led_strip) != ESP_OK) {
		ESP_LOGW(TAG, "Failed to initialize a LED strip");
	}
	ESP_LOGI(TAG, "Initialized a LED strip with %d LEDs at GPIO %d", led_num, led_gpio);
	const rgb_t color_white = { .r = 0x0f, .g = 0x0f, .b = 0x0f };
	ledstrip_set_color(color_white);
}


void board_init() {
    uint8_t mac_addr_array[8];
    mac_addr_array[6] = mac_addr_array[7] = 0;
    esp_efuse_mac_get_default(mac_addr_array);
    uint64_t *mac_addr = (uint64_t*) (mac_addr_array);
    ESP_LOGI(TAG, "Board MAC 0x%016llX", *mac_addr);

    xSemaphoreLed = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphoreLed);

    m_board.sdp_gpio.irq = 0;

	m_board.bmp_gpio.sda = 0;
	m_board.bmp_gpio.scl = 0;

    switch (*mac_addr) {
    case M5CORE2_SDP810_MAC:
    	m_board.spi_gpio.miso = 38;
    	m_board.spi_gpio.mosi = 23;
    	m_board.spi_gpio.clk  = 18;
    	m_board.spi_gpio.cs   = 4;

    	m_board.sdp_gpio.i2c_addr = 0x25;
    	m_board.sdp_gpio.sda = 14;
    	m_board.sdp_gpio.scl = 13;

//    	m_board.bmp_gpio.sda = m_board.sdp_gpio.sda;
//    	m_board.bmp_gpio.scl = m_board.sdp_gpio.scl;

    	m_board.led_init        = &m5core2_led_init;
    	m_board.led_set_error   = &m5core2_led_set_error;
    	m_board.led_clear_error = &m5core2_led_clear_error;
    	break;
    case TTGO_T8_SDP32_MAC:
    	m_board.bmp_gpio.sda = 5;
    	m_board.bmp_gpio.scl = 18;
    case TTGO_T8_SDP806_MAC:
    	m_board.spi_gpio.miso = 2;
    	m_board.spi_gpio.mosi = 15;
    	m_board.spi_gpio.clk  = 14;
    	m_board.spi_gpio.cs   = 13;

    	m_board.sdp_gpio.i2c_addr = 0x21;
    	m_board.sdp_gpio.sda = 19;
    	m_board.sdp_gpio.scl = 23;
//    	m_board.sdp_gpio.irq = 4;

    	m_board.led_init        = &ttgo_t8_led_init;
    	m_board.led_set_error   = &ttgo_t8_led_set_error;
    	m_board.led_clear_error = &ttgo_t8_led_clear_error;
    	break;
    default:
    	ESP_LOGE(TAG, "Unknown board!");
		vTaskDelay(pdMS_TO_TICKS(2000));
		esp_restart();
    	break;
    }

    m_board.led_init();
    xTaskCreatePinnedToCore(task_led_off, "led_off", 1024, NULL, LED_TASK_PRIORITY, NULL, APP_CPU_NUM);
}


const Board_t* board_get() {
	return (const Board_t*) &m_board;
}


#ifdef __cplusplus
}
#endif
