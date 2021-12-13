/*
 * bmp_task.c
 *
 *  Created on: Dec 3, 2021
 *      Author: Danylo Ulianych
 */


#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <M5Core2.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "SHT3x.h"

#include "esp_log.h"
#include "esp32-hal-log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp_log.h"
#include "sdcard.h"
#include "record.h"
#include "board.h"

#define BMP280_PRIORITY               1

SHT3x sht30(0x44, &Wire);
Adafruit_BMP280 bmp(&Wire);

static TaskHandle_t bmp_task_handle;
static QueueHandle_t xQueueBMP;
static int8_t bmp_initialized = 0;
static FILE *fileBMP;  // atm. pressure, temp., humidity

static const char *TAG = "bmp280";

static FILE* open_fileBMP();
static void bmp_task_same_wire();


static FILE* open_fileBMP() {
	static int trial = 0;
	char fpath[128];
    snprintf(fpath, sizeof(fpath), "%s/BMP-%03d.BIN", sdcard_get_record_dir(), trial++);
	FILE *file = fopen(fpath, "w");
	if (file == NULL) {
		BSP_LOGE(TAG, "Could not open fileBMP");
	} else {
		BSP_LOGI(TAG, "Opened fileBMP %s", fpath);
	}
	return file;
}


void record_bmp_read() {
	static BMPRecord bmp_record;
    static int64_t last_update = 0;

    if (bmp_initialized == 0) return;

    int64_t start = esp_timer_get_time();

    if (esp_timer_get_time() - last_update > BMP280_SAMPLE_PERIOD_MS) {
        if (bmp_record.pressure == 0) {
            bmp_record.pressure = bmp.readPressure();
            log_i("BMP took %lld us", esp_timer_get_time() - start);
        } else if (sht30.readTemperatureHumidity(&bmp_record.temperature, &bmp_record.humidity)) {
            xQueueSend(xQueueBMP, &bmp_record, 0);
            bmp_record.pressure = 0;
            last_update = esp_timer_get_time();
            log_i("SHT30 took %lld us", last_update - start);
        }
    }
}


static void bmp_lcd_print(const BMPRecord *bmp_record) {
  int16_t x = M5.lcd.getCursorX();
  int16_t y = M5.lcd.getCursorY();
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(LIGHTGREY);
  M5.Lcd.setTextSize(2);
  M5.lcd.fillRect(0, 0, 320, 16, BLACK);
  M5.Lcd.printf("T %3.1f H %2.0f%% P %6.0f Pa\n", bmp_record->temperature, bmp_record->humidity, bmp_record->pressure);
  M5.Lcd.setCursor(x, y);
}


static void bmp_write(const BMPRecord *bmp_record) {
	if (fileBMP == NULL) {
	    fileBMP = open_fileBMP();
	}
	if (fileBMP == NULL) {
		return;
	}
	size_t wcnt = fwrite(bmp_record, sizeof(BMPRecord), 1, fileBMP);
	int fflush_res = fflush(fileBMP);
	int fsync_res = fsync(fileno(fileBMP));
    if (wcnt != 1 || fflush_res != 0 || fsync_res != 0) {
        BSP_LOGE(TAG, "fwrite BMP failed");
        fclose(fileBMP);
        fileBMP = open_fileBMP();
        if (fileBMP != NULL) {
            wcnt = fwrite(bmp_record, sizeof(BMPRecord), 1, fileBMP);
            fflush(fileBMP);
            fsync(fileno(fileBMP));
        }
    }
}


static void bmp_task_same_wire()
{
	BMPRecord bmp_record;
	BSP_LOGI(TAG, "bmp_task_same_wire started");
	const TickType_t ticks_half = pdMS_TO_TICKS(BMP280_SAMPLE_PERIOD_MS / 2);

    while (1) {
		if (xQueueReceive(xQueueBMP, &bmp_record, ticks_half) == pdTRUE) {
			bmp_record.time = esp_timer_get_time();
            bmp_lcd_print(&bmp_record);
	    	bmp_write(&bmp_record);
		}
        vTaskDelay(ticks_half);
    }
}


esp_err_t record_bmp_start() {
	const Board_t *board = board_get();
    if (!bmp.begin(0x76)) {
        BSP_LOGE(TAG, "Failed to init a BMP280 sensor");
        return ESP_FAIL;
    }
    BSP_LOGI(TAG, "Initialized a BMP280 sensor");
    TaskFunction_t bmp_task;
    if (board->bmp_gpio.sda == board->sdp_gpio.sda || board->bmp_gpio.scl == board->sdp_gpio.scl) {
        xQueueBMP = xQueueCreate( 10, sizeof(BMPRecord) );
    	bmp_task = (TaskFunction_t) bmp_task_same_wire;
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }
    bmp_initialized = 1;

    BMPRecord bmp_record;
    bmp_record.pressure = bmp.readPressure();
    if (sht30.readTemperatureHumidity(&bmp_record.temperature, &bmp_record.humidity)) {
        bmp_lcd_print(&bmp_record);
    }

	xTaskCreatePinnedToCore(bmp_task, "bmp280", 3072, NULL, BMP280_PRIORITY, &bmp_task_handle, APP_CPU_NUM);
	return ESP_OK;
}


void record_bmp_stop() {
	if (bmp_initialized) vTaskDelete(bmp_task_handle);
	BSP_LOGI(TAG, "BMP task stopped");
}

