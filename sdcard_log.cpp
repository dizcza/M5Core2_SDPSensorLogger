/*
 * sdcard_log.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Danylo Ulianych
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sdcard.h"

static xSemaphoreHandle sync_sdcard_log = NULL;
static File log_file;
static int8_t log_active = 0;

static const char *TAG = "sdcard";


size_t sdcard_log(char *msg) {
	xSemaphoreTake(sync_sdcard_log, portMAX_DELAY);
	// A thread can enter this block after another thread
	// closes the file with sdcard_log_stop().
    size_t sz = 0;
	if (log_active && log_file) {
	    sz = log_file.println(msg);
	}
    xSemaphoreGive(sync_sdcard_log);
    return sz;
}


esp_err_t sdcard_log_start(fs::FS &fs) {
    char log_path[128];
    if (sync_sdcard_log == NULL) {
    	sync_sdcard_log = xSemaphoreCreateBinary();
    }
    snprintf(log_path, sizeof(log_path), "%s/log.log", sdcard_get_record_dir());
	log_file = fs.open(log_path, FILE_WRITE);
	if (log_file) {
		ESP_LOGI(TAG, "logging to '%s'", log_path);
		log_active = 1;
		xSemaphoreGive(sync_sdcard_log);
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "Could not open file for logging: %s", log_path);
	}
	return ESP_FAIL;
}


esp_err_t sdcard_log_stop() {
	if (sync_sdcard_log == NULL) {
		// not started
		return ESP_OK;
	}
	xSemaphoreTake(sync_sdcard_log, portMAX_DELAY);
	int ret = 0;
	if (log_active) {
		if (log_file) {
            log_file.close();
			log_file = (File) NULL;
			ESP_LOGI(TAG, "Closed the SD card log file.");
		}
	}
	log_active = 0;
	xSemaphoreGive(sync_sdcard_log);
	return ret == 0 ? ESP_OK : ESP_FAIL;
}

