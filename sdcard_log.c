/*
 * sdcard_log.c
 *
 *  Created on: Sep 14, 2021
 *      Author: Danylo Ulianych
 */

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp32-hal-log.h"

#include "sdcard.h"
#include "board.h"

static xSemaphoreHandle sync_sdcard_log = NULL;
static FILE *log_file;
static int8_t log_active = 0;

static const char *TAG = "sdcard";


static FILE *open_log_file() {
    char log_path[128];
    snprintf(log_path, sizeof(log_path), "%s/LOG.LOG", sdcard_get_record_dir());
    FILE *file = fopen(log_path, "a");
    return file;
}


size_t sdcard_log(char *msg) {
    if (sync_sdcard_log == NULL) return 0;
	xSemaphoreTake(sync_sdcard_log, portMAX_DELAY);
	// A thread can enter this block after another thread
	// closes the file with sdcard_log_stop().
    size_t wcnt = 0;
	if (log_active) {
        if (log_file == NULL) {
            log_file = open_log_file();
        }
        if (log_file == NULL) {
            xSemaphoreGive(sync_sdcard_log);
            return wcnt;
        }
        const size_t len = strlen(msg);
        wcnt = fwrite(msg, sizeof(char), len, log_file);
        if (wcnt != len) {
            fclose(log_file);
            log_file = open_log_file();
            if (log_file == NULL) {
                xSemaphoreGive(sync_sdcard_log);
                return wcnt;
            }
            wcnt += fwrite(&msg[wcnt], sizeof(char), len - wcnt, log_file);
        }
		int fflush_res = fflush(log_file);
		int fsync_res = fsync(fileno(log_file));
        if (fflush_res != 0 || fsync_res != 0) {
            fclose(log_file);
            log_file = open_log_file();
            if (log_file != NULL) {
                wcnt = fwrite(msg, sizeof(char), len, log_file);
            }
        }
	}
    xSemaphoreGive(sync_sdcard_log);
    return wcnt;
}


esp_err_t sdcard_log_start() {
    if (sync_sdcard_log == NULL) {
    	sync_sdcard_log = xSemaphoreCreateBinary();
    }
    log_file = open_log_file();
    esp_err_t err;
	if (log_file == NULL) {
		ESP_LOGE(TAG, "Could not open logging file");
		err = ESP_FAIL;
	} else {
	    ESP_LOGI(TAG, "Opened logging file");
        err = ESP_OK;
    }
	log_active = 1;
	xSemaphoreGive(sync_sdcard_log);
	return err;
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
            ret = fclose(log_file);
			log_file = NULL;
			ESP_LOGI(TAG, "Closed the SD card log file.");
		}
	}
	log_active = 0;
	xSemaphoreGive(sync_sdcard_log);
	return ret == 0 ? ESP_OK : ESP_FAIL;
}

