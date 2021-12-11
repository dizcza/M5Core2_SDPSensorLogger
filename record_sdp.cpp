/*
 * sdptasks.c
 *
 *  Created on: Sep 9, 2021
 *      Author: Danylo Ulianych
 */

#include <M5Core2.h>

#include <math.h>
#include <unistd.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "bsp_log.h"
#include "onlinemean.h"
#include "SDPSensors.h"
#include "sdcard.h"
#include "record.h"
#include "board.h"

#define READ_SENSORS_PRIORITY    3
#define WRITE_TO_SDCARD_PRIORITY 2
#define WATCHDOG_UPDATE_MS       1000L

#define ESP32_SDPSENSOR_DEBUG         0

/**
 * Buffer recordings to 512 bytes,
 * which corresponds to the sector
 * size of an SD card.
 * One record occupies 8+2 bytes.
 */
#define RECORDS_BUFFER_SIZE      52

static esp_timer_handle_t sdp_timer;

static TaskHandle_t read_sensor_task_handle;
static TaskHandle_t write_task_handle;

static QueueHandle_t xQueueSDP;
static QueueHandle_t xQueueSDPErrors;
static uint64_t records_received = 0;

static const char *TAG = "sdptask";

SDPSensor sdp(0x25);

static void sdptask_read_sensor();
static void sdptask_write();
static void sdp_timer_callback(void* arg);
static void xqueue_check_errors();
static FILE* open_fileSDP();


static void sdp_timer_callback(void* arg)
{
    xTaskNotifyGive(read_sensor_task_handle);
}


/**
 * Writes SDP sensor info to the SD card.
 *
 * Must be called after both the SD card
 * and the sensor are initialized.
 */
static esp_err_t write_sdpinfo() {
    char fpath[128];
    snprintf(fpath, sizeof(fpath), "%s/SENSOR.TXT", sdcard_get_record_dir());
    FILE *file = fopen(fpath, "w");
    if (file == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t model_number, range_pa;
    switch(sdp.getModel()) {
    case SDP31_500:
      model_number = 31;
      range_pa = 500;
      break;
    case SDP32_125:
      model_number = 32;
      range_pa = 125;
      break;
    case SDP800_500:
      model_number = 800;
      range_pa = 500;
      break;
    case SDP810_500:
      model_number = 810;
      range_pa = 500;
      break;
    case SDP801_500:
      model_number = 801;
      range_pa = 500;
      break;
    case SDP811_500:
      model_number = 811;
      range_pa = 500;
      break;
    case SDP800_125:
      model_number = 800;
      range_pa = 125;
      break;
    case SDP810_125:
      model_number = 810;
      range_pa = 125;
      break;
    default:
      model_number = 0;
      range_pa = 0;
      break;
    }
    uint32_t product_id;
    uint64_t serial;
    sdp.readProductID(&product_id, &serial);
    BSP_LOGI(TAG, "Initialized SDP%d %dPa sensor", model_number, range_pa);
    uint16_t pressure_scale = sdp.getPressureScale();
    fprintf(file, "Model number: %u\n", model_number);
    fprintf(file, "Range Pa: %u\n", range_pa);
    fprintf(file, "Pressure scale: %u\n", pressure_scale);
    fprintf(file, "Product ID: 0x%08X\n", product_id);
    fprintf(file, "Serial: 0x%016llX\n", serial);
    fclose(file);

    BSP_LOGI(TAG, "Wrote sensor info to '%s'", fpath);
    return ESP_OK;
}



static void sdptask_read_sensor() {
    bool success;
    SDPRecord sdp_record;  // diff pressure
    const Board_t *board = board_get();
    const int8_t is_bmp_same_wire = board->bmp_gpio.sda == board->sdp_gpio.sda
        || board->bmp_gpio.scl == board->sdp_gpio.scl;
    int64_t bmp_last_update = -BMP280_SAMPLE_PERIOD_MS;
    const esp_err_t err_fail = ESP_FAIL;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sdp_record.time = esp_timer_get_time();
        success = sdp.readMeasurement(&sdp_record.diff_pressure_raw, NULL, NULL);
        log_i("sdp took %lld us", esp_timer_get_time() - sdp_record.time);

        if (success) {
            xQueueSend(xQueueSDP, &sdp_record, portMAX_DELAY);
        } else {
            xQueueSend(xQueueSDPErrors, &err_fail, 0);
            // Set the LED error flag ON
            board->led_set_error();
        }

        if (is_bmp_same_wire && (sdp_record.time - bmp_last_update) > BMP280_SAMPLE_PERIOD_MS) {
            //record_bmp_read();
        }
    }
}


static FILE* open_fileSDP() {
    static int trial = 0;
    char fpath[128];
    snprintf(fpath, sizeof(fpath), "%s/SDP-%03d.BIN", sdcard_get_record_dir(), trial++);
    FILE *file = fopen(fpath, "w");
    int64_t t_last = esp_timer_get_time();
    while (file == NULL) {
        int64_t t_curr = esp_timer_get_time();
        if (t_curr - t_last > 100000) {
          SD.end();
          while (!SD.begin(TFCARD_CS_PIN, SPI, 40000000)) delay(10);
          BSP_LOGW(TAG, "SD restarted");
          t_last = t_curr;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        file = fopen(fpath, "w");
    }
    BSP_LOGI(TAG, "Opened %s", fpath);
    return file;
}


static void xqueue_check_errors() {
    static uint64_t errors_cnt = 0;

    esp_err_t err;
    const uint64_t n_total = records_received + uxQueueMessagesWaiting(xQueueSDP);
    while (uxQueueMessagesWaiting(xQueueSDPErrors)) {
        if (xQueueReceive(xQueueSDPErrors, &err, pdMS_TO_TICKS(10)) == pdTRUE) {
            errors_cnt++;
            BSP_LOGW(TAG, "[%llu / %llu] SDP sensor read failed: %s",
                    errors_cnt, n_total, esp_err_to_name(err));
        }
    }
}


static void sdptask_write() {
    SDPRecord sdp_records[RECORDS_BUFFER_SIZE];
    uint32_t rid;     // diff pressure record id
    UBaseType_t messages_cnt_max = 0;
    OnlineMean dt_mean;
    int64_t wd_update, dt, dt_min = INT64_MAX, dt_max = 0;
    int16_t dp_max = 0;
    size_t wcnt;

    OnlineMean_Init(&dt_mean);

    const Board_t *board = board_get();
    FILE *fileSDP = open_fileSDP();
	const TickType_t sleep_ticks = pdMS_TO_TICKS(10);

    while (1) {
        int8_t print_update = 0;
        OnlineMean_Reset(&dt_mean);
   		wd_update = esp_timer_get_time();
        do {  /* fwrite until the queue length is not enough */
            if (esp_timer_get_time() - wd_update > WATCHDOG_UPDATE_MS * 1000L) {
				vTaskDelay(sleep_ticks);
				wd_update = esp_timer_get_time();
			}

            if (messages_cnt_max < uxQueueMessagesWaiting(xQueueSDP)) {
                messages_cnt_max = uxQueueMessagesWaiting(xQueueSDP);
                print_update = 1;
            }
            for (rid = 0; rid < RECORDS_BUFFER_SIZE; rid++) {
                while (xQueueReceive(xQueueSDP, sdp_records + rid, pdMS_TO_TICKS(20)) != pdTRUE) {
                    xqueue_check_errors();
                }
                records_received++;
                xqueue_check_errors();

                if (sdp_records[rid].diff_pressure_raw > dp_max) {
                    dp_max = sdp_records[rid].diff_pressure_raw;
                }
                if (rid > 0) {
                    dt = sdp_records[rid].time - sdp_records[rid-1].time;
                    OnlineMean_Update(&dt_mean, dt);
                    if (dt > dt_max) {
                        dt_max = dt;
                        print_update = 1;
                    }
                    if (dt < dt_min) {
                        dt_min = dt;
                        print_update = 1;
                    }
                }
            }
            wcnt = fwrite(sdp_records, sizeof(SDPRecord), RECORDS_BUFFER_SIZE, fileSDP);
            while (wcnt != RECORDS_BUFFER_SIZE) {
                BSP_LOGE(TAG, "fwrite(fileSDP) failed. Retrying...");
                board->led_set_error();
                fclose(fileSDP);
                fileSDP = open_fileSDP();
                wcnt += fwrite(&sdp_records[wcnt], sizeof(SDPRecord), RECORDS_BUFFER_SIZE - wcnt, fileSDP);
            }
        } while (uxQueueMessagesWaiting(xQueueSDP) >= RECORDS_BUFFER_SIZE);
        fflush(fileSDP);
        fsync(fileno(fileSDP));

        if (print_update || ESP32_SDPSENSOR_DEBUG) {
            BSP_LOGI(TAG, "[q %u] [r %.0f %lld %lld] [dp %d]",
                    messages_cnt_max,
                    OnlineMean_GetMean(&dt_mean), dt_min, dt_max,
                    dp_max);
        }

        // TODO M5.Lcd print header info queue

        vTaskDelay(sleep_ticks);
    }
}


void record_sdp_start() {
    const Board_t *board = board_get();
    while (!sdp.stopContinuous());
    while (sdp.begin() == 0);
    while (!sdp.startContinuous(false));

    write_sdpinfo();

    int16_t temperature;
    if (sdp.readMeasurement(NULL, &temperature, NULL)) {
        BSP_LOGI(TAG, "SDP sensor t° %.1f", temperature / 200.0f);
    }

    const esp_timer_create_args_t sdp_timer_args = {
        .callback = &sdp_timer_callback,
        .name = "sdp_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&sdp_timer_args, &sdp_timer));

    xQueueSDP = xQueueCreate( 5000 * RECORDS_BUFFER_SIZE, sizeof(SDPRecord) );
    xQueueSDPErrors = xQueueCreate( 100, sizeof(esp_err_t) );

    xTaskCreatePinnedToCore((TaskFunction_t) sdptask_read_sensor, "sdp_read", 2048, NULL, READ_SENSORS_PRIORITY, &read_sensor_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore((TaskFunction_t) sdptask_write, "sdp_write", 8192, NULL, WRITE_TO_SDCARD_PRIORITY, &write_task_handle, PRO_CPU_NUM);

    // let SDP & BMP tasks start
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_ERROR_CHECK(esp_timer_start_periodic(sdp_timer, SDPSENSOR_SAMPLE_PERIOUD_US));
    BSP_LOGI(TAG, "Started a periodic timer");

    BSP_LOGI(TAG, "SDP tasks started");
}


void record_sdp_stop() {
    vTaskDelete(read_sensor_task_handle);
    vTaskDelete(write_task_handle);
    BSP_LOGW(TAG, "SDP tasks stopped");
}
