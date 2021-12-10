/*
 * sdptasks.c
 *
 *  Created on: Sep 9, 2021
 *      Author: Danylo Ulianych
 */

#include <math.h>
#include <unistd.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "bsp_log.h"
#include "onlinemean.h"
#include "sdpsensor.h"
#include "sdcard.h"
#include "record.h"
#include "board.h"

#define READ_SENSORS_PRIORITY    3
#define WRITE_TO_SDCARD_PRIORITY 2

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

SDPSensor sdp(0x25, 0);

static void sdptask_read_sensor();
static void sdptask_write();
static void sdp_timer_callback(void* arg);
static void xqueue_check_errors();
static FILE* open_fileSDP();


static void sdp_timer_callback(void* arg)
{
    xTaskNotifyGive(read_sensor_task_handle);
}


static void IRAM_ATTR sdpsensor_irq_handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(read_sensor_task_handle, &xHigherPriorityTaskWoken);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
       should be performed to ensure the interrupt returns directly to the highest
       priority task.  The macro used for this purpose is dependent on the port in
       use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    uint32_t model_number, range_pa, product_id;
    uint64_t serial;
    sdp.getInfo(&model_number, &range_pa, &product_id, &serial);
//    BSP_LOGI(TAG, "Initialized SDP%d %dPa sensor", model_number, range_pa);
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
    esp_err_t err;
    SDPRecord sdp_record;  // diff pressure
    const Board_t *board = board_get();
    const int8_t is_bmp_same_wire = board->bmp_gpio.sda == board->sdp_gpio.sda
        || board->bmp_gpio.scl == board->sdp_gpio.scl;
    int64_t bmp_last_update = -BMP280_SAMPLE_PERIOD_MS;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sdp_record.time = esp_timer_get_time();
        err = sdp.readDiffPressure(&sdp_record.diff_pressure_raw);

        if (err == ESP_OK) {
            xQueueSend(xQueueSDP, &sdp_record, portMAX_DELAY);
        } else if (err == ESP_ERR_INVALID_CRC) {
            // ignore silently
        } else {
            xQueueSend(xQueueSDPErrors, &err, 0);
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
    while (file == NULL) {
        ESP_LOGE("fopen(fileSDP) failed");
        vTaskDelay(pdMS_TO_TICKS(1));
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
    int64_t dt, dt_min = INT64_MAX, dt_max = 0;
    int16_t dp_max = 0, dp_min = 0;
    size_t wcnt;

    OnlineMean_Init(&dt_mean);

    const Board_t *board = board_get();
    FILE *fileSDP = open_fileSDP();

    while (1) {
        int8_t print_update = 0;
        OnlineMean_Reset(&dt_mean);
        do {  /* fwrite until the queue length is not enough */
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
                if (sdp_records[rid].diff_pressure_raw < dp_min) {
                    dp_min = sdp_records[rid].diff_pressure_raw;
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
                BSP_LOGE(TAG, "fwrite(fileSDP) failed. Reopening...");
                board->led_set_error();
                fclose(fileSDP);
                fileSDP = open_fileSDP();
                wcnt += fwrite(&sdp_records[wcnt], sizeof(SDPRecord), RECORDS_BUFFER_SIZE - wcnt, fileSDP);
            }
        } while (uxQueueMessagesWaiting(xQueueSDP) >= RECORDS_BUFFER_SIZE);
        fflush(fileSDP);
        fsync(fileno(fileSDP));

        if (print_update || ESP32_SDPSENSOR_DEBUG) {
            BSP_LOGI(TAG, "[q %u] read=%.0f (min=%lld, max=%lld); dp (%d, %d)",
                    messages_cnt_max,
                    OnlineMean_GetMean(&dt_mean), dt_min, dt_max,
                    dp_min, dp_max);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void record_sdp_start() {
    const Board_t *board = board_get();
    sdp.initI2C(board->sdp_gpio.sda, board->sdp_gpio.scl);
    while (sdp.stopContinuous() != ESP_OK);
    while (sdp.begin() != ESP_OK);
    while (sdp.startContinuous() != ESP_OK);

    write_sdpinfo();

    float temperature;
    if (sdp.readDiffPressureTemperature(NULL, &temperature) == ESP_OK) {
        BSP_LOGI(TAG, "SDP sensor t° %.1f", temperature);
    }

    const esp_timer_create_args_t sdp_timer_args = {
        .callback = &sdp_timer_callback,
        .name = "sdp_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&sdp_timer_args, &sdp_timer));

    xQueueSDP = xQueueCreate( 100 * RECORDS_BUFFER_SIZE, sizeof(SDPRecord) );
    xQueueSDPErrors = xQueueCreate( 100, sizeof(esp_err_t) );

    xTaskCreatePinnedToCore((TaskFunction_t) sdptask_read_sensor, "sdp_read", 2048, NULL, READ_SENSORS_PRIORITY, &read_sensor_task_handle, APP_CPU_NUM);
    xTaskCreatePinnedToCore((TaskFunction_t) sdptask_write, "sdp_write", 4096, NULL, WRITE_TO_SDCARD_PRIORITY, &write_task_handle, PRO_CPU_NUM);

    // let SDP & BMP tasks start
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (board->sdp_gpio.irq != 0) {
        sdp.attachIRQHandler(board->sdp_gpio.irq, sdpsensor_irq_handler);
        xTaskNotifyGive(read_sensor_task_handle);
    } else {
        ESP_ERROR_CHECK(esp_timer_start_periodic(sdp_timer, SDPSENSOR_SAMPLE_PERIOUD_US));
        BSP_LOGI(TAG, "Started a periodic timer");
    }

    BSP_LOGI(TAG, "SDP tasks started");
}


void record_sdp_stop() {
    vTaskDelete(read_sensor_task_handle);
    vTaskDelete(write_task_handle);
    BSP_LOGW(TAG, "SDP tasks stopped");
}
