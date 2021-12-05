#include <M5Core2.h>

#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>

#include <math.h>
#include <unistd.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "sdpsensor.h"
#include "sdcard.h"
#include "record.h"
#include "onlinemean.h"
#include "SHT3x.h"

#define SDP_READ_PRIORITY             3
#define SDP_WRITE2SDCARD_PRIORITY     2
#define BMP280_PRIORITY               1

#define ESP32_SDPSENSOR_DEBUG         0

#define SDPSENSOR_SAMPLE_PERIOUD_US   1000
#define BMP280_SAMPLE_PERIOD_MS       2000

/**
 * Buffer SDP records before writing to an SD card.
 * One record occupies 8 + 2 bytes.
 */
#define RECORDS_BUFFER_SIZE           52


static esp_timer_handle_t sdp_timer;

static TaskHandle_t read_sensor_task_handle;
static TaskHandle_t write_task_handle;

static File fileSDP;   // SDP sensor: diff pressure
static File fileBMP;

static QueueHandle_t xQueueSDP;
static QueueHandle_t xQueueSDPErrors;
static uint64_t records_received = 0;

static TaskHandle_t bmp_task_handle;
static bool bmp_initialized = false;

static const char *TAG = "main";

static void sdptask_read_sensor();
static void sdptask_write();
static void sdptask_led_off();
static void sdp_timer_callback(void* arg);
static void xqueue_check_errors();
static File open_fileSDP();
static File open_fileBMP();
esp_err_t write_sdpinfo(fs::FS &fs);
static void bmp280_read(File fileBMP);
static void bmp280_task(void *not_used);

SDPSensor sdp(0x25, 0);
SHT3x sht30(0x44, &Wire1);
Adafruit_BMP280 bmp(&Wire1);


static void sdp_timer_callback(void* arg)
{
  xTaskNotifyGive(read_sensor_task_handle);
}


static void sdptask_read_sensor(void *not_used) {
  esp_err_t err;
  SDPRecord sdp_record;  // diff pressure

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
    }
  }
}


static File open_fileSDP() {
  char fpath[128];
  snprintf(fpath, sizeof(fpath), "%s/SDP.bin", sdcard_get_record_dir());
  File file = SD.open(fpath, FILE_WRITE);
  if (file) {
    ESP_LOGI(TAG, "Opened file for writing diff pressure: '%s'", fpath);
  }
  return file;
}


static void xqueue_check_errors() {
  static uint64_t errors_cnt = 0;

  esp_err_t err;
  const uint64_t n_total = records_received + uxQueueMessagesWaiting(xQueueSDP);
  while (uxQueueMessagesWaiting(xQueueSDPErrors)) {
    if (xQueueReceive(xQueueSDPErrors, &err, pdMS_TO_TICKS(10)) == pdTRUE) {
      ESP_LOGW(TAG, "[%llu / %llu] SDP sensor read failed: %s",
           errors_cnt, n_total, esp_err_to_name(err));
      errors_cnt++;
    }
  }
}


static void sdptask_write(void *not_used) {
  SDPRecord sdp_records[RECORDS_BUFFER_SIZE];
  uint32_t rid;     // diff pressure record id
  UBaseType_t messages_cnt, messages_cnt_max = 0;
  OnlineMean dt_mean;
  int64_t dt, dt_max = 0;
  int16_t dp_max = 0, dp_min = 0;
  size_t written_cnt;

  OnlineMean_Init(&dt_mean);

  SDPRecord rec_dummy = {
      .time = 0,
      .diff_pressure_raw = 0
  };

  while (1) {
    messages_cnt = uxQueueMessagesWaiting(xQueueSDP);
    OnlineMean_Reset(&dt_mean);
    do {  /* fwrite until the queue length is not enough */
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
          if (dt > dt_max) dt_max = dt;
        }
      }
      written_cnt = fileSDP.write((const uint8_t*) sdp_records, sizeof(SDPRecord) * RECORDS_BUFFER_SIZE);
      if (written_cnt != sizeof(SDPRecord) * RECORDS_BUFFER_SIZE) {
        ESP_LOGE(TAG, "fwrite(recordsDP) failed");
      }
    } while (uxQueueMessagesWaiting(xQueueSDP) >= RECORDS_BUFFER_SIZE);
    fileSDP.flush();

    if (messages_cnt > messages_cnt_max) messages_cnt_max = messages_cnt;

#if ESP32_SDPSENSOR_DEBUG
    ESP_LOGI(TAG, "[q %3u, m %u] read=%.3f (max=%lld us); dp (%d, %d)",
        messages_cnt, messages_cnt_max,
        OnlineMean_GetMean(&dt_mean), dt_max, dp_min, dp_max);
#endif  /* ESP32_SDPSENSOR_DEBUG */

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void record_sdp_start() {
  sdp.initI2C(14, 13);  // PortC;
  while (sdp.stopContinuous() != ESP_OK);
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);

  write_sdpinfo(SD);
  
  const esp_timer_create_args_t sdp_timer_args = {
            .callback = &sdp_timer_callback,
            .name = "sdp_timer"
  };
  ESP_ERROR_CHECK(esp_timer_create(&sdp_timer_args, &sdp_timer));

  fileSDP = open_fileSDP();    // diff pressure
  if (!fileSDP) {
    ESP_LOGE(TAG, "Could not open a file for writing SDP data");
    esp_restart();
  }

  float temperature;
  if (sdp.readDiffPressureTemperature(NULL, &temperature) == ESP_OK) {
    ESP_LOGI(TAG, "SDP sensor t° %.1f", temperature);
  }

  xQueueSDP = xQueueCreate( 100 * RECORDS_BUFFER_SIZE, sizeof(SDPRecord) );
  xQueueSDPErrors = xQueueCreate( 100, sizeof(esp_err_t) );

  xTaskCreatePinnedToCore(sdptask_read_sensor, "sdp_read", 2048, NULL, SDP_READ_PRIORITY, &read_sensor_task_handle, APP_CPU_NUM);
  xTaskCreatePinnedToCore(sdptask_write, "sdp_write", 4096, NULL, SDP_WRITE2SDCARD_PRIORITY, &write_task_handle, PRO_CPU_NUM);

  ESP_ERROR_CHECK(esp_timer_start_periodic(sdp_timer, SDPSENSOR_SAMPLE_PERIOUD_US));

  ESP_LOGI(TAG, "SDP tasks started");
}


void record_sdp_stop() {
  vTaskDelete(read_sensor_task_handle);
  vTaskDelete(write_task_handle);
  ESP_LOGW(TAG, "SDP tasks stopped");
}


static File open_fileBMP() {
  char fpath[128];
  snprintf(fpath, sizeof(fpath), "%s/BMP.bin", sdcard_get_record_dir());
  File file = SD.open(fpath, FILE_WRITE);
  if (file) {
    ESP_LOGI(TAG, "Opened file for writing atm. pressure: '%s'", fpath);
  }
  return file;
}


/**
 * @brief BMP280 atm. pressure read-out and dump to the SD card
 */
static void bmp280_task(void *not_used)
{
  BMPRecord bmp_record;

//  File fileBMP = open_fileBMP();  // atm. pressure, temp., humidity
  ESP_LOGI(TAG, "BMP task started");

  while (1) {
    bmp280_read(fileBMP);
    vTaskDelay(pdMS_TO_TICKS(BMP280_SAMPLE_PERIOD_MS));
  }
}


esp_err_t record_bmp_start() {
  Wire1.begin(32, 33);
  if (!bmp.begin(0x76)) {
    ESP_LOGE(TAG, "Failed to init a BMP280 sensor");
    return ESP_FAIL;
  }
  bmp_initialized = true;
  ESP_LOGI(TAG, "Initialized a BMP280 sensor");
  fileBMP = open_fileBMP();
//  xTaskCreatePinnedToCore(bmp280_task, "bmp280", 4096, NULL, BMP280_PRIORITY, &bmp_task_handle, PRO_CPU_NUM);
  return ESP_OK;
}


static void bmp280_read(File fileBMP) {
  if (!bmp_initialized) return;
  
  BMPRecord bmp_record;
  bmp_record.pressure = bmp.readPressure();
  if (!sht30.readTemperatureHumidity(&bmp_record.temperature, &bmp_record.humidity)) {
    ESP_LOGE(TAG, "sht30.readTemperatureHumidity failed");
    return;
  }

  M5.lcd.setCursor(0,50);
  M5.lcd.fillRect(0,50,100,60,BLACK);
  M5.Lcd.printf("Temp: %2.1f  \r\nHumi: %2.0f%%  \r\nPressure:%2.0fPa\r\n",
                 bmp_record.temperature, bmp_record.humidity, bmp_record.pressure);

  bmp_record.time = esp_timer_get_time();
  fileBMP.write((const uint8_t*) &bmp_record, sizeof(BMPRecord));
  fileBMP.flush();
}


void record_bmp_stop() {
  if (bmp_initialized) vTaskDelete(bmp_task_handle);
  ESP_LOGI(TAG, "BMP task stopped");
}


/**
 * Writes SDP sensor info to the SD card.
 *
 * Must be called after both the SD card
 * and the sensor are initialized.
 */
esp_err_t write_sdpinfo(fs::FS &fs) {
  char fpath[128];
  snprintf(fpath, sizeof(fpath), "%s/SENSOR.txt", sdcard_get_record_dir());
  File file = fs.open(fpath, FILE_WRITE);
  if (!file) {
    return ESP_ERR_NOT_FOUND;
  }

  uint32_t model_number, range_pa, product_id;
  uint64_t serial;
  sdp.getInfo(&model_number, &range_pa, &product_id, &serial);
  uint16_t pressure_scale = sdp.getPressureScale();
  file.printf("Model number: %u\n", model_number);
  file.printf("Range Pa: %u\n", range_pa);
  file.printf("Pressure scale: %u\n", pressure_scale);
  file.printf("Product ID: 0x%08X\n", product_id);
  file.printf("Serial: 0x%016llX\n", serial);
  file.close();

  ESP_LOGI(TAG, "Wrote sensor info to '%s'", fpath);
  return ESP_OK;
}



void setup() {
  bool LCDEnable = true;
  bool SDEnable = true;
  bool SerialEnable = true;
  bool I2CEnablePortA = false;  // BMP280 sensor
  M5.begin(LCDEnable, SDEnable, SerialEnable, I2CEnablePortA);
  M5.lcd.setTextSize(2);

  sdcard_init(SD);
//  sdcard_listdir(SD, "/RECORDS/063");
//  sdcard_print_content(SD, "/RECORDS/063/SENSOR.txt");

  record_sdp_start();
  record_bmp_start();
}


void loop() {
  bmp280_read(fileBMP);
  M5.update();
  vTaskDelay(pdMS_TO_TICKS(BMP280_SAMPLE_PERIOD_MS));
}
