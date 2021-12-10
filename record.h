/*
 * record.h
 *
 *  Created on: Sep 9, 2021
 *      Author: Danylo Ulianych
 */

#ifndef MAIN_RECORD_H_
#define MAIN_RECORD_H_

#include <stdint.h>
#include "esp_err.h"

#define SDPSENSOR_SAMPLE_PERIOUD_US   1000
#define BMP280_SAMPLE_PERIOD_MS       10000


// SDPxx sensor: diff pressure
typedef struct __attribute__ ((__packed__)) SDPRecord {
   int64_t time;
   int16_t diff_pressure_raw;
} SDPRecord;


// BMP/BME280 sensor: atm. pressure, temperature, humidity
typedef struct __attribute__ ((__packed__)) BMPRecord {
   int64_t time;
   float pressure;
   float temperature;
   float humidity;
} BMPRecord;


void record_sdp_start();
void record_sdp_stop();

esp_err_t record_bmp_start();
void record_bmp_stop();
void record_bmp_read();


#endif /* MAIN_RECORD_H_ */
