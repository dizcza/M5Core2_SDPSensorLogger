/*
 * sdcard.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Danylo Ulianych
 */

#ifndef MAIN_SDCARD_H_
#define MAIN_SDCARD_H_

#include <stdint.h>
#include "esp_err.h"
#include "FS.h"
#include "SD.h"

void sdcard_print_info(fs::SDFS &fs);
void sdcard_create_record_dir(fs::FS &fs);
const char* sdcard_get_record_dir();
void sdcard_listdir(fs::FS &fs, const char *dirname, uint8_t levels=0);
esp_err_t sdcard_print_content(fs::FS &fs, char *fpath);

esp_err_t sdcard_log_start();
esp_err_t sdcard_log_stop();
int sdcard_log(const char *format, ...);

#endif /* MAIN_SDCARD_H_ */
