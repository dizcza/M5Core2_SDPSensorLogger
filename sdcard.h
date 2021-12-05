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

void sdcard_init(fs::SDFS &fs);
const char* sdcard_get_record_dir();
void sdcard_listdir(fs::FS &fs, const char *dirname, uint8_t levels=0);
esp_err_t sdcard_print_content(fs::FS &fs, char *fpath);

esp_err_t sdcard_log_start(fs::FS &fs);
size_t sdcard_log(char *msg);
esp_err_t sdcard_log_stop();

#endif /* MAIN_SDCARD_H_ */
