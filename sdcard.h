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


#ifdef __cplusplus
 extern "C" {
#endif

extern const char *sdcard_mount_point;

esp_err_t sdcard_init();
esp_err_t sdcard_format();
uint64_t sdcard_get_free_bytes();

uint32_t sdcard_get_record_id();
const char* sdcard_get_record_dir();
void sdcard_listdir(const char *name, int indent);
esp_err_t sdcard_print_content(char *fpath);

esp_err_t sdcard_log_start();
esp_err_t sdcard_log_stop();
int sdcard_log(const char *format, ...);

#ifdef __cplusplus
}
#endif


#endif /* MAIN_SDCARD_H_ */
