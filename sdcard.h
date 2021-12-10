/*
 * sdcard.h
 *
 *  Created on: Sep 8, 2021
 *      Author: Danylo Ulianych
 */

#ifndef MAIN_SDCARD_H_
#define MAIN_SDCARD_H_

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include "esp_err.h"


extern const char *sdcard_mount_point;

int sdcard_get_record_id();
const char* sdcard_get_record_dir();
void sdcard_create_record_dir();
void sdcard_listdir(const char *name, int indent);
esp_err_t sdcard_print_content(char *fpath);
esp_err_t sdcard_print_content_prev(char *fname);
int64_t sdcard_get_record_duration(int record_id);
uint64_t sdcard_get_free_bytes();

esp_err_t sdcard_log_start();
esp_err_t sdcard_log_stop();
size_t sdcard_log(char *msg);


#ifdef __cplusplus
}
#endif


#endif /* MAIN_SDCARD_H_ */
