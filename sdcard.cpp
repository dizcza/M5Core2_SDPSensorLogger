/*
 * sdcard.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Danylo Ulianych
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "sdcard.h"
#include "SD.h"

static uint32_t m_record_id = 0;
static char *m_sdcard_record_dir = NULL;
static char *m_sdcard_records_root = "/RECORDS";

static const char *TAG = "sdcard";

static int sdcard_count_dirs(char *dirpath);


void sdcard_listdir(fs::FS &fs, const char *dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                sdcard_listdir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
    root.close();
}


static int sdcard_count_dirs(fs::FS &fs, char *dirpath) {
    int dircnt = 0;
    File root = fs.open(dirpath);
    if (!root || !root.isDirectory()) return 0;

    File file = root.openNextFile();
    while (file) {
        dircnt += file.isDirectory();
        file = root.openNextFile();
    }
    root.close();
    return dircnt;
}


void sdcard_create_record_dir(fs::FS &fs) {
    char rec_folder[128];

    if (!fs.exists(m_sdcard_records_root)) {
        fs.mkdir(m_sdcard_records_root);
    }

    m_record_id = sdcard_count_dirs(fs, m_sdcard_records_root);
    if (m_record_id > 0) {
        m_record_id--;
        snprintf(rec_folder, sizeof(rec_folder), "%s/%03u/SDP.bin", m_sdcard_records_root, m_record_id);
        if (fs.exists(rec_folder)) {
            // file exists; increment ID
            m_record_id++;
        }
        if (m_record_id >= 1000) {
            // SD cards don't like long paths
            m_record_id = 0;
        }
    }

    snprintf(rec_folder, sizeof(rec_folder), "%s/%03u", m_sdcard_records_root, m_record_id);
    if (!fs.exists(rec_folder)) {
        fs.mkdir(rec_folder);
    }
    char *rec_dir_heap = (char*) malloc(strlen(rec_folder));
    strcpy(rec_dir_heap, rec_folder);
    if (m_sdcard_record_dir != NULL) {
        // clean-up from the previous call
        free(m_sdcard_record_dir);
    }
    m_sdcard_record_dir = rec_dir_heap;
    printf("Created record dir: %s\n", m_sdcard_record_dir);
}


const char* sdcard_get_record_dir() {
    return (const char*) m_sdcard_record_dir;
}


void sdcard_print_info(fs::SDFS &fs) {
    uint64_t total = fs.cardSize();
    uint64_t used = fs.usedBytes();
    uint64_t freeMb = (total - used) >> 20;
    sdcard_type_t cardType = fs.cardType();
    printf("Type: ");
    switch (cardType) {
        case CARD_MMC:
            printf("MMC\n");
            break;
        case CARD_SD:
            printf("SDSC\n");
            break;
        case CARD_SDHC:
            printf("SDHC\n");
            break;
        default:
            printf("UNKNOWN\n");
            break;
    }
    printf("Size: %lluMB\n", total >> 20);
    printf("Free: %lluMB\n", freeMb);
}


esp_err_t sdcard_print_content(fs::FS &fs, char *fpath) {
    File f = fs.open(fpath);
    if (f == NULL) {
        ESP_LOGW(TAG, "No such file: '%s'", fpath);
        return ESP_ERR_NOT_FOUND;
    }
    printf("\n>>> BEGIN '%s'\n", fpath);
    while (f.available()) {
        printf("%c", (char) f.read());
    }
    f.close();
    printf("<<< END '%s'\n\n", fpath);
    return ESP_OK;
}

