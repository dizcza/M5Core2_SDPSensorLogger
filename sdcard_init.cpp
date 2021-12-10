/*
 * sdcard.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Danylo Ulianych
 */

#include <stdio.h>
#include <string.h>

#include "sdcard.h"
#include "SD.h"

static void sdcard_print_info(fs::SDFS &fs);


void sdcard_init(fs::SDFS &fs) {
    sdcard_print_info(fs);
    sdcard_create_record_dir();
    sdcard_log_start();
}


static void sdcard_print_info(fs::SDFS &fs) {
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
