/*
 * sdcard.c
 *
 *  Created on: Sep 8, 2021
 *      Author: Danylo Ulianych
 */

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "vfs_fat_internal.h"

#include "sdcard.h"

#define SDCARD_SPI_GPIO_MISO 38
#define SDCARD_SPI_GPIO_MOSI 23
#define SDCARD_SPI_GPIO_CLK  18
#define SDCARD_SPI_GPIO_CS   4

#define SDCARD_ALLOCATION_UNIT_SIZE (16 * 1024)

static sdmmc_card_t *m_card = NULL;
static uint32_t m_record_id = 0;

const char *sdcard_mount_point = "/sdcard";
static char *m_sdcard_record_dir = NULL;

static const char *TAG = "sdcard";

static void sdcard_create_record_dir();
static int sdcard_count_dirs(char *dirpath);


esp_err_t sdcard_format() {
	if (m_card == NULL) {
		ESP_LOGE(TAG, "Initialize the SD card prior to formatting");
		return ESP_ERR_INVALID_STATE;
	}

	const char drv[3] = { '0', ':', 0 };
	const size_t workbuf_size = 4096;
	void *workbuf = NULL;
	esp_err_t err;
	ESP_LOGW(TAG, "Formatting the SD card...");

	workbuf = ff_memalloc(workbuf_size);
	if (workbuf == NULL) {
		return ESP_ERR_NO_MEM;
	}

	size_t alloc_unit_size = esp_vfs_fat_get_allocation_unit_size(
			m_card->csd.sector_size,
			SDCARD_ALLOCATION_UNIT_SIZE);

	FRESULT res = f_mkfs(drv, FM_ANY, alloc_unit_size, workbuf, workbuf_size);
	if (res == FR_OK) {
		err = ESP_OK;
		ESP_LOGI(TAG, "Successfully formatted the SD card");
		m_record_id = 0;
	} else {
		err = ESP_FAIL;
		ESP_LOGE(TAG, "f_mkfs failed (%d)", res);
	}

	free(workbuf);

	sdcard_create_record_dir();

	return err;
}


void sdcard_listdir(const char *name, int indent) {
	DIR *dir;
	struct dirent *entry;
	struct stat fstat;

	if (!(dir = opendir(name)))
		return;

	char path[512];
	while ((entry = readdir(dir)) != NULL) {
		snprintf(path, sizeof(path), "%s/%s", name, entry->d_name);
		if (entry->d_type == DT_DIR) {
			if (strcmp(entry->d_name, ".") == 0
					|| strcmp(entry->d_name, "..") == 0)
				continue;
			printf("%*s[%s]\n", indent, "", entry->d_name);
			sdcard_listdir(path, indent + 2);
		} else {
			stat(path, &fstat);
			printf("%*s- %s: %ld bytes\n", indent, "", entry->d_name, fstat.st_size);
		}
	}
	closedir(dir);
}


static int sdcard_count_dirs(char *dirpath) {
	int dircnt = 0;
	DIR *directory;
	struct dirent *entry;

	directory = opendir(dirpath);
	if (directory == NULL) {
		return 0;
	}

	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_type == DT_DIR) { /* a directory */
			dircnt++;
		}
	}

	closedir(directory);
	return dircnt;
}

uint32_t sdcard_get_record_id() {
	return m_record_id;
}


uint64_t sdcard_get_free_bytes() {
    FATFS *fs;
    size_t free_clusters;
    uint64_t free_bytes = 0;
    /* Get volume information and free clusters of drive 0 */
    if (f_getfree("0:", &free_clusters, &fs) == FR_OK) {
    	uint64_t free_sectors = ((uint64_t) free_clusters) * fs->csize;
    	free_bytes = free_sectors * fs->ssize;
    }
    return free_bytes;
}


static void sdcard_create_record_dir() {
    char rec_folder[128];
	struct stat dstat = { 0 };
    snprintf(rec_folder, sizeof(rec_folder), "%s/RECORDS", sdcard_mount_point);
	if (stat(rec_folder, &dstat) == -1) {
		mkdir(rec_folder, 0700);
	}
    m_record_id = sdcard_count_dirs(rec_folder);
    if (m_record_id > 0) {
    	m_record_id--;
        snprintf(rec_folder, sizeof(rec_folder), "%s/RECORDS/%03u/SDP.bin", sdcard_mount_point, m_record_id);
    	if (stat(rec_folder, &dstat) == 0) {
    		// file exists; increment ID
    		m_record_id++;
    	}
        if (m_record_id >= 1000) {
        	// SD cards don't like long paths
        	m_record_id = 0;
        }
    }

    snprintf(rec_folder, sizeof(rec_folder), "%s/RECORDS/%03u", sdcard_mount_point, m_record_id);
	if (stat(rec_folder, &dstat) == -1) {
		mkdir(rec_folder, 0700);
	}
	char *rec_dir_heap = (char*) malloc(strlen(rec_folder));
	strcpy(rec_dir_heap, rec_folder);
	if (m_sdcard_record_dir != NULL) {
		// clean-up from the previous call
		free(m_sdcard_record_dir);
	}
	m_sdcard_record_dir = rec_dir_heap;
}


const char* sdcard_get_record_dir() {
	return (const char*) m_sdcard_record_dir;
}


esp_err_t sdcard_init() {
	// Options for mounting the filesystem.
	// If format_if_mount_failed is set to true, SD card will be partitioned and
	// formatted in case when mounting fails.
	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
		.format_if_mount_failed = false,
		.max_files = 5,
		.allocation_unit_size = SDCARD_ALLOCATION_UNIT_SIZE
	};

	// Use settings defined above to initialize SD card and mount FAT filesystem.
	// Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
	// Please check its source code and implement error recovery when developing
	// production applications.

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SDCARD_SPI_GPIO_MOSI,
        .miso_io_num = SDCARD_SPI_GPIO_MISO,
        .sclk_io_num = SDCARD_SPI_GPIO_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH1));

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SDCARD_SPI_GPIO_CS;
    slot_config.host_id = host.slot;

    esp_err_t err = esp_vfs_fat_sdspi_mount(sdcard_mount_point, &host, &slot_config,
    		&mount_config, &m_card);

	if (err == ESP_OK) {
		ESP_LOGI(TAG, "SD card mounted at %s", sdcard_mount_point);
		sdmmc_card_print_info(stdout, m_card);

		/* Print free memory size info */
		printf("Free: %lluMB\n", sdcard_get_free_bytes() / (1 << 20));

		sdcard_create_record_dir();
	} else {
		ESP_LOGW(TAG, "esp_vfs_fat_sdmmc_mount failed (%s)",
				esp_err_to_name(err));
	}

	return err;
}


esp_err_t sdcard_print_content(char *fpath) {
	FILE *f = fopen(fpath, "r");
	if (f == NULL) {
		ESP_LOGW(TAG, "No such file: '%s'", fpath);
		return ESP_ERR_NOT_FOUND;
	}
	printf("\n>>> BEGIN '%s'\n", fpath);
	int c;
	while ((c = fgetc(f)) != EOF) {
		printf("%c", c);
	}
	fclose(f);
	printf("<<< END '%s'\n\n", fpath);
	return ESP_OK;
}

