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
#include <time.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "esp_err.h"

#include "bsp_log.h"
#include "sdcard.h"
#include "record.h"

#define SDCARD_USE_SPI
#define SDCARD_ALLOCATION_UNIT_SIZE (16 * 1024)

static int m_record_id = 0;

const char *sdcard_mount_point = "/sd";
static char *m_sdcard_record_dir = NULL;

static const char *TAG = "sdcard";

static int sdcard_count_dirs(char *dirpath);


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


static int8_t sdcard_sdpfile_exists(uint32_t record_id) {
	char dirpath[128];
	DIR *directory;
	struct dirent *entry;
	if (record_id == -1) record_id = m_record_id - 1;
	const char *sdp_pref = "SDP-";

    snprintf(dirpath, sizeof(dirpath), "%s/RECORDS/%03d", sdcard_mount_point, record_id);

	directory = opendir(dirpath);
	if (directory == NULL) {
		return 0;
	}

	int8_t sdp_exists = 0;
	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_type == DT_REG   /* a regular file */
				&& strstr(entry->d_name, sdp_pref) == entry->d_name) {  /* a prefix match */
			sdp_exists = 1;
			break;
		}
	}
	closedir(directory);

	return sdp_exists;
}


static void sdcard_remove_record_files(int record_id) {
	char dirpath[300];
	DIR *directory;
	struct dirent *entry;
	if (record_id == -1) record_id = m_record_id - 1;

    snprintf(dirpath, sizeof(dirpath), "%s/RECORDS/%03d", sdcard_mount_point, record_id);

	directory = opendir(dirpath);
	if (directory == NULL) {
		return;
	}
	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_type == DT_REG ) {   /* a regular file */
			snprintf(dirpath, sizeof(dirpath), "%s/RECORDS/%03d/%s", sdcard_mount_point, record_id, entry->d_name);
			remove(dirpath);
		}
	}
	closedir(directory);

	BSP_LOGI(TAG, "Removed files in record %d", record_id);
}


static esp_err_t sdcard_get_sdpfile_path(char *path, int record_id) {
	char dirpath[300];
	DIR *directory;
	struct dirent *entry;
	if (record_id == -1) record_id = m_record_id - 1;
	const char *sdp_pref = "SDP-";

    snprintf(dirpath, sizeof(dirpath), "%s/RECORDS/%03d", sdcard_mount_point, record_id);

	directory = opendir(dirpath);
	if (directory == NULL) {
		return ESP_ERR_NOT_FOUND;
	}

	int sdp_cnt = 0;
	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_type == DT_REG   /* a regular file */
				&& strstr(entry->d_name, sdp_pref) == entry->d_name) {  /* a prefix match */
			// the files are sorted
			sdp_cnt++;
		}
	}
	if (sdp_cnt == 0) {
		return ESP_ERR_NOT_FOUND;
	}
	sprintf(path, "%s/%s%03d.bin", dirpath, sdp_pref, sdp_cnt - 1);
	closedir(directory);

	return ESP_OK;
}


int sdcard_get_record_id() {
	return m_record_id;
}


void sdcard_create_record_dir() {
    char rec_folder[128];
	struct stat dstat = { 0 };
    snprintf(rec_folder, sizeof(rec_folder), "%s/RECORDS", sdcard_mount_point);
    if (stat(rec_folder, &dstat) == -1) {
		mkdir(rec_folder, 0700);
	}
    m_record_id = sdcard_count_dirs(rec_folder);
    if (m_record_id > 0) {
    	m_record_id--;
        if (sdcard_sdpfile_exists(m_record_id)) {
    		// file exists; increment ID
    		m_record_id++;
    	}
    	sdcard_get_record_duration(-1);
    	// remove logs, BMP files, etc.
    	sdcard_remove_record_files(m_record_id);
        if (m_record_id >= 1000) {
        	// SD cards don't like long paths
        	m_record_id = 0;
        }
    }

    snprintf(rec_folder, sizeof(rec_folder), "%s/RECORDS/%03d", sdcard_mount_point, m_record_id);
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


esp_err_t sdcard_print_content(char *fpath) {
	FILE *f = fopen(fpath, "r");
	if (f == NULL) {
		BSP_LOGW(TAG, "No such file: '%s'", fpath);
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


int64_t sdcard_get_record_duration(int record_id) {
	char fpath[512];
	if (record_id == -1) record_id = m_record_id - 1;
	if (sdcard_get_sdpfile_path(fpath, record_id) != ESP_OK) {
		return 0;
	}
    FILE *f = fopen(fpath, "r");
    if (f == NULL) {
    	return 0;
    }
    fseek(f, 0L, SEEK_END);
    long int fsize = ftell(f);
    if (fsize % sizeof(SDPRecord) != 0) {
    	BSP_LOGW(TAG, "%s file is corrupted", fpath);
    }
    long int records_cnt = fsize / sizeof(SDPRecord);
    fseek(f, (records_cnt - 1) * sizeof(SDPRecord), SEEK_SET);
    SDPRecord record;
    fread(&record, sizeof(SDPRecord), 1, f);
    fclose(f);
    time_t seconds = (time_t) (record.time / 1000000);
    BSP_LOGI(TAG, "Record %d ended %s", record_id, ctime(&seconds));
    return record.time;
}


esp_err_t sdcard_print_content_prev(char *fname) {
	char fpath[128];
	if (m_record_id == 0) {
		return ESP_ERR_NOT_FOUND;
	}
    snprintf(fpath, sizeof(fpath), "%s/RECORDS/%03d/%s", sdcard_mount_point, m_record_id - 1, fname);
    return sdcard_print_content(fpath);
}

