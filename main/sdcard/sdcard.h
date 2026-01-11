/**
 * @file sdcard.h
 * @brief SD Card driver for M5Paper
 */

#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize SD card and mount FAT filesystem
 * @return ESP_OK on success
 */
esp_err_t sdcard_init(void);

/**
 * Deinitialize SD card
 */
void sdcard_deinit(void);

/**
 * Check if SD card is mounted
 * @return true if mounted
 */
bool sdcard_is_mounted(void);

/**
 * Get total size of SD card in bytes
 * @return Size in bytes, 0 on error
 */
uint64_t sdcard_get_total_size(void);

/**
 * Get free space on SD card in bytes
 * @return Free space in bytes, 0 on error
 */
uint64_t sdcard_get_free_size(void);

#ifdef __cplusplus
}
#endif

#endif // SDCARD_H
