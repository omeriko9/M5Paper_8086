/**
 * @file hdd_image.h
 * @brief Hard disk image provisioning helpers (C: drive backing store)
 */

#ifndef HDD_IMAGE_H
#define HDD_IMAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Ensure a FAT16-partitioned hard disk image exists at `image_path`.
// If missing, creates a new image of `size_bytes` and formats it as a single FAT16 partition.
bool hdd_image_ensure_fat16(const char *image_path, uint64_t size_bytes);

#ifdef __cplusplus
}
#endif

#endif // HDD_IMAGE_H

