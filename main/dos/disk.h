/**
 * @file disk.h
 * @brief Disk emulation for DOS (reads from SD card image files)
 */

#ifndef DISK_H
#define DISK_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of drives
#define MAX_DRIVES      4

// Drive types
#define DRIVE_NONE      0
#define DRIVE_FLOPPY    1
#define DRIVE_HDD       2

// Floppy types
#define FLOPPY_360K     0   // 360KB 5.25"
#define FLOPPY_1200K    1   // 1.2MB 5.25"
#define FLOPPY_720K     2   // 720KB 3.5"
#define FLOPPY_1440K    3   // 1.44MB 3.5"
#define FLOPPY_2880K    4   // 2.88MB 3.5"

// Disk geometry
typedef struct {
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors;
    uint32_t total_sectors;
} disk_geometry_t;

// Drive structure
typedef struct {
    uint8_t type;           // Drive type
    bool inserted;          // Media present
    bool write_protected;   // Write protection
    bool media_changed;     // Media changed since last query/reset
    FILE *image_file;       // Image file handle
    char image_path[64];    // Image file path
    disk_geometry_t geometry;
    uint32_t image_size;    // Size in bytes
} drive_t;

// Initialize disk subsystem
void disk_init(void);

// Mount disk image
bool disk_mount(uint8_t drive_num, const char *image_path, uint8_t type);

// Unmount disk
void disk_unmount(uint8_t drive_num);

// Check if drive is ready
bool disk_is_ready(uint8_t drive_num);

// Get drive geometry
bool disk_get_geometry(uint8_t drive_num, disk_geometry_t *geom);

// Read sectors (CHS)
bool disk_read_chs(uint8_t drive_num, uint16_t cyl, uint8_t head, 
                   uint8_t sector, uint8_t count, uint8_t *buffer);

// Write sectors (CHS)
bool disk_write_chs(uint8_t drive_num, uint16_t cyl, uint8_t head,
                    uint8_t sector, uint8_t count, const uint8_t *buffer);

// Read sectors (LBA)
bool disk_read_lba(uint8_t drive_num, uint32_t lba, uint8_t count, uint8_t *buffer);

// Write sectors (LBA)
bool disk_write_lba(uint8_t drive_num, uint32_t lba, uint8_t count, const uint8_t *buffer);

// Convert CHS to LBA
uint32_t disk_chs_to_lba(uint8_t drive_num, uint16_t cyl, uint8_t head, uint8_t sector);

// Convert LBA to CHS
void disk_lba_to_chs(uint8_t drive_num, uint32_t lba, uint16_t *cyl, uint8_t *head, uint8_t *sector);

// Get last error
uint8_t disk_get_error(void);

// Sync/flush disk
void disk_sync(uint8_t drive_num);

// Media change tracking (for DOS floppy cache invalidation).
void disk_mark_media_changed(uint8_t drive_num);
bool disk_take_media_changed(uint8_t drive_num);
bool disk_mark_media_changed_for_path(const char *image_path);

// Replace media for a drive at runtime (diskette swap).
// For staging swaps (e.g. upload), prefer `disk_replace_image_from_staging` to avoid races.
bool disk_replace_image(uint8_t drive_num, const char *image_path, uint8_t type);
bool disk_replace_image_from_staging(uint8_t drive_num, const char *staging_path, const char *final_path, uint8_t type);

#ifdef __cplusplus
}
#endif

#endif // DISK_H
