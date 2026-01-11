/**
 * @file disk.c
 * @brief Disk emulation for DOS
 * 
 * Reads from SD card image files to emulate floppy/hard drives.
 */

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "disk.h"
#include "settings.h"

static const char *TAG = "DISK";

static uint16_t read_le16(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static uint32_t read_le32(const uint8_t *buf)
{
    return (uint32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
}

// Drive array
static drive_t drives[MAX_DRIVES];

// Last error code
static uint8_t last_error = 0;

static SemaphoreHandle_t s_disk_lock;

static void disk_lock(void)
{
    if (s_disk_lock) {
        xSemaphoreTake(s_disk_lock, portMAX_DELAY);
    }
}

static void disk_unlock(void)
{
    if (s_disk_lock) {
        xSemaphoreGive(s_disk_lock);
    }
}

// Floppy geometry lookup
static const disk_geometry_t floppy_geom[] = {
    {40, 2, 9, 720},        // 360KB
    {80, 2, 15, 2400},      // 1.2MB
    {80, 2, 9, 1440},       // 720KB
    {80, 2, 18, 2880},      // 1.44MB
    {80, 2, 36, 5760},      // 2.88MB
};

static void detect_geometry(drive_t *drive);

static bool detect_floppy_geometry_bpb(drive_t *drive)
{
    uint8_t boot_sector[512];

    if (fseek(drive->image_file, 0, SEEK_SET) != 0) {
        ESP_LOGW(TAG, "BPB: seek failed");
        return false;
    }

    size_t bytes_read = fread(boot_sector, 1, sizeof(boot_sector), drive->image_file);
    fseek(drive->image_file, 0, SEEK_SET);
    if (bytes_read != sizeof(boot_sector)) {
        ESP_LOGW(TAG, "BPB: short read (%u bytes)", (unsigned)bytes_read);
        return false;
    }

    uint16_t bytes_per_sector = read_le16(boot_sector + 11);
    uint16_t sectors_per_track = read_le16(boot_sector + 24);
    uint16_t heads = read_le16(boot_sector + 26);
    uint16_t total_sectors16 = read_le16(boot_sector + 19);
    uint32_t total_sectors32 = read_le32(boot_sector + 32);
    uint32_t total_sectors = total_sectors16 ? total_sectors16 : total_sectors32;

    if (bytes_per_sector != 512 || sectors_per_track == 0 || heads == 0) {
        return false;
    }

    if (sectors_per_track > 255 || heads > 255) {
        return false;
    }

    uint32_t max_sectors = drive->image_size / 512;
    if (total_sectors == 0 || max_sectors == 0) {
        return false;
    }
    if (total_sectors > max_sectors) {
        ESP_LOGW(TAG, "BPB: total sectors %lu > image sectors %lu, clamping",
                 total_sectors, max_sectors);
        total_sectors = max_sectors;
    }

    drive->geometry.sectors = (uint8_t)sectors_per_track;
    drive->geometry.heads = (uint8_t)heads;
    drive->geometry.total_sectors = total_sectors;
    uint32_t denom = (uint32_t)drive->geometry.heads * drive->geometry.sectors;
    drive->geometry.cylinders = (uint16_t)((total_sectors + denom - 1) / denom);
    if (drive->geometry.cylinders == 0) {
        drive->geometry.cylinders = 1;
    }

    ESP_LOGI(TAG, "Geometry(BPB): C=%d H=%d S=%d (total=%lu)",
             drive->geometry.cylinders, drive->geometry.heads,
             drive->geometry.sectors, drive->geometry.total_sectors);
    return true;
}

static bool disk_mount_idx(uint8_t idx, const char *image_path, uint8_t type)
{
    drive_t *drive = &drives[idx];

    // Close any existing image
    if (drive->image_file) {
        fclose(drive->image_file);
        drive->image_file = NULL;
    }
    drive->write_protected = false;

    // Open new image
    ESP_LOGI(TAG, "Mounting %s as drive %d", image_path, idx);

    drive->image_file = fopen(image_path, "r+b");
    if (!drive->image_file) {
        // Try read-only
        drive->image_file = fopen(image_path, "rb");
        if (!drive->image_file) {
            ESP_LOGE(TAG, "Failed to open image: %s", image_path);
            return false;
        }
        drive->write_protected = true;
    }

    // Get file size
    fseek(drive->image_file, 0, SEEK_END);
    long raw_size = ftell(drive->image_file);
    fseek(drive->image_file, 0, SEEK_SET);

    if (raw_size < 512) {
        ESP_LOGE(TAG, "Image size too small: %ld bytes", raw_size);
        fclose(drive->image_file);
        drive->image_file = NULL;
        return false;
    }

    drive->image_size = (uint32_t)raw_size;
    if ((drive->image_size % 512) != 0) {
        uint32_t trimmed = drive->image_size - (drive->image_size % 512);
        ESP_LOGW(TAG, "Image size %lu not sector-aligned, truncating to %lu",
                 drive->image_size, trimmed);
        drive->image_size = trimmed;
    }

    ESP_LOGI(TAG, "Image size: %lu bytes", drive->image_size);

    // Validate floppy size
    bool nonstandard_floppy = false;
    if (type == DRIVE_FLOPPY) {
        bool valid_floppy = false;
        if (drive->image_size == 368640 ||  // 360K
            drive->image_size == 737280 ||  // 720K
            drive->image_size == 1228800 || // 1.2M
            drive->image_size == 1474560 || // 1.44M
            drive->image_size == 2949120)   // 2.88M
        {
            valid_floppy = true;
        }

        if (!valid_floppy) {
            ESP_LOGW(TAG, "Image size %lu not standard floppy size", drive->image_size);
            nonstandard_floppy = true;
        }
    }

    // Set drive type and geometry
    drive->type = type;
    drive->inserted = true;
    drive->media_changed = false;
    strncpy(drive->image_path, image_path, sizeof(drive->image_path) - 1);
    drive->image_path[sizeof(drive->image_path) - 1] = '\0';

    if (type == DRIVE_FLOPPY && nonstandard_floppy) {
        if (!detect_floppy_geometry_bpb(drive)) {
            detect_geometry(drive);
        }
    } else {
        detect_geometry(drive);
    }

    return true;
}

/**
 * Initialize disk subsystem
 */
void disk_init(void)
{
    ESP_LOGI(TAG, "Initializing disk subsystem");
    if (!s_disk_lock) {
        s_disk_lock = xSemaphoreCreateMutex();
    }
    disk_lock();
    memset(drives, 0, sizeof(drives));
    last_error = 0;
    disk_unlock();
}

/**
 * Detect disk geometry from image size
 */
static void detect_geometry(drive_t *drive)
{
    uint32_t sectors = drive->image_size / 512;
    
    if (drive->type == DRIVE_FLOPPY) {
        // Try to match known floppy sizes
        if (sectors <= 720) {
            drive->geometry = floppy_geom[FLOPPY_360K];
        } else if (sectors <= 1440) {
            drive->geometry = floppy_geom[FLOPPY_720K];
        } else if (sectors <= 2400) {
            drive->geometry = floppy_geom[FLOPPY_1200K];
        } else if (sectors <= 2880) {
            drive->geometry = floppy_geom[FLOPPY_1440K];
        } else {
            drive->geometry = floppy_geom[FLOPPY_2880K];
        }
    } else {
        // Hard disk
        // Determine geometry strategy based on settings
        app_settings_t cfg;
        bool use_legacy = false;
        if (app_settings_get(&cfg) == ESP_OK) {
            use_legacy = cfg.c_drive_dos33_compat;
        }

        if (use_legacy) {
            // "Legacy" DOS 3.3 mode (CHS)
            const uint16_t spt = 17;
            static const uint16_t head_candidates[] = {4, 8, 16};
            uint16_t heads = 4;
            uint16_t cylinders = 1;

            for (size_t i = 0; i < sizeof(head_candidates) / sizeof(head_candidates[0]); i++) {
                heads = head_candidates[i];
                uint32_t denom = (uint32_t)heads * spt;
                if (denom == 0) continue;
                cylinders = (uint16_t)((sectors + denom - 1) / denom);
                if (cylinders == 0) cylinders = 1;
                if (cylinders <= 1024) {
                    break;
                }
            }
            if (cylinders > 1024) cylinders = 1024;

            drive->geometry.heads = (uint8_t)heads;
            drive->geometry.sectors = (uint8_t)spt;
            drive->geometry.cylinders = cylinders;
            drive->geometry.total_sectors = sectors;
            ESP_LOGI(TAG, "Geometry (Legacy): C=%d H=%d S=%d", cylinders, heads, spt);
        } else {
            // "Modern" LBA-translated geometry
            static const uint16_t head_candidates[] = {16, 32, 64, 128, 255};
            const uint16_t spt = 63;
            uint16_t heads = head_candidates[0];
            uint16_t cylinders = 1;

            for (size_t i = 0; i < sizeof(head_candidates) / sizeof(head_candidates[0]); i++) {
                heads = head_candidates[i];
                uint32_t denom = (uint32_t)heads * spt;
                if (denom == 0) continue;
                cylinders = (uint16_t)((sectors + denom - 1) / denom);
                if (cylinders == 0) cylinders = 1;
                if (cylinders <= 1024) {
                    break;
                }
            }

            if (cylinders > 1024) {
                cylinders = 1024;
                heads = 255;
            }

            drive->geometry.heads = (uint8_t)heads;
            drive->geometry.sectors = (uint8_t)spt;
            drive->geometry.cylinders = cylinders;
            drive->geometry.total_sectors = sectors;
            ESP_LOGI(TAG, "Geometry (Modern): C=%d H=%d S=%d", cylinders, heads, spt);
        }
    }
    
    ESP_LOGI(TAG, "Geometry: C=%d H=%d S=%d (total=%lu)", 
             drive->geometry.cylinders, drive->geometry.heads,
             drive->geometry.sectors, drive->geometry.total_sectors);
}

/**
 * Mount disk image
 */
bool disk_mount(uint8_t drive_num, const char *image_path, uint8_t type)
{
    disk_lock();
    if (drive_num < 0x80) {
        // Floppy drives 0x00-0x01
        if (drive_num >= 2) {
            ESP_LOGE(TAG, "Invalid floppy drive number: %d", drive_num);
            disk_unlock();
            return false;
        }
    } else {
        // Hard drives 0x80-0x83 -> indices 2-3
        drive_num = (drive_num - 0x80) + 2;
        if (drive_num >= MAX_DRIVES) {
            ESP_LOGE(TAG, "Invalid hard drive number");
            disk_unlock();
            return false;
        }
    }

    bool ok = disk_mount_idx(drive_num, image_path, type);
    disk_unlock();
    return ok;
}

/**
 * Unmount disk
 */
void disk_unmount(uint8_t drive_num)
{
    disk_lock();
    if (drive_num >= 0x80) {
        drive_num = (drive_num - 0x80) + 2;
    }
    
    if (drive_num >= MAX_DRIVES) {
        disk_unlock();
        return;
    }
    
    drive_t *drive = &drives[drive_num];
    
    if (drive->image_file) {
        fclose(drive->image_file);
        drive->image_file = NULL;
    }
    
    drive->inserted = false;
    drive->type = DRIVE_NONE;
    drive->media_changed = false;
    disk_unlock();
}

/**
 * Check if drive is ready
 */
bool disk_is_ready(uint8_t drive_num)
{
    disk_lock();
    if (drive_num >= 0x80) {
        drive_num = (drive_num - 0x80) + 2;
    }
    
    if (drive_num >= MAX_DRIVES) {
        disk_unlock();
        return false;
    }

    bool ready = drives[drive_num].inserted && drives[drive_num].image_file != NULL;
    disk_unlock();
    return ready;
}

/**
 * Get drive geometry
 */
bool disk_get_geometry(uint8_t drive_num, disk_geometry_t *geom)
{
    disk_lock();
    if (drive_num >= 0x80) {
        drive_num = (drive_num - 0x80) + 2;
    }
    
    if (drive_num >= MAX_DRIVES) {
        disk_unlock();
        return false;
    }
    if (!drives[drive_num].inserted) {
        disk_unlock();
        return false;
    }

    *geom = drives[drive_num].geometry;
    disk_unlock();
    return true;
}

/**
 * Convert CHS to LBA
 */
uint32_t disk_chs_to_lba(uint8_t drive_num, uint16_t cyl, uint8_t head, uint8_t sector)
{
    if (drive_num >= 0x80) {
        drive_num = (drive_num - 0x80) + 2;
    }
    
    if (drive_num >= MAX_DRIVES) return 0;
    
    drive_t *drive = &drives[drive_num];
    
    // LBA = (C * H + h) * S + (s - 1)
    uint32_t lba = ((uint32_t)cyl * drive->geometry.heads + head) * 
                   drive->geometry.sectors + (sector - 1);
    
    return lba;
}

/**
 * Convert LBA to CHS
 */
void disk_lba_to_chs(uint8_t drive_num, uint32_t lba, uint16_t *cyl, uint8_t *head, uint8_t *sector)
{
    if (drive_num >= 0x80) {
        drive_num = (drive_num - 0x80) + 2;
    }
    
    if (drive_num >= MAX_DRIVES) return;
    
    drive_t *drive = &drives[drive_num];
    
    *sector = (lba % drive->geometry.sectors) + 1;
    uint32_t tmp = lba / drive->geometry.sectors;
    *head = tmp % drive->geometry.heads;
    *cyl = tmp / drive->geometry.heads;
}

/**
 * Read sectors (CHS)
 */
bool disk_read_chs(uint8_t drive_num, uint16_t cyl, uint8_t head, 
                   uint8_t sector, uint8_t count, uint8_t *buffer)
{
    uint32_t lba = disk_chs_to_lba(drive_num, cyl, head, sector);
    return disk_read_lba(drive_num, lba, count, buffer);
}

/**
 * Write sectors (CHS)
 */
bool disk_write_chs(uint8_t drive_num, uint16_t cyl, uint8_t head,
                    uint8_t sector, uint8_t count, const uint8_t *buffer)
{
    uint32_t lba = disk_chs_to_lba(drive_num, cyl, head, sector);
    return disk_write_lba(drive_num, lba, count, buffer);
}

/**
 * Read sectors (LBA)
 */
bool disk_read_lba(uint8_t drive_num, uint32_t lba, uint8_t count, uint8_t *buffer)
{
    disk_lock();
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    
    if (idx >= MAX_DRIVES) {
        last_error = 0x01;  // Invalid command
        disk_unlock();
        return false;
    }
    
    drive_t *drive = &drives[idx];
    
    if (!drive->inserted || !drive->image_file) {
        last_error = 0x80;  // Timeout / not ready
        disk_unlock();
        return false;
    }

    uint32_t max_sectors = drive->image_size / 512;
    if ((uint64_t)lba + count > max_sectors) {
        ESP_LOGE(TAG, "Read beyond image: drive=%d LBA=%lu count=%u max=%lu",
                 idx, lba, count, max_sectors);
        last_error = 0x04; // Sector not found
        disk_unlock();
        return false;
    }
    
    // Seek to position
    uint32_t offset = lba * 512;
    if (fseek(drive->image_file, offset, SEEK_SET) != 0) {
        last_error = 0x40;  // Seek failed
        ESP_LOGE(TAG, "Seek failed: drive=%d LBA=%lu offset=%lu", idx, lba, offset);
        disk_unlock();
        return false;
    }
    
    // Read data
    size_t bytes_read = fread(buffer, 1, count * 512, drive->image_file);
    if (bytes_read != count * 512) {
        ESP_LOGW(TAG, "Read %d bytes, expected %d (drive=%d LBA=%lu count=%u)",
                 bytes_read, count * 512, idx, lba, count);
        last_error = 0x04;
        disk_unlock();
        return false;
    }
    
    last_error = 0x00;
    disk_unlock();
    return true;
}

/**
 * Write sectors (LBA)
 */
bool disk_write_lba(uint8_t drive_num, uint32_t lba, uint8_t count, const uint8_t *buffer)
{
    disk_lock();
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    
    if (idx >= MAX_DRIVES) {
        last_error = 0x01;
        disk_unlock();
        return false;
    }
    
    drive_t *drive = &drives[idx];
    
    if (!drive->inserted || !drive->image_file) {
        last_error = 0x80;
        disk_unlock();
        return false;
    }
    
    if (drive->write_protected) {
        last_error = 0x03;  // Write protected
        disk_unlock();
        return false;
    }

    uint32_t max_sectors = drive->image_size / 512;
    if ((uint64_t)lba + count > max_sectors) {
        ESP_LOGE(TAG, "Write beyond image: drive=%d LBA=%lu count=%u max=%lu",
                 idx, lba, count, max_sectors);
    }
    
    // Seek to position
    uint32_t offset = lba * 512;
    if (fseek(drive->image_file, offset, SEEK_SET) != 0) {
        last_error = 0x04;
        ESP_LOGE(TAG, "Seek failed: drive=%d LBA=%lu offset=%lu", idx, lba, offset);
        disk_unlock();
        return false;
    }
    
    // Write data
    size_t bytes_written = fwrite(buffer, 1, count * 512, drive->image_file);
    if (bytes_written != count * 512) {
        ESP_LOGE(TAG, "Write %d bytes, expected %d (drive=%d LBA=%lu count=%u)",
                 bytes_written, count * 512, idx, lba, count);
        last_error = 0x04;
        disk_unlock();
        return false;
    }
    
    fflush(drive->image_file);
    
    last_error = 0x00;
    disk_unlock();
    return true;
}

/**
 * Get last error
 */
uint8_t disk_get_error(void)
{
    return last_error;
}

/**
 * Sync disk
 */
void disk_sync(uint8_t drive_num)
{
    disk_lock();
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    
    if (idx >= MAX_DRIVES) {
        disk_unlock();
        return;
    }
    
    if (drives[idx].image_file) {
        fflush(drives[idx].image_file);
    }
    disk_unlock();
}

void disk_mark_media_changed(uint8_t drive_num)
{
    disk_lock();
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    if (idx < MAX_DRIVES) {
        drives[idx].media_changed = true;
    }
    disk_unlock();
}

bool disk_take_media_changed(uint8_t drive_num)
{
    bool changed = false;
    disk_lock();
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    if (idx < MAX_DRIVES) {
        changed = drives[idx].media_changed;
        drives[idx].media_changed = false;
    }
    disk_unlock();
    return changed;
}

bool disk_mark_media_changed_for_path(const char *image_path)
{
    if (!image_path || !image_path[0]) {
        return false;
    }

    bool marked = false;
    disk_lock();
    for (uint8_t idx = 0; idx < MAX_DRIVES; idx++) {
        drive_t *drive = &drives[idx];
        if (!drive->inserted || drive->image_path[0] == '\0') {
            continue;
        }
        if (strcmp(drive->image_path, image_path) == 0) {
            drive->media_changed = true;
            marked = true;
        }
    }
    disk_unlock();
    return marked;
}

bool disk_replace_image(uint8_t drive_num, const char *image_path, uint8_t type)
{
    // disk_mount already handles closing/reopening; keep this wrapper for API clarity.
    bool ok = disk_mount(drive_num, image_path, type);
    if (ok) {
        disk_mark_media_changed(drive_num);
    }
    return ok;
}

bool disk_replace_image_from_staging(uint8_t drive_num, const char *staging_path, const char *final_path, uint8_t type)
{
    uint8_t idx = drive_num;
    if (drive_num >= 0x80) {
        idx = (drive_num - 0x80) + 2;
    }
    if (idx >= MAX_DRIVES) {
        return false;
    }

    disk_lock();

    drive_t *drive = &drives[idx];
    char old_path[sizeof(drive->image_path)];
    old_path[0] = '\0';
    strncpy(old_path, drive->image_path, sizeof(old_path) - 1);

    if (drive->image_file) {
        fclose(drive->image_file);
        drive->image_file = NULL;
    }
    drive->inserted = false;

    // Replace the on-disk image file atomically (best-effort on FATFS).
    remove(final_path);
    if (rename(staging_path, final_path) != 0) {
        ESP_LOGE(TAG, "Failed to rename staging image: %s (%d)", strerror(errno), errno);
        // Best-effort restore previous media.
        if (old_path[0] != '\0') {
            (void)disk_mount_idx(idx, old_path, drive->type ? drive->type : type);
        }
        disk_unlock();
        return false;
    }

    // Mount the new image (reuse internal state for this drive index).
    bool ok = disk_mount_idx(idx, final_path, type);
    if (ok) {
        drive->media_changed = true;
    }
    disk_unlock();
    return ok;
}
