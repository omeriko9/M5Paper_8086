/**
 * @file hdd_image.c
 * @brief Hard disk image provisioning helpers
 *
 * Creates a basic MBR + single FAT16 partition so DOS can mount it as C:.
 */

#include "hdd_image.h"

#include "config_defs.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_log.h"

static const char *TAG = "HDDIMG";

#define SECTOR_SIZE 512u
#define DEFAULT_HDD_START_LBA 63u
#define DOS33_MAX_PART_SECTORS 0xFFFFu

// ESP-IDF main task stack can be tight; keep sector buffers off the stack.
static uint8_t s_sector_buf[SECTOR_SIZE];
static bool s_in_use = false;

static void write_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void write_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static bool is_regular_file(const char *path, uint64_t *size_out)
{
    struct stat st;
    if (stat(path, &st) != 0) {
        return false;
    }
    if ((st.st_mode & S_IFREG) == 0) {
        return false;
    }
    if (size_out) {
        if (st.st_size < 0) {
            *size_out = 0;
        } else {
            *size_out = (uint64_t)st.st_size;
        }
    }
    return true;
}

static bool ensure_dir(const char *dir_path)
{
    if (mkdir(dir_path, 0775) == 0) {
        return true;
    }
    if (errno == EEXIST) {
        struct stat st;
        if (stat(dir_path, &st) == 0 && (st.st_mode & S_IFDIR) != 0) {
            return true;
        }
        ESP_LOGE(TAG, "Path exists but is not a directory: %s", dir_path);
        return false;
    }
    ESP_LOGE(TAG, "mkdir failed: path=%s errno=%d (%s)", dir_path, errno, strerror(errno));
    return false;
}

static bool write_sector(FILE *f, uint32_t lba, const uint8_t sector[SECTOR_SIZE])
{
    const uint32_t offset = lba * SECTOR_SIZE;
    if (fseek(f, (long)offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "seek failed: lba=%lu offset=%lu errno=%d (%s)",
                 (unsigned long)lba, (unsigned long)offset, errno, strerror(errno));
        return false;
    }
    size_t written = fwrite(sector, 1, SECTOR_SIZE, f);
    if (written != SECTOR_SIZE) {
        ESP_LOGE(TAG, "short write: lba=%lu wrote=%u errno=%d (%s)",
                 (unsigned long)lba, (unsigned)written, errno, strerror(errno));
        return false;
    }
    return true;
}

static bool zero_sectors(FILE *f, uint32_t lba, uint32_t count)
{
    memset(s_sector_buf, 0, sizeof(s_sector_buf));
    for (uint32_t i = 0; i < count; i++) {
        if (!write_sector(f, lba + i, s_sector_buf)) {
            return false;
        }
    }
    return true;
}

static bool read_mbr_partition0(FILE *f, uint32_t *out_lba, uint32_t *out_sectors, uint8_t *out_type)
{
    if (!f || !out_lba || !out_sectors) {
        return false;
    }

    if (fseek(f, 0, SEEK_SET) != 0) {
        return false;
    }
    size_t read = fread(s_sector_buf, 1, sizeof(s_sector_buf), f);
    if (read != sizeof(s_sector_buf)) {
        return false;
    }
    if (s_sector_buf[510] != 0x55 || s_sector_buf[511] != 0xAA) {
        return false;
    }

    for (int i = 0; i < 4; i++) {
        const uint8_t *pte = &s_sector_buf[446 + (i * 16)];
        const uint8_t type = pte[4];
        const uint32_t lba = read_le32(pte + 8);
        const uint32_t sectors = read_le32(pte + 12);
        if (type != 0x00 && lba != 0 && sectors != 0) {
            *out_lba = lba;
            *out_sectors = sectors;
            if (out_type) {
                *out_type = type;
            }
            return true;
        }
    }

    return false;
}

static bool backup_path_for_image(const char *image_path, char *out, size_t out_len)
{
    if (!image_path || !out || out_len == 0) {
        return false;
    }
    // Try a few suffixes to avoid clobbering an old backup.
    for (int i = 0; i < 10; i++) {
        if (i == 0) {
            if (snprintf(out, out_len, "%s.large", image_path) < 0) {
                return false;
            }
        } else {
            if (snprintf(out, out_len, "%s.large.%d", image_path, i) < 0) {
                return false;
            }
        }
        struct stat st;
        if (stat(out, &st) != 0) {
            return true;
        }
    }
    return false;
}

static void choose_hdd_chs(uint32_t total_sectors, uint16_t *out_cyl, uint16_t *out_heads, uint16_t *out_spt)
{
    static const uint16_t head_candidates[] = {16, 32, 64, 128, 255};
    const uint16_t spt = 63;

    uint16_t heads = head_candidates[0];
    uint16_t cylinders = 1;
    for (size_t i = 0; i < sizeof(head_candidates) / sizeof(head_candidates[0]); i++) {
        heads = head_candidates[i];
        uint32_t denom = (uint32_t)heads * spt;
        if (denom == 0) continue;
        cylinders = (uint16_t)((total_sectors + denom - 1) / denom);
        if (cylinders == 0) cylinders = 1;
        if (cylinders <= 1024) {
            break;
        }
    }
    if (cylinders > 1024) {
        cylinders = 1024;
        heads = 255;
    }

    *out_cyl = cylinders;
    *out_heads = heads;
    *out_spt = spt;
}

static void encode_chs(uint32_t lba, uint16_t heads, uint16_t spt, uint8_t out_chs[3])
{
    if (!out_chs || heads == 0 || spt == 0) {
        return;
    }
    uint32_t sector = (lba % spt) + 1u;
    uint32_t tmp = lba / spt;
    uint32_t head = tmp % heads;
    uint32_t cyl = tmp / heads;
    if (cyl > 1023u) {
        cyl = 1023u;
        head = (heads > 0) ? (heads - 1u) : 0u;
        sector = spt;
    }
    out_chs[0] = (uint8_t)head;
    out_chs[1] = (uint8_t)((sector & 0x3Fu) | ((cyl >> 2) & 0xC0u));
    out_chs[2] = (uint8_t)(cyl & 0xFFu);
}

static uint8_t choose_fat16_spc(uint32_t total_sectors)
{
    // Conservative FAT16 cluster size selection (sectors/cluster) to keep clusters <= 65524.
    // Use typical DOS/Windows defaults; keep <= 32KB clusters (64 sectors).
    const uint32_t mib = 1024u * 1024u;
    uint64_t bytes = (uint64_t)total_sectors * SECTOR_SIZE;

    if (bytes <= 64u * mib) return 4;        // 2KB
    if (bytes <= 128u * mib) return 8;       // 4KB
    if (bytes <= 256u * mib) return 16;      // 8KB
    if (bytes <= 512u * mib) return 32;      // 16KB
    return 64;                               // 32KB (up to ~2GB FAT16)
}

static bool fat16_compute(uint32_t vol_sectors,
                          uint8_t spc,
                          uint16_t reserved_sectors,
                          uint8_t fats,
                          uint16_t root_entries,
                          uint16_t *out_spf,
                          uint32_t *out_clusters)
{
    const uint32_t root_dir_sectors = ((uint32_t)root_entries * 32u + (SECTOR_SIZE - 1u)) / SECTOR_SIZE;
    uint16_t spf = 1;

    for (int iter = 0; iter < 32; iter++) {
        uint32_t data_sectors = vol_sectors - reserved_sectors - root_dir_sectors - (uint32_t)fats * spf;
        uint32_t clusters = data_sectors / spc;
        uint32_t fat_entries = clusters + 2u;
        uint32_t fat_bytes = fat_entries * 2u;
        uint16_t needed_spf = (uint16_t)((fat_bytes + (SECTOR_SIZE - 1u)) / SECTOR_SIZE);
        if (needed_spf == 0) needed_spf = 1;

        if (needed_spf == spf) {
            if (clusters < 4085u || clusters > 65524u) {
                return false;
            }
            *out_spf = spf;
            *out_clusters = clusters;
            return true;
        }
        spf = needed_spf;
    }
    return false;
}

static bool format_mbr_fat16(FILE *f, uint32_t disk_sectors)
{
    if (disk_sectors < (DEFAULT_HDD_START_LBA + 4096u)) {
        ESP_LOGE(TAG, "Disk too small for FAT16: sectors=%lu", (unsigned long)disk_sectors);
        return false;
    }

    const uint32_t part_lba = DEFAULT_HDD_START_LBA;
    uint32_t part_sectors = disk_sectors - part_lba;

    if (g_c_drive_dos33_compat) {
        if (part_sectors > DOS33_MAX_PART_SECTORS) {
            ESP_LOGW(TAG, "DOS 3.30 compatibility: limiting partition to 32MB (sectors=%lu -> %lu); remaining space unused",
                     (unsigned long)(disk_sectors - part_lba), (unsigned long)DOS33_MAX_PART_SECTORS);
            part_sectors = DOS33_MAX_PART_SECTORS;
        }
    }

    uint16_t cyl = 0;
    uint16_t heads = 0;
    uint16_t spt = 0;
    choose_hdd_chs(disk_sectors, &cyl, &heads, &spt);

    const uint16_t reserved = 1;
    const uint8_t fats = 2;
    const uint16_t root_entries = 512;
    uint8_t spc = choose_fat16_spc(part_sectors);

    uint16_t spf = 0;
    uint32_t clusters = 0;
    if (!fat16_compute(part_sectors, spc, reserved, fats, root_entries, &spf, &clusters)) {
        // Try smaller clusters in case this volume size sits just above a boundary.
        while (spc > 1) {
            spc >>= 1;
            if (fat16_compute(part_sectors, spc, reserved, fats, root_entries, &spf, &clusters)) {
                break;
            }
        }
    }
    if (spf == 0) {
        ESP_LOGE(TAG, "Failed to compute FAT16 layout: part_sectors=%lu", (unsigned long)part_sectors);
        return false;
    }

    const uint32_t root_dir_sectors = ((uint32_t)root_entries * 32u + (SECTOR_SIZE - 1u)) / SECTOR_SIZE;
    const uint32_t fat1_lba = part_lba + reserved;
    const uint32_t fat2_lba = fat1_lba + spf;
    const uint32_t root_lba = fat2_lba + spf;
    const uint32_t data_lba = root_lba + root_dir_sectors;

    ESP_LOGI(TAG, "Formatting FAT16: disk=%lu sectors, part@%lu size=%lu",
             (unsigned long)disk_sectors, (unsigned long)part_lba, (unsigned long)part_sectors);
    ESP_LOGI(TAG, "FAT16 params: SPC=%u RS=%u FATs=%u Root=%u SPF=%u DataLBA=%lu Clusters=%lu CHS=%ux%ux%u",
             spc, reserved, fats, root_entries, spf, (unsigned long)data_lba, (unsigned long)clusters,
             (unsigned)cyl, (unsigned)heads, (unsigned)spt);

    // MBR
    memset(s_sector_buf, 0, sizeof(s_sector_buf));
    uint8_t *pte = &s_sector_buf[446];
    pte[0] = 0x80;  // bootable flag (helps DOS/FDISK treat it as the primary active partition)
    uint8_t chs_start[3] = {0};
    uint8_t chs_end[3] = {0};
    encode_chs(part_lba, heads, spt, chs_start);
    encode_chs(part_lba + part_sectors - 1u, heads, spt, chs_end);
    pte[1] = chs_start[0];
    pte[2] = chs_start[1];
    pte[3] = chs_start[2];
    if (g_c_drive_dos33_compat) {
        pte[4] = 0x04;  // FAT16 (<32MB)
    } else {
        pte[4] = (part_sectors <= 0xFFFFu) ? 0x04 : 0x06;  // FAT16 (<32MB) or FAT16 (>=32MB)
    }
    pte[5] = chs_end[0];
    pte[6] = chs_end[1];
    pte[7] = chs_end[2];
    write_le32(&pte[8], part_lba);
    write_le32(&pte[12], part_sectors);
    s_sector_buf[510] = 0x55;
    s_sector_buf[511] = 0xAA;
    if (!write_sector(f, 0, s_sector_buf)) {
        return false;
    }

    // Partition boot sector (VBR)
    memset(s_sector_buf, 0, sizeof(s_sector_buf));
    s_sector_buf[0] = 0xEB; s_sector_buf[1] = 0x3C; s_sector_buf[2] = 0x90;  // JMP short
    memcpy(&s_sector_buf[3], "M5PDOS  ", 8);
    write_le16(&s_sector_buf[11], SECTOR_SIZE);        // bytes/sector
    s_sector_buf[13] = spc;                            // sectors/cluster
    write_le16(&s_sector_buf[14], reserved);           // reserved sectors
    s_sector_buf[16] = fats;                           // number of FATs
    write_le16(&s_sector_buf[17], root_entries);       // root entries
    if (part_sectors <= 0xFFFFu) {
        write_le16(&s_sector_buf[19], (uint16_t)part_sectors); // total sectors (16-bit)
    } else {
        write_le16(&s_sector_buf[19], 0);
        write_le32(&s_sector_buf[32], part_sectors);           // total sectors (32-bit)
    }
    s_sector_buf[21] = 0xF8;                           // media descriptor
    write_le16(&s_sector_buf[22], spf);                // sectors/FAT
    write_le16(&s_sector_buf[24], spt);                // sectors/track
    write_le16(&s_sector_buf[26], heads);              // number of heads
    write_le32(&s_sector_buf[28], part_lba);           // hidden sectors
    s_sector_buf[36] = 0x80;                           // drive number
    s_sector_buf[38] = 0x29;                           // boot signature
    // volume ID
    uint32_t vol_id = (uint32_t)time(NULL) ^ (part_sectors * 2654435761u);
    write_le32(&s_sector_buf[39], vol_id);
    memcpy(&s_sector_buf[43], "M5PAPERDOS ", 11);
    memcpy(&s_sector_buf[54], "FAT16   ", 8);
    s_sector_buf[510] = 0x55;
    s_sector_buf[511] = 0xAA;
    if (!write_sector(f, part_lba, s_sector_buf)) {
        return false;
    }

    // FATs + root directory
    if (!zero_sectors(f, fat1_lba, spf)) return false;
    if (!zero_sectors(f, fat2_lba, spf)) return false;
    if (!zero_sectors(f, root_lba, root_dir_sectors)) return false;

    memset(s_sector_buf, 0, sizeof(s_sector_buf));
    s_sector_buf[0] = 0xF8;
    s_sector_buf[1] = 0xFF;
    s_sector_buf[2] = 0xFF;
    s_sector_buf[3] = 0xFF;
    if (!write_sector(f, fat1_lba, s_sector_buf)) return false;
    if (!write_sector(f, fat2_lba, s_sector_buf)) return false;

    fflush(f);
    return true;
}

static bool looks_like_mbr(const uint8_t mbr[SECTOR_SIZE])
{
    if (mbr[510] != 0x55 || mbr[511] != 0xAA) {
        return false;
    }
    // Basic sanity: at least one partition entry has a nonzero type.
    for (int i = 0; i < 4; i++) {
        const uint8_t *pte = &mbr[446 + (i * 16)];
        if (pte[4] != 0x00) {
            return true;
        }
    }
    return false;
}

static bool fixup_mbr_chs(const char *image_path, uint64_t size_bytes)
{
    if (!image_path || size_bytes < SECTOR_SIZE) {
        return false;
    }

    FILE *f = fopen(image_path, "r+b");
    if (!f) {
        return false;
    }

    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return false;
    }
    size_t read = fread(s_sector_buf, 1, sizeof(s_sector_buf), f);
    if (read != sizeof(s_sector_buf) || !looks_like_mbr(s_sector_buf)) {
        fclose(f);
        return false;
    }

    uint8_t *pte = &s_sector_buf[446];
    uint32_t part_lba = read_le32(pte + 8);
    uint32_t part_sectors = read_le32(pte + 12);
    if (pte[4] == 0x00 || part_lba == 0 || part_sectors == 0) {
        fclose(f);
        return false;
    }

    const bool start_sentinel = (pte[1] == 0xFE && pte[2] == 0xFF && pte[3] == 0xFF);
    const bool end_sentinel = (pte[5] == 0xFE && pte[6] == 0xFF && pte[7] == 0xFF);
    if (!start_sentinel && !end_sentinel) {
        fclose(f);
        return true;
    }

    uint32_t disk_sectors = (uint32_t)(size_bytes / SECTOR_SIZE);
    uint16_t cyl = 0, heads = 0, spt = 0;
    choose_hdd_chs(disk_sectors, &cyl, &heads, &spt);
    (void)cyl;

    uint8_t chs_start[3] = {0};
    uint8_t chs_end[3] = {0};
    encode_chs(part_lba, heads, spt, chs_start);
    encode_chs(part_lba + part_sectors - 1u, heads, spt, chs_end);
    pte[1] = chs_start[0];
    pte[2] = chs_start[1];
    pte[3] = chs_start[2];
    pte[5] = chs_end[0];
    pte[6] = chs_end[1];
    pte[7] = chs_end[2];

    ESP_LOGW(TAG, "Fixing MBR CHS fields for DOS compatibility: path=%s start_lba=%lu sectors=%lu",
             image_path, (unsigned long)part_lba, (unsigned long)part_sectors);

    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return false;
    }
    if (fwrite(s_sector_buf, 1, sizeof(s_sector_buf), f) != sizeof(s_sector_buf)) {
        fclose(f);
        return false;
    }
    fflush(f);
    fclose(f);
    return true;
}

bool hdd_image_ensure_fat16(const char *image_path, uint64_t size_bytes)
{
    if (!image_path || image_path[0] == '\0') {
        return false;
    }

    if (s_in_use) {
        ESP_LOGE(TAG, "hdd_image_ensure_fat16 called concurrently");
        return false;
    }
    s_in_use = true;

    // If it already exists, leave it untouched.
    uint64_t existing_size = 0;
    if (is_regular_file(image_path, &existing_size)) {
        if (existing_size < SECTOR_SIZE) {
            ESP_LOGE(TAG, "Existing image too small: path=%s size=%llu", image_path, (unsigned long long)existing_size);
            s_in_use = false;
            return false;
        }
        if (g_c_drive_dos33_compat) {
        FILE *f = fopen(image_path, "rb");
        uint32_t part_lba = 0;
        uint32_t part_sectors = 0;
        bool have_part = false;
        if (f) {
            have_part = read_mbr_partition0(f, &part_lba, &part_sectors, NULL);
            fclose(f);
        }
        if (have_part && part_sectors > DOS33_MAX_PART_SECTORS) {
            ESP_LOGW(TAG, "Existing C: image partition is too large for DOS 3.30 (sectors=%lu). Recreating a compatible image...",
                     (unsigned long)part_sectors);
            char backup[160];
            if (backup_path_for_image(image_path, backup, sizeof(backup)) && rename(image_path, backup) == 0) {
                ESP_LOGW(TAG, "Moved old image aside: %s", backup);
            } else {
                ESP_LOGW(TAG, "Failed to move old image aside; reformatting in-place (data will be lost): %s", image_path);
                FILE *rf = fopen(image_path, "r+b");
                if (rf) {
                    bool ok = format_mbr_fat16(rf, (uint32_t)(existing_size / SECTOR_SIZE));
                    fclose(rf);
                    if (ok) {
                        (void)fixup_mbr_chs(image_path, existing_size);
                        s_in_use = false;
                        return true;
                    }
                }
                s_in_use = false;
                return false;
            }
            // Fall through to create a new compatible image at the original path.
        } else {
            ESP_LOGI(TAG, "Using existing C: image: %s (%llu bytes)", image_path, (unsigned long long)existing_size);
            (void)fixup_mbr_chs(image_path, existing_size);
            s_in_use = false;
            return true;
        }
        } else {
        ESP_LOGI(TAG, "Using existing C: image: %s (%llu bytes)", image_path, (unsigned long long)existing_size);
        (void)fixup_mbr_chs(image_path, existing_size);
        s_in_use = false;
        return true;
        }
    }

    // Create parent dir if needed (best-effort: assume ".../file.img").
    const char *slash = strrchr(image_path, '/');
    if (slash) {
        char dir[128];
        size_t len = (size_t)(slash - image_path);
        if (len < sizeof(dir)) {
            memcpy(dir, image_path, len);
            dir[len] = '\0';
            if (!ensure_dir(dir)) {
                s_in_use = false;
                return false;
            }
        }
    }

    if ((size_bytes % SECTOR_SIZE) != 0) {
        size_bytes -= (size_bytes % SECTOR_SIZE);
    }
    if (g_c_drive_dos33_compat) {
        const uint64_t dos33_size = (uint64_t)SECTOR_SIZE * (DEFAULT_HDD_START_LBA + (uint64_t)DOS33_MAX_PART_SECTORS);
        if (size_bytes > dos33_size) {
            size_bytes = dos33_size;
        }
    }
    if (size_bytes < (uint64_t)SECTOR_SIZE * (DEFAULT_HDD_START_LBA + 4096u)) {
        size_bytes = (uint64_t)SECTOR_SIZE * (DEFAULT_HDD_START_LBA + 4096u);
    }
    // Keep within 32-bit signed file APIs (newlib long/offset). This is "about 2GB".
    const uint64_t max_size = 2147483136ull; // 0x7FFFFE00 (sector-aligned)
    if (size_bytes > max_size) {
        size_bytes = max_size;
    }

    ESP_LOGI(TAG, "Creating C: image: %s size=%llu bytes", image_path, (unsigned long long)size_bytes);

    FILE *f = fopen(image_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to create image: path=%s errno=%d (%s)", image_path, errno, strerror(errno));
        s_in_use = false;
        return false;
    }

    // Extend file to requested size. FATFS doesn't support sparse files; this may take time.
    if (fseek(f, (long)(size_bytes - 1), SEEK_SET) != 0 || fputc(0, f) == EOF) {
        ESP_LOGE(TAG, "Failed to size image: path=%s errno=%d (%s)", image_path, errno, strerror(errno));
        fclose(f);
        remove(image_path);
        s_in_use = false;
        return false;
    }
    fflush(f);
    fclose(f);

    f = fopen(image_path, "r+b");
    if (!f) {
        ESP_LOGE(TAG, "Failed to reopen image: path=%s errno=%d (%s)", image_path, errno, strerror(errno));
        remove(image_path);
        s_in_use = false;
        return false;
    }

    uint32_t disk_sectors = (uint32_t)(size_bytes / SECTOR_SIZE);
    bool ok = format_mbr_fat16(f, disk_sectors);
    fclose(f);
    if (!ok) {
        remove(image_path);
        s_in_use = false;
        return false;
    }

    // Quick verify signature (no heavy parsing).
    f = fopen(image_path, "rb");
    if (f) {
        size_t read = fread(s_sector_buf, 1, sizeof(s_sector_buf), f);
        fclose(f);
        if (read == sizeof(s_sector_buf) && looks_like_mbr(s_sector_buf)) {
            s_in_use = false;
            return true;
        }
    }

    ESP_LOGW(TAG, "Image created, but MBR verification failed: %s", image_path);
    s_in_use = false;
    return true;
}
