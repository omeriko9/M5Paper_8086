#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fat16_image fat16_image_t;

typedef struct fat16_dirent_info
{
  char name[13]; // "NAME.EXT" (8.3), upper-case
  bool is_dir;
  uint32_t size;
  uint16_t fat_date;
  uint16_t fat_time;
  uint8_t attr;
} fat16_dirent_info_t;

typedef bool (*fat16_list_cb)(const fat16_dirent_info_t *info, void *arg);

typedef struct fat16_write_ctx
{
  fat16_image_t *img;
  uint32_t dirent_lba;
  uint16_t dirent_off;
  uint32_t size;
  uint16_t first_cluster;
  uint16_t last_cluster;
  uint32_t cluster_bytes_used;
  uint8_t sector_buf[512];
  uint16_t sector_used;
  uint8_t sector_in_cluster;
  bool active;
} fat16_write_ctx_t;

bool fat16_image_open(fat16_image_t **out, const char *image_path, bool write, char *err, size_t err_len);
void fat16_image_close(fat16_image_t *img);

bool fat16_image_list_dir(fat16_image_t *img, const char *path, fat16_list_cb cb, void *arg, char *err, size_t err_len);

// Create a directory `name` inside `dir_path` (both use "/" separators).
// Only supports 8.3 names.
bool fat16_image_mkdir(fat16_image_t *img, const char *dir_path, const char *name, char *err, size_t err_len);

bool fat16_image_delete(fat16_image_t *img, const char *path, char *err, size_t err_len);
bool fat16_image_rename(fat16_image_t *img, const char *path, const char *new_name, char *err, size_t err_len);

bool fat16_image_write_begin(fat16_image_t *img,
                            const char *dir_path,
                            const char *name,
                            uint32_t total_size,
                            fat16_write_ctx_t *out,
                            char *err,
                            size_t err_len);
bool fat16_image_write_append(fat16_write_ctx_t *ctx, const uint8_t *data, size_t len, char *err, size_t err_len);
bool fat16_image_write_finish(fat16_write_ctx_t *ctx, char *err, size_t err_len);
void fat16_image_write_abort(fat16_write_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
