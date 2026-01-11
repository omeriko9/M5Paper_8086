#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SETTINGS_BOOT_IMAGE_PATH_MAX 256

typedef struct app_settings
{
  float display_fps;
  bool display_partial_refresh;
  bool display_pause_cpu_on_refresh;
  bool display_clear_on_bottom;
  bool wifi_enabled;
  bool bt_keyboard_enabled;
  bool gameport_enabled;
  bool c_drive_dos33_compat;
  char boot_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  char c_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  char a_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  char b_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
} app_settings_t;

esp_err_t app_settings_init(void);

uint32_t app_settings_generation(void);

float app_settings_display_fps(void);
bool app_settings_display_partial_refresh(void);
bool app_settings_display_pause_cpu_on_refresh(void);
bool app_settings_display_clear_on_bottom(void);
bool app_settings_wifi_enabled(void);
bool app_settings_bt_keyboard_enabled(void);
bool app_settings_gameport_enabled(void);
bool app_settings_get_boot_image_path(char *out, size_t out_len);
bool app_settings_get_c_drive_image_path(char *out, size_t out_len);
bool app_settings_get_a_drive_image_path(char *out, size_t out_len);
bool app_settings_get_b_drive_image_path(char *out, size_t out_len);

esp_err_t app_settings_set_display_fps(float fps);
esp_err_t app_settings_set_display_partial_refresh(bool enabled);
esp_err_t app_settings_set_display_pause_cpu_on_refresh(bool enabled);
esp_err_t app_settings_set_display_clear_on_bottom(bool enabled);
esp_err_t app_settings_set_wifi_enabled(bool enabled);
esp_err_t app_settings_set_bt_keyboard_enabled(bool enabled);
esp_err_t app_settings_set_gameport_enabled(bool enabled);
esp_err_t app_settings_set_boot_image_path(const char *path);
esp_err_t app_settings_set_c_drive_image_path(const char *path);
esp_err_t app_settings_set_a_drive_image_path(const char *path);
esp_err_t app_settings_set_b_drive_image_path(const char *path);

esp_err_t app_settings_set_log_level(const char *tag, int level);
esp_err_t app_settings_get_log_level(const char *tag, int *out_level);

esp_err_t app_settings_get(app_settings_t *out);
esp_err_t app_settings_set(const app_settings_t *in);

#ifdef __cplusplus
}
#endif
