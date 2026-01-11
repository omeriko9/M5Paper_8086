#include "settings.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "SETTINGS";

#define NVS_NS "app_cfg"
#define KEY_DISPLAY_FPS_BITS "disp_fps"
#define KEY_DISPLAY_PARTIAL "disp_part"
#define KEY_DISPLAY_PAUSE_CPU "disp_pause"
#define KEY_DISPLAY_CLEAR_BOTTOM "disp_clrbot"
#define KEY_BT_KB_ENABLED "bt_kb_en"
#define KEY_GAMEPORT_ENABLED "gameport_en"
#define KEY_BOOT_IMAGE_PATH "boot_img"
#define KEY_C_DRIVE_IMAGE_PATH "c_img"
#define KEY_A_DRIVE_IMAGE_PATH "a_img"
#define KEY_B_DRIVE_IMAGE_PATH "b_img"

static portMUX_TYPE s_settings_mux = portMUX_INITIALIZER_UNLOCKED;
static float s_display_fps = 2.0f;
static bool s_display_partial = true;
static bool s_display_pause_cpu = true;
static bool s_display_clear_bottom = false;
static bool s_wifi_enabled = true;
static bool s_bt_kb_enabled = true;
static bool s_gameport_enabled = true;
static char s_boot_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = "/sdcard/msdos.img";
static char s_c_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = "/sdcard/c_drive/hd.img";
static char s_a_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = "/sdcard/disk1.img";
static char s_b_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = "/sdcard/disk2.img";
static uint32_t s_generation = 1;

static esp_err_t ensure_nvs_ready(void)
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_LOGW(TAG, "NVS init needs erase: err=%d", (int)err);
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", (int)err);
  }
  return err;
}

static bool nvs_read_u32(nvs_handle_t nvs, const char *key, uint32_t *out)
{
  if (!out)
    return false;
  uint32_t tmp = 0;
  esp_err_t err = nvs_get_u32(nvs, key, &tmp);
  if (err != ESP_OK)
    return false;
  *out = tmp;
  return true;
}

static bool nvs_read_u8(nvs_handle_t nvs, const char *key, uint8_t *out)
{
  if (!out)
    return false;
  uint8_t tmp = 0;
  esp_err_t err = nvs_get_u8(nvs, key, &tmp);
  if (err != ESP_OK)
    return false;
  *out = tmp;
  return true;
}

static uint32_t float_to_u32_bits(float f)
{
  uint32_t bits = 0;
  memcpy(&bits, &f, sizeof(bits));
  return bits;
}

static float u32_bits_to_float(uint32_t bits)
{
  float f = 0.0f;
  memcpy(&f, &bits, sizeof(f));
  return f;
}

esp_err_t app_settings_init(void)
{
  esp_err_t err = ensure_nvs_ready();
  if (err != ESP_OK)
  {
    return err;
  }

  nvs_handle_t nvs;
  err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "No settings namespace yet; using defaults (err=%d)", (int)err);
    return ESP_OK;
  }

  uint32_t fps_bits = 0;
  uint8_t partial = 0;
  uint8_t pause_cpu = 0;
  uint8_t clear_bottom = 0;
  uint8_t bt_kb_en = 1;
  uint8_t gameport_en = 1;
  char boot_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  size_t boot_path_len = sizeof(boot_path);
  char c_drive_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  size_t c_drive_path_len = sizeof(c_drive_path);
  char a_drive_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  size_t a_drive_path_len = sizeof(a_drive_path);
  char b_drive_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  size_t b_drive_path_len = sizeof(b_drive_path);
  bool have_fps = nvs_read_u32(nvs, KEY_DISPLAY_FPS_BITS, &fps_bits);
  bool have_partial = nvs_read_u8(nvs, KEY_DISPLAY_PARTIAL, &partial);
  bool have_pause_cpu = nvs_read_u8(nvs, KEY_DISPLAY_PAUSE_CPU, &pause_cpu);
  bool have_clear_bottom = nvs_read_u8(nvs, KEY_DISPLAY_CLEAR_BOTTOM, &clear_bottom);
  bool have_bt_kb = nvs_read_u8(nvs, KEY_BT_KB_ENABLED, &bt_kb_en);
  bool have_gameport = nvs_read_u8(nvs, KEY_GAMEPORT_ENABLED, &gameport_en);
  bool have_boot = (nvs_get_str(nvs, KEY_BOOT_IMAGE_PATH, boot_path, &boot_path_len) == ESP_OK) && boot_path[0];
  bool have_c_drive = (nvs_get_str(nvs, KEY_C_DRIVE_IMAGE_PATH, c_drive_path, &c_drive_path_len) == ESP_OK) && c_drive_path[0];
  bool have_a_drive = (nvs_get_str(nvs, KEY_A_DRIVE_IMAGE_PATH, a_drive_path, &a_drive_path_len) == ESP_OK) && a_drive_path[0];
  bool have_b_drive = (nvs_get_str(nvs, KEY_B_DRIVE_IMAGE_PATH, b_drive_path, &b_drive_path_len) == ESP_OK) && b_drive_path[0];
  nvs_close(nvs);

  portENTER_CRITICAL(&s_settings_mux);
  s_wifi_enabled = true; // Always default to true on boot
  if (have_fps)
  {
    float fps = u32_bits_to_float(fps_bits);
    if (isfinite(fps) && fps >= 0.0f)
    {
      s_display_fps = fps;
    }
  }
  if (have_partial)
  {
    s_display_partial = (partial != 0);
  }
  if (have_pause_cpu)
  {
    s_display_pause_cpu = (pause_cpu != 0);
  }
  if (have_clear_bottom)
  {
    s_display_clear_bottom = (clear_bottom != 0);
  }
  if (have_bt_kb)
  {
    s_bt_kb_enabled = (bt_kb_en != 0);
  }
  if (have_gameport)
  {
    s_gameport_enabled = (gameport_en != 0);
  }
  if (have_boot)
  {
    strncpy(s_boot_image_path, boot_path, sizeof(s_boot_image_path) - 1);
    s_boot_image_path[sizeof(s_boot_image_path) - 1] = '\0';
  }
  if (have_c_drive)
  {
    strncpy(s_c_drive_image_path, c_drive_path, sizeof(s_c_drive_image_path) - 1);
    s_c_drive_image_path[sizeof(s_c_drive_image_path) - 1] = '\0';
  }
  if (have_a_drive)
  {
    strncpy(s_a_drive_image_path, a_drive_path, sizeof(s_a_drive_image_path) - 1);
    s_a_drive_image_path[sizeof(s_a_drive_image_path) - 1] = '\0';
  }
  if (have_b_drive)
  {
    strncpy(s_b_drive_image_path, b_drive_path, sizeof(s_b_drive_image_path) - 1);
    s_b_drive_image_path[sizeof(s_b_drive_image_path) - 1] = '\0';
  }
  s_generation++;
  portEXIT_CRITICAL(&s_settings_mux);

  ESP_LOGI(TAG, "Settings: display_fps=%.6g partial=%d pause_cpu=%d clear_bottom=%d wifi=%d bt_kb=%d gameport=%d boot_img=%s c_img=%s a_img=%s b_img=%s",
           (double)app_settings_display_fps(),
           app_settings_display_partial_refresh() ? 1 : 0,
           app_settings_display_pause_cpu_on_refresh() ? 1 : 0,
           app_settings_display_clear_on_bottom() ? 1 : 0,
           app_settings_wifi_enabled() ? 1 : 0,
           app_settings_bt_keyboard_enabled() ? 1 : 0,
           app_settings_gameport_enabled() ? 1 : 0,
           s_boot_image_path,
           s_c_drive_image_path,
           s_a_drive_image_path,
           s_b_drive_image_path);
  return ESP_OK;
}

uint32_t app_settings_generation(void)
{
  uint32_t gen;
  portENTER_CRITICAL(&s_settings_mux);
  gen = s_generation;
  portEXIT_CRITICAL(&s_settings_mux);
  return gen;
}

float app_settings_display_fps(void)
{
  float fps;
  portENTER_CRITICAL(&s_settings_mux);
  fps = s_display_fps;
  portEXIT_CRITICAL(&s_settings_mux);
  return fps;
}

bool app_settings_display_partial_refresh(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_display_partial;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_display_pause_cpu_on_refresh(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_display_pause_cpu;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_display_clear_on_bottom(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_display_clear_bottom;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_wifi_enabled(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_wifi_enabled;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_bt_keyboard_enabled(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_bt_kb_enabled;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_gameport_enabled(void)
{
  bool enabled;
  portENTER_CRITICAL(&s_settings_mux);
  enabled = s_gameport_enabled;
  portEXIT_CRITICAL(&s_settings_mux);
  return enabled;
}

bool app_settings_get_boot_image_path(char *out, size_t out_len)
{
  if (!out || out_len == 0)
    return false;
  portENTER_CRITICAL(&s_settings_mux);
  strncpy(out, s_boot_image_path, out_len - 1);
  out[out_len - 1] = '\0';
  portEXIT_CRITICAL(&s_settings_mux);
  return out[0] != '\0';
}

bool app_settings_get_c_drive_image_path(char *out, size_t out_len)
{
  if (!out || out_len == 0)
    return false;
  portENTER_CRITICAL(&s_settings_mux);
  strncpy(out, s_c_drive_image_path, out_len - 1);
  out[out_len - 1] = '\0';
  portEXIT_CRITICAL(&s_settings_mux);
  return out[0] != '\0';
}

bool app_settings_get_a_drive_image_path(char *out, size_t out_len)
{
  if (!out || out_len == 0)
    return false;
  portENTER_CRITICAL(&s_settings_mux);
  strncpy(out, s_a_drive_image_path, out_len - 1);
  out[out_len - 1] = '\0';
  portEXIT_CRITICAL(&s_settings_mux);
  return out[0] != '\0';
}

bool app_settings_get_b_drive_image_path(char *out, size_t out_len)
{
  if (!out || out_len == 0)
    return false;
  portENTER_CRITICAL(&s_settings_mux);
  strncpy(out, s_b_drive_image_path, out_len - 1);
  out[out_len - 1] = '\0';
  portEXIT_CRITICAL(&s_settings_mux);
  return out[0] != '\0';
}

esp_err_t app_settings_get(app_settings_t *out)
{
  if (!out)
    return ESP_ERR_INVALID_ARG;
  portENTER_CRITICAL(&s_settings_mux);
  out->display_fps = s_display_fps;
  out->display_partial_refresh = s_display_partial;
  out->display_pause_cpu_on_refresh = s_display_pause_cpu;
  out->display_clear_on_bottom = s_display_clear_bottom;
  out->wifi_enabled = s_wifi_enabled;
  out->bt_keyboard_enabled = s_bt_kb_enabled;
  out->gameport_enabled = s_gameport_enabled;
  strncpy(out->boot_image_path, s_boot_image_path, sizeof(out->boot_image_path) - 1);
  out->boot_image_path[sizeof(out->boot_image_path) - 1] = '\0';
  strncpy(out->c_drive_image_path, s_c_drive_image_path, sizeof(out->c_drive_image_path) - 1);
  out->c_drive_image_path[sizeof(out->c_drive_image_path) - 1] = '\0';
  strncpy(out->a_drive_image_path, s_a_drive_image_path, sizeof(out->a_drive_image_path) - 1);
  out->a_drive_image_path[sizeof(out->a_drive_image_path) - 1] = '\0';
  strncpy(out->b_drive_image_path, s_b_drive_image_path, sizeof(out->b_drive_image_path) - 1);
  out->b_drive_image_path[sizeof(out->b_drive_image_path) - 1] = '\0';
  portEXIT_CRITICAL(&s_settings_mux);
  return ESP_OK;
}

static esp_err_t write_to_nvs(const app_settings_t *s)
{
  esp_err_t err = ensure_nvs_ready();
  if (err != ESP_OK)
    return err;

  nvs_handle_t nvs;
  err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
  if (err != ESP_OK)
    return err;

  const uint32_t fps_bits = float_to_u32_bits(s->display_fps);
  err = nvs_set_u32(nvs, KEY_DISPLAY_FPS_BITS, fps_bits);
  if (err == ESP_OK)
  {
    err = nvs_set_u8(nvs, KEY_DISPLAY_PARTIAL, s->display_partial_refresh ? 1 : 0);
  }
  if (err == ESP_OK)
  {
    err = nvs_set_u8(nvs, KEY_DISPLAY_PAUSE_CPU, s->display_pause_cpu_on_refresh ? 1 : 0);
  }
  if (err == ESP_OK)
  {
    err = nvs_set_u8(nvs, KEY_DISPLAY_CLEAR_BOTTOM, s->display_clear_on_bottom ? 1 : 0);
  }
  if (err == ESP_OK)
  {
    err = nvs_set_u8(nvs, KEY_BT_KB_ENABLED, s->bt_keyboard_enabled ? 1 : 0);
  }
  if (err == ESP_OK)
  {
    err = nvs_set_u8(nvs, KEY_GAMEPORT_ENABLED, s->gameport_enabled ? 1 : 0);
  }
  if (err == ESP_OK)
  {
    if (s->boot_image_path[0])
    {
      err = nvs_set_str(nvs, KEY_BOOT_IMAGE_PATH, s->boot_image_path);
    }
    else
    {
      (void)nvs_erase_key(nvs, KEY_BOOT_IMAGE_PATH);
    }
  }
  if (err == ESP_OK)
  {
    if (s->c_drive_image_path[0])
    {
      err = nvs_set_str(nvs, KEY_C_DRIVE_IMAGE_PATH, s->c_drive_image_path);
    }
    else
    {
      (void)nvs_erase_key(nvs, KEY_C_DRIVE_IMAGE_PATH);
    }
  }
  if (err == ESP_OK)
  {
    if (s->a_drive_image_path[0])
    {
      err = nvs_set_str(nvs, KEY_A_DRIVE_IMAGE_PATH, s->a_drive_image_path);
    }
    else
    {
      (void)nvs_erase_key(nvs, KEY_A_DRIVE_IMAGE_PATH);
    }
  }
  if (err == ESP_OK)
  {
    if (s->b_drive_image_path[0])
    {
      err = nvs_set_str(nvs, KEY_B_DRIVE_IMAGE_PATH, s->b_drive_image_path);
    }
    else
    {
      (void)nvs_erase_key(nvs, KEY_B_DRIVE_IMAGE_PATH);
    }
  }
  if (err == ESP_OK)
  {
    err = nvs_commit(nvs);
  }
  nvs_close(nvs);
  return err;
}

esp_err_t app_settings_set(const app_settings_t *in)
{
  if (!in)
    return ESP_ERR_INVALID_ARG;
  if (!isfinite(in->display_fps) || in->display_fps < 0.0f)
    return ESP_ERR_INVALID_ARG;

  app_settings_t next = *in;
  next.boot_image_path[sizeof(next.boot_image_path) - 1] = '\0';
  next.c_drive_image_path[sizeof(next.c_drive_image_path) - 1] = '\0';
  next.a_drive_image_path[sizeof(next.a_drive_image_path) - 1] = '\0';
  next.b_drive_image_path[sizeof(next.b_drive_image_path) - 1] = '\0';

  portENTER_CRITICAL(&s_settings_mux);
  s_display_fps = next.display_fps;
  s_display_partial = next.display_partial_refresh;
  s_display_pause_cpu = next.display_pause_cpu_on_refresh;
  s_display_clear_bottom = next.display_clear_on_bottom;
  s_wifi_enabled = next.wifi_enabled;
  s_bt_kb_enabled = next.bt_keyboard_enabled;
  s_gameport_enabled = next.gameport_enabled;
  strncpy(s_boot_image_path, next.boot_image_path, sizeof(s_boot_image_path) - 1);
  s_boot_image_path[sizeof(s_boot_image_path) - 1] = '\0';
  strncpy(s_c_drive_image_path, next.c_drive_image_path, sizeof(s_c_drive_image_path) - 1);
  s_c_drive_image_path[sizeof(s_c_drive_image_path) - 1] = '\0';
  strncpy(s_a_drive_image_path, next.a_drive_image_path, sizeof(s_a_drive_image_path) - 1);
  s_a_drive_image_path[sizeof(s_a_drive_image_path) - 1] = '\0';
  strncpy(s_b_drive_image_path, next.b_drive_image_path, sizeof(s_b_drive_image_path) - 1);
  s_b_drive_image_path[sizeof(s_b_drive_image_path) - 1] = '\0';
  s_generation++;
  portEXIT_CRITICAL(&s_settings_mux);

  esp_err_t err = write_to_nvs(&next);
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "Failed to persist settings: %d", (int)err);
  }
  return err;
}

esp_err_t app_settings_set_display_fps(float fps)
{
  if (!isfinite(fps) || fps < 0.0f)
    return ESP_ERR_INVALID_ARG;

  app_settings_t s;
  (void)app_settings_get(&s);
  s.display_fps = fps;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_display_partial_refresh(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.display_partial_refresh = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_display_pause_cpu_on_refresh(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.display_pause_cpu_on_refresh = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_display_clear_on_bottom(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.display_clear_on_bottom = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_wifi_enabled(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.wifi_enabled = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_bt_keyboard_enabled(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.bt_keyboard_enabled = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_gameport_enabled(bool enabled)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  s.gameport_enabled = enabled;
  return app_settings_set(&s);
}

esp_err_t app_settings_set_boot_image_path(const char *path)
{
  if (!path || !path[0])
  {
    return ESP_ERR_INVALID_ARG;
  }
  if (strlen(path) >= APP_SETTINGS_BOOT_IMAGE_PATH_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  app_settings_t s;
  (void)app_settings_get(&s);
  strncpy(s.boot_image_path, path, sizeof(s.boot_image_path) - 1);
  s.boot_image_path[sizeof(s.boot_image_path) - 1] = '\0';
  return app_settings_set(&s);
}

esp_err_t app_settings_set_c_drive_image_path(const char *path)
{
  if (!path || !path[0])
  {
    return ESP_ERR_INVALID_ARG;
  }
  if (strlen(path) >= APP_SETTINGS_BOOT_IMAGE_PATH_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  app_settings_t s;
  (void)app_settings_get(&s);
  strncpy(s.c_drive_image_path, path, sizeof(s.c_drive_image_path) - 1);
  s.c_drive_image_path[sizeof(s.c_drive_image_path) - 1] = '\0';
  return app_settings_set(&s);
}

esp_err_t app_settings_set_a_drive_image_path(const char *path)
{
  if (!path || !path[0])
  {
    return ESP_ERR_INVALID_ARG;
  }
  if (strlen(path) >= APP_SETTINGS_BOOT_IMAGE_PATH_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  app_settings_t s;
  (void)app_settings_get(&s);
  strncpy(s.a_drive_image_path, path, sizeof(s.a_drive_image_path) - 1);
  s.a_drive_image_path[sizeof(s.a_drive_image_path) - 1] = '\0';
  return app_settings_set(&s);
}

esp_err_t app_settings_set_b_drive_image_path(const char *path)
{
  if (!path || !path[0])
  {
    return ESP_ERR_INVALID_ARG;
  }
  if (strlen(path) >= APP_SETTINGS_BOOT_IMAGE_PATH_MAX)
  {
    return ESP_ERR_INVALID_ARG;
  }

  app_settings_t s;
  (void)app_settings_get(&s);
  strncpy(s.b_drive_image_path, path, sizeof(s.b_drive_image_path) - 1);
  s.b_drive_image_path[sizeof(s.b_drive_image_path) - 1] = '\0';
  return app_settings_set(&s);
}
esp_err_t app_settings_set_log_level(const char *tag, int level)
{
  if (!tag || !tag[0])
    return ESP_ERR_INVALID_ARG;

  // Key must be <= 15 chars. "L_" prefix takes 2. So tag must be <= 13.
  char key[16];
  int w = snprintf(key, sizeof(key), "L_%s", tag);
  if (w < 0 || w >= sizeof(key))
  {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
  if (err != ESP_OK)
    return err;

  err = nvs_set_u8(nvs, key, (uint8_t)level);
  if (err == ESP_OK)
  {
    err = nvs_commit(nvs);
  }
  nvs_close(nvs);
  return err;
}

esp_err_t app_settings_get_log_level(const char *tag, int *out_level)
{
  if (!tag || !tag[0] || !out_level)
    return ESP_ERR_INVALID_ARG;

  char key[16];
  int w = snprintf(key, sizeof(key), "L_%s", tag);
  if (w < 0 || w >= sizeof(key))
  {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
  if (err != ESP_OK)
    return err;

  uint8_t val = 0;
  err = nvs_get_u8(nvs, key, &val);
  nvs_close(nvs);

  if (err == ESP_OK)
  {
    *out_level = (int)val;
  }
  return err;
}
