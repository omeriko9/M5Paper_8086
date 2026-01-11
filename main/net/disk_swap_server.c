#include "net/disk_swap_server.h"

#include <errno.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "dos/bios.h"
#include "bt/bt_keyboard.h"
#include "dos/disk.h"
#include "dos/fat16_image.h"
#include "dos/hdd_image.h"
#include "dos/memory.h"
#include "dos/ports.h"
#include "dos/video.h"
#include "display/epd_driver.h"
#include "config_defs.h"
#include "settings.h"
#include "sdcard/sdcard.h"

static const char *TAG = "DISKSWAP";

static size_t json_escape_string(const char *src, char *dst, size_t dst_len);
static bool get_query_value(httpd_req_t *req, const char *key, char *out, size_t out_len);
static bool get_content_type(httpd_req_t *req, char *out, size_t out_len);
static const char *find_boundary_param(const char *content_type);
static bool content_type_is_multipart(const char *content_type);
static bool read_body_to_buffer(httpd_req_t *req, char *out, size_t out_len);
static esp_err_t recv_raw_body_to_file(httpd_req_t *req, FILE *f);
static esp_err_t recv_multipart_file_to_file(httpd_req_t *req, FILE *f, const char *boundary);
static uint32_t simple_hash(const char *buf, size_t len);
static bool parse_u32_value(const char *s, uint32_t *out);
static void update_floppy_count(void);
static bool is_graphics_mode(uint8_t mode);

// NOTE: Keep this an 8.3 filename (LFN may be disabled in FatFs), otherwise fopen/stat can fail with EINVAL.
#define DISK1_STAGING_PATH "/sdcard/disk1.tmp"
#define DISK1_FINAL_PATH "/sdcard/disk1.img"
#define DISK2_STAGING_PATH "/sdcard/disk2.tmp"
#define DISK2_FINAL_PATH "/sdcard/disk2.img"

static httpd_handle_t s_server;
static uint8_t s_last_screen_mode = 0xFF;
static bool s_warned_graphics_screen = false;
static uint8_t s_last_fb_mode = 0xFF;

extern const unsigned char _binary_index_html_start[] asm("_binary_index_html_start");
extern const unsigned char _binary_index_html_end[] asm("_binary_index_html_end");

static void log_path_stat(const char *label, const char *path)
{
  struct stat st;
  errno = 0;
  int rc = stat(path, &st);
  int err = errno;
  if (rc == 0)
  {
    ESP_LOGI(TAG, "%s stat ok: path=%s mode=0%o size=%ld", label, path, (unsigned)st.st_mode, (long)st.st_size);
  }
  else
  {
    ESP_LOGW(TAG, "%s stat failed: path=%s errno=%d (%s)", label, path, err, strerror(err));
  }
}

static void log_sdcard_space(const char *path)
{
  uint64_t total = 0;
  uint64_t freeb = 0;
  esp_err_t err = esp_vfs_fat_info(path, &total, &freeb);
  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "fatfs info: path=%s total=%llu free=%llu", path, (unsigned long long)total,
             (unsigned long long)freeb);
  }
  else
  {
    ESP_LOGW(TAG, "fatfs info failed: path=%s err=%d", path, (int)err);
  }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
  const size_t len = (size_t)(_binary_index_html_end - _binary_index_html_start);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, (const char *)_binary_index_html_start, len);
}

static bool form_get_value(const char *body, const char *key, char *out, size_t out_len)
{
  if (!body || !key || !out || out_len == 0)
    return false;

  size_t key_len = strlen(key);
  const char *p = body;
  while (*p)
  {
    if ((p == body || p[-1] == '&') && strncmp(p, key, key_len) == 0 && p[key_len] == '=')
    {
      const char *v = p + key_len + 1;
      size_t n = 0;
      while (v[n] && v[n] != '&' && n + 1 < out_len)
      {
        out[n] = v[n];
        n++;
      }
      out[n] = '\0';
      return true;
    }
    const char *amp = strchr(p, '&');
    if (!amp)
      break;
    p = amp + 1;
  }
  return false;
}

static int hex_value(int ch)
{
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  if (ch >= 'a' && ch <= 'f')
    return 10 + (ch - 'a');
  if (ch >= 'A' && ch <= 'F')
    return 10 + (ch - 'A');
  return -1;
}

static void url_decode_inplace(char *s)
{
  if (!s)
    return;
  char *r = s;
  char *w = s;
  while (*r)
  {
    if (*r == '+')
    {
      *w++ = ' ';
      r++;
      continue;
    }
    if (*r == '%' && r[1] && r[2])
    {
      int hi = hex_value((unsigned char)r[1]);
      int lo = hex_value((unsigned char)r[2]);
      if (hi >= 0 && lo >= 0)
      {
        *w++ = (char)((hi << 4) | lo);
        r += 3;
        continue;
      }
    }
    *w++ = *r++;
  }
  *w = '\0';
}

static bool form_get_value_decoded(const char *body, const char *key, char *out, size_t out_len)
{
  if (!form_get_value(body, key, out, out_len))
  {
    return false;
  }
  url_decode_inplace(out);
  return true;
}

static bool parse_bool_value(const char *s, bool *out)
{
  if (!s || !out)
    return false;

  if (strcmp(s, "1") == 0 || strcasecmp(s, "true") == 0 || strcasecmp(s, "on") == 0 || strcasecmp(s, "yes") == 0)
  {
    *out = true;
    return true;
  }
  if (strcmp(s, "0") == 0 || strcasecmp(s, "false") == 0 || strcasecmp(s, "off") == 0 || strcasecmp(s, "no") == 0)
  {
    *out = false;
    return true;
  }
  return false;
}

static bool parse_float_value(const char *s, float *out)
{
  if (!s || !out)
    return false;
  char *end = NULL;
  float v = strtof(s, &end);
  if (end == s)
    return false;
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
  {
    end++;
  }
  if (*end != '\0')
    return false;
  if (!isfinite(v) || v < 0.0f)
    return false;
  *out = v;
  return true;
}

static esp_err_t api_settings_get_handler(httpd_req_t *req)
{
  app_settings_t s;
  (void)app_settings_get(&s);
  const bool bt_connected = bt_keyboard_is_connected();

  char esc_boot[512];
  json_escape_string(s.boot_image_path, esc_boot, sizeof(esc_boot));

  char esc_c_drive[512];
  json_escape_string(s.c_drive_image_path, esc_c_drive, sizeof(esc_c_drive));

  char esc_a_drive[512];
  json_escape_string(s.a_drive_image_path, esc_a_drive, sizeof(esc_a_drive));

  char esc_b_drive[512];
  json_escape_string(s.b_drive_image_path, esc_b_drive, sizeof(esc_b_drive));

  char resp[1360];
  int n = snprintf(resp, sizeof(resp),
                   "{\"display_fps\":%.6g,\"partial_refresh\":%s,\"pause_cpu_on_refresh\":%s,\"clear_on_bottom\":%s,\"wifi_enabled\":%s,\"bt_keyboard_enabled\":%s,\"gameport_enabled\":%s,\"bt_keyboard_connected\":%s,\"c_drive_dos33_compat\":%s,\"boot_image_path\":\"%s\",\"c_drive_image_path\":\"%s\",\"a_drive_image_path\":\"%s\",\"b_drive_image_path\":\"%s\"}\n",
                   (double)s.display_fps, s.display_partial_refresh ? "true" : "false",
                   s.display_pause_cpu_on_refresh ? "true" : "false",
                   s.display_clear_on_bottom ? "true" : "false",
                   s.wifi_enabled ? "true" : "false",
                   s.bt_keyboard_enabled ? "true" : "false",
                   s.gameport_enabled ? "true" : "false",
                   bt_connected ? "true" : "false",
                   s.c_drive_dos33_compat ? "true" : "false",
                   esc_boot,
                   esc_c_drive,
                   esc_a_drive,
                   esc_b_drive);
  if (n < 0)
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, resp, (size_t)n);
  return ESP_OK;
}

static bool ends_with_img_ext(const char *path)
{
  if (!path)
    return false;
  const char *dot = strrchr(path, '.');
  if (!dot)
    return false;
  return strcasecmp(dot, ".img") == 0;
}

static bool path_is_under_sdcard(const char *path)
{
  return path && strncmp(path, "/sdcard/", 8) == 0;
}

static const char *get_c_drive_image_path(char *out, size_t out_len)
{
  if (!out || out_len == 0)
  {
    return g_c_drive_image_path;
  }

  app_settings_t s;
  (void)app_settings_get(&s);

  const char *path = s.c_drive_image_path[0] ? s.c_drive_image_path : g_c_drive_image_path;
  if (!path_is_under_sdcard(path) || !ends_with_img_ext(path))
  {
    path = g_c_drive_image_path;
  }

  strncpy(out, path, out_len - 1);
  out[out_len - 1] = '\0';
  return out;
}

static bool hd_image_path_is_valid(const char *path)
{
  if (!path || !path[0])
  {
    return false;
  }
  if (strstr(path, "..") != NULL)
  {
    return false;
  }
  return path_is_under_sdcard(path) && ends_with_img_ext(path);
}

static const char *get_hd_image_path_from_req(httpd_req_t *req, const char *body, char *out, size_t out_len)
{
  if (!out || out_len == 0)
  {
    return get_c_drive_image_path(out, out_len);
  }

  char requested[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  bool have = false;
  if (req && get_query_value(req, "image", requested, sizeof(requested)))
  {
    have = true;
  }
  if (!have && body && form_get_value_decoded(body, "image", requested, sizeof(requested)))
  {
    have = true;
  }

  if (have && hd_image_path_is_valid(requested))
  {
    strncpy(out, requested, out_len - 1);
    out[out_len - 1] = '\0';
    ESP_LOGI(TAG, "FM image override: image=%s", out);
    return out;
  }
  if (have)
  {
    ESP_LOGW(TAG, "FM image override ignored: image=%s", requested);
  }

  return get_c_drive_image_path(out, out_len);
}

static bool is_safe_filename_char(char c)
{
  if (c >= 'a' && c <= 'z')
    return true;
  if (c >= 'A' && c <= 'Z')
    return true;
  if (c >= '0' && c <= '9')
    return true;
  return (c == '.' || c == '_' || c == '-');
}

static bool sanitize_filename(const char *in, char *out, size_t out_len)
{
  if (!in || !out || out_len == 0)
    return false;
  out[0] = '\0';

  const char *name = in;
  for (const char *p = in; *p; p++)
  {
    if (*p == '/' || *p == '\\')
    {
      name = p + 1;
    }
  }
  if (!name[0])
    return false;

  size_t o = 0;
  for (const char *p = name; *p && o + 1 < out_len; p++)
  {
    char c = *p;
    if (c == '/' || c == '\\')
      continue;
    if (c == ' ')
      c = '_';
    if ((unsigned char)c < 0x20)
      continue;
    if (!is_safe_filename_char(c))
      c = '_';
    out[o++] = c;
  }
  out[o] = '\0';

  if (out[0] == '\0')
    return false;
  if (strcmp(out, ".") == 0 || strcmp(out, "..") == 0)
    return false;
  if (strstr(out, "..") != NULL)
    return false;
  return true;
}

static char to_upper_ascii(char c)
{
  if (c >= 'a' && c <= 'z')
    return (char)(c - ('a' - 'A'));
  return c;
}

static bool make_8dot3_filename(const char *in, char *out, size_t out_len)
{
  if (!in || !out || out_len == 0)
    return false;

  char tmp[96];
  if (!sanitize_filename(in, tmp, sizeof(tmp)))
  {
    return false;
  }

  const char *dot = strrchr(tmp, '.');
  const char *base_in = tmp;
  const char *ext_in = NULL;
  if (dot && dot != tmp && dot[1] != '\0')
  {
    ext_in = dot + 1;
  }

  char base[9];
  char ext[4];
  size_t bi = 0;
  for (const char *p = base_in; *p && (ext_in == NULL || p < dot) && bi < 8; p++)
  {
    char c = to_upper_ascii(*p);
    if (c == '.')
      break;
    if (!is_safe_filename_char(c))
      continue;
    if (c == '-')
      c = '_';
    base[bi++] = c;
  }
  base[bi] = '\0';

  if (base[0] == '\0')
  {
    strncpy(base, "FILE", sizeof(base));
    base[sizeof(base) - 1] = '\0';
  }

  size_t ei = 0;
  if (ext_in)
  {
    for (const char *p = ext_in; *p && ei < 3; p++)
    {
      char c = to_upper_ascii(*p);
      if (!is_safe_filename_char(c))
        continue;
      if (c == '.')
        continue;
      if (c == '-')
        c = '_';
      ext[ei++] = c;
    }
  }
  ext[ei] = '\0';

  if (ext[0])
  {
    if (snprintf(out, out_len, "%s.%s", base, ext) < 0)
      return false;
  }
  else
  {
    if (snprintf(out, out_len, "%s", base) < 0)
      return false;
  }

  // Final guard: enforce 8.3 length.
  const size_t n = strlen(out);
  if (n == 0 || n > 12)
    return false;
  return true;
}

static bool file_exists_path(const char *path)
{
  struct stat st;
  return path && (stat(path, &st) == 0);
}

static bool floppy_image_is_acceptable(const char *path, long *out_size)
{
  if (!path)
    return false;
  struct stat st;
  if (stat(path, &st) != 0 || (st.st_mode & S_IFREG) == 0)
    return false;
  if (st.st_size < 512 || (st.st_size % 512) != 0)
    return false;
  if (st.st_size > 1474560)
    return false;
  if (out_size)
    *out_size = (long)st.st_size;
  return true;
}

static bool choose_unique_sd_filename_8dot3(const char *requested_name, char *out_name, size_t out_name_len)
{
  if (!requested_name || !out_name || out_name_len == 0)
    return false;

  char name83[16] = {0};
  if (!make_8dot3_filename(requested_name, name83, sizeof(name83)))
  {
    return false;
  }

  char path[160];
  int pn = snprintf(path, sizeof(path), "/sdcard/%s", name83);
  if (pn < 0 || (size_t)pn >= sizeof(path))
  {
    return false;
  }

  if (!file_exists_path(path))
  {
    strncpy(out_name, name83, out_name_len - 1);
    out_name[out_name_len - 1] = '\0';
    return true;
  }

  // Collision: use ~N scheme.
  char base[9] = {0};
  char ext[4] = {0};
  const char *dot = strrchr(name83, '.');
  if (dot)
  {
    size_t bn = (size_t)(dot - name83);
    if (bn > 8)
      bn = 8;
    memcpy(base, name83, bn);
    base[bn] = '\0';
    strncpy(ext, dot + 1, sizeof(ext) - 1);
    ext[sizeof(ext) - 1] = '\0';
  }
  else
  {
    strncpy(base, name83, sizeof(base) - 1);
    base[sizeof(base) - 1] = '\0';
  }

  for (int i = 1; i <= 99; i++)
  {
    char suffix[8];
    int sn = snprintf(suffix, sizeof(suffix), "~%d", i);
    if (sn < 0 || (size_t)sn >= sizeof(suffix))
      continue;

    const size_t suffix_len = strlen(suffix);
    size_t base_len = strlen(base);
    if (base_len + suffix_len > 8)
    {
      base_len = 8 - suffix_len;
    }

    char cand[16];
    if (ext[0])
    {
      int cn = snprintf(cand, sizeof(cand), "%.*s%s.%s", (int)base_len, base, suffix, ext);
      if (cn < 0 || (size_t)cn >= sizeof(cand))
        continue;
    }
    else
    {
      int cn = snprintf(cand, sizeof(cand), "%.*s%s", (int)base_len, base, suffix);
      if (cn < 0 || (size_t)cn >= sizeof(cand))
        continue;
    }

    pn = snprintf(path, sizeof(path), "/sdcard/%s", cand);
    if (pn < 0 || (size_t)pn >= sizeof(path))
      continue;

    if (!file_exists_path(path))
    {
      strncpy(out_name, cand, out_name_len - 1);
      out_name[out_name_len - 1] = '\0';
      return true;
    }
  }

  return false;
}

static esp_err_t api_sd_upload_post_handler(httpd_req_t *req)
{
  if (!sdcard_is_mounted())
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD not mounted");
    return ESP_FAIL;
  }
  if (req->content_len <= 0)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  char name_q[128] = {0};
  if (!get_query_value(req, "name", name_q, sizeof(name_q)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
    return ESP_FAIL;
  }

  char safe_name[16] = {0};
  if (!choose_unique_sd_filename_8dot3(name_q, safe_name, sizeof(safe_name)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid name (need 8.3)");
    return ESP_FAIL;
  }

  char dest_path[128];
  int pn = snprintf(dest_path, sizeof(dest_path), "/sdcard/%s", safe_name);
  if (pn < 0 || (size_t)pn >= sizeof(dest_path))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name too long");
    return ESP_FAIL;
  }

  FILE *f = fopen(dest_path, "wb");
  if (!f)
  {
    int err = errno;
    ESP_LOGE(TAG, "SD upload open failed: path=%s errno=%d (%s)", dest_path, err, strerror(err));
    log_path_stat("sdcard_root", "/sdcard");
    log_path_stat("dest_path", dest_path);
    log_sdcard_space("/sdcard");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
    return ESP_FAIL;
  }

  char content_type[128] = {0};
  bool has_ct = get_content_type(req, content_type, sizeof(content_type));
  esp_err_t rx_err = ESP_FAIL;
  if (has_ct && content_type_is_multipart(content_type))
  {
    const char *boundary = find_boundary_param(content_type);
    rx_err = recv_multipart_file_to_file(req, f, boundary);
  }
  else
  {
    rx_err = recv_raw_body_to_file(req, f);
  }

  fflush(f);
  fclose(f);

  if (rx_err != ESP_OK)
  {
    remove(dest_path);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive/write failed");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char esc_path[256];
  json_escape_string(dest_path, esc_path, sizeof(esc_path));
  char resp[320];
  int n = snprintf(resp, sizeof(resp), "{\"path\":\"%s\",\"bytes\":%d}\n", esc_path, (int)req->content_len);
  if (n < 0 || (size_t)n >= sizeof(resp))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }

  httpd_resp_send(req, resp, (size_t)n);
  return ESP_OK;
}

typedef struct boot_images_ctx
{
  httpd_req_t *req;
  bool first;
  int emitted;
  int max_images;
  int max_depth;
  long long min_size;
  bool ok;
} boot_images_ctx_t;

static bool boot_images_emit(boot_images_ctx_t *ctx, const char *path, long long size)
{
  if (!ctx || !ctx->req || !path || !path[0] || !ctx->ok)
    return false;

  char esc_path[512];
  json_escape_string(path, esc_path, sizeof(esc_path));

  if (!ctx->first)
  {
    if (httpd_resp_sendstr_chunk(ctx->req, ",") != ESP_OK)
    {
      ctx->ok = false;
      return false;
    }
  }
  ctx->first = false;

  if (httpd_resp_sendstr_chunk(ctx->req, "{\"path\":\"") != ESP_OK ||
      httpd_resp_sendstr_chunk(ctx->req, esc_path) != ESP_OK ||
      httpd_resp_sendstr_chunk(ctx->req, "\",\"size\":") != ESP_OK)
  {
    ctx->ok = false;
    return false;
  }

  char size_buf[32];
  int n = snprintf(size_buf, sizeof(size_buf), "%lld", size);
  if (n < 0 || (size_t)n >= sizeof(size_buf))
  {
    ctx->ok = false;
    return false;
  }

  if (httpd_resp_sendstr_chunk(ctx->req, size_buf) != ESP_OK ||
      httpd_resp_sendstr_chunk(ctx->req, "}\n") != ESP_OK)
  {
    ctx->ok = false;
    return false;
  }

  ctx->emitted++;
  return true;
}

static void boot_images_scan_dir(boot_images_ctx_t *ctx, const char *dir_path, int depth)
{
  if (!ctx || !ctx->ok || !dir_path || ctx->emitted >= ctx->max_images)
    return;
  if (depth > ctx->max_depth)
    return;

  DIR *dir = opendir(dir_path);
  if (!dir)
  {
    return;
  }

  for (;;)
  {
    struct dirent *ent = readdir(dir);
    if (!ent || !ctx->ok || ctx->emitted >= ctx->max_images)
      break;

    const char *name = ent->d_name;
    if (!name || !name[0] || strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
      continue;

    char full[256];
    int pn = snprintf(full, sizeof(full), "%s/%s", dir_path, name);
    if (pn < 0 || (size_t)pn >= sizeof(full))
    {
      continue;
    }

    struct stat st;
    if (stat(full, &st) != 0)
    {
      continue;
    }

    if ((st.st_mode & S_IFDIR) != 0)
    {
      boot_images_scan_dir(ctx, full, depth + 1);
      continue;
    }

    if ((st.st_mode & S_IFREG) == 0)
    {
      continue;
    }

    if (!ends_with_img_ext(full) || !path_is_under_sdcard(full))
    {
      continue;
    }

    if (ctx->min_size > 0 && (long long)st.st_size < ctx->min_size)
    {
      continue;
    }

    (void)boot_images_emit(ctx, full, (long long)st.st_size);
  }

  closedir(dir);
}

static esp_err_t api_boot_images_get_handler(httpd_req_t *req)
{
  app_settings_t s;
  (void)app_settings_get(&s);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char esc_current[512];
  json_escape_string(s.boot_image_path, esc_current, sizeof(esc_current));

  if (httpd_resp_sendstr_chunk(req, "{\"current\":\"") != ESP_OK ||
      httpd_resp_sendstr_chunk(req, esc_current) != ESP_OK ||
      httpd_resp_sendstr_chunk(req, "\",\"images\":[") != ESP_OK)
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "send failed");
    return ESP_FAIL;
  }

  boot_images_ctx_t ctx = {
      .req = req,
      .first = true,
      .emitted = 0,
      .max_images = 256,
      .max_depth = 6,
      .min_size = 0,
      .ok = true,
  };
  boot_images_scan_dir(&ctx, "/sdcard", 0);

  httpd_resp_sendstr_chunk(req, "]}\n");
  httpd_resp_sendstr_chunk(req, NULL);
  return ctx.ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t api_hdd_images_get_handler(httpd_req_t *req)
{
  app_settings_t s;
  (void)app_settings_get(&s);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char esc_current[512];
  json_escape_string(s.c_drive_image_path, esc_current, sizeof(esc_current));

  if (httpd_resp_sendstr_chunk(req, "{\"current\":\"") != ESP_OK ||
      httpd_resp_sendstr_chunk(req, esc_current) != ESP_OK ||
      httpd_resp_sendstr_chunk(req, "\",\"images\":[") != ESP_OK)
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "send failed");
    return ESP_FAIL;
  }

  boot_images_ctx_t ctx = {
      .req = req,
      .first = true,
      .emitted = 0,
      .max_images = 256,
      .max_depth = 6,
      .min_size = 16ll * 1024ll * 1024ll,
      .ok = true,
  };
  boot_images_scan_dir(&ctx, "/sdcard", 0);

  httpd_resp_sendstr_chunk(req, "]}\n");
  httpd_resp_sendstr_chunk(req, NULL);
  return ctx.ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t api_hdd_generate_post_handler(httpd_req_t *req)
{
  if (!sdcard_is_mounted())
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD not mounted");
    return ESP_FAIL;
  }

  char body[512];
  if (!read_body_to_buffer(req, body, sizeof(body)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    return ESP_FAIL;
  }

  char path_buf[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  if (!form_get_value_decoded(body, "path", path_buf, sizeof(path_buf)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
    return ESP_FAIL;
  }
  if (!path_is_under_sdcard(path_buf) || !ends_with_img_ext(path_buf) || strstr(path_buf, "..") != NULL)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid path");
    return ESP_FAIL;
  }

  char size_buf[32] = {0};
  bool have_size = form_get_value(body, "size_mb", size_buf, sizeof(size_buf)) ||
                   form_get_value(body, "size", size_buf, sizeof(size_buf));
  if (!have_size)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing size_mb");
    return ESP_FAIL;
  }

  uint32_t size_mb = 0;
  if (!parse_u32_value(size_buf, &size_mb) || size_mb == 0)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid size_mb");
    return ESP_FAIL;
  }

  char dos33_buf[16] = {0};
  bool dos33 = false;
  bool have_dos33 = form_get_value(body, "dos33", dos33_buf, sizeof(dos33_buf)) ||
                    form_get_value(body, "c_drive_dos33", dos33_buf, sizeof(dos33_buf)) ||
                    form_get_value(body, "c_drive_dos33_compat", dos33_buf, sizeof(dos33_buf));
  if (have_dos33)
  {
    if (!parse_bool_value(dos33_buf, &dos33))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid dos33");
      return ESP_FAIL;
    }
  }

  if (file_exists_path(path_buf))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "file exists");
    return ESP_FAIL;
  }

  const uint64_t size_bytes = (uint64_t)size_mb * 1024ull * 1024ull;
  const int prev_dos33 = g_c_drive_dos33_compat;
  g_c_drive_dos33_compat = dos33 ? 1 : 0;
  const bool ok = hdd_image_ensure_fat16(path_buf, size_bytes);
  g_c_drive_dos33_compat = prev_dos33;
  if (!ok)
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "create failed");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char esc_path[512];
  json_escape_string(path_buf, esc_path, sizeof(esc_path));
  char resp[640];
  int n = snprintf(resp, sizeof(resp), "{\"path\":\"%s\",\"size_mb\":%lu}\n", esc_path, (unsigned long)size_mb);
  if (n < 0 || (size_t)n >= sizeof(resp))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }
  httpd_resp_send(req, resp, (size_t)n);
  return ESP_OK;
}

static bool parse_u8_value(const char *s, uint8_t *out)
{
  if (!s || !out)
    return false;

  char *end = NULL;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s)
    return false;
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
  {
    end++;
  }
  if (*end != '\0')
    return false;
  if (v > 255UL)
    return false;
  *out = (uint8_t)v;
  return true;
}

static bool parse_u32_value(const char *s, uint32_t *out)
{
  if (!s || !out)
    return false;

  char *end = NULL;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s)
    return false;
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
  {
    end++;
  }
  if (*end != '\0')
    return false;
  if (v > 0xFFFFFFFFul)
    return false;
  *out = (uint32_t)v;
  return true;
}

static bool parse_i8_value(const char *s, int8_t *out)
{
  if (!s || !out)
    return false;

  char *end = NULL;
  long v = strtol(s, &end, 0);
  if (end == s)
    return false;
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
  {
    end++;
  }
  if (*end != '\0')
    return false;

  if (v < -127L || v > 127L)
    return false;
  *out = (int8_t)v;
  return true;
}

static size_t json_escape_string(const char *src, char *dst, size_t dst_len)
{
  if (!dst || dst_len == 0)
    return 0;
  if (!src)
  {
    dst[0] = '\0';
    return 0;
  }

  size_t o = 0;
  for (const unsigned char *p = (const unsigned char *)src; *p; p++)
  {
    const unsigned char c = *p;
    if (c == '\\' || c == '"')
    {
      if (o + 2 >= dst_len)
        break;
      dst[o++] = '\\';
      dst[o++] = (char)c;
    }
    else if (c == '\n')
    {
      if (o + 2 >= dst_len)
        break;
      dst[o++] = '\\';
      dst[o++] = 'n';
    }
    else if (c == '\r')
    {
      if (o + 2 >= dst_len)
        break;
      dst[o++] = '\\';
      dst[o++] = 'r';
    }
    else if (c == '\t')
    {
      if (o + 2 >= dst_len)
        break;
      dst[o++] = '\\';
      dst[o++] = 't';
    }
    else if (c < 0x20)
    {
      if (o + 6 >= dst_len)
        break;
      static const char hex[] = "0123456789ABCDEF";
      dst[o++] = '\\';
      dst[o++] = 'u';
      dst[o++] = '0';
      dst[o++] = '0';
      dst[o++] = hex[(c >> 4) & 0x0F];
      dst[o++] = hex[c & 0x0F];
    }
    else
    {
      if (o + 1 >= dst_len)
        break;
      dst[o++] = (char)c;
    }
  }
  dst[o] = '\0';
  return o;
}

static bool is_graphics_mode(uint8_t mode)
{
  return (mode == 0x04 || mode == 0x05 || mode == 0x06 || mode == 0x13);
}

static esp_err_t api_terminal_screen_get_handler(httpd_req_t *req)
{
  const uint8_t mode = (uint8_t)(video_get_mode() & 0x7F);
  const bool is_graphics = is_graphics_mode(mode);
  const bool bt_connected = bt_keyboard_is_connected();

  if (mode != s_last_screen_mode)
  {
    ESP_LOGI(TAG, "Terminal screen: mode=%02X (%s)", mode, is_graphics ? "graphics" : "text");
    s_last_screen_mode = mode;
    if (!is_graphics)
    {
      s_warned_graphics_screen = false;
    }
  }
  if (is_graphics && !s_warned_graphics_screen)
  {
    ESP_LOGW(TAG, "Terminal screen requested in graphics mode; use /api/terminal/framebuffer for visuals");
    s_warned_graphics_screen = true;
  }

  uint16_t cols = mem_read_word(BDA_VIDEO_COLS);
  if (cols == 0 || cols > 80)
  {
    cols = 80;
  }
  uint8_t rows = (uint8_t)(mem_read_byte(BDA_VIDEO_ROWS) + 1);
  if (rows == 0 || rows > 25)
  {
    rows = 25;
  }

  const size_t raw_cap = (size_t)rows * ((size_t)cols + 1) + 1;
  char *raw = (char *)malloc(raw_cap);
  if (!raw)
  {
    ESP_LOGE(TAG, "Terminal screen OOM: raw_cap=%u", (unsigned)raw_cap);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  if (!video_get_terminal_snapshot(raw, raw_cap, rows, cols))
  {
    ESP_LOGE(TAG, "Terminal screen snapshot failed: mode=%02X rows=%u cols=%u", mode, rows, cols);
    free(raw);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "snapshot failed");
    return ESP_FAIL;
  }

  // Compute a cheap hash over the raw text so clients can avoid re-downloading unchanged screens.
  size_t raw_len = strlen(raw);
  uint32_t version = simple_hash(raw, raw_len);
  ESP_LOGD(TAG, "Terminal screen: mode=%02X rows=%u cols=%u hash=%u", mode, rows, cols, (unsigned)version);

  // Check client version parameter
  char vbuf[32] = {0};
  if (get_query_value(req, "v", vbuf, sizeof(vbuf)))
  {
    unsigned long client_v = strtoul(vbuf, NULL, 10);
    if ((uint32_t)client_v == version)
    {
      // Unchanged
      ESP_LOGD(TAG, "Terminal screen unchanged: version=%u", (unsigned)version);
      free(raw);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Cache-Control", "no-store");
      char unchanged[160];
      int un = snprintf(unchanged, sizeof(unchanged),
                        "{\"unchanged\":true,\"mode\":%u,\"graphics\":%s,\"bt_keyboard_connected\":%s}\n",
                        (unsigned)mode, is_graphics ? "true" : "false",
                        bt_connected ? "true" : "false");
      if (un < 0 || (size_t)un >= sizeof(unchanged))
      {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
        return ESP_FAIL;
      }
      httpd_resp_send(req, unchanged, (size_t)un);
      return ESP_OK;
    }
  }

  const size_t esc_cap = raw_cap * 6 + 1;
  char *escaped = (char *)malloc(esc_cap);
  if (!escaped)
  {
    ESP_LOGE(TAG, "Terminal screen OOM: esc_cap=%u", (unsigned)esc_cap);
    free(raw);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }
  (void)json_escape_string(raw, escaped, esc_cap);

  uint8_t cursor_row = 0;
  uint8_t cursor_col = 0;
  video_get_cursor_pos(&cursor_row, &cursor_col);

  const size_t resp_cap = strlen(escaped) + 320;
  char *resp = (char *)malloc(resp_cap);
  if (!resp)
  {
    ESP_LOGE(TAG, "Terminal screen OOM: resp_cap=%u", (unsigned)resp_cap);
    free(escaped);
    free(raw);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  int n = snprintf(resp, resp_cap,
                   "{\"mode\":%u,\"cols\":%u,\"rows\":%u,\"cursor_row\":%u,\"cursor_col\":%u,\"graphics\":%s,\"bt_keyboard_connected\":%s,\"version\":%u,\"text\":\"%s\"}\n",
                   (unsigned)mode, (unsigned)cols, (unsigned)rows, (unsigned)cursor_row, (unsigned)cursor_col,
                   is_graphics ? "true" : "false",
                   bt_connected ? "true" : "false",
                   (unsigned)version, escaped);
  free(escaped);
  free(raw);
  if (n < 0 || (size_t)n >= resp_cap)
  {
    free(resp);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, resp, (size_t)n);
  free(resp);
  return ESP_OK;
} 

static esp_err_t api_terminal_framebuffer_get_handler(httpd_req_t *req)
{
  const uint8_t mode = (uint8_t)(video_get_mode() & 0x7F);
  bool is_graphics = is_graphics_mode(mode);

  if (mode != s_last_fb_mode)
  {
    ESP_LOGI(TAG, "Terminal framebuffer: mode=%02X (%s)", mode, is_graphics ? "graphics" : "text");
    s_last_fb_mode = mode;
  }
  ESP_LOGD(TAG, "Terminal framebuffer: mode=%02X graphics=%d", mode, is_graphics ? 1 : 0);

  if (!is_graphics)
  {
    ESP_LOGD(TAG, "Terminal framebuffer requested in text mode");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, "{\"graphics\":false}\n", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }

  uint8_t *fb = epd_get_framebuffer();
  if (!fb)
  {
    ESP_LOGE(TAG, "Terminal framebuffer: epd_get_framebuffer returned NULL");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no framebuffer");
    return ESP_FAIL;
  }

  const int src_w = EPD_WIDTH;
  const int src_h = EPD_HEIGHT;
  uint16_t dst_w = 320;
  uint16_t dst_h = 200;

  char wbuf[16] = {0};
  if (get_query_value(req, "w", wbuf, sizeof(wbuf)))
  {
    uint32_t w = 0;
    if (parse_u32_value(wbuf, &w))
    {
      if (w < 16) w = 16;
      if (w > (uint32_t)src_w) w = (uint32_t)src_w;
      dst_w = (uint16_t)w;
    }
  }

  char hbuf[16] = {0};
  if (get_query_value(req, "h", hbuf, sizeof(hbuf)))
  {
    uint32_t h = 0;
    if (parse_u32_value(hbuf, &h))
    {
      if (h < 16) h = 16;
      if (h > (uint32_t)src_h) h = (uint32_t)src_h;
      dst_h = (uint16_t)h;
    }
  }

  ESP_LOGD(TAG, "Terminal framebuffer downscale: %dx%d -> %dx%d", src_w, src_h, dst_w, dst_h);

  size_t raw_size = (size_t)dst_w * (size_t)dst_h;
  if (raw_size == 0)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid size");
    return ESP_FAIL;
  }

  uint8_t *down = (uint8_t *)malloc(raw_size);
  if (!down)
  {
    ESP_LOGE(TAG, "Terminal framebuffer OOM: downscale=%dx%d", dst_w, dst_h);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  for (int y = 0; y < (int)dst_h; y++)
  {
    int sy = (y * src_h) / (int)dst_h;
    for (int x = 0; x < (int)dst_w; x++)
    {
      int sx = (x * src_w) / (int)dst_w;
      down[(size_t)y * dst_w + x] = fb[sy * src_w + sx];
    }
  }

  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t b64_size = ((raw_size + 2) / 3) * 4;
  char *b64_buf = (char *)malloc(b64_size + 1);
  if (!b64_buf)
  {
    ESP_LOGE(TAG, "Terminal framebuffer OOM: b64_size=%u", (unsigned)b64_size);
    free(down);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  size_t bi = 0;
  for (size_t i = 0; i < raw_size; i += 3)
  {
    uint32_t n = ((uint32_t)down[i]) << 16;
    if (i + 1 < raw_size) n |= ((uint32_t)down[i + 1]) << 8;
    if (i + 2 < raw_size) n |= (uint32_t)down[i + 2];
    b64_buf[bi++] = b64[(n >> 18) & 0x3F];
    b64_buf[bi++] = b64[(n >> 12) & 0x3F];
    b64_buf[bi++] = (i + 1 < raw_size) ? b64[(n >> 6) & 0x3F] : '=';
    b64_buf[bi++] = (i + 2 < raw_size) ? b64[n & 0x3F] : '=';
  }
  b64_buf[bi] = '\0';
  free(down);

  size_t resp_cap = b64_size + 128;
  char *resp = (char *)malloc(resp_cap);
  if (!resp)
  {
    ESP_LOGE(TAG, "Terminal framebuffer OOM: resp_cap=%u", (unsigned)resp_cap);
    free(b64_buf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    return ESP_FAIL;
  }

  int rn = snprintf(resp, resp_cap, "{\"graphics\":true,\"width\":%d,\"height\":%d,\"data\":\"%s\"}\n", dst_w, dst_h, b64_buf);
  free(b64_buf);
  if (rn < 0 || (size_t)rn >= resp_cap)
  {
    free(resp);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, resp, (size_t)rn);
  free(resp);
  return ESP_OK;
}

static esp_err_t api_terminal_key_post_handler(httpd_req_t *req)
{
  if (req->content_len <= 0 || req->content_len > 256)
  {
    ESP_LOGW(TAG, "Terminal key invalid content length: %d", req->content_len);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content length");
    return ESP_FAIL;
  }

  char body[257];
  int remaining = req->content_len;
  int off = 0;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, body + off, remaining);
    if (recv_len <= 0)
    {
      ESP_LOGE(TAG, "Terminal key receive failed");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive failed");
      return ESP_FAIL;
    }
    off += recv_len;
    remaining -= recv_len;
  }
  body[off] = '\0';

  char sc_buf[16] = {0};
  char ascii_buf[16] = {0};
  char pressed_buf[16] = {0};
  char ext_buf[16] = {0};
  char numlock_buf[16] = {0};
  bool have_sc = form_get_value(body, "sc", sc_buf, sizeof(sc_buf)) || form_get_value(body, "scancode", sc_buf, sizeof(sc_buf));
  bool have_ascii = form_get_value(body, "ascii", ascii_buf, sizeof(ascii_buf));
  bool have_pressed = form_get_value(body, "pressed", pressed_buf, sizeof(pressed_buf));
  bool have_ext = form_get_value(body, "ext", ext_buf, sizeof(ext_buf)) || form_get_value(body, "extended", ext_buf, sizeof(ext_buf));
  bool have_numlock = form_get_value(body, "numlock", numlock_buf, sizeof(numlock_buf)) ||
                      form_get_value(body, "num", numlock_buf, sizeof(numlock_buf));

  if (!have_pressed)
  {
    ESP_LOGW(TAG, "Terminal key missing pressed");
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing pressed");
    return ESP_FAIL;
  }

  bool pressed = false;
  if (!parse_bool_value(pressed_buf, &pressed))
  {
    ESP_LOGW(TAG, "Terminal key invalid pressed=%s", pressed_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid pressed");
    return ESP_FAIL;
  }

  uint8_t scancode = 0;
  if (have_sc && !parse_u8_value(sc_buf, &scancode))
  {
    ESP_LOGW(TAG, "Terminal key invalid scancode=%s", sc_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid scancode");
    return ESP_FAIL;
  }

  uint8_t ascii = 0;
  if (have_ascii && !parse_u8_value(ascii_buf, &ascii))
  {
    ESP_LOGW(TAG, "Terminal key invalid ascii=%s", ascii_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid ascii");
    return ESP_FAIL;
  }

  bool ext = false;
  if (have_ext && !parse_bool_value(ext_buf, &ext))
  {
    ESP_LOGW(TAG, "Terminal key invalid ext=%s", ext_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid ext");
    return ESP_FAIL;
  }

  bool numlock = false;
  if (have_numlock && !parse_bool_value(numlock_buf, &numlock))
  {
    ESP_LOGW(TAG, "Terminal key invalid numlock=%s", numlock_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid numlock");
    return ESP_FAIL;
  }
  if (have_numlock)
  {
    bios_set_numlock(numlock);
  }

  ESP_LOGI(TAG, "Terminal key: sc=%u ascii=%u pressed=%d ext=%d num=%d",
           (unsigned)scancode, (unsigned)ascii, pressed ? 1 : 0, ext ? 1 : 0,
           have_numlock ? (numlock ? 1 : 0) : -1);

  if (scancode != 0) {
    bios_kbd_update_flags(scancode, pressed, ext);
  }

  bool is_modifier = (scancode == 0x2A || scancode == 0x36 || scancode == 0x1D || scancode == 0x38);
  if (pressed && !is_modifier)
  {
    bios_key_enqueue(scancode, ascii, ext);
  }
  if (scancode != 0)
  {
    uint8_t code = pressed ? scancode : (uint8_t)(scancode | 0x80);
    kb_set_scancode_ext(code, ext);
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t api_terminal_mouse_post_handler(httpd_req_t *req)
{
  if (req->content_len <= 0 || req->content_len > 256)
  {
    ESP_LOGW(TAG, "Terminal mouse invalid content length: %d", req->content_len);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content length");
    return ESP_FAIL;
  }

  char body[257];
  int remaining = req->content_len;
  int off = 0;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, body + off, remaining);
    if (recv_len <= 0)
    {
      ESP_LOGE(TAG, "Terminal mouse receive failed");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive failed");
      return ESP_FAIL;
    }
    off += recv_len;
    remaining -= recv_len;
  }
  body[off] = '\0';

  char dx_buf[16] = {0};
  char dy_buf[16] = {0};
  char buttons_buf[16] = {0};
  bool have_dx = form_get_value(body, "dx", dx_buf, sizeof(dx_buf));
  bool have_dy = form_get_value(body, "dy", dy_buf, sizeof(dy_buf));
  bool have_buttons = form_get_value(body, "buttons", buttons_buf, sizeof(buttons_buf));

  int8_t dx = 0;
  if (have_dx && !parse_i8_value(dx_buf, &dx))
  {
    ESP_LOGW(TAG, "Terminal mouse invalid dx=%s", dx_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid dx");
    return ESP_FAIL;
  }

  int8_t dy = 0;
  if (have_dy && !parse_i8_value(dy_buf, &dy))
  {
    ESP_LOGW(TAG, "Terminal mouse invalid dy=%s", dy_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid dy");
    return ESP_FAIL;
  }

  uint8_t buttons = 0;
  if (have_buttons && !parse_u8_value(buttons_buf, &buttons))
  {
    ESP_LOGW(TAG, "Terminal mouse invalid buttons=%s", buttons_buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid buttons");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Terminal mouse: dx=%d dy=%d buttons=0x%02X", (int)dx, (int)dy, (unsigned)buttons);

  mouse_ps2_enqueue(dx, dy, (uint8_t)(buttons & 0x07));

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static bool get_query_value(httpd_req_t *req, const char *key, char *out, size_t out_len)
{
  if (!req || !key || !out || out_len == 0)
  {
    return false;
  }
  out[0] = '\0';

  char query[256];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
  {
    return false;
  }

  if (httpd_query_key_value(query, key, out, out_len) != ESP_OK)
  {
    return false;
  }
  url_decode_inplace(out);
  return true;
}

static uint32_t simple_hash(const char *buf, size_t len)
{
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < len; i++)
  {
    h ^= (uint8_t)buf[i];
    h *= 16777619u;
  }
  return h;
}

typedef struct hd_list_ctx
{
  httpd_req_t *req;
  bool first;
  bool ok;
  int count;
} hd_list_ctx_t;

static bool hd_list_emit(const fat16_dirent_info_t *info, void *arg)
{
  hd_list_ctx_t *ctx = (hd_list_ctx_t *)arg;
  if (!ctx || !ctx->req || !info)
  {
    return false;
  }

  char esc_name[32];
  json_escape_string(info->name, esc_name, sizeof(esc_name));

  char line[192];
  int n = snprintf(line, sizeof(line),
                   "%s{\"name\":\"%s\",\"is_dir\":%s,\"size\":%lu,\"date\":%u,\"time\":%u}\n",
                   ctx->first ? "" : ",",
                   esc_name,
                   info->is_dir ? "true" : "false",
                   (unsigned long)info->size,
                   (unsigned)info->fat_date,
                   (unsigned)info->fat_time);
  if (n < 0 || (size_t)n >= sizeof(line))
  {
    ctx->ok = false;
    return false;
  }

  if (httpd_resp_sendstr_chunk(ctx->req, line) != ESP_OK)
  {
    ctx->ok = false;
    return false;
  }

  ctx->first = false;
  ctx->count++;
  return true;
}

static esp_err_t api_hd_list_get_handler(httpd_req_t *req)
{
  char path[256] = "/";
  (void)get_query_value(req, "path", path, sizeof(path));

  char image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  const char *img_path = get_hd_image_path_from_req(req, NULL, image_path, sizeof(image_path));
  ESP_LOGI(TAG, "FM list: image=%s path=%s", img_path, path);

  fat16_image_t *img = NULL;
  char errbuf[64] = {0};
  if (!fat16_image_open(&img, img_path, false, errbuf, sizeof(errbuf)))
  {
    ESP_LOGW(TAG, "FM list open failed: image=%s err=%s", img_path, errbuf);
    const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
    if (hdd_image_ensure_fat16(img_path, size_bytes) &&
        fat16_image_open(&img, img_path, false, errbuf, sizeof(errbuf)))
    {
      // ok (created)
    }
    else
    {
      ESP_LOGW(TAG, "FM list create failed: image=%s err=%s", img_path, errbuf);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "open failed");
      return ESP_FAIL;
    }
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  char esc_path[300];
  json_escape_string(path, esc_path, sizeof(esc_path));

  char header[360];
  int hn = snprintf(header, sizeof(header), "{\"path\":\"%s\",\"entries\":[", esc_path);
  if (hn < 0 || (size_t)hn >= sizeof(header))
  {
    fat16_image_close(img);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "format error");
    return ESP_FAIL;
  }
  if (httpd_resp_sendstr_chunk(req, header) != ESP_OK)
  {
    fat16_image_close(img);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "send failed");
    return ESP_FAIL;
  }

  hd_list_ctx_t ctx = {.req = req, .first = true, .ok = true, .count = 0};
  bool ok = fat16_image_list_dir(img, path, hd_list_emit, &ctx, errbuf, sizeof(errbuf));
  fat16_image_close(img);

  if (!ok || !ctx.ok)
  {
    ESP_LOGW(TAG, "FM list failed: image=%s path=%s err=%s", img_path, path, errbuf[0] ? errbuf : "list failed");
    char esc_err[128];
    json_escape_string(errbuf[0] ? errbuf : "list failed", esc_err, sizeof(esc_err));
    char tail[196];
    int tn = snprintf(tail, sizeof(tail), "],\"error\":\"%s\"}\n", esc_err);
    if (tn > 0 && (size_t)tn < sizeof(tail))
    {
      httpd_resp_sendstr_chunk(req, tail);
    }
    else
    {
      httpd_resp_sendstr_chunk(req, "]}\n");
    }
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
  }

  ESP_LOGI(TAG, "FM list ok: image=%s path=%s entries=%d", img_path, path, ctx.count);
  httpd_resp_sendstr_chunk(req, "]}\n");
  httpd_resp_sendstr_chunk(req, NULL);
  return ESP_OK;
}

static bool read_body_to_buffer(httpd_req_t *req, char *out, size_t out_len)
{
  if (!req || !out || out_len == 0)
  {
    return false;
  }
  if (req->content_len <= 0 || (size_t)req->content_len >= out_len)
  {
    return false;
  }
  int remaining = req->content_len;
  int off = 0;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, out + off, remaining);
    if (recv_len <= 0)
    {
      return false;
    }
    off += recv_len;
    remaining -= recv_len;
  }
  out[off] = '\0';
  return true;
}

static esp_err_t api_hd_delete_post_handler(httpd_req_t *req)
{
  char body[512];
  if (!read_body_to_buffer(req, body, sizeof(body)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    return ESP_FAIL;
  }

  char path[256] = {0};
  if (!form_get_value_decoded(body, "path", path, sizeof(path)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  char image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  const char *img_path = get_hd_image_path_from_req(req, body, image_path, sizeof(image_path));
  ESP_LOGI(TAG, "FM delete: image=%s path=%s", img_path, path);

  fat16_image_t *img = NULL;
  char errbuf[64] = {0};
  bool ok = fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  if (!ok)
  {
    const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
    ok = hdd_image_ensure_fat16(img_path, size_bytes) &&
         fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  }
  ok = ok && fat16_image_delete(img, path, errbuf, sizeof(errbuf));
  if (img)
  {
    fat16_image_close(img);
  }
  system_cpu_pause_release(5000);

  if (!ok)
  {
    ESP_LOGW(TAG, "FM delete failed: image=%s path=%s err=%s", img_path, path, errbuf[0] ? errbuf : "delete failed");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "delete failed");
    return ESP_FAIL;
  }

  (void)disk_mark_media_changed_for_path(img_path);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "FM delete ok: image=%s path=%s", img_path, path);
  return ESP_OK;
}

static esp_err_t api_hd_rename_post_handler(httpd_req_t *req)
{
  char body[512];
  if (!read_body_to_buffer(req, body, sizeof(body)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    return ESP_FAIL;
  }

  char path[256] = {0};
  char name[64] = {0};
  if (!form_get_value_decoded(body, "path", path, sizeof(path)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing path");
    return ESP_FAIL;
  }
  if (!form_get_value_decoded(body, "name", name, sizeof(name)) &&
      !form_get_value_decoded(body, "new_name", name, sizeof(name)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  char image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  const char *img_path = get_hd_image_path_from_req(req, body, image_path, sizeof(image_path));

  fat16_image_t *img = NULL;
  char errbuf[64] = {0};
  bool ok = fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  if (!ok)
  {
    const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
    ok = hdd_image_ensure_fat16(img_path, size_bytes) &&
         fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  }
  ok = ok && fat16_image_rename(img, path, name, errbuf, sizeof(errbuf));
  if (img)
  {
    fat16_image_close(img);
  }
  system_cpu_pause_release(5000);

  if (!ok)
  {
    ESP_LOGW(TAG, "FM rename failed: image=%s path=%s new=%s err=%s", img_path, path, name, errbuf[0] ? errbuf : "rename failed");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "rename failed");
    return ESP_FAIL;
  }

  (void)disk_mark_media_changed_for_path(img_path);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "FM rename ok: image=%s path=%s new=%s", img_path, path, name);
  return ESP_OK;
}

static esp_err_t api_hd_mkdir_post_handler(httpd_req_t *req)
{
  char body[512];
  if (!read_body_to_buffer(req, body, sizeof(body)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
    return ESP_FAIL;
  }

  char path[256] = "/";
  char name[64] = {0};
  (void)form_get_value_decoded(body, "path", path, sizeof(path));
  if (!form_get_value_decoded(body, "name", name, sizeof(name)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  char image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  const char *img_path = get_hd_image_path_from_req(req, body, image_path, sizeof(image_path));

  fat16_image_t *img = NULL;
  char errbuf[64] = {0};
  bool ok = fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  if (!ok)
  {
    const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
    ok = hdd_image_ensure_fat16(img_path, size_bytes) &&
         fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf));
  }
  ok = ok && fat16_image_mkdir(img, path, name, errbuf, sizeof(errbuf));
  if (img)
  {
    fat16_image_close(img);
  }
  system_cpu_pause_release(5000);

  if (!ok)
  {
    ESP_LOGW(TAG, "FM mkdir failed: image=%s path=%s name=%s err=%s", img_path, path, name, errbuf[0] ? errbuf : "mkdir failed");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "mkdir failed");
    return ESP_FAIL;
  }

  (void)disk_mark_media_changed_for_path(img_path);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "FM mkdir ok: image=%s path=%s name=%s", img_path, path, name);
  return ESP_OK;
}

static esp_err_t api_hd_upload_post_handler(httpd_req_t *req)
{
  char dir_path[256] = "/";
  char name[128] = {0};
  if (get_query_value(req, "path", dir_path, sizeof(dir_path)))
  {
    // decoded already
  }
  if (!get_query_value(req, "name", name, sizeof(name)))
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing name");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  char image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX];
  const char *img_path = get_hd_image_path_from_req(req, NULL, image_path, sizeof(image_path));
  ESP_LOGI(TAG, "FM upload: image=%s dir=%s name=%s bytes=%d", img_path, dir_path, name, (int)req->content_len);

  fat16_image_t *img = NULL;
  char errbuf[64] = {0};
  if (!fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf)))
  {
    const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
    if (hdd_image_ensure_fat16(img_path, size_bytes) &&
        fat16_image_open(&img, img_path, true, errbuf, sizeof(errbuf)))
    {
      // ok (created)
    }
    else
    {
      system_cpu_pause_release(5000);
      ESP_LOGW(TAG, "FM upload open failed: image=%s dir=%s name=%s err=%s", img_path, dir_path, name, errbuf[0] ? errbuf : "open failed");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "open failed");
      return ESP_FAIL;
    }
  }

  fat16_write_ctx_t w;
  if (!fat16_image_write_begin(img, dir_path, name, (uint32_t)req->content_len, &w, errbuf, sizeof(errbuf)))
  {
    fat16_image_close(img);
    system_cpu_pause_release(5000);
    ESP_LOGW(TAG, "FM upload create failed: image=%s dir=%s name=%s err=%s", img_path, dir_path, name, errbuf[0] ? errbuf : "create failed");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "create failed");
    return ESP_FAIL;
  }

  uint8_t buf[1024];
  int remaining = req->content_len;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, (char *)buf, (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf));
    if (recv_len <= 0)
    {
      fat16_image_write_abort(&w);
      fat16_image_close(img);
      system_cpu_pause_release(5000);
      ESP_LOGW(TAG, "FM upload recv failed: image=%s dir=%s name=%s", img_path, dir_path, name);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive failed");
      return ESP_FAIL;
    }
    if (!fat16_image_write_append(&w, buf, (size_t)recv_len, errbuf, sizeof(errbuf)))
    {
      fat16_image_write_abort(&w);
      fat16_image_close(img);
      system_cpu_pause_release(5000);
      ESP_LOGW(TAG, "FM upload write failed: image=%s dir=%s name=%s err=%s", img_path, dir_path, name, errbuf[0] ? errbuf : "write failed");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "write failed");
      return ESP_FAIL;
    }
    remaining -= recv_len;
  }

  if (!fat16_image_write_finish(&w, errbuf, sizeof(errbuf)))
  {
    fat16_image_write_abort(&w);
    fat16_image_close(img);
    system_cpu_pause_release(5000);
    ESP_LOGW(TAG, "FM upload finalize failed: image=%s dir=%s name=%s err=%s", img_path, dir_path, name, errbuf[0] ? errbuf : "finalize failed");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, errbuf[0] ? errbuf : "finalize failed");
    return ESP_FAIL;
  }

  fat16_image_close(img);
  system_cpu_pause_release(5000);

  (void)disk_mark_media_changed_for_path(img_path);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "FM upload ok: image=%s dir=%s name=%s", img_path, dir_path, name);
  return ESP_OK;
}

static esp_err_t api_settings_post_handler(httpd_req_t *req)
{
  if (req->content_len <= 0 || req->content_len > 1024)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid content length");
    return ESP_FAIL;
  }

  char body[1025];
  int remaining = req->content_len;
  int off = 0;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, body + off, remaining);
    if (recv_len <= 0)
    {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "receive failed");
      return ESP_FAIL;
    }
    off += recv_len;
    remaining -= recv_len;
  }
  body[off] = '\0';

  char fps_buf[32] = {0};
  char partial_buf[16] = {0};
  char pause_buf[16] = {0};
  char clear_bottom_buf[16] = {0};
  char wifi_buf[16] = {0};
  char bt_kb_buf[16] = {0};
  char gameport_buf[16] = {0};
  char boot_buf[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  char c_drive_buf[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  char a_drive_buf[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  char b_drive_buf[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
  bool have_fps = form_get_value(body, "fps", fps_buf, sizeof(fps_buf)) ||
                  form_get_value(body, "display_fps", fps_buf, sizeof(fps_buf));
  bool have_partial = form_get_value(body, "partial_refresh", partial_buf, sizeof(partial_buf)) ||
                      form_get_value(body, "partial", partial_buf, sizeof(partial_buf));
  bool have_pause = form_get_value(body, "pause_cpu_on_refresh", pause_buf, sizeof(pause_buf)) ||
                    form_get_value(body, "pause_cpu", pause_buf, sizeof(pause_buf)) ||
                    form_get_value(body, "pause", pause_buf, sizeof(pause_buf));
  bool have_clear_bottom = form_get_value(body, "clear_on_bottom", clear_bottom_buf, sizeof(clear_bottom_buf)) ||
                           form_get_value(body, "clear_bottom", clear_bottom_buf, sizeof(clear_bottom_buf));
  bool have_wifi = form_get_value(body, "wifi_enabled", wifi_buf, sizeof(wifi_buf)) ||
                   form_get_value(body, "wifi", wifi_buf, sizeof(wifi_buf));
  bool have_bt_kb = form_get_value(body, "bt_keyboard_enabled", bt_kb_buf, sizeof(bt_kb_buf)) ||
                    form_get_value(body, "bt_keyboard", bt_kb_buf, sizeof(bt_kb_buf)) ||
                    form_get_value(body, "bt_kb", bt_kb_buf, sizeof(bt_kb_buf));
  bool have_gameport = form_get_value(body, "gameport_enabled", gameport_buf, sizeof(gameport_buf)) ||
                       form_get_value(body, "gameport", gameport_buf, sizeof(gameport_buf)) ||
                       form_get_value(body, "joystick_enabled", gameport_buf, sizeof(gameport_buf));
  bool have_boot = form_get_value_decoded(body, "boot_image_path", boot_buf, sizeof(boot_buf)) ||
                   form_get_value_decoded(body, "boot_image", boot_buf, sizeof(boot_buf)) ||
                   form_get_value_decoded(body, "boot", boot_buf, sizeof(boot_buf));
  bool have_c_drive = form_get_value_decoded(body, "c_drive_image_path", c_drive_buf, sizeof(c_drive_buf)) ||
                      form_get_value_decoded(body, "c_drive", c_drive_buf, sizeof(c_drive_buf)) ||
                      form_get_value_decoded(body, "hdd_image_path", c_drive_buf, sizeof(c_drive_buf)) ||
                      form_get_value_decoded(body, "hdd", c_drive_buf, sizeof(c_drive_buf));
  bool have_a_drive = form_get_value_decoded(body, "a_drive_image_path", a_drive_buf, sizeof(a_drive_buf)) ||
                      form_get_value_decoded(body, "a_drive", a_drive_buf, sizeof(a_drive_buf)) ||
                      form_get_value_decoded(body, "drive_a", a_drive_buf, sizeof(a_drive_buf));
  bool have_b_drive = form_get_value_decoded(body, "b_drive_image_path", b_drive_buf, sizeof(b_drive_buf)) ||
                      form_get_value_decoded(body, "b_drive", b_drive_buf, sizeof(b_drive_buf)) ||
                      form_get_value_decoded(body, "drive_b", b_drive_buf, sizeof(b_drive_buf));

  char c_drive_dos33_buf[16] = {0};
  bool have_c_drive_dos33 = form_get_value(body, "c_drive_dos33", c_drive_dos33_buf, sizeof(c_drive_dos33_buf)) ||
                            form_get_value(body, "c_drive_dos33_compat", c_drive_dos33_buf, sizeof(c_drive_dos33_buf));

  if (!have_fps && !have_partial && !have_pause && !have_clear_bottom && !have_wifi && !have_bt_kb && !have_gameport &&
      !have_boot && !have_c_drive && !have_a_drive && !have_b_drive)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no settings provided");
    return ESP_FAIL;
  }

  app_settings_t s;
  (void)app_settings_get(&s);

  if (have_fps)
  {
    float fps = 0.0f;
    if (!parse_float_value(fps_buf, &fps))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid fps");
      return ESP_FAIL;
    }
    s.display_fps = fps;
  }
  if (have_partial)
  {
    bool enabled = false;
    if (!parse_bool_value(partial_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid partial_refresh");
      return ESP_FAIL;
    }
    s.display_partial_refresh = enabled;
  }
  if (have_pause)
  {
    bool enabled = false;
    if (!parse_bool_value(pause_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid pause_cpu_on_refresh");
      return ESP_FAIL;
    }
    s.display_pause_cpu_on_refresh = enabled;
  }
  if (have_clear_bottom)
  {
    bool enabled = false;
    if (!parse_bool_value(clear_bottom_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid clear_on_bottom");
      return ESP_FAIL;
    }
    s.display_clear_on_bottom = enabled;
  }
  if (have_wifi)
  {
    bool enabled = false;
    if (!parse_bool_value(wifi_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid wifi_enabled");
      return ESP_FAIL;
    }
    s.wifi_enabled = enabled;
  }
  if (have_bt_kb)
  {
    bool enabled = false;
    if (!parse_bool_value(bt_kb_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid bt_keyboard_enabled");
      return ESP_FAIL;
    }
    s.bt_keyboard_enabled = enabled;
  }
  if (have_gameport)
  {
    bool enabled = false;
    if (!parse_bool_value(gameport_buf, &enabled))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid gameport_enabled");
      return ESP_FAIL;
    }
    s.gameport_enabled = enabled;
  }
  if (have_boot)
  {
    if (strcasecmp(boot_buf, "C:") == 0)
    {
      (void)get_c_drive_image_path(boot_buf, sizeof(boot_buf));
    }

    if (!path_is_under_sdcard(boot_buf) || !ends_with_img_ext(boot_buf))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid boot_image_path");
      return ESP_FAIL;
    }

    struct stat st;
    if (stat(boot_buf, &st) != 0 || (st.st_mode & S_IFREG) == 0)
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "boot image not found");
      return ESP_FAIL;
    }

    strncpy(s.boot_image_path, boot_buf, sizeof(s.boot_image_path) - 1);
    s.boot_image_path[sizeof(s.boot_image_path) - 1] = '\0';
  }

  if (have_c_drive)
  {
    if (!path_is_under_sdcard(c_drive_buf) || !ends_with_img_ext(c_drive_buf))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid c_drive_image_path");
      return ESP_FAIL;
    }

    strncpy(s.c_drive_image_path, c_drive_buf, sizeof(s.c_drive_image_path) - 1);
    s.c_drive_image_path[sizeof(s.c_drive_image_path) - 1] = '\0';
  }

  if (have_a_drive)
  {
    if (!path_is_under_sdcard(a_drive_buf) || !ends_with_img_ext(a_drive_buf))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid a_drive_image_path");
      return ESP_FAIL;
    }
    if (!floppy_image_is_acceptable(a_drive_buf, NULL))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid a: image size");
      return ESP_FAIL;
    }
    strncpy(s.a_drive_image_path, a_drive_buf, sizeof(s.a_drive_image_path) - 1);
    s.a_drive_image_path[sizeof(s.a_drive_image_path) - 1] = '\0';
  }

  if (have_b_drive)
  {
    if (!path_is_under_sdcard(b_drive_buf) || !ends_with_img_ext(b_drive_buf))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid b_drive_image_path");
      return ESP_FAIL;
    }
    if (!floppy_image_is_acceptable(b_drive_buf, NULL))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid b: image size");
      return ESP_FAIL;
    }
    strncpy(s.b_drive_image_path, b_drive_buf, sizeof(s.b_drive_image_path) - 1);
    s.b_drive_image_path[sizeof(s.b_drive_image_path) - 1] = '\0';
  }

  if (have_c_drive_dos33)
  {
    bool v = false;
    if (!parse_bool_value(c_drive_dos33_buf, &v))
    {
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid c_drive_dos33");
      return ESP_FAIL;
    }
    s.c_drive_dos33_compat = v;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  bool paused = false;
  if (have_a_drive || have_b_drive)
  {
    if (!system_cpu_pause_request(5000))
    {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
      return ESP_FAIL;
    }
    paused = true;
  }

  if (have_a_drive)
  {
    if (!disk_replace_image(0x00, s.a_drive_image_path, DRIVE_FLOPPY))
    {
      if (paused) {
        system_cpu_pause_release(5000);
      }
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "mount A: failed");
      return ESP_FAIL;
    }
  }

  if (have_b_drive)
  {
    if (!disk_replace_image(0x01, s.b_drive_image_path, DRIVE_FLOPPY))
    {
      if (paused) {
        system_cpu_pause_release(5000);
      }
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "mount B: failed");
      return ESP_FAIL;
    }
  }

  if (have_a_drive || have_b_drive)
  {
    update_floppy_count();
  }
  if (paused) {
    system_cpu_pause_release(5000);
  }

  esp_err_t err = app_settings_set(&s);
  if (err != ESP_OK)
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "persist failed");
    return ESP_FAIL;
  }

  if (have_wifi && !s.wifi_enabled)
  {
    ESP_LOGW(TAG, "WiFi disabled via API; shutting down network services in 1s...");
    // We can't stop the server from within a handler easily without potentially 
    // cutting off the response. We'll use a one-shot timer or just a simple task.
    extern void system_request_network_shutdown(void);
    system_request_network_shutdown();
  }

  return api_settings_get_handler(req);
}

static esp_err_t api_reboot_post_handler(httpd_req_t *req)
{
  ESP_LOGW(TAG, "Reboot requested via API...");
  httpd_resp_send(req, "Rebooting...\n", HTTPD_RESP_USE_STRLEN);

  // Delay reboot slightly to allow response to be sent
  extern void system_request_reboot(void);
  system_request_reboot();

  return ESP_OK;
}

static const char *LOG_TAGS[] = {
    "DISKSWAP", "CPU_TRACE", "CPU_DIAG", "BT_GAP", "KBD", "SPK", "TTY", "BIOS_TTY", "VIDEO", "PORT", "CGA",
    NULL
};

static esp_err_t api_logs_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char buffer[256];
    httpd_resp_send_chunk(req, "{", 1);
    
    bool first = true;
    for (int i = 0; LOG_TAGS[i]; i++) {
        const char* tag = LOG_TAGS[i];
        int level = -1;
        if (app_settings_get_log_level(tag, &level) == ESP_OK) {
             if (level == 255) level = -1;
        } else {
             level = -1;
        }

        if (!first) {
            httpd_resp_send_chunk(req, ",", 1);
        }
        first = false;

        int len = snprintf(buffer, sizeof(buffer), "\"%s\":%d", tag, level);
        httpd_resp_send_chunk(req, buffer, len);
    }
    httpd_resp_send_chunk(req, "}", 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t api_logs_post_handler(httpd_req_t *req)
{
    char buf[2048];
    int ret, remaining = req->content_len;
    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }

    if (remaining > 0) {
        if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }
        buf[remaining] = '\0';
    } else {
        buf[0] = '\0';
    }

    for (int i = 0; LOG_TAGS[i]; i++) {
        const char *tag = LOG_TAGS[i];
        char val_str[16];
        if (form_get_value(buf, tag, val_str, sizeof(val_str))) {
            int val = atoi(val_str);
            if (val >= -1 && val <= 5) {
                int store_val = (val == -1) ? 255 : val;
                app_settings_set_log_level(tag, store_val);
                
                if (val >= 0) {
                    esp_log_level_set(tag, (esp_log_level_t)val);
                    ESP_LOGI(TAG, "Log %s -> %d", tag, val);
                }
            }
        }
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static void update_floppy_count(void)
{
  uint8_t floppy_count = 0;
  if (disk_is_ready(0x00))
    floppy_count++;
  if (disk_is_ready(0x01))
    floppy_count++;
  bios_set_floppy_drives(floppy_count);
}

static bool get_content_type(httpd_req_t *req, char *out, size_t out_len)
{
  if (!out || out_len == 0)
    return false;
  out[0] = '\0';
  if (httpd_req_get_hdr_value_str(req, "Content-Type", out, out_len) != ESP_OK)
    return false;
  return out[0] != '\0';
}

static const char *find_boundary_param(const char *content_type)
{
  if (!content_type)
    return NULL;
  const char *p = strstr(content_type, "boundary=");
  if (!p)
    return NULL;
  return p + strlen("boundary=");
}

static bool content_type_is_multipart(const char *content_type)
{
  if (!content_type)
    return false;
  // case-insensitive prefix match for "multipart/form-data"
  const char *needle = "multipart/form-data";
  while (*needle && *content_type)
  {
    if (tolower((unsigned char)*needle) != tolower((unsigned char)*content_type))
      return false;
    needle++;
    content_type++;
  }
  return *needle == '\0';
}

static esp_err_t recv_raw_body_to_file(httpd_req_t *req, FILE *f)
{
  char buf[1024];
  int remaining = req->content_len;
  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, buf, (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf));
    if (recv_len <= 0)
    {
      return ESP_FAIL;
    }
    if (fwrite(buf, 1, recv_len, f) != (size_t)recv_len)
    {
      return ESP_FAIL;
    }
    remaining -= recv_len;
  }
  return ESP_OK;
}

static esp_err_t recv_multipart_file_to_file(httpd_req_t *req, FILE *f, const char *boundary)
{
  if (!boundary || boundary[0] == '\0')
  {
    return ESP_FAIL;
  }

  // We look for the boundary marker that terminates the file data:
  // "\r\n--" + boundary
  char marker[128];
  int marker_len = snprintf(marker, sizeof(marker), "\r\n--%s", boundary);
  if (marker_len <= 0 || marker_len >= (int)sizeof(marker))
  {
    return ESP_FAIL;
  }

  char buf[1024];
  int remaining = req->content_len;

  bool in_data = false;
  // Track the end-of-headers marker "\r\n\r\n" across recv() boundaries.
  uint32_t header_window = 0;

  // Keep a tail to detect boundary across chunk boundaries (cap to marker size).
  char pending[128];
  int pending_len = 0;
  int keep = marker_len;

  while (remaining > 0)
  {
    int recv_len = httpd_req_recv(req, buf, (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf));
    if (recv_len <= 0)
    {
      return ESP_FAIL;
    }
    remaining -= recv_len;

    int offset = 0;
    while (offset < recv_len)
    {
      if (!in_data)
      {
        // Consume bytes until we see "\r\n\r\n", then switch into data mode.
        while (offset < recv_len)
        {
          header_window = (header_window << 8) | (uint8_t)buf[offset++];
          if (header_window == 0x0D0A0D0A)
          {
            in_data = true;
            break;
          }
        }
        if (!in_data)
          continue;
      }

      // In file data. We need to write bytes while searching for marker.
      int chunk_len = recv_len - offset;
      if (chunk_len <= 0)
      {
        break;
      }

      if (marker_len > (int)sizeof(pending))
      {
        ESP_LOGE(TAG, "multipart boundary too long for pending: marker_len=%d pending=%d", marker_len, (int)sizeof(pending));
        return ESP_FAIL;
      }

      const char *chunk = buf + offset;
      char combined[sizeof(pending) + sizeof(buf)];
      int combined_len = 0;
      if (pending_len > 0)
      {
        memcpy(combined, pending, pending_len);
        combined_len += pending_len;
      }
      memcpy(combined + combined_len, chunk, chunk_len);
      combined_len += chunk_len;

      // Search for marker in [pending + chunk].
      int found_at = -1;
      for (int i = 0; i + marker_len <= combined_len; i++)
      {
        if (memcmp(combined + i, marker, marker_len) == 0)
        {
          found_at = i;
          break;
        }
      }

      if (found_at >= 0)
      {
        // Flush all bytes before the marker (file payload), then stop.
        if (found_at > 0)
        {
          if (fwrite(combined, 1, found_at, f) != (size_t)found_at)
          {
            return ESP_FAIL;
          }
        }
        return ESP_OK;
      }

      // No marker found: flush all but the last `keep` bytes (which might be start of a marker).
      int flush_len = combined_len - keep;
      if (flush_len > 0)
      {
        if (fwrite(combined, 1, flush_len, f) != (size_t)flush_len)
        {
          return ESP_FAIL;
        }
        pending_len = keep;
        memcpy(pending, combined + flush_len, pending_len);
      }
      else
      {
        // Not enough data yet to safely flush; keep everything.
        pending_len = combined_len;
        memcpy(pending, combined, pending_len);
      }

      offset = recv_len; // consumed this recv chunk in data mode
    }
  }

  // If we never saw the terminating marker, treat as failure.
  return ESP_FAIL;
}

static esp_err_t disk2_post_handler(httpd_req_t *req)
{
  char content_type[128] = {0};
  bool has_ct = get_content_type(req, content_type, sizeof(content_type));
  ESP_LOGI(TAG, "Upload start: content_len=%d has_content_type=%d content_type=%s", (int)req->content_len,
           has_ct ? 1 : 0, has_ct ? content_type : "(none)");

  if (!sdcard_is_mounted())
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD not mounted");
    return ESP_FAIL;
  }

  if (req->content_len <= 0)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Opening staging file: path=%s mode=wb", DISK2_STAGING_PATH);
  FILE *f = fopen(DISK2_STAGING_PATH, "wb");
  if (!f)
  {
    int err = errno;
    ESP_LOGE(TAG, "Failed to open staging file: path=%s errno=%d (%s)", DISK2_STAGING_PATH, err, strerror(err));
    log_path_stat("sdcard_root", "/sdcard");
    log_path_stat("staging_path", DISK2_STAGING_PATH);
    log_path_stat("final_path", DISK2_FINAL_PATH);
    log_sdcard_space("/sdcard");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open staging file");
    return ESP_FAIL;
  }

  esp_err_t rx_err = ESP_FAIL;
  if (has_ct && content_type_is_multipart(content_type))
  {
    const char *boundary = find_boundary_param(content_type);
    ESP_LOGI(TAG, "Receiving multipart upload: boundary=%s", boundary ? boundary : "(null)");
    rx_err = recv_multipart_file_to_file(req, f, boundary);
  }
  else
  {
    ESP_LOGI(TAG, "Receiving raw upload");
    rx_err = recv_raw_body_to_file(req, f);
  }

  if (rx_err != ESP_OK)
  {
    fclose(f);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive/write failed");
    return ESP_FAIL;
  }

  fflush(f);
  long size = ftell(f);
  fclose(f);
  ESP_LOGI(TAG, "Upload complete: staging=%s size=%ld", DISK2_STAGING_PATH, size);

  if (size < 512)
  {
    remove(DISK2_STAGING_PATH);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image too small");
    return ESP_FAIL;
  }

  if ((size % 512) != 0)
  {
    ESP_LOGE(TAG, "Uploaded image not sector-aligned: size=%ld", size);
    remove(DISK2_STAGING_PATH);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image not sector-aligned");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  bool ok = disk_replace_image_from_staging(0x01, DISK2_STAGING_PATH, DISK2_FINAL_PATH, DRIVE_FLOPPY);
  system_cpu_pause_release(5000);
  if (!ok)
  {
    ESP_LOGE(TAG, "Disk swap failed (staging=%s final=%s)", DISK2_STAGING_PATH, DISK2_FINAL_PATH);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Disk swap failed");
    return ESP_FAIL;
  }

  update_floppy_count();

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "Replaced B: image at runtime");
  return ESP_OK;
}

static esp_err_t disk1_post_handler(httpd_req_t *req)
{
  char content_type[128] = {0};
  bool has_ct = get_content_type(req, content_type, sizeof(content_type));
  ESP_LOGI(TAG, "Upload start (A:): content_len=%d has_content_type=%d content_type=%s", (int)req->content_len,
           has_ct ? 1 : 0, has_ct ? content_type : "(none)");

  if (!sdcard_is_mounted())
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD not mounted");
    return ESP_FAIL;
  }

  if (req->content_len <= 0)
  {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Opening staging file: path=%s mode=wb", DISK1_STAGING_PATH);
  FILE *f = fopen(DISK1_STAGING_PATH, "wb");
  if (!f)
  {
    int err = errno;
    ESP_LOGE(TAG, "Failed to open staging file: path=%s errno=%d (%s)", DISK1_STAGING_PATH, err, strerror(err));
    log_path_stat("sdcard_root", "/sdcard");
    log_path_stat("staging_path", DISK1_STAGING_PATH);
    log_path_stat("final_path", DISK1_FINAL_PATH);
    log_sdcard_space("/sdcard");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to open staging file");
    return ESP_FAIL;
  }

  esp_err_t rx_err = ESP_FAIL;
  if (has_ct && content_type_is_multipart(content_type))
  {
    const char *boundary = find_boundary_param(content_type);
    ESP_LOGI(TAG, "Receiving multipart upload: boundary=%s", boundary ? boundary : "(null)");
    rx_err = recv_multipart_file_to_file(req, f, boundary);
  }
  else
  {
    ESP_LOGI(TAG, "Receiving raw upload");
    rx_err = recv_raw_body_to_file(req, f);
  }

  if (rx_err != ESP_OK)
  {
    fclose(f);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive/write failed");
    return ESP_FAIL;
  }

  fflush(f);
  long size = ftell(f);
  fclose(f);
  ESP_LOGI(TAG, "Upload complete: staging=%s size=%ld", DISK1_STAGING_PATH, size);

  if (size < 512)
  {
    remove(DISK1_STAGING_PATH);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image too small");
    return ESP_FAIL;
  }

  if ((size % 512) != 0)
  {
    ESP_LOGE(TAG, "Uploaded image not sector-aligned: size=%ld", size);
    remove(DISK1_STAGING_PATH);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image not sector-aligned");
    return ESP_FAIL;
  }

  extern bool system_cpu_pause_request(uint32_t timeout_ms);
  extern void system_cpu_pause_release(uint32_t timeout_ms);
  if (!system_cpu_pause_request(5000))
  {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cpu busy");
    return ESP_FAIL;
  }

  bool ok = disk_replace_image_from_staging(0x00, DISK1_STAGING_PATH, DISK1_FINAL_PATH, DRIVE_FLOPPY);
  system_cpu_pause_release(5000);
  if (!ok)
  {
    ESP_LOGE(TAG, "Disk swap failed (staging=%s final=%s)", DISK1_STAGING_PATH, DISK1_FINAL_PATH);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Disk swap failed");
    return ESP_FAIL;
  }

  update_floppy_count();

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
  ESP_LOGI(TAG, "Replaced A: image at runtime");
  return ESP_OK;
}

esp_err_t disk_swap_server_start(void)
{
  ESP_LOGI(TAG, "Starting disk swap server...");
  if (s_server)
  {
    return ESP_OK;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 24;
  // The upload handler does streaming SD writes; give the HTTPD thread more stack headroom.
  config.stack_size = 16384;

  esp_err_t ret = httpd_start(&s_server, &config);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "httpd_start failed: %d", (int)ret);
    return ret;
  }

  httpd_uri_t root = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = root_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t disk2 = {
      .uri = "/disk2",
      .method = HTTP_POST,
      .handler = disk2_post_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t disk1 = {
      .uri = "/disk1",
      .method = HTTP_POST,
      .handler = disk1_post_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_settings_get = {
      .uri = "/api/settings",
      .method = HTTP_GET,
      .handler = api_settings_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_settings_post = {
      .uri = "/api/settings",
      .method = HTTP_POST,
      .handler = api_settings_post_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_boot_images_get = {
      .uri = "/api/boot-images",
      .method = HTTP_GET,
      .handler = api_boot_images_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_hdd_images_get = {
      .uri = "/api/hdd-images",
      .method = HTTP_GET,
      .handler = api_hdd_images_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_hdd_generate_post = {
      .uri = "/api/hdd-generate",
      .method = HTTP_POST,
      .handler = api_hdd_generate_post_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_sd_upload_post = {
      .uri = "/api/sd/upload",
      .method = HTTP_POST,
      .handler = api_sd_upload_post_handler,
      .user_ctx = NULL,
  };

  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &disk1));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &disk2));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_settings_get));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_settings_post));

  httpd_uri_t api_logs_get = {
      .uri = "/api/logs",
      .method = HTTP_GET,
      .handler = api_logs_get_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_logs_get));

  httpd_uri_t api_logs_post = {
      .uri = "/api/logs",
      .method = HTTP_POST,
      .handler = api_logs_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_logs_post));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_boot_images_get));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hdd_images_get));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hdd_generate_post));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_sd_upload_post));

  httpd_uri_t api_terminal_screen_get = {
      .uri = "/api/terminal/screen",
      .method = HTTP_GET,
      .handler = api_terminal_screen_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_terminal_framebuffer_get = {
      .uri = "/api/terminal/framebuffer",
      .method = HTTP_GET,
      .handler = api_terminal_framebuffer_get_handler,
      .user_ctx = NULL,
  };

  httpd_uri_t api_terminal_key_post = {
      .uri = "/api/terminal/key",
      .method = HTTP_POST,
      .handler = api_terminal_key_post_handler,
      .user_ctx = NULL,
  };

  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_terminal_screen_get));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_terminal_framebuffer_get));
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_terminal_key_post));

  httpd_uri_t api_terminal_mouse_post = {
      .uri = "/api/terminal/mouse",
      .method = HTTP_POST,
      .handler = api_terminal_mouse_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_terminal_mouse_post));

  httpd_uri_t api_hd_list_get = {
      .uri = "/api/hd/list",
      .method = HTTP_GET,
      .handler = api_hd_list_get_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hd_list_get));

  httpd_uri_t api_hd_upload_post = {
      .uri = "/api/hd/upload",
      .method = HTTP_POST,
      .handler = api_hd_upload_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hd_upload_post));

  httpd_uri_t api_hd_delete_post = {
      .uri = "/api/hd/delete",
      .method = HTTP_POST,
      .handler = api_hd_delete_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hd_delete_post));

  httpd_uri_t api_hd_rename_post = {
      .uri = "/api/hd/rename",
      .method = HTTP_POST,
      .handler = api_hd_rename_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hd_rename_post));

  httpd_uri_t api_hd_mkdir_post = {
      .uri = "/api/hd/mkdir",
      .method = HTTP_POST,
      .handler = api_hd_mkdir_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_hd_mkdir_post));

  httpd_uri_t api_reboot_post = {
      .uri = "/api/reboot",
      .method = HTTP_POST,
      .handler = api_reboot_post_handler,
      .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &api_reboot_post));

  ESP_LOGI(TAG, "Disk swap server started");
  return ESP_OK;
}

esp_err_t disk_swap_server_stop(void)
{
  if (!s_server)
  {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping disk swap server...");
  esp_err_t ret = httpd_stop(s_server);
  if (ret == ESP_OK)
  {
    s_server = NULL;
  }
  return ret;
}
