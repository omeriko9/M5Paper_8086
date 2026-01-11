#include "net/wifi_ap.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "WIFI_AP";

static esp_netif_t *s_ap_netif;
static char s_ap_ssid[32];
static char s_ap_ip[16];

static void build_default_ssid(void)
{
  if (s_ap_ssid[0] != '\0')
  {
    return;
  }

  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) == ESP_OK)
  {
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "M5PaperDOS-%02X%02X", mac[4], mac[5]);
  }
  else
  {
    strncpy(s_ap_ssid, "M5PaperDOS", sizeof(s_ap_ssid) - 1);
    s_ap_ssid[sizeof(s_ap_ssid) - 1] = '\0';
  }
}

static esp_err_t ensure_wifi_base(void)
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", (int)ret);
    return ret;
  }

  ret = esp_netif_init();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "esp_netif_init failed: %d", (int)ret);
    return ret;
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "esp_event_loop_create_default failed: %d", (int)ret);
    return ret;
  }

  return ESP_OK;
}

esp_err_t wifi_ap_start(void)
{
  if (s_ap_netif)
  {
    return ESP_OK;
  }

  esp_err_t ret = ensure_wifi_base();
  if (ret != ESP_OK)
  {
    return ret;
  }

  s_ap_netif = esp_netif_create_default_wifi_ap();
  if (!s_ap_netif)
  {
    return ESP_ERR_NO_MEM;
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
  {
    ESP_LOGE(TAG, "esp_wifi_init failed: %d", (int)ret);
    return ret;
  }

  build_default_ssid();

  wifi_config_t wifi_config = {0};
  strncpy((char *)wifi_config.ap.ssid, s_ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
  wifi_config.ap.ssid_len = (uint8_t)strlen((const char *)wifi_config.ap.ssid);
  wifi_config.ap.channel = 1;
  wifi_config.ap.max_connection = 4;
  wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  wifi_config.ap.pmf_cfg.capable = true;
  wifi_config.ap.pmf_cfg.required = false;

  ret = esp_wifi_set_mode(WIFI_MODE_AP);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_wifi_set_mode failed: %d", (int)ret);
    return ret;
  }
  ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_wifi_set_config failed: %d", (int)ret);
    return ret;
  }
  ret = esp_wifi_start();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_wifi_start failed: %d", (int)ret);
    return ret;
  }

  s_ap_ip[0] = '\0';
  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(s_ap_netif, &ip_info) == ESP_OK)
  {
    snprintf(s_ap_ip, sizeof(s_ap_ip), IPSTR, IP2STR(&ip_info.ip));
  }
  if (s_ap_ip[0] == '\0')
  {
    strncpy(s_ap_ip, "192.168.4.1", sizeof(s_ap_ip) - 1);
    s_ap_ip[sizeof(s_ap_ip) - 1] = '\0';
  }

  ESP_LOGI(TAG, "SoftAP started: ssid=%s ip=%s", s_ap_ssid, s_ap_ip);
  return ESP_OK;
}

esp_err_t wifi_ap_stop(void)
{
  if (!s_ap_netif)
  {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping SoftAP...");
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_netif_destroy(s_ap_netif);
  s_ap_netif = NULL;
  s_ap_ip[0] = '\0';
  return ESP_OK;
}

const char *wifi_ap_get_ssid(void)
{
  return s_ap_ssid;
}

const char *wifi_ap_get_ip_str(void)
{
  return s_ap_ip;
}
