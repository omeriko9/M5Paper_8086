#include "net/wifi_sta.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "wifi_secrets.h"

static const char *TAG = "WIFI";

static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_wifi_netif;

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;

static int s_retry_num;
static const int s_max_retries = 10;

static wifi_sta_ip_callback_t s_ip_cb;
static char s_ip_str[16];

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < s_max_retries)
    {
      s_retry_num++;
      esp_wifi_connect();
      ESP_LOGW(TAG, "Retrying WiFi connection (%d/%d)", s_retry_num, s_max_retries);
    }
    else
    {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      ESP_LOGE(TAG, "WiFi connect failed");
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Got IP: %s", s_ip_str);
    if (s_ip_cb)
    {
      s_ip_cb(s_ip_str);
    }
  }
}

esp_err_t wifi_sta_start(void)
{
  if (strlen(WIFI_SSID) == 0)
  {
    ESP_LOGW(TAG, "WiFi disabled: WIFI_SSID is empty (check secrets.txt)");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret;

  ret = nvs_flash_init();
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
  if (ret != ESP_OK)
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

  s_wifi_event_group = xEventGroupCreate();
  if (!s_wifi_event_group)
  {
    return ESP_ERR_NO_MEM;
  }

  s_wifi_netif = esp_netif_create_default_wifi_sta();
  if (!s_wifi_netif)
  {
    return ESP_ERR_NO_MEM;
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_wifi_init failed: %d", (int)ret);
    return ret;
  }

  ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "event handler register failed: %d", (int)ret);
    return ret;
  }
  ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "event handler register failed: %d", (int)ret);
    return ret;
  }

  wifi_config_t wifi_config = {0};
  strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;

  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_wifi_set_mode failed: %d", (int)ret);
    return ret;
  }
  ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
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

  ESP_LOGI(TAG, "WiFi STA started");
  return ESP_OK;
}

esp_err_t wifi_sta_stop(void)
{
  if (!s_wifi_event_group)
  {
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Stopping WiFi...");
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
  vEventGroupDelete(s_wifi_event_group);
  s_wifi_event_group = NULL;
  if (s_wifi_netif)
  {
    esp_netif_destroy(s_wifi_netif);
  }
  s_wifi_netif = NULL;
  s_ip_str[0] = '\0';
  
  return ESP_OK;
}

bool wifi_sta_wait_connected(uint32_t timeout_ms)
{
  if (!s_wifi_event_group)
  {
    return false;
  }

  TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, ticks);
  return (bits & WIFI_CONNECTED_BIT) != 0;
}

void wifi_sta_set_ip_callback(wifi_sta_ip_callback_t cb)
{
  s_ip_cb = cb;
}

const char *wifi_sta_get_ip_str(void)
{
  return s_ip_str;
}
