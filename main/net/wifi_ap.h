#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wifi_ap_start(void);
esp_err_t wifi_ap_stop(void);
const char *wifi_ap_get_ssid(void);
const char *wifi_ap_get_ip_str(void);

#ifdef __cplusplus
}
#endif
