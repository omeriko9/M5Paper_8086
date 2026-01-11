#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*wifi_sta_ip_callback_t)(const char *ip_str);

esp_err_t wifi_sta_start(void);
esp_err_t wifi_sta_stop(void);
bool wifi_sta_wait_connected(uint32_t timeout_ms);
void wifi_sta_set_ip_callback(wifi_sta_ip_callback_t cb);
const char *wifi_sta_get_ip_str(void);

#ifdef __cplusplus
}
#endif
