#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t disk_swap_server_start(void);
esp_err_t disk_swap_server_stop(void);

#ifdef __cplusplus
}
#endif

