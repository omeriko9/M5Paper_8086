#include "config_defs.h"

// Display Configuration Defaults
float g_display_fps = 2.0f;
int g_display_partial_refresh = 1;
int g_display_pause_cpu_on_refresh = 1;

// C Drive Configuration Defaults
int g_c_drive_enabled = 1;
const char *g_c_drive_image_path = "/sdcard/c_drive/hd.img";
int g_c_drive_default_size_mb = 512;
int g_c_drive_dos33_compat = 1;
