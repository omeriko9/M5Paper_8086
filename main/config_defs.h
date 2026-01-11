#ifndef CONFIG_DEFS_H
#define CONFIG_DEFS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Display Configuration
extern float g_display_fps;
extern int g_display_partial_refresh;
extern int g_display_pause_cpu_on_refresh;

// C Drive Configuration
extern int g_c_drive_enabled;
extern const char *g_c_drive_image_path;
extern int g_c_drive_default_size_mb;
extern int g_c_drive_dos33_compat;

#ifdef __cplusplus
}
#endif

#endif // CONFIG_DEFS_H
