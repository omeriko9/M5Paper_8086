/**
 * @file main.c
 * @brief M5Paper DOS - Main entry point
 *
 * DOS emulator for M5Paper e-ink display
 * Uses 8086tiny-based emulator to run MS-DOS from SD card image
 */

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "sdcard/sdcard.h"
#include "display/epd_driver.h"
#include "dos/memory.h"
#include "dos/cpu8086.h"
#include "dos/disk.h"
#include "dos/bios.h"
#include "dos/hdd_image.h"
#include "dos/video.h"
#include "dos/ports.h"
#include "dos/interrupts.h"
#include "dos/xms.h"
#include "emulator/embedded_8086tiny_bios.h"
#include "bt/bt_keyboard.h"
#include "config_defs.h"
#include "settings.h"

#include "net/wifi_sta.h"
#include "net/wifi_ap.h"
#include "net/disk_swap_server.h"

#include "M5Unified.h"
#include "M5GFX.h"

//static int TFT_WHITE       = 0xFFFF;      /* 255, 255, 255 */

static const char *TAG = "M5PaperDOS";
static const char *CPU_TAG = "CPU_TRACE";

// DOS disk image path on SD card
#define DOS_IMAGE_PATH "/sdcard/msdos.img"
#define DOS_IMAGE_PATH_A "/sdcard/disk1.img"
#define DOS_IMAGE_PATH_B "/sdcard/disk2.img"

// CPU emulation task parameters
#define CPU_TASK_STACK_SIZE (32 * 1024)
#define CPU_TASK_PRIORITY 6  // Higher priority than display (was 5)

// Display refresh task parameters
#define DISPLAY_TASK_STACK_SIZE (8 * 1024)
#define DISPLAY_TASK_PRIORITY 3

// Timer tick interval in microseconds (18.2 Hz = ~54945 us)
#define TIMER_TICK_INTERVAL_US 54945

// CPU cycles per tick
// Execute more cycles per batch for smoother emulation
// 10000 instructions per batch is a good balance
// UPDATE: Reduced to 1000 to improve hardware interrupt latency (e.g. Prince of Persia)
#define CPU_CYCLES_PER_TICK 1000

// Global CPU state
static cpu8086_t g_cpu;

// Task handles
static TaskHandle_t cpu_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;

// Network services are started after DOS tasks are created to avoid starving
// critical task stack allocations during WiFi/HTTP init.
static bool g_enable_network_task = false;

// Timer handle
static esp_timer_handle_t timer_tick_handle = NULL;

// Running flag
static volatile bool g_running = false;
static volatile bool g_cpu_pause_requested = false;
static volatile bool g_cpu_paused = false;

static bool waitForDisplayReady(uint32_t timeout_ms);

static portMUX_TYPE g_status_line_mux = portMUX_INITIALIZER_UNLOCKED;
static char g_status_line[81];
static volatile bool g_status_line_dirty = false;
static volatile bool g_status_line_valid = false;
static volatile uint32_t g_status_show_until_ms = 0;

static void display_set_status_line(const char *line)
{
    if (!line) {
        return;
    }

    char tmp[81];
    size_t n = strnlen(line, 80);
    memcpy(tmp, line, n);
    for (size_t i = n; i < 80; i++) {
        tmp[i] = ' ';
    }
    tmp[80] = '\0';

    portENTER_CRITICAL(&g_status_line_mux);
    memcpy(g_status_line, tmp, sizeof(g_status_line));
    g_status_line_dirty = true;
    g_status_line_valid = true;
    g_status_show_until_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) + 60000;
    portEXIT_CRITICAL(&g_status_line_mux);
}

static void wifi_ip_callback(const char *ip_str)
{
    ESP_LOGI(TAG, "WiFi IP: %s", ip_str ? ip_str : "");
    char line[81];
    snprintf(line, sizeof(line), "WiFi IP: %s", ip_str ? ip_str : "");
    display_set_status_line(line);
}

static void wifi_ap_status_line(void)
{
    const char *ssid = wifi_ap_get_ssid();
    const char *ip = wifi_ap_get_ip_str();
    char line[81];
    if (ssid && ssid[0] && ip && ip[0]) {
        snprintf(line, sizeof(line), "WiFi AP: %s %s", ssid, ip);
    } else if (ssid && ssid[0]) {
        snprintf(line, sizeof(line), "WiFi AP: %s", ssid);
    } else if (ip && ip[0]) {
        snprintf(line, sizeof(line), "WiFi AP IP: %s", ip);
    } else {
        snprintf(line, sizeof(line), "WiFi AP active");
    }
    display_set_status_line(line);
}

static bool display_take_status_line(char *out81, bool *dirty)
{
    bool have = false;
    bool is_dirty = false;

    portENTER_CRITICAL(&g_status_line_mux);
    if (g_status_line_valid) {
        memcpy(out81, g_status_line, 81);
        have = true;
    }
    is_dirty = g_status_line_dirty;
    g_status_line_dirty = false;
    portEXIT_CRITICAL(&g_status_line_mux);

    if (dirty) {
        *dirty = is_dirty;
    }
    return have;
}

static void draw_status_line_overlay(bool force)
{
    char status[81];
    bool dirty = false;
    if (!display_take_status_line(status, &dirty)) {
        return;
    }

    // Only draw if content changed or forced (e.g. after full screen refresh)
    if (!force && !dirty) {
        return;
    }

    // Draw at the bottom of the screen to avoid obscuring DOS text
    // Screen height 540. Text area ends at 525 (25 * 21).
    // We use the bottom 16 pixels (524-540) for status, which is 4-pixel aligned.
    // This overlaps the last pixel of the text area, but that's just padding.
    int y_pos = 524;
    int height = 16;

    // Ensure alignment for IT8951
    // y_pos is 524 (divisible by 4)
    // height is 16 (divisible by 4)

    ESP_LOGD(TAG, "Drawing status overlay: %s", status);

    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.fillRect(0, y_pos, EPD_WIDTH, height, TFT_WHITE);
    M5.Display.setCursor(4, y_pos + 4); // Add some padding
    M5.Display.print(status);
    M5.Display.display(0, y_pos, EPD_WIDTH, height);
    
    // Short wait to ensure command is sent
    // waitForDisplayReady(2000); // Removed to prevent blocking, M5GFX handles busy check
}

static void network_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Network task started");

    bool sta_ok = false;
    if (wifi_sta_start() == ESP_OK) {
        if (wifi_sta_wait_connected(20000)) {
            sta_ok = true;
            (void)disk_swap_server_start();
        } else {
            ESP_LOGW(TAG, "WiFi not connected; falling back to SoftAP");
            wifi_sta_stop();
        }
    } else {
        ESP_LOGW(TAG, "WiFi STA start failed; falling back to SoftAP");
        wifi_sta_stop();
    }

    if (!sta_ok) {
        if (wifi_ap_start() == ESP_OK) {
            wifi_ap_status_line();
            (void)disk_swap_server_start();
        } else {
            ESP_LOGE(TAG, "SoftAP start failed; web UI disabled");
        }
    }

    vTaskDelete(NULL);
}

static void network_shutdown_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Executing requested network shutdown...");
    disk_swap_server_stop();
    wifi_ap_stop();
    wifi_sta_stop();
    display_set_status_line("WiFi Disabled");
}

extern "C" void system_request_network_shutdown(void)
{
    static esp_timer_handle_t shutdown_timer = NULL;
    if (shutdown_timer) return;

    const esp_timer_create_args_t args = {
        .callback = &network_shutdown_timer_cb,
        .name = "net_shutdown"
    };
    if (esp_timer_create(&args, &shutdown_timer) == ESP_OK) {
        esp_timer_start_once(shutdown_timer, 1000000); // 1 second delay
    }
}

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Executing requested reboot...");
    esp_restart();
}

extern "C" void system_request_reboot(void)
{
    static esp_timer_handle_t reboot_timer = NULL;
    if (reboot_timer) return;

    const esp_timer_create_args_t args = {
        .callback = &reboot_timer_cb,
        .name = "reboot"
    };
    if (esp_timer_create(&args, &reboot_timer) == ESP_OK) {
        esp_timer_start_once(reboot_timer, 1000000); // 1 second delay
    }
}

static bool cpu_pause_request(uint32_t timeout_ms)
{
    if (!cpu_task_handle) {
        return true;
    }

    g_cpu_pause_requested = true;
    const uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!g_cpu_paused) {
        if (timeout_ms != 0) {
            const uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_ms;
            if (elapsed_ms >= timeout_ms) {
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
}

static void cpu_pause_release(uint32_t timeout_ms)
{
    if (!cpu_task_handle) {
        return;
    }

    g_cpu_pause_requested = false;
    if (timeout_ms == 0) {
        return;
    }

    const uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (g_cpu_paused) {
        const uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_ms;
        if (elapsed_ms >= timeout_ms) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" bool system_cpu_pause_request(uint32_t timeout_ms)
{
    return cpu_pause_request(timeout_ms);
}

extern "C" void system_cpu_pause_release(uint32_t timeout_ms)
{
    cpu_pause_release(timeout_ms);
}

static bool waitForDisplayReady(uint32_t timeout_ms)
{
    const uint32_t start_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool was_busy = false;

    while (M5.Display.displayBusy())
    {
        was_busy = true;
        const uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_ms;
        if (timeout_ms != 0 && elapsed_ms >= timeout_ms)
        {
            ESP_LOGW(TAG, "waitForDisplayReady timeout after %lu ms", elapsed_ms);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (was_busy) {
        const uint32_t elapsed_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_ms;
        ESP_LOGD(TAG, "Display was busy for %lu ms", elapsed_ms);
    }

    return true;
}

static bool file_exists(const char *path)
{
    struct stat st;
    return (stat(path, &st) == 0);
}

static uint8_t count_hard_drives_ready(void)
{
    uint8_t count = 0;
    if (disk_is_ready(0x80)) {
        count++;
    }
    if (disk_is_ready(0x81)) {
        count++;
    }
    return count;
}

/**
 * Timer tick callback - called ~18.2 times per second
 * Simulates the PC timer interrupt
 */
static void timer_tick_callback(void *arg)
{
    // Increment BIOS timer counter
    bios_timer_tick();

    // Update PIT counters
    // ports_tick();

    // Raise timer interrupt if enabled
    // port_irq(IRQ_TIMER);
}

/**
 * CPU emulation task - OPTIMIZED for performance
 * Runs on dedicated core (core 1) to avoid contention
 * Minimizes delays to maximize instruction throughput
 */
static void cpu_task(void *pvParameters)
{
    ESP_LOGI(TAG, "CPU task started on core %d", xPortGetCoreID());

    int cycles_executed;
    bool wdt_enabled = false;
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err == ESP_OK) {
        wdt_enabled = true;
    } else if (wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to add cpu_task to WDT: %d", (int)wdt_err);
    }

    uint64_t last_log_us = esp_timer_get_time();
    uint16_t last_cs = g_cpu.sregs[SEG_CS];
    uint16_t last_ip = g_cpu.ip;
    uint8_t stuck_count = 0;

    uint32_t batch_count = 0;
    uint64_t last_idle_yield_us = esp_timer_get_time();
    
    g_enable_network_task = true;

    while (g_running)
    {
        if (g_cpu_pause_requested) {
            g_cpu_paused = true;
            while (g_running && g_cpu_pause_requested) {
                if (wdt_enabled) {
                    esp_task_wdt_reset();
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            g_cpu_paused = false;
        }

        // Execute a batch of CPU cycles
        cycles_executed = cpu_exec_cycles(&g_cpu, CPU_CYCLES_PER_TICK);

        // Update PIT state
        ports_tick();

        // Check for pending hardware interrupts
        if (interrupts_are_enabled())
        {
            interrupt_hardware(&g_cpu);
        }

        batch_count++;
        
        // Reset WDT every batch to prevent timeout
        if (wdt_enabled) {
            esp_task_wdt_reset();
        }

        // Yield periodically so FreeRTOS IDLE can run (avoids task_wdt spam/backtraces).
        // Using vTaskDelay(1) (not taskYIELD) allows lower-priority tasks like IDLE to run.
        uint64_t now_us = esp_timer_get_time();
        if ((now_us - last_idle_yield_us) >= 20000) { // ~20ms
            vTaskDelay(1);
            last_idle_yield_us = esp_timer_get_time();
        }

#if 1
        now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 1000000) {
            uint16_t cs = g_cpu.sregs[SEG_CS];
            uint16_t ip = g_cpu.ip;
            uint32_t addr = (cs << 4) + ip;
            uint8_t op0 = mem_read_byte(addr);
            uint8_t op1 = mem_read_byte(addr + 1);
            uint8_t op2 = mem_read_byte(addr + 2);
            const uint16_t ax = g_cpu.regs16[REG_AX];
            const uint16_t dx = g_cpu.regs16[REG_DX];

            const char *io_hint = "";
            uint16_t io_port = 0;
            if (op0 == 0xE4 || op0 == 0xE5 || op0 == 0xE6 || op0 == 0xE7) {
                io_hint = (op0 == 0xE4 || op0 == 0xE5) ? " IN" : " OUT";
                io_port = op1;
            } else if (op0 == 0xEC || op0 == 0xED || op0 == 0xEE || op0 == 0xEF) {
                io_hint = (op0 == 0xEC || op0 == 0xED) ? " IN" : " OUT";
                io_port = dx;
            }

            if (io_hint[0] != '\0') {
                ESP_LOGI(CPU_TAG, "CPU: CS:IP=%04X:%04X AX=%04X op=%02X %02X %02X%s port=%04X cycles=%llu",
                         cs, ip, ax, op0, op1, op2, io_hint, (unsigned)io_port, g_cpu.cycles);
            } else {
                ESP_LOGI(CPU_TAG, "CPU: CS:IP=%04X:%04X AX=%04X op=%02X %02X %02X cycles=%llu",
                         cs, ip, ax, op0, op1, op2, g_cpu.cycles);
            }
            if (cs == last_cs && ip == last_ip) {
                stuck_count++;
                if (stuck_count >= 5) {
                    ESP_LOGW(CPU_TAG, "CPU seems stuck at %04X:%04X", cs, ip);
                }
            } else {
                stuck_count = 0;
            }
            last_cs = cs;
            last_ip = ip;
            last_log_us = now_us;
        }
#endif
    }

    ESP_LOGI(TAG, "CPU task ended");
    vTaskDelete(NULL);
}

/**
 * Display refresh task
 * Monitors video memory for changes and updates the EPD
 */
static uint32_t compute_min_update_interval_ms_from_fps(float fps)
{
    if (!(fps > 0.0f) || !isfinite(fps)) {
        return 0;
    }
    float interval_ms_f = 1000.0f / fps;
    if (!isfinite(interval_ms_f) || interval_ms_f <= 0.0f) {
        return 0;
    }
    uint32_t interval_ms = (uint32_t)ceilf(interval_ms_f);
    if (interval_ms == 0) {
        interval_ms = 1;
    }
    return interval_ms;
}

static void display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display task started");

    uint32_t last_update = 0;
    uint32_t min_update_interval_ms = 0;
    uint32_t last_settings_gen = 0;
    bool pause_cpu_on_refresh = true;
    bool probe_drawn = false;
    uint32_t last_video_log = 0;

    while (g_running)
    {
        uint32_t gen = app_settings_generation();
        if (gen != last_settings_gen) {
            float fps = app_settings_display_fps();
            min_update_interval_ms = compute_min_update_interval_ms_from_fps(fps);
            pause_cpu_on_refresh = app_settings_display_pause_cpu_on_refresh();
            last_settings_gen = gen;
            ESP_LOGI(TAG, "Display settings updated: fps=%.6g min_interval_ms=%u partial=%d pause_cpu=%d",
                     (double)fps, (unsigned)min_update_interval_ms,
                     app_settings_display_partial_refresh() ? 1 : 0,
                     pause_cpu_on_refresh ? 1 : 0);
        }

        const bool is_graphics = (video_get_mode() == 0x13);

        // WiFi status overlay (text mode only; avoid stealing time from BT/WiFi in graphics mode).
        if (!is_graphics) {
            draw_status_line_overlay(false);
        }

        if (!probe_drawn)
        {
            ESP_LOGI(TAG, "Display probe: drawing test message");
            epd_clear();
            epd_draw_string(0, 0, "EPD PROBE\nIf you see this, display OK.", 0x07);
            if (pause_cpu_on_refresh) {
                if (!cpu_pause_request(200)) {
                    ESP_LOGW(TAG, "CPU pause timeout before display probe refresh");
                }
            }
            epd_refresh_full();
            if (pause_cpu_on_refresh) {
                cpu_pause_release(200);
            }
            probe_drawn = true;
            if (!is_graphics) {
                draw_status_line_overlay(true);
            }
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Urgent TTY-driven refresh: bypass FPS limiter to keep text output stable/responsive.
        uint8_t urgent_first = 0;
        uint8_t urgent_last = 0;
        if (video_take_urgent_refresh(&urgent_first, &urgent_last)) {
            if (!is_graphics && pause_cpu_on_refresh) {
                if (!cpu_pause_request(200)) {
                    ESP_LOGW(TAG, "CPU pause timeout before urgent display refresh");
                }
            }
            video_render_text_rows_to_epd(urgent_first, urgent_last);
            if (!is_graphics && pause_cpu_on_refresh) {
                cpu_pause_release(200);
            }
            last_update = now;
            if (!is_graphics) {
                draw_status_line_overlay(true);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Check if video memory has changed
        if (video_needs_refresh())
        {
            // Rate limit updates to prevent excessive EPD refreshes
            if (min_update_interval_ms == 0 || (now - last_update) >= min_update_interval_ms)
            {
                // In graphics mode (mode 13h), EPD refresh can take a long time (esp. with WiFi on core 0).
                // Pausing the 8086 for the whole refresh makes games appear "frozen". Allow tearing instead.
                if (!is_graphics && pause_cpu_on_refresh) {
                    if (!cpu_pause_request(200)) {
                        ESP_LOGW(TAG, "CPU pause timeout before display refresh");
                    }
                }
                // Render video memory to EPD
                video_render_to_epd();
                if (!is_graphics && pause_cpu_on_refresh) {
                    cpu_pause_release(200);
                }
                last_update = now;
                if (!is_graphics) {
                    draw_status_line_overlay(true);
                }
            }
        }

        // Check every 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Display task ended");
    vTaskDelete(NULL);
}

/**
 * Initialize timer tick for PC emulation
 */
static esp_err_t init_timer_tick(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_tick_callback,
        .name = "dos_timer"};

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_tick_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_tick_handle, TIMER_TICK_INTERVAL_US));

    return ESP_OK;
}

/**
 * Display boot message on EPD
 */
static void display_boot_message(const char *message)
{
    ESP_LOGI(TAG, "%s", message);
    
    // Use video subsystem to render message so it uses the correct font size
    video_clear_screen(0x07); // Clear to Light Gray on Black (standard DOS)
    video_write_string(0, 0, message, 0x07);
    
    // Force render to EPD
    video_force_refresh();
    video_render_to_epd();
}

static lgfx::epd_mode_t display_map_epd_mode(epd_refresh_mode_t mode)
{
    switch (mode) {
        case EPD_REFRESH_FAST:
            return lgfx::epd_mode_t::epd_fastest;
        case EPD_REFRESH_PARTIAL:
            return lgfx::epd_mode_t::epd_fast;
        case EPD_REFRESH_FULL:
        default:
            return lgfx::epd_mode_t::epd_quality;
    }
}

static void display_apply_epd_mode(epd_refresh_mode_t mode)
{
    static lgfx::epd_mode_t last = (lgfx::epd_mode_t)0;
    lgfx::epd_mode_t target = display_map_epd_mode(mode);
    if (last != target) {
        M5.Display.setEpdMode(target);
        last = target;
    }
}

/**
 * Flush display callback
 */
static void flush_display_cb(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data)
{
    ESP_LOGD(TAG, "flush_display_cb: %ux%u at %u,%u",
             (unsigned)w, (unsigned)h, (unsigned)x, (unsigned)y);

    display_apply_epd_mode(epd_get_refresh_mode());

    // Push 8-bit grayscale image to M5GFX
    M5.Display.pushImage(x, y, w, h, data);

    const bool full = (x == 0 && y == 0 && w == EPD_WIDTH && h == EPD_HEIGHT);
    if (full) {
        M5.Display.display();
    } else {
        M5.Display.display(x, y, w, h);
    }
    waitForDisplayReady(full ? 6000 : 2000);
}

static uint16_t read_le16(const uint8_t *buf)
{
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static uint32_t read_le32(const uint8_t *buf)
{
    return (uint32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
}

static bool is_power_of_two_u8(uint8_t v)
{
    return v != 0 && (v & (v - 1)) == 0;
}

static bool looks_like_fat_boot_sector(const uint8_t *sector)
{
    if (!sector) {
        return false;
    }
    if (sector[510] != 0x55 || sector[511] != 0xAA) {
        return false;
    }

    const uint8_t jmp = sector[0];
    if (jmp != 0xEB && jmp != 0xE9) {
        return false;
    }

    const uint16_t bytes_per_sector = read_le16(sector + 11);
    const uint8_t sectors_per_cluster = sector[13];
    const uint16_t reserved_sectors = read_le16(sector + 14);
    const uint8_t fats = sector[16];
    const uint16_t root_entries = read_le16(sector + 17);
    const uint16_t total_sectors16 = read_le16(sector + 19);
    const uint32_t total_sectors32 = read_le32(sector + 32);

    if (bytes_per_sector != 512) {
        return false;
    }
    if (!is_power_of_two_u8(sectors_per_cluster)) {
        return false;
    }
    if (reserved_sectors == 0 || fats == 0 || fats > 2) {
        return false;
    }
    if (root_entries == 0) {
        return false;
    }
    if (total_sectors16 == 0 && total_sectors32 == 0) {
        return false;
    }

    return true;
}

static bool mbr_find_active_partition(const uint8_t *mbr,
                                      uint32_t *out_lba,
                                      uint32_t *out_sectors,
                                      bool *out_has_partition)
{
    if (!mbr || mbr[510] != 0x55 || mbr[511] != 0xAA) {
        if (out_has_partition) {
            *out_has_partition = false;
        }
        return false;
    }

    bool has_partition = false;
    bool found_active = false;

    for (int i = 0; i < 4; i++) {
        const uint8_t *pte = mbr + 446 + (i * 16);
        const uint8_t status = pte[0];
        const uint8_t type = pte[4];
        const uint32_t lba = read_le32(pte + 8);
        const uint32_t sectors = read_le32(pte + 12);

        if (type != 0x00 && lba != 0 && sectors != 0) {
            has_partition = true;
            if (!found_active && status == 0x80) {
                if (out_lba) {
                    *out_lba = lba;
                }
                if (out_sectors) {
                    *out_sectors = sectors;
                }
                found_active = true;
            }
        }
    }

    if (out_has_partition) {
        *out_has_partition = has_partition;
    }
    return found_active;
}

static bool root_dir_has_system_files(uint8_t drive, const uint8_t *boot_sector);

static bool hdd_boot_sector_ready(uint8_t drive)
{
    uint8_t sector[512];
    if (!disk_read_chs(drive, 0, 0, 1, 1, sector)) {
        ESP_LOGW(TAG, "HDD boot probe failed (DL=%02X err=%02X)", drive, disk_get_error());
        return false;
    }

    if (looks_like_fat_boot_sector(sector)) {
        return root_dir_has_system_files(drive, sector);
    }

    bool has_partition = false;
    uint32_t part_lba = 0;
    if (!mbr_find_active_partition(sector, &part_lba, NULL, &has_partition)) {
        if (!has_partition) {
            ESP_LOGW(TAG, "HDD MBR has no partitions");
        } else {
            ESP_LOGW(TAG, "HDD MBR has no active partition");
        }
        return false;
    }

    uint8_t vbr[512];
    if (!disk_read_lba(drive, part_lba, 1, vbr)) {
        ESP_LOGW(TAG, "HDD VBR read failed (LBA=%lu DL=%02X err=%02X)",
                 (unsigned long)part_lba, drive, disk_get_error());
        return false;
    }

    if (!looks_like_fat_boot_sector(vbr)) {
        ESP_LOGW(TAG, "HDD VBR missing FAT BPB (LBA=%lu DL=%02X)", (unsigned long)part_lba, drive);
        return false;
    }

    return root_dir_has_system_files(drive, vbr);
}

static bool read_image_sector0(const char *path, uint8_t out[512])
{
    if (!path || !out) {
        return false;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }
    size_t read = fread(out, 1, 512, f);
    fclose(f);
    return read == 512;
}

static bool is_standard_floppy_size(uint64_t size)
{
    return size == 368640u ||  // 360K
           size == 737280u ||  // 720K
           size == 1228800u || // 1.2M
           size == 1474560u || // 1.44M
           size == 2949120u;   // 2.88M
}

static bool should_boot_as_hdd(const char *boot_path, const char *c_drive_path)
{
    if (!boot_path) {
        return false;
    }
    if (c_drive_path && strcmp(boot_path, c_drive_path) == 0) {
        return true;
    }

    struct stat st;
    uint64_t size = 0;
    if (stat(boot_path, &st) == 0 && st.st_size > 0) {
        size = (uint64_t)st.st_size;
    }
    if (size && is_standard_floppy_size(size)) {
        return false;
    }

    uint8_t sector[512];
    if (read_image_sector0(boot_path, sector)) {
        if (looks_like_fat_boot_sector(sector)) {
            return false;
        }
        bool has_partition = false;
        if (mbr_find_active_partition(sector, NULL, NULL, &has_partition)) {
            return true;
        }
        if (has_partition) {
            return true;
        }
    }

    return size > 2949120u;
}

static void sanitize_ascii(char *dst, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t c = src[i];
        dst[i] = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
    }
    dst[len] = '\0';
}

static void log_boot_bpb(const uint8_t *boot_sector)
{
    char oem[9];
    char vol_label[12];
    char fs_type[9];
    sanitize_ascii(oem, boot_sector + 3, 8);

    uint16_t bytes_per_sector = read_le16(boot_sector + 11);
    uint8_t sectors_per_cluster = boot_sector[13];
    uint16_t reserved_sectors = read_le16(boot_sector + 14);
    uint8_t fats = boot_sector[16];
    uint16_t root_entries = read_le16(boot_sector + 17);
    uint16_t total_sectors16 = read_le16(boot_sector + 19);
    uint8_t media = boot_sector[21];
    uint16_t sectors_per_fat16 = read_le16(boot_sector + 22);
    uint16_t sectors_per_track = read_le16(boot_sector + 24);
    uint16_t heads = read_le16(boot_sector + 26);
    uint32_t hidden_sectors = read_le32(boot_sector + 28);
    uint32_t total_sectors32 = read_le32(boot_sector + 32);
    uint8_t drive_num = boot_sector[36];
    uint8_t boot_sig = boot_sector[38];
    sanitize_ascii(vol_label, boot_sector + 43, 11);
    sanitize_ascii(fs_type, boot_sector + 54, 8);

    ESP_LOGI(TAG, "Boot: JMP=%02X %02X %02X OEM=%s",
             boot_sector[0], boot_sector[1], boot_sector[2], oem);
    ESP_LOGI(TAG, "BPB: OEM=%s BPS=%u SPC=%u RS=%u FATs=%u Root=%u",
             oem, bytes_per_sector, sectors_per_cluster, reserved_sectors, fats, root_entries);
    ESP_LOGI(TAG, "BPB: TS16=%u TS32=%lu Media=%02X SPF=%u SPT=%u Heads=%u Hidden=%lu",
             total_sectors16, total_sectors32, media, sectors_per_fat16, sectors_per_track, heads, hidden_sectors);
    ESP_LOGI(TAG, "BPB: Drive=%02X BootSig=%02X Label=%s FSType=%s",
             drive_num, boot_sig, vol_label, fs_type);
}

static void log_boot_sector_patterns(const uint8_t *boot_sector)
{
    int int10_off = -1;
    int int13_off = -1;
    int int18_off = -1;
    int int19_off = -1;

    for (int i = 0; i < 511; i++) {
        if (boot_sector[i] == 0xCD) {
            uint8_t intnum = boot_sector[i + 1];
            if (intnum == 0x10 && int10_off < 0) {
                int10_off = i;
            } else if (intnum == 0x13 && int13_off < 0) {
                int13_off = i;
            } else if (intnum == 0x18 && int18_off < 0) {
                int18_off = i;
            } else if (intnum == 0x19 && int19_off < 0) {
                int19_off = i;
            }
        }
    }

    ESP_LOGI(TAG, "Boot: INT10=%s INT13=%s INT18=%s INT19=%s",
             (int10_off >= 0) ? "yes" : "no",
             (int13_off >= 0) ? "yes" : "no",
             (int18_off >= 0) ? "yes" : "no",
             (int19_off >= 0) ? "yes" : "no");
    if (int13_off >= 0) {
        ESP_LOGI(TAG, "Boot: INT 13h opcode at +%d", int13_off);
    }
}

static void log_boot_sector_slice(const uint8_t *boot_sector, uint16_t start, uint16_t len)
{
    if (start >= 512) {
        return;
    }
    if (start + len > 512) {
        len = (uint16_t)(512 - start);
    }
    for (uint16_t offset = 0; offset < len; offset += 16) {
        uint16_t line = (uint16_t)(start + offset);
        uint16_t remaining = (uint16_t)(len - offset);
        uint16_t count = remaining > 16 ? 16 : remaining;
        uint8_t bytes[16] = {0};
        for (uint16_t i = 0; i < count; i++) {
            bytes[i] = boot_sector[line + i];
        }
        ESP_LOGI(TAG, "Boot+%02X: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 line,
                 bytes[0], bytes[1], bytes[2], bytes[3],
                 bytes[4], bytes[5], bytes[6], bytes[7],
                 bytes[8], bytes[9], bytes[10], bytes[11],
                 bytes[12], bytes[13], bytes[14], bytes[15]);
    }
}

typedef struct root_dir_scan_t {
    bool has_io;
    bool has_msdos;
    bool has_kernel;
    bool has_ibmbio;
    bool has_ibmdos;
} root_dir_scan_t;

static bool scan_root_dir(uint8_t drive, const uint8_t *boot_sector, bool log_entries, root_dir_scan_t *out)
{
    uint16_t bytes_per_sector = read_le16(boot_sector + 11);
    uint16_t reserved_sectors = read_le16(boot_sector + 14);
    uint8_t fats = boot_sector[16];
    uint16_t root_entries = read_le16(boot_sector + 17);
    uint16_t sectors_per_fat16 = read_le16(boot_sector + 22);
    uint32_t hidden_sectors = read_le32(boot_sector + 28);

    if (bytes_per_sector == 0 || sectors_per_fat16 == 0 || root_entries == 0) {
        ESP_LOGW(TAG, "RootDir: invalid BPB, skipping root scan");
        return false;
    }

    uint16_t root_dir_sectors = (uint16_t)(((uint32_t)root_entries * 32 + (bytes_per_sector - 1)) / bytes_per_sector);
    if (root_dir_sectors == 0) {
        ESP_LOGW(TAG, "RootDir: zero-sized root directory");
        return false;
    }

    uint32_t root_dir_lba = hidden_sectors + (uint32_t)reserved_sectors + (uint32_t)fats * sectors_per_fat16;
    uint32_t buf_len = (uint32_t)root_dir_sectors * bytes_per_sector;
    uint8_t *buf = (uint8_t *)malloc(buf_len);
    if (!buf) {
        ESP_LOGW(TAG, "RootDir: buffer alloc failed (%lu bytes)", buf_len);
        return false;
    }

    if (!disk_read_lba(drive, root_dir_lba, (uint8_t)root_dir_sectors, buf)) {
        ESP_LOGE(TAG, "RootDir: read failed LBA=%lu sectors=%u", root_dir_lba, root_dir_sectors);
        free(buf);
        return false;
    }

    root_dir_scan_t scan = {0};
    int logged = 0;

    for (uint32_t offset = 0; offset + 32 <= buf_len; offset += 32) {
        const uint8_t *ent = buf + offset;
        if (ent[0] == 0x00) {
            break;  // end of directory
        }
        if (ent[0] == 0xE5 || ent[11] == 0x0F) {
            continue;  // deleted or long filename
        }

        if (memcmp(ent, "IO      SYS", 11) == 0) {
            scan.has_io = true;
        } else if (memcmp(ent, "MSDOS   SYS", 11) == 0) {
            scan.has_msdos = true;
        } else if (memcmp(ent, "KERNEL  SYS", 11) == 0) {
            scan.has_kernel = true;
        } else if (memcmp(ent, "IBMBIO  COM", 11) == 0) {
            scan.has_ibmbio = true;
        } else if (memcmp(ent, "IBMDOS  COM", 11) == 0) {
            scan.has_ibmdos = true;
        }

        if (log_entries && logged < 10) {
            char name[13];
            sanitize_ascii(name, ent, 8);
            char ext[4];
            sanitize_ascii(ext, ent + 8, 3);
            uint16_t first_cluster = read_le16(ent + 26);
            uint32_t size = read_le32(ent + 28);
            ESP_LOGI(TAG, "RootDir: %.8s.%.3s attr=%02X cl=%u size=%lu",
                     name, ext, ent[11], first_cluster, size);
            logged++;
        }
    }

    if (log_entries) {
        ESP_LOGI(TAG, "RootDir: IO.SYS=%s MSDOS.SYS=%s KERNEL.SYS=%s IBMBIO.COM=%s IBMDOS.COM=%s",
                 scan.has_io ? "yes" : "no",
                 scan.has_msdos ? "yes" : "no",
                 scan.has_kernel ? "yes" : "no",
                 scan.has_ibmbio ? "yes" : "no",
                 scan.has_ibmdos ? "yes" : "no");
    }
    if (out) {
        *out = scan;
    }
    free(buf);
    return true;
}

static void log_root_dir(uint8_t drive, const uint8_t *boot_sector)
{
    (void)scan_root_dir(drive, boot_sector, true, NULL);
}

static bool root_dir_has_system_files(uint8_t drive, const uint8_t *boot_sector)
{
    root_dir_scan_t scan = {0};
    if (!scan_root_dir(drive, boot_sector, false, &scan)) {
        return false;
    }
    if ((scan.has_io && scan.has_msdos) || scan.has_kernel || (scan.has_ibmbio && scan.has_ibmdos)) {
        return true;
    }
    ESP_LOGW(TAG, "HDD boot: missing system files (IO.SYS=%s MSDOS.SYS=%s KERNEL.SYS=%s IBMBIO.COM=%s IBMDOS.COM=%s)",
             scan.has_io ? "yes" : "no",
             scan.has_msdos ? "yes" : "no",
             scan.has_kernel ? "yes" : "no",
             scan.has_ibmbio ? "yes" : "no",
             scan.has_ibmdos ? "yes" : "no");
    return false;
}

static bool load_boot_sector(uint8_t drive)
{
    uint8_t boot_sector[512];
    if (!disk_read_chs(drive, 0, 0, 1, 1, boot_sector))
    {
        ESP_LOGE(TAG, "Boot sector read failed (DL=%02X err=%02X)",
                 drive, disk_get_error());
        return false;
    }

    bool used_vbr = false;
    uint32_t vbr_lba = 0;
    if (!looks_like_fat_boot_sector(boot_sector)) {
        bool has_partition = false;
        uint32_t part_lba = 0;
        if (mbr_find_active_partition(boot_sector, &part_lba, NULL, &has_partition)) {
            uint8_t vbr[512];
            if (disk_read_lba(drive, part_lba, 1, vbr)) {
                if (looks_like_fat_boot_sector(vbr)) {
                    memcpy(boot_sector, vbr, sizeof(boot_sector));
                    used_vbr = true;
                    vbr_lba = part_lba;
                } else {
                    ESP_LOGW(TAG, "Boot VBR missing FAT BPB (LBA=%lu DL=%02X)",
                             (unsigned long)part_lba, drive);
                }
            } else {
                ESP_LOGW(TAG, "Boot VBR read failed (LBA=%lu DL=%02X err=%02X)",
                         (unsigned long)part_lba, drive, disk_get_error());
            }
        } else if (has_partition) {
            ESP_LOGW(TAG, "Boot MBR has no active partition (DL=%02X)", drive);
        }
    }

    mem_write_block(0x7C00, boot_sector, sizeof(boot_sector));
    ESP_LOGI(TAG, "Boot sector loaded%s (sig=%02X%02X)",
             used_vbr ? " (VBR)" : "",
             boot_sector[510], boot_sector[511]);
    if (used_vbr) {
        ESP_LOGI(TAG, "Boot: using active partition VBR at LBA=%lu", (unsigned long)vbr_lba);
    }
    log_boot_bpb(boot_sector);
    log_boot_sector_patterns(boot_sector);
    log_boot_sector_slice(boot_sector, 0x30, 0x30);
    log_boot_sector_slice(boot_sector, 0x60, 0x30);
    log_boot_sector_slice(boot_sector, 0x90, 0x30);
    log_root_dir(drive, boot_sector);
    return true;
}

static void set_log_level_from_settings(const char *tag, esp_log_level_t default_level)
{
    int level = -1;
    if (app_settings_get_log_level(tag, &level) == ESP_OK)
    {
        // If 255 (unset) or invalid (>5 or <0), use default.
        // Valid levels are 0..5
        if (level >= 0 && level <= 5)
        {
            ESP_LOGI("LOGLEVEL", "Setting log level for tag '%s' to %d from settings", tag, level);
            esp_log_level_set(tag, (esp_log_level_t)level);
            return;
        }
    }

    ESP_LOGI("LOGLEVEL", "Setting log level for tag '%s' to %d from settings", tag, level);
    esp_log_level_set(tag, default_level);
}

/**
 * Main application entry point
 */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "M5PaperDOS starting...");

    // Configure log levels
    esp_log_level_set("CPU_TRACE", ESP_LOG_WARN);
    esp_log_level_set("CPU_DIAG", ESP_LOG_WARN);
    esp_log_level_set("DISKSWAP", ESP_LOG_WARN);
    esp_log_level_set("PORT", ESP_LOG_WARN);
    esp_log_level_set("BT_GAP", ESP_LOG_WARN);
    esp_log_level_set("KBD", ESP_LOG_WARN);
    esp_log_level_set("SPK", ESP_LOG_WARN);
    esp_log_level_set("TTY", ESP_LOG_INFO);
    esp_log_level_set("BIOS_TTY", ESP_LOG_WARN);
    esp_log_level_set("VIDEO", ESP_LOG_WARN);
    esp_log_level_set("CGA", ESP_LOG_WARN);
    esp_log_level_set("EPD", ESP_LOG_INFO);

    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());

    auto cfg = M5.config(); // keep main power rail

    // Configure device-specific settings
#ifdef CONFIG_EBOOK_DEVICE_M5PAPERS3
    // M5PaperS3 has IMU enabled for gyroscope-based rotation
    cfg.internal_imu = true;
    cfg.internal_rtc = true;
    cfg.internal_spk = false;
    cfg.internal_mic = false;
    // Ensure display init even if auto-detect fails.
    cfg.fallback_board = m5::board_t::board_M5PaperS3;
#else
    // Original M5Paper - no IMU
    cfg.internal_imu = false;
    cfg.internal_rtc = true;
    cfg.internal_spk = false;
    cfg.internal_mic = false;
    // Ensure display init even if auto-detect fails.
    cfg.fallback_board = m5::board_t::board_M5Paper;
#endif

    M5.begin(cfg);
    ESP_LOGI(TAG, "M5 initialized. Board type: %d", (int)M5.getBoard());

    // Ensure display is awake and powered
    M5.Display.wakeup();
    M5.Display.setRotation(1);  // Landscape: 960x540 for M5Paper
    M5.Display.setColorDepth(8);

    M5.Display.setEpdMode(lgfx::epd_mode_t::epd_quality);
    M5.Display.invertDisplay(true); // Ensure 0 is Black and 255 is White
    if (!waitForDisplayReady(6000))
    {
        ESP_LOGW(TAG, "Display busy on boot; continuing without wait");
    }
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.display();

    (void)app_settings_init();

    // Configure Log Levels
    set_log_level_from_settings("CPU_TRACE", ESP_LOG_INFO);    // was M5PAPER_CPU_TRACE_LOGS 1
    set_log_level_from_settings("CPU_DIAG", ESP_LOG_WARN);
    set_log_level_from_settings("DISKSWAP", ESP_LOG_INFO);
    set_log_level_from_settings("BT_GAP", ESP_LOG_ERROR);      // was M5PAPER_BT_VERBOSE_LOGS 0
    set_log_level_from_settings("KBD", ESP_LOG_ERROR);         // was M5PAPER_KBD_DEBUG 0
    set_log_level_from_settings("SPK", ESP_LOG_ERROR);         // was M5PAPER_SPEAKER_DEBUG 0
    set_log_level_from_settings("TTY", ESP_LOG_INFO);          // was M5PAPER_TTY_DEBUG 1
    set_log_level_from_settings("BIOS_TTY", ESP_LOG_ERROR);    // was M5PAPER_BIOS_TTY_CHAR_DEBUG 0
    set_log_level_from_settings("VIDEO", ESP_LOG_ERROR);       // was M5PAPER_VIDEO_CHAR_DEBUG 0
    set_log_level_from_settings("PORT", ESP_LOG_INFO);         // was M5PAPER_PORT_TRACE 1
    set_log_level_from_settings("CGA", ESP_LOG_ERROR);         // was M5PAPER_VIDEO_CGA_DEBUG 0

    // Initialize EPD display
    ESP_LOGI(TAG, "Initializing EPD display...");
    if (epd_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize EPD display");
        return;
    }

    // Register flush callback to use M5GFX for display updates
    epd_register_flush_cb(flush_display_cb);

    // DEBUG: Color Test
    /*
    ESP_LOGI(TAG, "DEBUG: Running Color Test");
    uint8_t* test_buf = (uint8_t*)heap_caps_malloc(EPD_WIDTH * 100, MALLOC_CAP_SPIRAM);
    if (test_buf) {
        // Strip 1: 0x00 (Should be Black)
        memset(test_buf, 0x00, EPD_WIDTH * 100); 
        flush_display_cb(0, 0, EPD_WIDTH, 100, test_buf);
        
        // Strip 2: 0x55 (Should be Dark Gray)
        memset(test_buf, 0x55, EPD_WIDTH * 100); 
        flush_display_cb(0, 100, EPD_WIDTH, 100, test_buf);
        
        // Strip 3: 0xAA (Should be Light Gray)
        memset(test_buf, 0xAA, EPD_WIDTH * 100); 
        flush_display_cb(0, 200, EPD_WIDTH, 100, test_buf);
        
        // Strip 4: 0xFF (Should be White)
        memset(test_buf, 0xFF, EPD_WIDTH * 100); 
        flush_display_cb(0, 300, EPD_WIDTH, 100, test_buf);
        
        free(test_buf);
        ESP_LOGI(TAG, "DEBUG: Color Test Complete. Waiting 5s...");
        vTaskDelay(pdMS_TO_TICKS(5000)); 
    }
    */

    // Initialize DOS subsystems early for boot messages
    ESP_LOGI(TAG, "Initializing DOS subsystems (early)...");
    dos_mem_init();
    xms_init();
    ports_init();
    video_init();

    display_boot_message("M5Paper DOS v1.0\nInitializing...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize SD card
    ESP_LOGI(TAG, "Initializing SD card...");
    if (sdcard_init() != ESP_OK)
    {
        display_boot_message("ERROR: SD card init failed!");
        ESP_LOGE(TAG, "Failed to initialize SD card");
        return;
    }

    display_boot_message("M5Paper DOS v1.0\nSD card OK\nLoading DOS...");

    // Initialize remaining DOS subsystems
    ESP_LOGI(TAG, "Initializing remaining DOS subsystems...");

    // Load BIOS into memory
    if (!mem_load_bios(embedded_8086tiny_bios, embedded_8086tiny_bios_len))
    {
        display_boot_message("ERROR: BIOS load failed!");
        ESP_LOGE(TAG, "Failed to load BIOS");
        return;
    }

    // Initialize other subsystems
    interrupts_init();
    bios_init();
    disk_init();
    if (app_settings_bt_keyboard_enabled()) {
        ESP_LOGI(TAG, "Bluetooth enabled in settings, Initializing Bluetooth keyboard...");
        bt_keyboard_init();
    } else {
        ESP_LOGI(TAG, "Bluetooth keyboard disabled by settings");
    }

    uint8_t boot_drive = 0x00;

    char boot_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
    const char *boot_path = DOS_IMAGE_PATH;
    if (app_settings_get_boot_image_path(boot_image_path, sizeof(boot_image_path))) {
        boot_path = boot_image_path;
    }
    if (!file_exists(boot_path) && strcmp(boot_path, DOS_IMAGE_PATH) != 0) {
        ESP_LOGW(TAG, "Configured boot image not found: %s (falling back to %s)", boot_path, DOS_IMAGE_PATH);
        boot_path = DOS_IMAGE_PATH;
    }

    char c_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
    const char *c_drive_path = g_c_drive_image_path;
    if (app_settings_get_c_drive_image_path(c_drive_image_path, sizeof(c_drive_image_path))) {
        c_drive_path = c_drive_image_path;
    }

    char a_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
    const char *a_drive_path = DOS_IMAGE_PATH_A;
    if (app_settings_get_a_drive_image_path(a_drive_image_path, sizeof(a_drive_image_path))) {
        a_drive_path = a_drive_image_path;
    }
    if (!file_exists(a_drive_path)) {
        if (file_exists(DOS_IMAGE_PATH_A)) {
            a_drive_path = DOS_IMAGE_PATH_A;
        } else if (file_exists(DOS_IMAGE_PATH)) {
            a_drive_path = DOS_IMAGE_PATH;
        }
    }

    char b_drive_image_path[APP_SETTINGS_BOOT_IMAGE_PATH_MAX] = {0};
    const char *b_drive_path = DOS_IMAGE_PATH_B;
    if (app_settings_get_b_drive_image_path(b_drive_image_path, sizeof(b_drive_image_path))) {
        b_drive_path = b_drive_image_path;
    }
    if (!file_exists(b_drive_path)) {
        b_drive_path = NULL;
    }

    // Mount DOS disk image from SD card
    ESP_LOGI(TAG, "Mounting boot image: %s", boot_path);
    const bool boot_as_hdd = should_boot_as_hdd(boot_path, c_drive_path);
    if (boot_as_hdd)
    {
        if (disk_mount(0x80, boot_path, DRIVE_HDD))
        {
            boot_drive = 0x80;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to mount as HDD, trying as floppy...");
            if (!disk_mount(0x00, boot_path, DRIVE_FLOPPY))
            {
                display_boot_message("ERROR: Cannot mount boot image!");
                ESP_LOGE(TAG, "Failed to mount boot image: %s", boot_path);
                return;
            }
            boot_drive = 0x00;
        }
    }
    else
    {
        if (disk_mount(0x00, boot_path, DRIVE_FLOPPY))
        {
            boot_drive = 0x00;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to mount as floppy, trying as HDD...");
            if (!disk_mount(0x80, boot_path, DRIVE_HDD))
            {
                display_boot_message("ERROR: Cannot mount boot image!");
                ESP_LOGE(TAG, "Failed to mount boot image: %s", boot_path);
                return;
            }
            boot_drive = 0x80;
        }
    }

    if (boot_drive == 0x80) {
        if (!hdd_boot_sector_ready(boot_drive)) {
            const char *fallback_path = (a_drive_path && file_exists(a_drive_path)) ? a_drive_path : DOS_IMAGE_PATH;
            if (!file_exists(fallback_path)) {
                ESP_LOGW(TAG, "HDD boot invalid and fallback image missing: %s", fallback_path);
            } else if (!disk_mount(0x00, fallback_path, DRIVE_FLOPPY)) {
                ESP_LOGW(TAG, "HDD boot invalid; fallback mount failed: %s", fallback_path);
            } else {
                ESP_LOGW(TAG, "HDD boot invalid; falling back to %s as A:", fallback_path);
                disk_unmount(0x80);
                boot_drive = 0x00;
                boot_path = fallback_path;
            }
        }
    }

    if (boot_drive == 0x80 && a_drive_path && file_exists(a_drive_path)) {
        ESP_LOGI(TAG, "Mounting A: image: %s", a_drive_path);
        if (!disk_mount(0x00, a_drive_path, DRIVE_FLOPPY)) {
            ESP_LOGW(TAG, "Failed to mount A: image %s", a_drive_path);
        }
    }

    if (b_drive_path) {
        ESP_LOGI(TAG, "Mounting B: image: %s", b_drive_path);
        if (!disk_mount(0x01, b_drive_path, DRIVE_FLOPPY)) {
            ESP_LOGW(TAG, "Failed to mount B: image %s", b_drive_path);
        }
    }

    // Provision and mount a persistent hard disk for C: (or D: if booting from HDD).
    if (g_c_drive_enabled) {
        const uint8_t data_hdd_drive = (boot_drive >= 0x80) ? 0x81 : 0x80;
        if ((boot_drive < 0x80) || (strcmp(boot_path, c_drive_path) != 0)) {
            if (!disk_is_ready(data_hdd_drive)) {
                const uint64_t size_bytes = (uint64_t)g_c_drive_default_size_mb * 1024ull * 1024ull;
                if (!hdd_image_ensure_fat16(c_drive_path, size_bytes)) {
                    ESP_LOGW(TAG, "C: image not available (path=%s)", c_drive_path);
                } else if (!disk_mount(data_hdd_drive, c_drive_path, DRIVE_HDD)) {
                    ESP_LOGW(TAG, "Failed to mount hard drive (DL=%02X path=%s)", data_hdd_drive, c_drive_path);
                } else {
                    ESP_LOGI(TAG, "Mounted hard drive at DL=%02X (DOS %c:)", data_hdd_drive, (data_hdd_drive == 0x80) ? 'C' : 'D');
                }
            }
        } else {
            ESP_LOGI(TAG, "Booting from %s; not mounting a duplicate persistent drive", c_drive_path);
        }
    }

    uint8_t floppy_count = 0;
    if (disk_is_ready(0x00)) {
        floppy_count++;
    }
    if (disk_is_ready(0x01)) {
        floppy_count++;
    }
    bios_set_floppy_drives(floppy_count);
    bios_set_hard_drives(count_hard_drives_ready());

    wifi_sta_set_ip_callback(wifi_ip_callback);

    // Enable WiFi + web server for runtime disk swaps (non-blocking).
    // Start happens later (after `cpu_task`/`display_task` creation).
    // g_enable_network_task = true;

    // Initialize CPU
    ESP_LOGI(TAG, "Initializing 8086 CPU...");
    cpu_init(&g_cpu);
    g_cpu.regs16[REG_DX] = (g_cpu.regs16[REG_DX] & 0xFF00) | boot_drive;

    if (!load_boot_sector(boot_drive))
    {
        display_boot_message("ERROR: Boot sector read failed!");
        return;
    }

    g_cpu.sregs[SEG_CS] = 0x0000;
    g_cpu.ip = 0x7C00;
    g_cpu.sregs[SEG_DS] = 0x0000;
    g_cpu.sregs[SEG_ES] = 0x0000;
    g_cpu.sregs[SEG_SS] = 0x0000;
    g_cpu.regs16[REG_SP] = 0x7C00;

    display_boot_message("M5Paper DOS v1.0\nBooting MS-DOS...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait longer for user to see message

    // Clear screen for DOS
    epd_clear();
    epd_refresh_full();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for clear to complete

    // Start emulation
    g_running = true;

    // Initialize timer tick
    init_timer_tick();

    // Create CPU task
    if (xTaskCreatePinnedToCore(cpu_task, "cpu_task", CPU_TASK_STACK_SIZE,
                               NULL, CPU_TASK_PRIORITY, &cpu_task_handle, 1) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create cpu_task (stack=%u, prio=%u)", (unsigned)CPU_TASK_STACK_SIZE, (unsigned)CPU_TASK_PRIORITY);
        display_boot_message("ERROR: Failed to start CPU task!");
        return;
    }

    // Create display task
    if (xTaskCreatePinnedToCore(display_task, "display_task", DISPLAY_TASK_STACK_SIZE,
                               NULL, DISPLAY_TASK_PRIORITY, &display_task_handle, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create display_task (stack=%u, prio=%u)", (unsigned)DISPLAY_TASK_STACK_SIZE, (unsigned)DISPLAY_TASK_PRIORITY);
        display_boot_message("ERROR: Failed to start display task!");
        return;
    }

    g_enable_network_task = app_settings_wifi_enabled();
    ESP_LOGI(TAG, "g_enable_network_task=%d", (int)g_enable_network_task);

    if (g_enable_network_task) {
        // Keep this lower than display/CPU tasks. WiFi creates its own high-priority driver task.
        const UBaseType_t net_prio = 2;
        const uint32_t net_stack = 4096;
        if (xTaskCreatePinnedToCore(network_task, "network_task", net_stack,
                                   NULL, net_prio, NULL, 0) != pdPASS)
        {
            ESP_LOGW(TAG, "Failed to create network_task (stack=%u, prio=%u)", (unsigned)net_stack, (unsigned)net_prio);
        }
    }

    ESP_LOGI(TAG, "DOS emulation started");

    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Failed to add main task to WDT: %d", (int)wdt_err);
    }

    // Main loop - monitor system status
    int log_counter = 0;
    while (g_running)
    {
        esp_task_wdt_reset();

        if (++log_counter >= 5) {
            // Print some status info periodically
            ESP_LOGD(TAG, "CPU cycles: %llu, IP: %04X:%04X",
                     g_cpu.cycles, g_cpu.sregs[SEG_CS], g_cpu.ip);
            log_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
