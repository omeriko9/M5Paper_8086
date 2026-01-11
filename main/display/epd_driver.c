/**
 * @file epd_driver.c
 * @brief E-Paper display driver for M5Paper (IT8951)
 * 
 * This driver interfaces with the IT8951 e-paper controller
 * used in the M5Paper device.
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "epd_driver.h"
#include "font8x16.h"
#include "../config_defs.h"

static const char *TAG = "EPD";

// M5Paper IT8951 SPI pins
#define EPD_SPI_HOST    SPI2_HOST
#define EPD_PIN_MOSI    GPIO_NUM_12
#define EPD_PIN_MISO    GPIO_NUM_13
#define EPD_PIN_SCLK    GPIO_NUM_14
#define EPD_PIN_CS      GPIO_NUM_15
#define EPD_PIN_DC      GPIO_NUM_33     // Also called HRDY on IT8951
#define EPD_PIN_RST     GPIO_NUM_23
#define EPD_PIN_BUSY    GPIO_NUM_27

// IT8951 commands
#define IT8951_CMD_SYS_RUN          0x0001
#define IT8951_CMD_STANDBY          0x0002
#define IT8951_CMD_SLEEP            0x0003
#define IT8951_CMD_REG_RD           0x0010
#define IT8951_CMD_REG_WR           0x0011
#define IT8951_CMD_MEM_BST_RD_T     0x0012
#define IT8951_CMD_MEM_BST_RD_S     0x0013
#define IT8951_CMD_MEM_BST_WR       0x0014
#define IT8951_CMD_MEM_BST_END      0x0015
#define IT8951_CMD_LD_IMG           0x0020
#define IT8951_CMD_LD_IMG_AREA      0x0021
#define IT8951_CMD_LD_IMG_END       0x0022

// IT8951 registers
#define IT8951_REG_DISPLAY_BASE     0x1000
#define IT8951_REG_LUT0EWHR         (IT8951_REG_DISPLAY_BASE + 0x00)
#define IT8951_REG_LUT0XYR          (IT8951_REG_DISPLAY_BASE + 0x40)
#define IT8951_REG_LUT0BADDR        (IT8951_REG_DISPLAY_BASE + 0x80)
#define IT8951_REG_LUT0MFN          (IT8951_REG_DISPLAY_BASE + 0xC0)
#define IT8951_REG_UPIWADDR         (IT8951_REG_DISPLAY_BASE + 0x200)

// IT8951 display modes
#define IT8951_MODE_INIT    0   // Full initialization
#define IT8951_MODE_DU      1   // Direct Update (fast, monochrome)
#define IT8951_MODE_GC16    2   // 16-level grayscale
#define IT8951_MODE_GL16    3   // Ghost-less 16-level grayscale
#define IT8951_MODE_A2      6   // Fast monochrome (for animation)

// IT8951 pixel formats
#define IT8951_2BPP         0
#define IT8951_3BPP         1
#define IT8951_4BPP         2
#define IT8951_8BPP         3

// Framebuffer
static uint8_t *framebuffer = NULL;
static uint32_t fb_size = 0;

// SPI handle
#if 0
static spi_device_handle_t spi_handle = NULL;

// IT8951 device info
typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t img_buf_addr;
    char lut_version[16];
} it8951_dev_info_t;

static it8951_dev_info_t dev_info;
#endif

static epd_refresh_mode_t current_refresh_mode = EPD_REFRESH_PARTIAL;

// Forward declarations
#if 0
static void epd_wait_busy(void);
static void epd_write_cmd(uint16_t cmd);
static void epd_write_data(uint16_t data);
static uint16_t epd_read_data(void);
static void epd_write_args(uint16_t cmd, const uint16_t *args, uint16_t count);
#endif

/**
 * Wait for IT8951 to be ready
 */
#if 0
static void epd_wait_busy(void)
{
    while (gpio_get_level(EPD_PIN_BUSY) == 0) {
        vTaskDelay(1);
    }
}

/**
 * Write a command to IT8951
 */
static void epd_write_cmd(uint16_t cmd)
{
    epd_wait_busy();
    
    uint8_t tx_data[4];
    tx_data[0] = 0x60;  // Preamble for command
    tx_data[1] = 0x00;
    tx_data[2] = (cmd >> 8) & 0xFF;
    tx_data[3] = cmd & 0xFF;
    
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_data,
    };
    
    gpio_set_level(EPD_PIN_CS, 0);
    spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(EPD_PIN_CS, 1);
}

/**
 * Write data to IT8951
 */
static void epd_write_data(uint16_t data)
{
    epd_wait_busy();
    
    uint8_t tx_data[4];
    tx_data[0] = 0x00;  // Preamble for data
    tx_data[1] = 0x00;
    tx_data[2] = (data >> 8) & 0xFF;
    tx_data[3] = data & 0xFF;
    
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_data,
    };
    
    gpio_set_level(EPD_PIN_CS, 0);
    spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(EPD_PIN_CS, 1);
}

/**
 * Read data from IT8951
 */
static uint16_t epd_read_data(void)
{
    epd_wait_busy();
    
    uint8_t tx_data[4] = {0x10, 0x00, 0x00, 0x00};  // Read preamble + dummy
    uint8_t rx_data[4] = {0};
    
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    gpio_set_level(EPD_PIN_CS, 0);
    spi_device_polling_transmit(spi_handle, &t);
    
    // Read actual data
    t.length = 16;
    t.tx_buffer = NULL;
    spi_device_polling_transmit(spi_handle, &t);
    gpio_set_level(EPD_PIN_CS, 1);
    
    return (rx_data[0] << 8) | rx_data[1];
}

/**
 * Write command with arguments
 */
static void epd_write_args(uint16_t cmd, const uint16_t *args, uint16_t count)
{
    epd_write_cmd(cmd);
    for (uint16_t i = 0; i < count; i++) {
        epd_write_data(args[i]);
    }
}

/**
 * Get device info from IT8951
 */
static void epd_get_device_info(void)
{
    epd_write_cmd(0x0302);  // Get device info command
    
    epd_wait_busy();
    
    // Read device info
    dev_info.width = epd_read_data();
    dev_info.height = epd_read_data();
    dev_info.img_buf_addr = epd_read_data();
    dev_info.img_buf_addr |= ((uint32_t)epd_read_data()) << 16;
    
    // Read LUT version (8 words)
    for (int i = 0; i < 8; i++) {
        uint16_t val = epd_read_data();
        dev_info.lut_version[i*2] = val >> 8;
        dev_info.lut_version[i*2 + 1] = val & 0xFF;
    }
    
    ESP_LOGI(TAG, "IT8951: %dx%d, buf=0x%08lX", 
             dev_info.width, dev_info.height, dev_info.img_buf_addr);
}

/**
 * Set IT8951 VCOM value
 */
static void epd_set_vcom(uint16_t vcom)
{
    epd_write_cmd(0x0039);  // Set VCOM command
    epd_write_data(vcom);
}

/**
 * Load image area to IT8951
 */
static void epd_load_image_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
                                 const uint8_t *data)
{
    // Set image buffer address
    epd_write_cmd(IT8951_CMD_REG_WR);
    epd_write_data(IT8951_REG_UPIWADDR);
    epd_write_data(dev_info.img_buf_addr & 0xFFFF);
    epd_write_data((dev_info.img_buf_addr >> 16) & 0xFFFF);
    
    // Load image area settings
    uint16_t args[5];
    args[0] = (IT8951_8BPP << 4);  // 8bpp, rotate 0
    args[1] = x;
    args[2] = y;
    args[3] = w;
    args[4] = h;
    epd_write_args(IT8951_CMD_LD_IMG_AREA, args, 5);
    
    // Send pixel data
    epd_wait_busy();
    
    uint32_t size = w * h;
    uint8_t tx_preamble[2] = {0x00, 0x00};  // Data preamble
    
    gpio_set_level(EPD_PIN_CS, 0);
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_preamble,
    };
    spi_device_polling_transmit(spi_handle, &t);
    
    // Send pixel data in chunks
    const uint32_t chunk_size = 4096;
    for (uint32_t i = 0; i < size; i += chunk_size) {
        uint32_t len = (size - i > chunk_size) ? chunk_size : (size - i);
        t.length = len * 8;
        t.tx_buffer = data + i;
        spi_device_polling_transmit(spi_handle, &t);
    }
    
    gpio_set_level(EPD_PIN_CS, 1);
    
    // End load
    epd_write_cmd(IT8951_CMD_LD_IMG_END);
}

/**
 * Display area
 */
static void epd_display_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t mode)
{
    uint16_t args[7];
    args[0] = x;
    args[1] = y;
    args[2] = w;
    args[3] = h;
    args[4] = mode;
    args[5] = dev_info.img_buf_addr & 0xFFFF;
    args[6] = (dev_info.img_buf_addr >> 16) & 0xFFFF;
    
    epd_write_args(0x0037, args, 7);  // Display area command
}
#endif

/**
 * Initialize the EPD display
 */
esp_err_t epd_init(void)
{
    // Allocate framebuffer first - this is all we need for M5GFX-based rendering
    // The actual e-paper control is handled by M5Unified/M5GFX
    fb_size = EPD_WIDTH * EPD_HEIGHT;
    framebuffer = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!framebuffer) {
        ESP_LOGW(TAG, "SPIRAM allocation for framebuffer failed, trying internal memory");
        framebuffer = (uint8_t *)malloc(fb_size);
    }
    
    if (!framebuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Clear framebuffer to white
    memset(framebuffer, EPD_COLOR_WHITE, fb_size);
    
    ESP_LOGI(TAG, "EPD framebuffer allocated at %p (%lu bytes)", framebuffer, fb_size);
    return ESP_OK;

#if 0
    // NOTE: The code below is the original IT8951 SPI driver implementation.
    // It's disabled because M5Paper uses M5Unified/M5GFX for display control.
    // Keeping it for reference in case direct IT8951 control is needed later.

    ESP_LOGI(TAG, "Initializing EPD driver...");
    
    // Configure GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EPD_PIN_CS) | (1ULL << EPD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << EPD_PIN_BUSY);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // Set initial states
    gpio_set_level(EPD_PIN_CS, 1);
    gpio_set_level(EPD_PIN_RST, 1);
    
    // Reset IT8951
    gpio_set_level(EPD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EPD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = EPD_PIN_MOSI,
        .miso_io_num = EPD_PIN_MISO,
        .sclk_io_num = EPD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 65536,
    };
    
    esp_err_t ret = spi_bus_initialize(EPD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %d", ret);
        return ret;
    }
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 20 * 1000 * 1000,  // 20 MHz
        .mode = 0,
        .spics_io_num = -1,  // Manual CS control
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(EPD_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %d", ret);
        return ret;
    }
    
    // Wake up IT8951
    epd_write_cmd(IT8951_CMD_SYS_RUN);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Get device info
    epd_get_device_info();
    
    // Set VCOM (typical value for M5Paper is around -2.0V)
    epd_set_vcom(2000);
    
    // Allocate framebuffer
    fb_size = EPD_WIDTH * EPD_HEIGHT;
    framebuffer = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!framebuffer) {
        ESP_LOGW(TAG, "SPIRAM allocation for framebuffer failed, trying internal memory");
        framebuffer = (uint8_t *)malloc(fb_size);
    }
    
    if (!framebuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Framebuffer allocated at %p", framebuffer);
    
    // Clear framebuffer to white
    memset(framebuffer, EPD_COLOR_WHITE, fb_size);
    
    ESP_LOGI(TAG, "EPD initialized successfully");
    return ESP_OK;
#endif
}

/**
 * Deinitialize the EPD display
 */
void epd_deinit(void)
{
    if (framebuffer) {
        free(framebuffer);
        framebuffer = NULL;
    }
    
#if 0
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
    }
    
    spi_bus_free(EPD_SPI_HOST);
#endif
}

/**
 * Clear the display
 */
void epd_clear(void)
{
    if (!framebuffer) return;
    
    memset(framebuffer, EPD_COLOR_WHITE, fb_size);
}

static epd_flush_cb_t g_flush_cb = NULL;

void epd_register_flush_cb(epd_flush_cb_t cb)
{
    g_flush_cb = cb;
}

/**
 * Full refresh
 */
void epd_refresh_full(void)
{
    if (!framebuffer) return;
    
    if (g_flush_cb) {
        g_flush_cb(0, 0, EPD_WIDTH, EPD_HEIGHT, framebuffer);
    }
    
    // Wait for refresh to complete
    //vTaskDelay(pdMS_TO_TICKS(2000));
}

/**
 * Partial refresh
 */
void epd_refresh_partial(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if (!framebuffer) return;
    if (!g_flush_cb) return;
    
    // Align to 4 pixels (IT8951 requirement)
    // We expand the region to align it
    uint16_t x1 = (x / 4) * 4;
    uint16_t x2 = ((x + w + 3) / 4) * 4;
    
    // Also align y and h to 4 pixels just in case
    uint16_t y1 = (y / 4) * 4;
    uint16_t y2 = ((y + h + 3) / 4) * 4;
    
    x = x1;
    y = y1;
    w = x2 - x1;
    h = y2 - y1;
    
    if (x + w > EPD_WIDTH) w = EPD_WIDTH - x;
    if (y + h > EPD_HEIGHT) h = EPD_HEIGHT - y;
    if (w == 0 || h == 0) return;
    
    // If the region is full-width, it's already contiguous in the framebuffer.
    if (x == 0 && w == EPD_WIDTH) {
        g_flush_cb(x, y, w, h, framebuffer + (y * EPD_WIDTH));
        return;
    }

    // Use a small reusable stripe buffer to avoid large transient allocations
    // (which can fail/fragment heap when WiFi+lwIP are running).
    uint16_t stripe_h = h;
    if (stripe_h > 32) stripe_h = 32;
    stripe_h = (uint16_t)((stripe_h / 4) * 4);
    if (stripe_h == 0) stripe_h = 4;

    uint8_t *stripe = (uint8_t *)heap_caps_malloc((size_t)w * stripe_h, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!stripe) {
        stripe = (uint8_t *)heap_caps_malloc((size_t)w * stripe_h, MALLOC_CAP_8BIT);
    }
    if (!stripe && stripe_h > 4) {
        stripe_h = 4;
        stripe = (uint8_t *)heap_caps_malloc((size_t)w * stripe_h, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!stripe) {
            stripe = (uint8_t *)heap_caps_malloc((size_t)w * stripe_h, MALLOC_CAP_8BIT);
        }
    }

    if (!stripe) {
        ESP_LOGW(TAG, "Partial refresh buffer alloc failed (%ux%u), falling back to full refresh", (unsigned)w, (unsigned)h);
        epd_refresh_full();
        vTaskDelay(pdMS_TO_TICKS(300));
        return;
    }

    for (uint16_t row0 = 0; row0 < h; row0 = (uint16_t)(row0 + stripe_h)) {
        uint16_t cur_h = stripe_h;
        if ((uint16_t)(row0 + cur_h) > h) {
            cur_h = (uint16_t)(h - row0);
        }
        for (uint16_t row = 0; row < cur_h; row++) {
            memcpy(stripe + ((size_t)row * w),
                   framebuffer + ((size_t)(y + row0 + row) * EPD_WIDTH) + x,
                   w);
        }
        g_flush_cb(x, (uint16_t)(y + row0), w, cur_h, stripe);
    }

    free(stripe);
}

/**
 * Update a region
 */
void epd_update_region(uint16_t y1, uint16_t x1, uint16_t y2, uint16_t x2, const uint8_t *data)
{
    if (!framebuffer) return;
    
    uint16_t w = x2 - x1;
    uint16_t h = y2 - y1;
    
    // Copy data to framebuffer
    // Note: data is assumed to be already in the right format
    
    // Perform partial refresh
    epd_refresh_partial(x1, y1, w, h);
}

/**
 * Draw a pixel
 */
void epd_draw_pixel(uint16_t x, uint16_t y, uint8_t color)
{
    if (!framebuffer || x >= EPD_WIDTH || y >= EPD_HEIGHT) return;
    
    framebuffer[y * EPD_WIDTH + x] = color;
}

/**
 * Draw a character
 */
void epd_draw_char(uint16_t x, uint16_t y, char c, uint8_t fg, uint8_t bg)
{
    if (!framebuffer || x >= EPD_WIDTH - 8 || y >= EPD_HEIGHT - 16) return;
    
    const uint8_t *glyph = font8x16_get_glyph(c);
    
    for (int row = 0; row < 16; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            uint8_t color = (bits & (0x80 >> col)) ? fg : bg;
            framebuffer[(y + row) * EPD_WIDTH + x + col] = color;
        }
    }
}

/**
 * Draw string
 */
void epd_draw_string(uint8_t row, uint8_t col, const char *str, uint8_t attr)
{
    uint16_t x = col * EPD_TEXT_CHAR_WIDTH;
    uint16_t y = row * EPD_TEXT_CHAR_HEIGHT;
    
    uint8_t fg = (attr & 0x08) ? EPD_COLOR_BLACK : EPD_COLOR_BLACK;
    uint8_t bg = (attr & 0x70) ? EPD_COLOR_GRAY : EPD_COLOR_WHITE;
    
    while (*str) {
        if (*str == '\n') {
            y += EPD_TEXT_CHAR_HEIGHT;
            x = col * EPD_TEXT_CHAR_WIDTH;
        } else {
            epd_draw_char(x, y, *str, fg, bg);
            x += EPD_TEXT_CHAR_WIDTH;
            if (x >= EPD_WIDTH) {
                x = 0;
                y += EPD_TEXT_CHAR_HEIGHT;
            }
        }
        str++;
    }
}

/**
 * Draw DOS text character with attribute
 */
void epd_draw_text_char(uint8_t row, uint8_t col, uint8_t c, uint8_t attr)
{
    char safe = (c >= 0x20 && c <= 0x7E) ? (char)c : '.';
    ESP_LOGD(TAG, "epd_draw_text_char: %c (0x%02X) at %u,%u", safe, (unsigned)c, (unsigned)row, (unsigned)col);

    if (!framebuffer) return;
    if (row >= EPD_TEXT_ROWS || col >= EPD_TEXT_COLS) return;
    
    // Map DOS colors to EPD colors
    // DOS 0x07 is White on Black. On E-Paper we want Black on White.
    
    uint8_t fg_dos = attr & 0x0F;
    uint8_t bg_dos = (attr >> 4) & 0x07;
    
    // Paper Mode: Black text (0x00) on White background (0xFF)
    // If DOS FG is set (not black), use EPD Black (0x00). Else White (0xFF).
    uint8_t fg = (fg_dos > 0) ? 0x00 : 0xFF; 
    // If DOS BG is set (not black), use EPD Black (0x00). Else White (0xFF).
    uint8_t bg = (bg_dos > 0) ? 0x00 : 0xFF; 

    // Avoid per-frame logging from hot path (can severely slow emulation when WiFi is enabled).
    // Enable M5PAPER_TTY_DEBUG for detailed character-level logs instead.
    
    uint16_t x_pos = col * EPD_TEXT_CHAR_WIDTH;
    uint16_t y_pos = row * EPD_TEXT_CHAR_HEIGHT;
    
    const uint8_t* font_ptr = font8x16_get_glyph(c);
    
    // Scale 8x16 font to 12x21 cell
    for (int y = 0; y < 21; y++) {
        // Map target Y (0..20) to source Y (0..15)
        int src_y = (y * 16) / 21;
        uint8_t row_data = font_ptr[src_y];
        
        uint16_t target_y = y_pos + y;
        if (target_y >= EPD_HEIGHT) continue;
        
        uint32_t row_offset = target_y * EPD_WIDTH;
        
        for (int x = 0; x < 12; x++) {
            // Map target X (0..11) to source X (0..7)
            int src_x = (x * 8) / 12;
            
            // Check if pixel is set in font
            // Font is MSB first (bit 7 is left-most)
            bool pixel_set = (row_data & (1 << (7 - src_x)));
            
            uint8_t color = pixel_set ? fg : bg;
            
            uint16_t target_x = x_pos + x;
            if (target_x < EPD_WIDTH) {
                framebuffer[row_offset + target_x] = color;
            }
        }
    }
}

/**
 * Render entire DOS text mode buffer
 */
void epd_render_text_mode(const uint8_t *text_buffer)
{
    if (!framebuffer || !text_buffer) return;
    
    // Text buffer is 80x25, each character is 2 bytes (char + attr)
    for (int row = 0; row < EPD_TEXT_ROWS; row++) {
        for (int col = 0; col < EPD_TEXT_COLS; col++) {
            int idx = (row * EPD_TEXT_COLS + col) * 2;
            uint8_t ch = text_buffer[idx];
            uint8_t attr = text_buffer[idx + 1];
            epd_draw_text_char(row, col, ch, attr);
        }
    }
}

/**
 * Get framebuffer
 */
uint8_t *epd_get_framebuffer(void)
{
    return framebuffer;
}

/**
 * Set refresh mode
 */
void epd_set_refresh_mode(epd_refresh_mode_t mode)
{
    current_refresh_mode = mode;
}

epd_refresh_mode_t epd_get_refresh_mode(void)
{
    return current_refresh_mode;
}

/**
 * Flush framebuffer to display
 */
void epd_flush(void)
{
    epd_refresh_full();
}
