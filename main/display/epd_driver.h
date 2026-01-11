/**
 * @file epd_driver.h
 * @brief E-Paper display driver for M5Paper (IT8951)
 */

#ifndef EPD_DRIVER_H
#define EPD_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// M5Paper IT8951 display dimensions
#define EPD_WIDTH       960
#define EPD_HEIGHT      540

// Text mode dimensions (80x25 with 8x16 font fits nicely)
// We use 12x21 character cells for better readability on e-paper
#define EPD_TEXT_CHAR_WIDTH     12
#define EPD_TEXT_CHAR_HEIGHT    21
#define EPD_TEXT_COLS           80
#define EPD_TEXT_ROWS           25

// Grayscale levels
#define EPD_COLOR_WHITE     0xFF
#define EPD_COLOR_BLACK     0x00
#define EPD_COLOR_GRAY      0x80

// Refresh modes
typedef enum {
    EPD_REFRESH_FULL = 0,       // Full refresh (slowest, best quality)
    EPD_REFRESH_PARTIAL = 1,    // Partial refresh (faster, some ghosting)
    EPD_REFRESH_FAST = 2,       // Fast refresh (fastest, most ghosting)
} epd_refresh_mode_t;

// Callback for flushing display region
typedef void (*epd_flush_cb_t)(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* data);

/**
 * Register flush callback
 */
void epd_register_flush_cb(epd_flush_cb_t cb);

/**
 * Initialize the EPD display
 * @return ESP_OK on success
 */
esp_err_t epd_init(void);

/**
 * Deinitialize the EPD display
 */
void epd_deinit(void);

/**
 * Clear the entire display to white
 */
void epd_clear(void);

/**
 * Perform a full refresh of the display
 */
void epd_refresh_full(void);

/**
 * Perform a partial refresh of a specific region
 * @param x X coordinate
 * @param y Y coordinate
 * @param w Width
 * @param h Height
 */
void epd_refresh_partial(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/**
 * Update a region of the display with new data
 * @param y1 Start row (pixels)
 * @param x1 Start column (pixels)
 * @param y2 End row (pixels)
 * @param x2 End column (pixels)
 * @param data Framebuffer data
 */
void epd_update_region(uint16_t y1, uint16_t x1, uint16_t y2, uint16_t x2, const uint8_t *data);

/**
 * Draw a single pixel
 * @param x X coordinate
 * @param y Y coordinate
 * @param color Grayscale color (0=black, 255=white)
 */
void epd_draw_pixel(uint16_t x, uint16_t y, uint8_t color);

/**
 * Draw a character at the specified position
 * @param x X coordinate (pixels)
 * @param y Y coordinate (pixels)
 * @param c Character to draw
 * @param fg Foreground color
 * @param bg Background color
 */
void epd_draw_char(uint16_t x, uint16_t y, char c, uint8_t fg, uint8_t bg);

/**
 * Draw a string at the specified position
 * @param row Text row
 * @param col Text column
 * @param str String to draw
 * @param attr DOS text attribute
 */
void epd_draw_string(uint8_t row, uint8_t col, const char *str, uint8_t attr);

/**
 * Draw a DOS text mode character with attribute
 * @param row Character row (0-24)
 * @param col Character column (0-79)
 * @param c Character code
 * @param attr DOS attribute byte
 */
void epd_draw_text_char(uint8_t row, uint8_t col, uint8_t c, uint8_t attr);

/**
 * Render the entire DOS text framebuffer to the EPD
 * @param text_buffer Pointer to 80x25x2 text buffer (char + attr)
 */
void epd_render_text_mode(const uint8_t *text_buffer);

/**
 * Get the internal framebuffer for direct access
 * @return Pointer to framebuffer
 */
uint8_t *epd_get_framebuffer(void);

/**
 * Set refresh mode
 * @param mode Refresh mode
 */
void epd_set_refresh_mode(epd_refresh_mode_t mode);

/**
 * Get current refresh mode
 */
epd_refresh_mode_t epd_get_refresh_mode(void);

/**
 * Force a full screen update from the framebuffer
 */
void epd_flush(void);

#ifdef __cplusplus
}
#endif

#endif // EPD_DRIVER_H
