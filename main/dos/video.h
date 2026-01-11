/**
 * @file video.h
 * @brief Video emulation - renders DOS video memory to EPD
 */

#ifndef VIDEO_H
#define VIDEO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Video modes
#define VIDEO_MODE_TEXT_40x25_BW    0x00
#define VIDEO_MODE_TEXT_40x25_COLOR 0x01
#define VIDEO_MODE_TEXT_80x25_BW    0x02
#define VIDEO_MODE_TEXT_80x25_COLOR 0x03
#define VIDEO_MODE_CGA_320x200_4    0x04
#define VIDEO_MODE_CGA_320x200_BW   0x05
#define VIDEO_MODE_CGA_640x200_BW   0x06
#define VIDEO_MODE_MDA_80x25        0x07

// Default text mode parameters
#define TEXT_COLS_40    40
#define TEXT_COLS_80    80
#define TEXT_ROWS       25
#define CHAR_WIDTH      8
#define CHAR_HEIGHT     16

// Video memory layout for text mode
// Each character = 2 bytes: [char][attribute]
// Attribute: [blink:1][bg:3][fg:4]

// Video state
typedef struct {
    uint8_t mode;           // Current video mode
    uint8_t cols;           // Text columns
    uint8_t rows;           // Text rows
    uint8_t active_page;    // Active display page
    uint8_t cursor_row;     // Cursor row position
    uint8_t cursor_col;     // Cursor column position
    uint8_t cursor_start;   // Cursor start scanline
    uint8_t cursor_end;     // Cursor end scanline
    bool cursor_visible;    // Cursor visibility
    uint16_t page_offset;   // Video page offset
    uint16_t page_size;     // Size of one page
    bool dirty;             // Screen needs refresh
    uint32_t dirty_start;   // Dirty region start
    uint32_t dirty_end;     // Dirty region end
} video_state_t;

// Initialize video subsystem
void video_init(void);

// Set video mode
void video_set_mode(uint8_t mode);

// Get current video mode
uint8_t video_get_mode(void);

// Get video state
video_state_t *video_get_state(void);

// Cursor operations
void video_set_cursor_pos(uint8_t row, uint8_t col);
void video_get_cursor_pos(uint8_t *row, uint8_t *col);
void video_set_cursor_type(uint8_t start, uint8_t end);
void video_show_cursor(bool visible);

// Character operations
void video_write_char(uint8_t row, uint8_t col, uint8_t ch, uint8_t attr);
void video_read_char(uint8_t row, uint8_t col, uint8_t *ch, uint8_t *attr);

// Scrolling
void video_scroll_up(uint8_t lines, uint8_t attr, uint8_t top, uint8_t left, uint8_t bottom, uint8_t right);
void video_scroll_down(uint8_t lines, uint8_t attr, uint8_t top, uint8_t left, uint8_t bottom, uint8_t right);

// Clear screen
void video_clear_screen(uint8_t attr);
void video_clear_screen_partial(uint8_t attr);

// Page operations
void video_set_page(uint8_t page);
uint8_t video_get_page(void);

// String output
void video_write_string(uint8_t row, uint8_t col, const char *str, uint8_t attr);

// Check if screen needs update
bool video_needs_refresh(void);
void video_force_refresh(void);

// Render video memory to EPD
void video_render_to_epd(void);
void video_render_text_rows_to_epd(uint8_t first_row, uint8_t last_row);

// Urgent (TTY-driven) refresh support.
void video_request_urgent_refresh(uint8_t first_row, uint8_t last_row);
bool video_take_urgent_refresh(uint8_t *first_row, uint8_t *last_row);

// Get video buffer pointer
uint8_t *video_get_buffer(void);

// Snapshot "terminal" text for the Web UI.
// In text modes, reads from B800 (current page). In graphics modes, returns a shadow buffer
// maintained by the BIOS/video layer (so raw graphics bytes aren't misinterpreted as text).
bool video_get_terminal_snapshot(char *out, size_t out_cap, uint8_t rows, uint16_t cols);

// VGA DAC (palette) helpers for mode 13h
void video_vga_set_dac_index(uint8_t index);
void video_vga_write_dac(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // VIDEO_H
