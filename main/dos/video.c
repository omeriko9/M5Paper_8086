/**
 * @file video.c
 * @brief Video emulation for DOS (text mode)
 * 
 * Handles text mode display, cursor, and character rendering
 */

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "video.h"
#include "memory.h"
#include "config_defs.h"
#include "settings.h"
#include "../display/epd_driver.h"
#include "../display/epd_update.h"
#include "../display/font8x16.h"

static const char *TAG = "VIDEO";
static const char *TTY_TAG = "TTY";
static const char *CGA_TAG = "CGA";

// Current video state
static uint8_t current_mode = 0x03;  // 80x25 color
static uint8_t active_page = 0;
static uint8_t cursor_row = 0;
static uint8_t cursor_col = 0;
static uint8_t cursor_start = 6;
static uint8_t cursor_end = 7;
static bool cursor_visible = true;
static bool graphics_mode = false;

// Text mode dimensions
#define TEXT_ROWS   25
#define TEXT_COLS_MAX 80

// Mode 13h graphics dimensions
#define GRAPHICS_WIDTH  320
#define GRAPHICS_HEIGHT 200
#define GRAPHICS_SIZE   (GRAPHICS_WIDTH * GRAPHICS_HEIGHT)

// CGA graphics layout (modes 04h/05h): 320x200, 2bpp packed, odd/even scanlines interleaved
#define CGA_GRAPHICS_BASE            VIDEO_BASE
#define CGA_GRAPHICS_SIZE_BYTES      0x4000
#define CGA_BYTES_PER_SCANLINE_320   80
#define CGA_ODD_SCANLINE_OFFSET      0x2000

// Dirty tracking for partial refresh
static bool dirty_lines[TEXT_ROWS];
static bool full_redraw_needed = true;

static volatile bool urgent_refresh_pending = false;
static volatile uint8_t urgent_first_row = 0;
static volatile uint8_t urgent_last_row = 0;
static portMUX_TYPE urgent_refresh_mux = portMUX_INITIALIZER_UNLOCKED;

// VGA palette state (6-bit per channel)
static uint8_t vga_palette[256][3];
static uint8_t vga_palette_gray[256];
static uint8_t dac_write_index = 0;
static uint8_t dac_write_component = 0;
static uint8_t *vga_shadow = NULL;
static bool palette_dirty = false;

// Shadow "terminal" buffer for Web UI: 80x25 sanitized ASCII.
// This avoids misinterpreting CGA packed pixels at B800:0000 as text.
static char terminal_shadow[TEXT_ROWS][TEXT_COLS_MAX];

#define TEXT_SHADOW_INVALID 0xFFFFu
static uint16_t text_shadow[TEXT_ROWS][TEXT_COLS_MAX];

static inline uint16_t text_cell_signature(uint8_t ch, uint8_t attr)
{
    const uint8_t fg = (uint8_t)(attr & 0x0F);
    const uint8_t bg = (uint8_t)((attr >> 4) & 0x07);
    return (uint16_t)(((uint16_t)ch << 2) | (fg ? 0x02u : 0x00u) | (bg ? 0x01u : 0x00u));
}

static void text_shadow_reset(void)
{
    memset(text_shadow, 0xFF, sizeof(text_shadow));
}

static inline char terminal_sanitize_char(uint8_t ch)
{
    if (ch < 0x20 || ch == 0x7F) {
        return ' ';
    }
    if (ch <= 0x7E) {
        return (char)ch;
    }
    return '.';
}

static void terminal_shadow_clear(void)
{
    for (int r = 0; r < TEXT_ROWS; r++) {
        memset(terminal_shadow[r], ' ', TEXT_COLS_MAX);
    }
}

static uint8_t video_get_text_cols(void)
{
    uint16_t cols = mem_read_word(BDA_VIDEO_COLS);
    if (cols == 40 || cols == 80) {
        return (uint8_t)cols;
    }

    // Fallback: follow our mode mapping.
    switch (current_mode & 0x7F) {
        case 0x00:
        case 0x01:
        case 0x04:
        case 0x05:
        case 0x13:
            return 40;
        default:
            return 80;
    }
}

static uint16_t video_get_page_size_bytes(uint8_t cols)
{
    // BIOS uses 4KB pages for 80x25 text and 2KB pages for 40x25 text.
    // Keep BDA + address math consistent with common DOS expectations.
    return (cols >= 80) ? 0x1000 : 0x0800;
}

static inline uint32_t video_page_base(uint8_t page)
{
    const uint8_t cols = video_get_text_cols();
    return VIDEO_BASE + (uint32_t)page * (uint32_t)video_get_page_size_bytes(cols);
}

static inline uint32_t video_addr(uint8_t page, uint8_t row, uint8_t col)
{
    const uint8_t cols = video_get_text_cols();
    return video_page_base(page) + ((uint32_t)(row * cols + col) * 2U);
}

static uint8_t vga_rgb_to_gray(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t lum = (uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11;
    uint16_t gray = (uint16_t)((lum * 255 + 3150) / 6300);
    return (uint8_t)gray;
}

static void vga_palette_set(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    vga_palette[index][0] = (uint8_t)(r & 0x3F);
    vga_palette[index][1] = (uint8_t)(g & 0x3F);
    vga_palette[index][2] = (uint8_t)(b & 0x3F);
    vga_palette_gray[index] = vga_rgb_to_gray(vga_palette[index][0],
                                             vga_palette[index][1],
                                             vga_palette[index][2]);
}

static void vga_palette_reset(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t val = (uint8_t)(i >> 2);
        vga_palette_set((uint8_t)i, val, val, val);
    }
    dac_write_index = 0;
    dac_write_component = 0;
}

static void video_clear_graphics_mode13(uint8_t color)
{
    uint8_t *vga = mem_get_ptr(VIDEO_RAM_START);
    if (!vga) {
        return;
    }
    memset(vga, color, GRAPHICS_SIZE);
    mem_set_video_dirty(VIDEO_RAM_START, true);
    if (vga_shadow) {
        memset(vga_shadow, color, GRAPHICS_SIZE);
    }
    terminal_shadow_clear();
}

static void video_clear_graphics_cga(void)
{
    uint8_t *cga = mem_get_ptr(CGA_GRAPHICS_BASE);
    if (!cga) {
        return;
    }
    memset(cga, 0x00, CGA_GRAPHICS_SIZE_BYTES);
    mem_set_video_dirty(CGA_GRAPHICS_BASE, true);
    if (vga_shadow) {
        memset(vga_shadow, 0x00, GRAPHICS_SIZE);
    }
    terminal_shadow_clear();
}

static inline uint8_t cga_get_pixel_2bpp_320x200(const uint8_t *cga_base, int x, int y)
{
    // Each scanline is 80 bytes (4 pixels per byte). Even and odd scanlines are stored in separate 8KB banks.
    const uint32_t bank = (y & 1) ? CGA_ODD_SCANLINE_OFFSET : 0;
    const uint32_t row_off = (uint32_t)(y >> 1) * CGA_BYTES_PER_SCANLINE_320;
    const uint32_t byte_off = (uint32_t)(x >> 2);
    const uint8_t b = cga_base[bank + row_off + byte_off];
    const uint8_t shift = (uint8_t)(6 - ((x & 3) << 1));
    return (uint8_t)((b >> shift) & 0x03);
}

static inline void cga_set_pixel_2bpp_320x200(uint8_t *cga_base, int x, int y, uint8_t color)
{
    const uint32_t bank = (y & 1) ? CGA_ODD_SCANLINE_OFFSET : 0;
    const uint32_t row_off = (uint32_t)(y >> 1) * CGA_BYTES_PER_SCANLINE_320;
    const uint32_t byte_off = (uint32_t)(x >> 2);
    const uint32_t off = bank + row_off + byte_off;

    const uint8_t shift = (uint8_t)(6 - ((x & 3) << 1));
    const uint8_t mask = (uint8_t)(0x03u << shift);
    uint8_t b = cga_base[off];
    b = (uint8_t)((b & ~mask) | ((color & 0x03u) << shift));
    cga_base[off] = b;
}

static inline uint8_t font8x8_row(uint8_t c, uint8_t row)
{
    const uint8_t *glyph = font8x16_get_glyph(c);
    // Downsample 8x16 -> 8x8 by OR'ing row pairs (more readable than a single-sample).
    return (uint8_t)(glyph[row * 2] | glyph[row * 2 + 1]);
}

static void video_graphics_write_char(uint8_t row, uint8_t col, uint8_t ch, uint8_t color)
{
    const uint8_t mode = (uint8_t)(current_mode & 0x7F);
    const int cols = GRAPHICS_WIDTH / 8;
    const int rows = GRAPHICS_HEIGHT / 8;

    if ((int)row >= rows) row = (uint8_t)(rows - 1);
    if ((int)col >= cols) col = (uint8_t)(cols - 1);

    const int x0 = (int)col * 8;
    const int y0 = (int)row * 8;

    if (mode == 0x13) {
        uint8_t *vga = mem_get_ptr(VIDEO_RAM_START);
        if (!vga) return;

        const uint8_t fg = color;
        const uint8_t bg = 0x00;
        for (int y = 0; y < 8; y++) {
            uint8_t bits = font8x8_row(ch, (uint8_t)y);
            const int py = y0 + y;
            if (py < 0 || py >= GRAPHICS_HEIGHT) continue;
            const int row_off = py * GRAPHICS_WIDTH;
            for (int x = 0; x < 8; x++) {
                const int px = x0 + x;
                if (px < 0 || px >= GRAPHICS_WIDTH) continue;
                vga[row_off + px] = (bits & (0x80u >> x)) ? fg : bg;
            }
        }
        mem_set_video_dirty(VIDEO_RAM_START, true);
        return;
    }

    if (mode == 0x04 || mode == 0x05) {
        uint8_t *cga = mem_get_ptr(CGA_GRAPHICS_BASE);
        if (!cga) return;

        const uint8_t fg = (uint8_t)(color & 0x03u);
        const uint8_t bg = 0x00;
        for (int y = 0; y < 8; y++) {
            uint8_t bits = font8x8_row(ch, (uint8_t)y);
            const int py = y0 + y;
            if (py < 0 || py >= GRAPHICS_HEIGHT) continue;
            for (int x = 0; x < 8; x++) {
                const int px = x0 + x;
                if (px < 0 || px >= GRAPHICS_WIDTH) continue;
                const uint8_t pix = (bits & (0x80u >> x)) ? fg : bg;
                cga_set_pixel_2bpp_320x200(cga, px, py, pix);
            }
        }
        mem_set_video_dirty(CGA_GRAPHICS_BASE, true);
        return;
    }
}

/**
 * Initialize video subsystem
 */
void video_init(void)
{
    ESP_LOGI(TAG, "Initializing video subsystem");

    terminal_shadow_clear();
    text_shadow_reset();
    text_shadow_reset();

    if (!vga_shadow) {
        vga_shadow = (uint8_t *)malloc(GRAPHICS_SIZE);
        if (!vga_shadow) {
            ESP_LOGW(TAG, "Failed to allocate VGA shadow buffer");
        } else {
            memset(vga_shadow, 0, GRAPHICS_SIZE);
        }
    }
    vga_palette_reset();

    // Set default mode
    video_set_mode(0x03);
    
    // Clear dirty flags
    memset(dirty_lines, 0, sizeof(dirty_lines));
    full_redraw_needed = true;
    text_shadow_reset();
}

/**
 * Set video mode
 */
void video_set_mode(uint8_t mode)
{
    ESP_LOGI(TAG, "Setting video mode %02X", mode);
    
    switch (mode & 0x7F) {
        case 0x00:  // 40x25 B&W
        case 0x01:  // 40x25 color
        case 0x02:  // 80x25 B&W
        case 0x03:  // 80x25 color
        case 0x07:  // 80x25 monochrome
            current_mode = mode & 0x7F;
            graphics_mode = false;
            break;
        case 0x04:  // CGA 320x200 4-color
        case 0x05:  // CGA 320x200 B/W
            ESP_LOGI(CGA_TAG, "CGA graphics mode %02X (320x200 2bpp @ B800:0000)", mode & 0x7F);
            current_mode = mode & 0x7F;
            graphics_mode = true;
            break;
        case 0x13:  // 320x200 256-color
            current_mode = mode & 0x7F;
            graphics_mode = true;
            break;
        default:
            ESP_LOGW(TAG, "Unsupported video mode %02X, using 03h", mode);
            current_mode = 0x03;
            graphics_mode = false;
            break;
    }
    
    // Clear screen unless bit 7 is set
    if (!(mode & 0x80)) {
        if (graphics_mode) {
            if (current_mode == 0x13) {
                video_clear_graphics_mode13(0x00);
            } else if (current_mode == 0x04 || current_mode == 0x05) {
                video_clear_graphics_cga();
            }
        } else {
            video_clear_screen(0x07);
        }
    }
    
    // Update BDA
    mem_write_byte(BDA_VIDEO_MODE, current_mode);
    uint16_t cols = TEXT_COLS_MAX;
    switch (current_mode) {
        case 0x00:
        case 0x01:
        case 0x04:
        case 0x05:
        case 0x13:
            cols = 40;
            break;
        default:
            cols = TEXT_COLS_MAX;
            break;
    }
    mem_write_word(BDA_VIDEO_COLS, cols);
    mem_write_word(BDA_VIDEO_PAGE_SIZE, video_get_page_size_bytes((uint8_t)cols));
    mem_write_byte(BDA_VIDEO_ROWS, (uint8_t)(TEXT_ROWS - 1));
    mem_write_word(BDA_VIDEO_PAGE_OFF, 0);
    
    // Set cursor to home
    video_set_cursor_pos(0, 0);
    
    full_redraw_needed = true;
}

/**
 * Get current video mode
 */
uint8_t video_get_mode(void)
{
    return current_mode;
}

/**
 * Set active display page
 */
void video_set_page(uint8_t page)
{
    if (page < 8) {
        active_page = page;
        mem_write_byte(BDA_ACTIVE_PAGE, page);
        mem_write_word(BDA_VIDEO_PAGE_OFF, (uint16_t)(page * video_get_page_size_bytes(video_get_text_cols())));
        full_redraw_needed = true;
        text_shadow_reset();
    }
}

/**
 * Get active display page
 */
uint8_t video_get_page(void)
{
    return active_page;
}

/**
 * Set cursor position
 */
void video_set_cursor_pos(uint8_t row, uint8_t col)
{
    const uint8_t cols = video_get_text_cols();
    if (row >= TEXT_ROWS) row = TEXT_ROWS - 1;
    if (col >= cols) col = (uint8_t)(cols - 1);
    
    cursor_row = row;
    cursor_col = col;
    
    // Update BDA cursor position for current page
    uint16_t bda_addr = BDA_CURSOR_POS + (active_page * 2);
    mem_write_byte(bda_addr, col);
    mem_write_byte(bda_addr + 1, row);
    
    // Mark line as dirty (for cursor update)
    dirty_lines[row] = true;
}

/**
 * Get cursor position
 */
void video_get_cursor_pos(uint8_t *row, uint8_t *col)
{
    *row = cursor_row;
    *col = cursor_col;
}

/**
 * Set cursor type
 */
void video_set_cursor_type(uint8_t start, uint8_t end)
{
    cursor_start = start & 0x1F;
    cursor_end = end & 0x1F;
    cursor_visible = (start & 0x20) == 0;
    
    mem_write_word(BDA_CURSOR_TYPE, (start << 8) | end);
}

/**
 * Write character at position
 */
void video_write_char(uint8_t row, uint8_t col, uint8_t ch, uint8_t attr)
{
    const uint8_t cols = video_get_text_cols();
    if (row >= TEXT_ROWS || col >= cols) return;

    if (col < TEXT_COLS_MAX) {
        terminal_shadow[row][col] = terminal_sanitize_char(ch);
    }

    if (graphics_mode) {
        video_graphics_write_char(row, col, ch, (uint8_t)(attr & 0x0F));
        return;
    }
    
    char safe = (ch >= 0x20 && ch <= 0x7E) ? (char)ch : '.';
    ESP_LOGI(TAG, "video_write_char: %c (0x%02X) at %u,%u", safe, (unsigned)ch, (unsigned)row, (unsigned)col);

    uint32_t addr = video_addr(active_page, row, col);
    mem_write_byte(addr, ch);
    mem_write_byte(addr + 1, attr);
    
    dirty_lines[row] = true;
    mem_set_video_dirty(addr, true);
}

/**
 * Read character at position
 */
void video_read_char(uint8_t row, uint8_t col, uint8_t *ch, uint8_t *attr)
{
    const uint8_t cols = video_get_text_cols();
    if (row >= TEXT_ROWS || col >= cols || graphics_mode) {
        *ch = 0;
        *attr = 0x07;
        return;
    }
    
    uint32_t addr = video_addr(active_page, row, col);
    *ch = mem_read_byte(addr);
    *attr = mem_read_byte(addr + 1);
}

/**
 * Write string at position
 */
void video_write_string(uint8_t row, uint8_t col, const char *str, uint8_t attr)
{
    const uint8_t cols = video_get_text_cols();
    while (*str && col < cols) {
        video_write_char(row, col++, *str++, attr);
    }
}

/**
 * Clear screen with attribute
 */
void video_clear_screen(uint8_t attr)
{
    if (graphics_mode) {
        if ((current_mode & 0x7F) == 0x13) {
            video_clear_graphics_mode13(0x00);
        } else if ((current_mode & 0x7F) == 0x04 || (current_mode & 0x7F) == 0x05) {
            video_clear_graphics_cga();
        }
        full_redraw_needed = true;
        return;
    }

    terminal_shadow_clear();

    const uint8_t cols = video_get_text_cols();
    const uint32_t base = video_page_base(active_page);

    for (int i = 0; i < TEXT_ROWS * cols; i++) {
        mem_write_byte(base + (uint32_t)i * 2U, ' ');
        mem_write_byte(base + (uint32_t)i * 2U + 1U, attr);
    }

    memset(dirty_lines, true, sizeof(dirty_lines));
    full_redraw_needed = true;
}

void video_clear_screen_partial(uint8_t attr)
{
    if (graphics_mode) {
        video_clear_screen(attr);
        return;
    }

    terminal_shadow_clear();
    text_shadow_reset();

    const uint8_t cols = video_get_text_cols();
    const uint32_t base = video_page_base(active_page);

    for (int i = 0; i < TEXT_ROWS * cols; i++) {
        mem_write_byte(base + (uint32_t)i * 2U, ' ');
        mem_write_byte(base + (uint32_t)i * 2U + 1U, attr);
    }

    memset(dirty_lines, true, sizeof(dirty_lines));
    full_redraw_needed = false;
    mem_set_video_dirty(base, true);
}

/**
 * Scroll region up
 */
void video_scroll_up(uint8_t lines, uint8_t attr, 
                     uint8_t top_row, uint8_t left_col,
                     uint8_t bottom_row, uint8_t right_col)
{
    if (lines == 0) {
        // Clear entire window
        for (uint8_t row = top_row; row <= bottom_row; row++) {
            for (uint8_t col = left_col; col <= right_col; col++) {
                video_write_char(row, col, ' ', attr);
            }
        }
        return;
    }
    
    // Scroll content up
    for (uint8_t row = top_row; row <= bottom_row - lines; row++) {
        for (uint8_t col = left_col; col <= right_col; col++) {
            uint8_t ch, at;
            video_read_char(row + lines, col, &ch, &at);
            video_write_char(row, col, ch, at);
        }
    }
    
    // Clear bottom lines
    for (uint8_t row = bottom_row - lines + 1; row <= bottom_row; row++) {
        for (uint8_t col = left_col; col <= right_col; col++) {
            video_write_char(row, col, ' ', attr);
        }
    }
}

/**
 * Scroll region down
 */
void video_scroll_down(uint8_t lines, uint8_t attr,
                       uint8_t top_row, uint8_t left_col,
                       uint8_t bottom_row, uint8_t right_col)
{
    if (lines == 0) {
        // Clear entire window
        for (uint8_t row = top_row; row <= bottom_row; row++) {
            for (uint8_t col = left_col; col <= right_col; col++) {
                video_write_char(row, col, ' ', attr);
            }
        }
        return;
    }
    
    // Scroll content down
    for (int row = bottom_row; row >= top_row + lines; row--) {
        for (uint8_t col = left_col; col <= right_col; col++) {
            uint8_t ch, at;
            video_read_char(row - lines, col, &ch, &at);
            video_write_char(row, col, ch, at);
        }
    }
    
    // Clear top lines
    for (uint8_t row = top_row; row < top_row + lines; row++) {
        for (uint8_t col = left_col; col <= right_col; col++) {
            video_write_char(row, col, ' ', attr);
        }
    }
}

/**
 * Check if video needs refresh
 */
bool video_needs_refresh(void)
{
    if (urgent_refresh_pending) return true;

    if (graphics_mode) {
        return full_redraw_needed || palette_dirty || mem_is_video_dirty();
    }

    if (full_redraw_needed) return true;
    
    for (int i = 0; i < TEXT_ROWS; i++) {
        if (dirty_lines[i]) return true;
    }
    
    return mem_is_video_dirty();
}

void video_request_urgent_refresh(uint8_t first_row, uint8_t last_row)
{
    if (first_row >= TEXT_ROWS) first_row = TEXT_ROWS - 1;
    if (last_row >= TEXT_ROWS) last_row = TEXT_ROWS - 1;
    if (first_row > last_row) {
        uint8_t tmp = first_row;
        first_row = last_row;
        last_row = tmp;
    }

    taskENTER_CRITICAL(&urgent_refresh_mux);
    if (!urgent_refresh_pending) {
        urgent_first_row = first_row;
        urgent_last_row = last_row;
        urgent_refresh_pending = true;
    } else {
        if (first_row < urgent_first_row) urgent_first_row = first_row;
        if (last_row > urgent_last_row) urgent_last_row = last_row;
    }
    taskEXIT_CRITICAL(&urgent_refresh_mux);
}

bool video_take_urgent_refresh(uint8_t *first_row, uint8_t *last_row)
{
    bool have = false;
    taskENTER_CRITICAL(&urgent_refresh_mux);
    if (urgent_refresh_pending) {
        *first_row = urgent_first_row;
        *last_row = urgent_last_row;
        urgent_refresh_pending = false;
        have = true;
    }
    taskEXIT_CRITICAL(&urgent_refresh_mux);
    return have;
}

#define GRAPHICS_TILE_W 8
#define GRAPHICS_TILE_H 8

static void graphics_map_rect(uint16_t src_x0, uint16_t src_y0,
                              uint16_t src_x1, uint16_t src_y1,
                              uint16_t *dst_x0, uint16_t *dst_y0,
                              uint16_t *dst_x1, uint16_t *dst_y1)
{
    uint16_t x0 = (uint16_t)((src_x0 * EPD_WIDTH) / GRAPHICS_WIDTH);
    uint16_t x1 = (uint16_t)(((src_x1 + 1) * EPD_WIDTH - 1) / GRAPHICS_WIDTH);
    uint16_t y0 = (uint16_t)((src_y0 * EPD_HEIGHT) / GRAPHICS_HEIGHT);
    uint16_t y1 = (uint16_t)(((src_y1 + 1) * EPD_HEIGHT - 1) / GRAPHICS_HEIGHT);

    if (x1 >= EPD_WIDTH) x1 = EPD_WIDTH - 1;
    if (y1 >= EPD_HEIGHT) y1 = EPD_HEIGHT - 1;

    *dst_x0 = x0;
    *dst_x1 = x1;
    *dst_y0 = y0;
    *dst_y1 = y1;
}

static void graphics_render_rect(const uint8_t *src, const uint8_t *palette,
                                 uint16_t src_x0, uint16_t src_y0,
                                 uint16_t src_x1, uint16_t src_y1)
{
    uint8_t *fb = epd_get_framebuffer();
    if (!fb || !src || !palette) {
        return;
    }

    uint16_t dst_x0;
    uint16_t dst_y0;
    uint16_t dst_x1;
    uint16_t dst_y1;
    graphics_map_rect(src_x0, src_y0, src_x1, src_y1, &dst_x0, &dst_y0, &dst_x1, &dst_y1);

    for (uint16_t y = dst_y0; y <= dst_y1; y++) {
        uint16_t src_y = (uint16_t)((y * GRAPHICS_HEIGHT) / EPD_HEIGHT);
        uint8_t *fb_row = fb + (y * EPD_WIDTH);
        uint32_t src_row = (uint32_t)src_y * GRAPHICS_WIDTH;

        for (uint16_t x = dst_x0; x <= dst_x1; x++) {
            uint16_t src_x = (uint16_t)((x * GRAPHICS_WIDTH) / EPD_WIDTH);
            uint8_t color = src[src_row + src_x];
            fb_row[x] = palette[color];
        }
    }
}

static void graphics_render_rect_cga(const uint8_t *cga, const uint8_t *palette,
                                     uint16_t src_x0, uint16_t src_y0,
                                     uint16_t src_x1, uint16_t src_y1)
{
    uint8_t *fb = epd_get_framebuffer();
    if (!fb || !cga || !palette) {
        return;
    }

    uint16_t dst_x0;
    uint16_t dst_y0;
    uint16_t dst_x1;
    uint16_t dst_y1;
    graphics_map_rect(src_x0, src_y0, src_x1, src_y1, &dst_x0, &dst_y0, &dst_x1, &dst_y1);

    for (uint16_t y = dst_y0; y <= dst_y1; y++) {
        uint16_t src_y = (uint16_t)((y * GRAPHICS_HEIGHT) / EPD_HEIGHT);
        uint8_t *fb_row = fb + (y * EPD_WIDTH);
        for (uint16_t x = dst_x0; x <= dst_x1; x++) {
            uint16_t src_x = (uint16_t)((x * GRAPHICS_WIDTH) / EPD_WIDTH);
            uint8_t color = cga_get_pixel_2bpp_320x200(cga, (int)src_x, (int)src_y);
            fb_row[x] = palette[color & 0x03];
        }
    }
}

static void video_render_graphics_mode(void)
{
    const uint8_t mode = current_mode & 0x7F;
    epd_update_begin(EPD_UPDATE_GRAPHICS, app_settings_display_partial_refresh());

    if (mode == 0x13) {
        const uint8_t *src = mem_get_ptr(VIDEO_RAM_START);
        if (!src) {
            return;
        }

        const bool full_draw = full_redraw_needed || palette_dirty || !vga_shadow;
        if (full_draw) {
            if (vga_shadow) {
                memcpy(vga_shadow, src, GRAPHICS_SIZE);
            }
            graphics_render_rect(src, vga_palette_gray, 0, 0,
                                 (uint16_t)(GRAPHICS_WIDTH - 1),
                                 (uint16_t)(GRAPHICS_HEIGHT - 1));
            epd_update_add_rect(0, 0, EPD_WIDTH, EPD_HEIGHT);
        } else {
            const uint16_t tiles_x = (uint16_t)(GRAPHICS_WIDTH / GRAPHICS_TILE_W);
            const uint16_t tiles_y = (uint16_t)(GRAPHICS_HEIGHT / GRAPHICS_TILE_H);
            for (uint16_t ty = 0; ty < tiles_y; ty++) {
                const uint16_t src_y0 = (uint16_t)(ty * GRAPHICS_TILE_H);
                for (uint16_t tx = 0; tx < tiles_x; tx++) {
                    const uint16_t src_x0 = (uint16_t)(tx * GRAPHICS_TILE_W);
                    bool tile_dirty = false;

                    for (uint16_t y = 0; y < GRAPHICS_TILE_H; y++) {
                        const uint32_t row_off = (uint32_t)(src_y0 + y) * GRAPHICS_WIDTH + src_x0;
                        if (memcmp(vga_shadow + row_off, src + row_off, GRAPHICS_TILE_W) != 0) {
                            tile_dirty = true;
                            break;
                        }
                    }

                    if (!tile_dirty) {
                        continue;
                    }

                    for (uint16_t y = 0; y < GRAPHICS_TILE_H; y++) {
                        const uint32_t row_off = (uint32_t)(src_y0 + y) * GRAPHICS_WIDTH + src_x0;
                        memcpy(vga_shadow + row_off, src + row_off, GRAPHICS_TILE_W);
                    }

                    const uint16_t src_x1 = (uint16_t)(src_x0 + GRAPHICS_TILE_W - 1);
                    const uint16_t src_y1 = (uint16_t)(src_y0 + GRAPHICS_TILE_H - 1);
                    graphics_render_rect(src, vga_palette_gray, src_x0, src_y0, src_x1, src_y1);

                    uint16_t dst_x0;
                    uint16_t dst_y0;
                    uint16_t dst_x1;
                    uint16_t dst_y1;
                    graphics_map_rect(src_x0, src_y0, src_x1, src_y1, &dst_x0, &dst_y0, &dst_x1, &dst_y1);
                    epd_update_add_rect(dst_x0, dst_y0,
                                        (uint16_t)(dst_x1 - dst_x0 + 1),
                                        (uint16_t)(dst_y1 - dst_y0 + 1));
                }
            }
        }
    } else if (mode == 0x04 || mode == 0x05) {
        const uint8_t *cga = mem_get_ptr(CGA_GRAPHICS_BASE);
        if (!cga) {
            return;
        }

        static const uint8_t cga_gray[4] = { 0x00, 0x55, 0xAA, 0xFF };
        const bool have_shadow = (vga_shadow != NULL);
        const bool full_draw = full_redraw_needed || !have_shadow;

        if (full_draw) {
            if (have_shadow) {
                for (int y = 0; y < GRAPHICS_HEIGHT; y++) {
                    const int row_off = y * GRAPHICS_WIDTH;
                    for (int x = 0; x < GRAPHICS_WIDTH; x++) {
                        vga_shadow[row_off + x] = cga_get_pixel_2bpp_320x200(cga, x, y);
                    }
                }
                graphics_render_rect(vga_shadow, cga_gray, 0, 0,
                                     (uint16_t)(GRAPHICS_WIDTH - 1),
                                     (uint16_t)(GRAPHICS_HEIGHT - 1));
            } else {
                graphics_render_rect_cga(cga, cga_gray, 0, 0,
                                         (uint16_t)(GRAPHICS_WIDTH - 1),
                                         (uint16_t)(GRAPHICS_HEIGHT - 1));
            }
            epd_update_add_rect(0, 0, EPD_WIDTH, EPD_HEIGHT);
        } else {
            const uint16_t tiles_x = (uint16_t)(GRAPHICS_WIDTH / GRAPHICS_TILE_W);
            const uint16_t tiles_y = (uint16_t)(GRAPHICS_HEIGHT / GRAPHICS_TILE_H);
            for (uint16_t ty = 0; ty < tiles_y; ty++) {
                const uint16_t src_y0 = (uint16_t)(ty * GRAPHICS_TILE_H);
                for (uint16_t tx = 0; tx < tiles_x; tx++) {
                    const uint16_t src_x0 = (uint16_t)(tx * GRAPHICS_TILE_W);
                    bool tile_dirty = false;

                    for (uint16_t y = 0; y < GRAPHICS_TILE_H; y++) {
                        const int src_y = (int)(src_y0 + y);
                        const int row_off = src_y * GRAPHICS_WIDTH;
                        for (uint16_t x = 0; x < GRAPHICS_TILE_W; x++) {
                            const int src_x = (int)(src_x0 + x);
                            const int idx = row_off + src_x;
                            const uint8_t val = cga_get_pixel_2bpp_320x200(cga, src_x, src_y);
                            if (vga_shadow[idx] != val) {
                                vga_shadow[idx] = val;
                                tile_dirty = true;
                            }
                        }
                    }

                    if (!tile_dirty) {
                        continue;
                    }

                    const uint16_t src_x1 = (uint16_t)(src_x0 + GRAPHICS_TILE_W - 1);
                    const uint16_t src_y1 = (uint16_t)(src_y0 + GRAPHICS_TILE_H - 1);
                    graphics_render_rect(vga_shadow, cga_gray, src_x0, src_y0, src_x1, src_y1);

                    uint16_t dst_x0;
                    uint16_t dst_y0;
                    uint16_t dst_x1;
                    uint16_t dst_y1;
                    graphics_map_rect(src_x0, src_y0, src_x1, src_y1, &dst_x0, &dst_y0, &dst_x1, &dst_y1);
                    epd_update_add_rect(dst_x0, dst_y0,
                                        (uint16_t)(dst_x1 - dst_x0 + 1),
                                        (uint16_t)(dst_y1 - dst_y0 + 1));
                }
            }
        }
    } else {
        // Unknown/unsupported graphics mode: nothing to render.
        mem_clear_video_dirty();
        palette_dirty = false;
        full_redraw_needed = false;
        return;
    }

    epd_update_commit();

    mem_clear_video_dirty();
    palette_dirty = false;
    full_redraw_needed = false;
}

static void text_flush_run(uint8_t row, int *run_start, int *run_end, bool track_runs)
{
    if (!track_runs) {
        *run_start = -1;
        return;
    }

    if (*run_start < 0) {
        return;
    }

    const uint16_t x = (uint16_t)(*run_start * EPD_TEXT_CHAR_WIDTH);
    const uint16_t y = (uint16_t)(row * EPD_TEXT_CHAR_HEIGHT);
    const uint16_t w = (uint16_t)((*run_end - *run_start + 1) * EPD_TEXT_CHAR_WIDTH);
    epd_update_add_rect(x, y, w, EPD_TEXT_CHAR_HEIGHT);
    *run_start = -1;
}

static void text_update_cell(uint8_t row, uint8_t col, uint8_t ch, uint8_t attr,
                             bool force_draw, bool track_runs,
                             int *run_start, int *run_end)
{
    const uint16_t sig = text_cell_signature(ch, attr);
    if (force_draw || text_shadow[row][col] != sig) {
        text_shadow[row][col] = sig;
        epd_draw_text_char(row, col, ch, attr);
        if (track_runs) {
            if (*run_start < 0) {
                *run_start = col;
            }
            *run_end = col;
        }
        return;
    }

    text_flush_run(row, run_start, run_end, track_runs);
}

static void video_render_text_rows_range(uint8_t first_row, uint8_t last_row,
                                         bool scan_all_rows, bool force_full_draw,
                                         bool clear_mem_dirty)
{
    const uint8_t cols = video_get_text_cols();
    const uint8_t display_cols = (cols == 40) ? TEXT_COLS_MAX : cols;
    const uint32_t base = video_page_base(active_page);
    uint8_t *vmem = mem_get_ptr(base);
    if (!vmem) {
        return;
    }

    const bool allow_partial = app_settings_display_partial_refresh();
    const bool track_runs = !force_full_draw;

    epd_update_begin(EPD_UPDATE_TEXT, allow_partial);
    if (force_full_draw && !allow_partial) {
        epd_update_force_full();
    }

    for (uint8_t row = first_row; row <= last_row; row++) {
        if (!scan_all_rows && !dirty_lines[row]) {
            continue;
        }

        int run_start = -1;
        int run_end = -1;

        if (cols == 40) {
            for (uint8_t col = 0; col < 40; col++) {
                const uint32_t off = (uint32_t)((row * 40 + col) * 2);
                const uint8_t ch = vmem[off];
                const uint8_t attr = vmem[off + 1];
                const uint8_t dst_col = (uint8_t)(col * 2);

                text_update_cell(row, dst_col, ch, attr, force_full_draw, track_runs, &run_start, &run_end);
                if (dst_col + 1 < TEXT_COLS_MAX) {
                    text_update_cell(row, (uint8_t)(dst_col + 1), ' ', 0x07, force_full_draw,
                                     track_runs, &run_start, &run_end);
                }
            }
        } else {
            for (uint8_t col = 0; col < display_cols; col++) {
                const uint32_t off = (uint32_t)((row * display_cols + col) * 2);
                const uint8_t ch = vmem[off];
                const uint8_t attr = vmem[off + 1];
                text_update_cell(row, col, ch, attr, force_full_draw, track_runs, &run_start, &run_end);
            }
        }

        text_flush_run(row, &run_start, &run_end, track_runs);
        dirty_lines[row] = false;
    }

    if (force_full_draw) {
        uint16_t text_height_px = (uint16_t)(TEXT_ROWS * EPD_TEXT_CHAR_HEIGHT);
        if (text_height_px < EPD_HEIGHT) {
            uint8_t *fb = epd_get_framebuffer();
            if (fb) {
                memset(fb + (text_height_px * EPD_WIDTH), EPD_COLOR_WHITE,
                       (size_t)(EPD_HEIGHT - text_height_px) * EPD_WIDTH);
            }
            text_height_px = EPD_HEIGHT;
        }
        if (text_height_px > EPD_HEIGHT) {
            text_height_px = EPD_HEIGHT;
        }
        epd_update_add_rect(0, 0, EPD_WIDTH, text_height_px);
    }

    epd_update_commit();

    if (clear_mem_dirty) {
        mem_clear_video_dirty();
    }
}

void video_render_text_rows_to_epd(uint8_t first_row, uint8_t last_row)
{
    if (graphics_mode) {
        video_render_graphics_mode();
        return;
    }

    if (first_row >= TEXT_ROWS) first_row = TEXT_ROWS - 1;
    if (last_row >= TEXT_ROWS) last_row = TEXT_ROWS - 1;
    if (first_row > last_row) {
        uint8_t tmp = first_row;
        first_row = last_row;
        last_row = tmp;
    }

    video_render_text_rows_range(first_row, last_row, true, false, false);
}

/**
 * Render video to EPD framebuffer
 */
void video_render_to_epd(void)
{
    if (!video_needs_refresh()) return;

    ESP_LOGI(TTY_TAG, "video_render_to_epd: Refreshing...");

    if (graphics_mode) {
        video_render_graphics_mode();
        return;
    }

    uint8_t urgent_first = 0;
    uint8_t urgent_last = 0;
    if (video_take_urgent_refresh(&urgent_first, &urgent_last)) {
        video_render_text_rows_range(urgent_first, urgent_last, true, false, false);
        return;
    }

    bool any_dirty = false;
    for (int row = 0; row < TEXT_ROWS; row++) {
        if (dirty_lines[row]) {
            any_dirty = true;
            break;
        }
    }

    const bool mem_dirty = mem_is_video_dirty();
    const bool scan_all = full_redraw_needed || mem_dirty || !any_dirty;
    video_render_text_rows_range(0, (uint8_t)(TEXT_ROWS - 1), scan_all, full_redraw_needed, scan_all);

    full_redraw_needed = false;
    if (mem_is_video_dirty()) {
        full_redraw_needed = true;
    }
}

/**
 * Force full refresh on next render
 */
void video_force_refresh(void)
{
    full_redraw_needed = true;
}

/**
 * Get text buffer pointer
 */
uint8_t *video_get_buffer(void)
{
    return mem_get_ptr(video_page_base(active_page));
}

bool video_get_terminal_snapshot(char *out, size_t out_cap, uint8_t rows, uint16_t cols)
{
    if (!out || out_cap == 0) {
        return false;
    }

    if (cols == 0 || cols > TEXT_COLS_MAX) {
        cols = TEXT_COLS_MAX;
    }
    if (rows == 0 || rows > TEXT_ROWS) {
        rows = TEXT_ROWS;
    }

    const size_t needed = (size_t)rows * ((size_t)cols + 1) + 1;
    if (out_cap < needed) {
        ESP_LOGW(TAG, "terminal_snapshot: out_cap=%u needed=%u", (unsigned)out_cap, (unsigned)needed);
        return false;
    }

    size_t pos = 0;
    const uint8_t mode = (uint8_t)(current_mode & 0x7F);
    const bool is_text_mode = (mode <= 0x03) || (mode == 0x07);
    ESP_LOGD(TAG, "terminal_snapshot: mode=%02X text=%d rows=%u cols=%u",
             mode, is_text_mode ? 1 : 0, rows, (unsigned)cols);

    if (is_text_mode) {
        const uint32_t base = VIDEO_BASE + mem_read_word(BDA_VIDEO_PAGE_OFF);
        for (uint8_t r = 0; r < rows; r++) {
            for (uint16_t c = 0; c < cols; c++) {
                uint8_t ch = mem_read_byte(base + ((uint32_t)r * cols + c) * 2U);
                out[pos++] = terminal_sanitize_char(ch);
            }
            out[pos++] = '\n';
        }
    } else {
        for (uint8_t r = 0; r < rows; r++) {
            for (uint16_t c = 0; c < cols; c++) {
                out[pos++] = terminal_shadow[r][c];
            }
            out[pos++] = '\n';
        }
    }

    out[pos] = '\0';
    return true;
}

void video_vga_set_dac_index(uint8_t index)
{
    dac_write_index = index;
    dac_write_component = 0;
}

void video_vga_write_dac(uint8_t value)
{
    if (dac_write_component == 0) {
        vga_palette[dac_write_index][0] = (uint8_t)(value & 0x3F);
        dac_write_component = 1;
    } else if (dac_write_component == 1) {
        vga_palette[dac_write_index][1] = (uint8_t)(value & 0x3F);
        dac_write_component = 2;
    } else {
        vga_palette[dac_write_index][2] = (uint8_t)(value & 0x3F);
        vga_palette_gray[dac_write_index] = vga_rgb_to_gray(
            vga_palette[dac_write_index][0],
            vga_palette[dac_write_index][1],
            vga_palette[dac_write_index][2]);
        palette_dirty = true;
        dac_write_component = 0;
        dac_write_index++;
    }
}
