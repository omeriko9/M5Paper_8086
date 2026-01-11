/**
 * @file bios.c
 * @brief BIOS emulation for DOS
 * 
 * Handles BIOS interrupt services (INT 10h, 13h, 16h, etc.)
 */

#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "bios.h"
#include "memory.h"
#include "disk.h"
#include "video.h"
#include "ports.h"
#include "config_defs.h"
#include "settings.h"

static const char *TAG = "BIOS";
static const char *KBD_TAG = "KBD";
static const char *BIOS_TTY_TAG = "BIOS_TTY";
static const char *CPU_TRACE_TAG = "CPU_TRACE";

// BIOS timer tick counter
static volatile uint32_t timer_ticks = 0;

// Teletype log buffer for INT 10h/0Eh
static char tty_line_buf[81];
static uint8_t tty_line_len = 0;
static int64_t tty_last_flush_us = 0;
static uint8_t tty_dirty_first_row = 0xFF;
static uint8_t tty_dirty_last_row = 0;

#define TTY_REFRESH_INTERVAL_US 120000

static void tty_mark_dirty_row(uint8_t row)
{
    if (row >= 25) return;
    if (tty_dirty_first_row == 0xFF) {
        tty_dirty_first_row = row;
        tty_dirty_last_row = row;
        return;
    }
    if (row < tty_dirty_first_row) tty_dirty_first_row = row;
    if (row > tty_dirty_last_row) tty_dirty_last_row = row;
}

static void tty_flush_dirty_rows(bool force)
{
    if (tty_dirty_first_row == 0xFF) return;

    const int64_t now = esp_timer_get_time();
    if (!force && tty_last_flush_us != 0 && (now - tty_last_flush_us) < TTY_REFRESH_INTERVAL_US) {
        return;
    }

    video_request_urgent_refresh(tty_dirty_first_row, tty_dirty_last_row);
    tty_dirty_first_row = 0xFF;
    tty_dirty_last_row = 0;
    tty_last_flush_us = now;
}

// Disk op log counters
static uint32_t disk_read_log_count = 0;
static uint32_t disk_write_log_count = 0;
static bool disk_params_logged[4] = {0};

// 8-bit register helpers using 16-bit register storage
#define GET_AL() ((uint8_t)(cpu->regs16[REG_AX] & 0xFF))
#define GET_AH() ((uint8_t)(cpu->regs16[REG_AX] >> 8))
#define GET_BL() ((uint8_t)(cpu->regs16[REG_BX] & 0xFF))
#define GET_BH() ((uint8_t)(cpu->regs16[REG_BX] >> 8))
#define GET_CL() ((uint8_t)(cpu->regs16[REG_CX] & 0xFF))
#define GET_CH() ((uint8_t)(cpu->regs16[REG_CX] >> 8))
#define GET_DL() ((uint8_t)(cpu->regs16[REG_DX] & 0xFF))
#define GET_DH() ((uint8_t)(cpu->regs16[REG_DX] >> 8))

#define SET_AL(v) do { cpu->regs16[REG_AX] = (cpu->regs16[REG_AX] & 0xFF00) | (uint8_t)(v); } while (0)
#define SET_AH(v) do { cpu->regs16[REG_AX] = (cpu->regs16[REG_AX] & 0x00FF) | ((uint16_t)(uint8_t)(v) << 8); } while (0)
#define SET_BL(v) do { cpu->regs16[REG_BX] = (cpu->regs16[REG_BX] & 0xFF00) | (uint8_t)(v); } while (0)
#define SET_BH(v) do { cpu->regs16[REG_BX] = (cpu->regs16[REG_BX] & 0x00FF) | ((uint16_t)(uint8_t)(v) << 8); } while (0)
#define SET_CL(v) do { cpu->regs16[REG_CX] = (cpu->regs16[REG_CX] & 0xFF00) | (uint8_t)(v); } while (0)
#define SET_CH(v) do { cpu->regs16[REG_CX] = (cpu->regs16[REG_CX] & 0x00FF) | ((uint16_t)(uint8_t)(v) << 8); } while (0)
#define SET_DL(v) do { cpu->regs16[REG_DX] = (cpu->regs16[REG_DX] & 0xFF00) | (uint8_t)(v); } while (0)
#define SET_DH(v) do { cpu->regs16[REG_DX] = (cpu->regs16[REG_DX] & 0x00FF) | ((uint16_t)(uint8_t)(v) << 8); } while (0)

static bool should_log_disk(uint32_t *counter)
{
    (*counter)++;
    return (*counter <= 200) || ((*counter % 200) == 0);
}

static void log_disk_failure(const char *op, uint8_t dl, uint16_t cyl,
                             uint8_t head, uint8_t sector, uint8_t count)
{
    uint32_t lba = disk_chs_to_lba(dl, cyl, head, sector);
    ESP_LOGE(TAG, "INT 13h/%s failed: DL=%02X C=%u H=%u S=%u count=%u LBA=%lu err=%02X",
             op, dl, cyl, head, sector, count, lba, disk_get_error());
}
// Keyboard buffer (BIOS-backed) plus mirror into BDA ring buffer for games.
#define KB_BUFFER_SIZE  16
static uint16_t kb_buffer[KB_BUFFER_SIZE];
static volatile uint8_t kb_head = 0;
static volatile uint8_t kb_tail = 0;
static uint8_t auto_enter_remaining = 0;
static portMUX_TYPE kb_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t bios_get_floppy_count(void)
{
    uint8_t count = 0;
    if (disk_is_ready(0x00)) {
        count++;
    }
    if (disk_is_ready(0x01)) {
        count++;
    }
    if (count == 0) {
        count = 1;
    }
    return count;
}

void bios_set_floppy_drives(uint8_t count)
{
    uint16_t equip = mem_read_word(BDA_EQUIP_FLAG);
    equip &= (uint16_t)~(0xC0 | 0x01);

    if (count > 0) {
        if (count > 4) {
            count = 4;
        }
        equip |= 0x01;
        equip |= (uint16_t)((count - 1) << 6);
    }

    mem_write_word(BDA_EQUIP_FLAG, equip);
}

static uint8_t bios_get_hdd_count(void)
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

void bios_set_hard_drives(uint8_t count)
{
    mem_write_byte(BDA_HD_COUNT, count);
}

/**
 * Initialize BIOS
 */
void bios_init(void)
{
    ESP_LOGI(TAG, "Initializing BIOS");
    
    // Initialize BIOS Data Area
    uint8_t *bda = mem_get_ptr(BDA_BASE);
    if (bda) {
        // Equipment flags: 80x25 color, 1 floppy, 1 serial port (COM1), PS/2 Mouse.
        // Bit 0: Floppy installed
        // Bit 2: Pointing device installed (PS/2)
        // Bit 4-5: Video mode (10 = 80x25 color)
        // Bit 6-7: Number of floppies (00 = 1 drive)
        // Bit 9-11: Number of serial ports (001 = 1)
        mem_write_word(BDA_EQUIP_FLAG, 0x0225);

        // COM port base addresses (for serial mouse drivers).
        mem_write_word(BDA_COM1_PORT, PORT_COM1_BASE);
        mem_write_word(BDA_COM2_PORT, 0x0000);
        mem_write_word(BDA_COM3_PORT, 0x0000);
        mem_write_word(BDA_COM4_PORT, 0x0000);

        // Hard disk count (updated later once disks are mounted).
        mem_write_byte(BDA_HD_COUNT, 0);
        
        // Memory size in KB (640KB conventional)
        mem_write_word(BDA_MEM_SIZE, 640);
        
        // Video mode: 80x25 color
        mem_write_byte(BDA_VIDEO_MODE, 0x03);
        mem_write_word(BDA_VIDEO_COLS, 80);
        mem_write_word(BDA_VIDEO_PAGE_SIZE, 0x1000);
        mem_write_word(BDA_VIDEO_PAGE_OFF, 0x0000);
        mem_write_byte(BDA_ACTIVE_PAGE, 0);
        mem_write_byte(BDA_VIDEO_ROWS, 24);  // 25 rows (0-24)
        mem_write_word(BDA_CHAR_HEIGHT, 16);
        
        // Cursor position (page 0)
        mem_write_word(BDA_CURSOR_POS, 0x0000);
        
        // Cursor type
        mem_write_word(BDA_CURSOR_TYPE, 0x0607);
        
        // Keyboard buffer pointers (linear addresses for programs reading BDA directly).
        uint16_t kb_start = BDA_KB_BUFFER;
        uint16_t kb_end = (uint16_t)(BDA_KB_BUFFER + 32);
        mem_write_word(BDA_KB_BUF_HEAD, kb_start);
        mem_write_word(BDA_KB_BUF_TAIL, kb_start);
        mem_write_word(BDA_KB_BUF_START, kb_start);
        mem_write_word(BDA_KB_BUF_END, kb_end);
        
        // CRTC port
        mem_write_word(BDA_CRTC_PORT, 0x03D4);
    }
    
    // Initialize timer
    timer_ticks = 0;
    
    // Initialize keyboard buffer
    kb_head = kb_tail = 0;
    auto_enter_remaining = 2;
}

/**
 * Timer tick handler
 */
void bios_timer_tick(void)
{
    timer_ticks++;
    
    // Update BDA timer counter
    mem_write_dword(BDA_TIMER_COUNT, timer_ticks);
    
    // Check for midnight rollover (0x1800B0 ticks per day)
    if (timer_ticks >= 0x1800B0) {
        timer_ticks = 0;
        mem_write_byte(BDA_TIMER_OVERFLOW, 1);
    }
}

static uint8_t bios_scancode_to_ascii(uint8_t scancode)
{
    // Simple scancode to ASCII mapping (incomplete)
    static const char scancode_to_ascii[] = {
        0, 27, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', 8,   // 00-0E
        9, 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', 13,     // 0F-1C
        0, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '`',        // 1D-29
        0, '\\', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/',             // 2A-35
        0, '*', 0, ' '                                                          // 36-39
    };

    if (scancode < sizeof(scancode_to_ascii)) {
        return (uint8_t)scancode_to_ascii[scancode];
    }
    return 0;
}

/**
 * Add key to keyboard buffer
 */
void bios_key_enqueue(uint8_t scancode, uint8_t ascii, bool extended)
{
    (void)extended;
    uint16_t key = (uint16_t)((scancode << 8) | ascii);

    if (!xPortInIsrContext()) {
        ESP_LOGI(KBD_TAG, "BIOS ENQUEUE: sc=%02X ascii=%02X", scancode, ascii);
    }

    portENTER_CRITICAL(&kb_mux);

    // Add to local BIOS buffer if not full.
    uint8_t next_head = (uint8_t)((kb_head + 1) % KB_BUFFER_SIZE);
    if (next_head != kb_tail) {
        kb_buffer[kb_head] = key;
        kb_head = next_head;
    }

    // Mirror into BDA ring buffer for programs that read it directly.
    uint16_t start = mem_read_word(BDA_KB_BUF_START);
    uint16_t end = mem_read_word(BDA_KB_BUF_END);
    const uint16_t default_start = BDA_KB_BUFFER;
    const uint16_t default_end = (uint16_t)(BDA_KB_BUFFER + 32);
    if (start == 0 || end == 0 || start >= end) {
        start = default_start;
        end = default_end;
        mem_write_word(BDA_KB_BUF_START, start);
        mem_write_word(BDA_KB_BUF_END, end);
    }

    uint16_t head = mem_read_word(BDA_KB_BUF_HEAD);
    uint16_t tail = mem_read_word(BDA_KB_BUF_TAIL);
    if (head < start || head >= end || (head & 1) != 0) {
        head = start;
    }
    if (tail < start || tail >= end || (tail & 1) != 0) {
        tail = start;
    }

    uint16_t next = (uint16_t)(head + 2);
    if (next >= end) {
        next = start;
    }

    if (next == tail) {
        portEXIT_CRITICAL(&kb_mux);
        return;
    }

    mem_write_word(head, key);
    mem_write_word(BDA_KB_BUF_HEAD, next);

    portEXIT_CRITICAL(&kb_mux);
}

void bios_key_press(uint8_t scancode, bool pressed)
{
    if (!pressed) return;  // Ignore key releases for now

    uint8_t ascii = bios_scancode_to_ascii(scancode);
    bios_key_enqueue(scancode, ascii, false);
}

void bios_kbd_update_flags(uint8_t scancode, bool pressed, bool extended)
{
    uint8_t flags = mem_read_byte(BDA_KB_FLAG);
    uint8_t flags2 = mem_read_byte(BDA_KB_FLAG_1);
    uint8_t code = (uint8_t)(scancode & 0x7F);

    switch (code) {
        case 0x2A:  // Left Shift
            if (pressed) flags |= 0x02;
            else flags &= (uint8_t)~0x02;
            break;
        case 0x36:  // Right Shift
            if (pressed) flags |= 0x01;
            else flags &= (uint8_t)~0x01;
            break;
        case 0x1D:  // Ctrl
            if (pressed) flags |= 0x04;
            else flags &= (uint8_t)~0x04;
            if (!extended) {
                if (pressed) flags2 |= 0x01;
                else flags2 &= (uint8_t)~0x01;
            }
            break;
        case 0x38:  // Alt
            if (pressed) flags |= 0x08;
            else flags &= (uint8_t)~0x08;
            if (!extended) {
                if (pressed) flags2 |= 0x02;
                else flags2 &= (uint8_t)~0x02;
            }
            break;
        case 0x3A:  // Caps Lock
            if (pressed) flags ^= 0x40;
            break;
        case 0x45:  // Num Lock
            if (pressed) flags ^= 0x20;
            break;
        case 0x46:  // Scroll Lock
            if (pressed) flags ^= 0x10;
            break;
        default:
            break;
    }

    mem_write_byte(BDA_KB_FLAG, flags);
    mem_write_byte(BDA_KB_FLAG_1, flags2);
}

/**
 * Check if key is available
 */
bool bios_key_available(void)
{
    bool available;
    portENTER_CRITICAL(&kb_mux);
    available = (kb_head != kb_tail);
    portEXIT_CRITICAL(&kb_mux);
    return available;
}

/**
 * Get key from buffer
 */
uint16_t bios_get_key(void)
{
    portENTER_CRITICAL(&kb_mux);
    if (kb_head == kb_tail) {
        portEXIT_CRITICAL(&kb_mux);
        return 0;
    }

    uint16_t key = kb_buffer[kb_tail];
    kb_tail = (uint8_t)((kb_tail + 1) % KB_BUFFER_SIZE);

    // Advance BDA tail to keep it roughly in sync with BIOS reads.
    uint16_t start = mem_read_word(BDA_KB_BUF_START);
    uint16_t end = mem_read_word(BDA_KB_BUF_END);
    uint16_t head = mem_read_word(BDA_KB_BUF_HEAD);
    uint16_t tail = mem_read_word(BDA_KB_BUF_TAIL);

    if (start != 0 && end != 0 && start < end && head != tail) {
        tail = (uint16_t)(tail + 2);
        if (tail >= end) {
            tail = start;
        }
        mem_write_word(BDA_KB_BUF_TAIL, tail);
    }

    portEXIT_CRITICAL(&kb_mux);
    return key;
}

/**
 * Handle INT 10h - Video Services
 */
static bool handle_int10(cpu8086_t *cpu)
{
    uint8_t ah = GET_AH();  // AH
    uint8_t al = GET_AL();  // AL
    
    switch (ah) {
        case 0x00:  // Set video mode
            ESP_LOGD(TAG, "INT 10h/00: Set video mode %02X", al);
            video_set_mode(al);
            break;
            
        case 0x01:  // Set cursor type
            ESP_LOGD(TAG, "INT 10h/01: Set cursor type");
            video_set_cursor_type(GET_CH(), GET_CL());  // CH, CL
            break;
            
        case 0x02:  // Set cursor position
            ESP_LOGD(TAG, "INT 10h/02: Set cursor pos (%d,%d)", 
                     GET_DH(), GET_DL());  // DH=row, DL=col
            video_set_cursor_pos(GET_DH(), GET_DL());
            break;
            
        case 0x03:  // Get cursor position
            {
                uint8_t row, col;
                video_get_cursor_pos(&row, &col);
                SET_DH(row);  // DH
                SET_DL(col);  // DL
                cpu->regs16[REG_CX] = mem_read_word(BDA_CURSOR_TYPE);
            }
            break;
            
        case 0x05:  // Set active display page
            video_set_page(al);
            break;
            
        case 0x06:  // Scroll up
            ESP_LOGD(TAG, "INT 10h/06: Scroll up %d lines", al);
            video_scroll_up(al, GET_BH(), GET_CH(), GET_CL(),
                           GET_DH(), GET_DL());  // AL, BH, CH, CL, DH, DL
            break;
            
        case 0x07:  // Scroll down
            video_scroll_down(al, GET_BH(), GET_CH(), GET_CL(),
                             GET_DH(), GET_DL());
            break;
            
        case 0x08:  // Read character and attribute
            {
                uint8_t ch, attr;
                uint8_t row, col;
                video_get_cursor_pos(&row, &col);
                video_read_char(row, col, &ch, &attr);
                SET_AL(ch);   // AL
                SET_AH(attr); // AH
            }
            break;
            
        case 0x09:  // Write character and attribute
            {
                uint8_t row, col;
                video_get_cursor_pos(&row, &col);
                uint8_t cols = (uint8_t)(mem_read_word(BDA_VIDEO_COLS) & 0xFF);
                if (cols == 0) cols = 80;
                uint16_t count = cpu->regs16[REG_CX];
                uint8_t attr = GET_BL();  // BL
                for (uint16_t i = 0; i < count; i++) {
                    video_write_char(row, col, al, attr);
                    col++;
                    if (col >= cols) {
                        col = 0;
                        row++;
                    }
                }
            }
            break;
            
        case 0x0A:  // Write character only
            {
                uint8_t row, col;
                uint8_t ch, attr;
                video_get_cursor_pos(&row, &col);
                video_read_char(row, col, &ch, &attr);  // Get current attr
                uint8_t cols = (uint8_t)(mem_read_word(BDA_VIDEO_COLS) & 0xFF);
                if (cols == 0) cols = 80;
                uint16_t count = cpu->regs16[REG_CX];
                for (uint16_t i = 0; i < count; i++) {
                    video_write_char(row, col, al, attr);
                    col++;
                    if (col >= cols) {
                        col = 0;
                        row++;
                    }
                }
            }
            break;
            
        case 0x0E:  // Teletype output
            {
                uint8_t row, col;
                video_get_cursor_pos(&row, &col);
                uint8_t cols = (uint8_t)(mem_read_word(BDA_VIDEO_COLS) & 0xFF);
                if (cols == 0) cols = 80;
                uint8_t rows = (uint8_t)(mem_read_byte(BDA_VIDEO_ROWS) + 1);
                if (rows == 0) rows = 25;

                ESP_LOGI(BIOS_TTY_TAG, "INT 10h/0E: char=%02X ('%c') at %d,%d", al, (al >= 0x20 && al <= 0x7E) ? al : '.', row, col);

                const bool allow_urgent = ((video_get_mode() & 0x7F) != 0x13);
                
                if (al == 0x0D) {  // CR
                    col = 0;
                    if (allow_urgent) {
                        tty_mark_dirty_row(row);
                        tty_flush_dirty_rows(true);
                    }
                    if (tty_line_len > 0) {
                        tty_line_buf[tty_line_len] = '\0';
                        ESP_LOGI(TAG, "TTY: %s", tty_line_buf);
                        tty_line_len = 0;
                    }
                } else if (al == 0x0A) {  // LF
                    if (allow_urgent) {
                        tty_mark_dirty_row(row);
                        tty_flush_dirty_rows(true);
                    }
                    row++;
                    if (tty_line_len > 0) {
                        tty_line_buf[tty_line_len] = '\0';
                        ESP_LOGI(TAG, "TTY: %s", tty_line_buf);
                        tty_line_len = 0;
                    }
                    if (row >= rows) {
                        if (allow_urgent && app_settings_display_clear_on_bottom()) {
                            video_clear_screen_partial(0x07);
                            video_request_urgent_refresh(0, (uint8_t)(rows - 1));
                            tty_dirty_first_row = 0xFF;
                            tty_dirty_last_row = 0;
                            row = 0;
                            col = 0;
                        } else {
                            video_scroll_up(1, 0x07, 0, 0, (uint8_t)(rows - 1), (uint8_t)(cols - 1));
                            if (allow_urgent) {
                                video_request_urgent_refresh(0, (uint8_t)(rows - 1));
                                tty_dirty_first_row = 0xFF;
                                tty_dirty_last_row = 0;
                                tty_last_flush_us = esp_timer_get_time();
                            }
                            row = (uint8_t)(rows - 1);
                        }
                    }
                } else if (al == 0x08) {  // BS
                    if (col > 0) col--;
                } else if (al == 0x07) {  // Bell
                    // Ignore bell
                } else {
                    video_write_char(row, col, al, 0x07);
                    if (allow_urgent) {
                        tty_mark_dirty_row(row);
                    }
                    col++;
                    if (col >= cols) {
                        col = 0;
                        if (allow_urgent) {
                            tty_flush_dirty_rows(true);
                        }
                        row++;
                        if (row >= rows) {
                            if (allow_urgent && app_settings_display_clear_on_bottom()) {
                                video_clear_screen_partial(0x07);
                                video_request_urgent_refresh(0, (uint8_t)(rows - 1));
                                tty_dirty_first_row = 0xFF;
                                tty_dirty_last_row = 0;
                                row = 0;
                                col = 0;
                            } else {
                                video_scroll_up(1, 0x07, 0, 0, (uint8_t)(rows - 1), (uint8_t)(cols - 1));
                                if (allow_urgent) {
                                    video_request_urgent_refresh(0, (uint8_t)(rows - 1));
                                    tty_dirty_first_row = 0xFF;
                                    tty_dirty_last_row = 0;
                                    tty_last_flush_us = esp_timer_get_time();
                                }
                                row = (uint8_t)(rows - 1);
                            }
                        }
                    }
                    if (al >= 0x20 && tty_line_len < sizeof(tty_line_buf) - 1) {
                        tty_line_buf[tty_line_len++] = (char)al;
                    }
                    if (allow_urgent) {
                        tty_flush_dirty_rows(false);
                    }
                }
                
                video_set_cursor_pos(row, col);
            }
            break;
            
        case 0x0F:  // Get video mode
            SET_AL(video_get_mode());  // AL
            SET_AH((uint8_t)(mem_read_word(BDA_VIDEO_COLS) & 0xFF));  // AH = columns
            SET_BH(video_get_page());  // BH = page
            break;
            
        case 0x10:  // Set palette (ignore for e-paper)
            break;
            
        case 0x11:  // Character generator
            break;
            
        case 0x12:  // Alternate select
            // Return video memory size
            SET_BL(0x03);  // BL = 256KB video memory
            break;
            
        case 0x1A:  // Get/set display combination
            if (al == 0x00) {
                SET_AL(0x1A);  // Function supported
                SET_BL(0x08);  // VGA with color analog display
            }
            break;
            
        default:
            ESP_LOGD(TAG, "INT 10h/%02X: Unhandled", ah);
            break;
    }
    
    return true;
}

/**
 * Handle INT 11h - Equipment List
 */
static bool handle_int11(cpu8086_t *cpu)
{
    cpu->regs16[REG_AX] = mem_read_word(BDA_EQUIP_FLAG);
    return true;
}

/**
 * Handle INT 12h - Memory Size
 */
static bool handle_int12(cpu8086_t *cpu)
{
    cpu->regs16[REG_AX] = mem_read_word(BDA_MEM_SIZE);
    return true;
}

/**
 * Handle INT 13h - Disk Services
 */
static bool handle_int13(cpu8086_t *cpu)
{
    static bool root_dir_logged = false;
    uint8_t ah = GET_AH();
    uint8_t dl = GET_DL();  // Drive number
    
    switch (ah) {
        case 0x00:  // Reset disk system
            ESP_LOGD(TAG, "INT 13h/00: Reset drive %02X", dl);
            (void)disk_take_media_changed(dl);
            SET_AH(0);  // AH = success
            cpu->flags &= ~FLAG_CF;
            break;
            
        case 0x01:  // Get disk status
            SET_AH(disk_get_error());
            cpu->flags &= ~FLAG_CF;
            break;
            
        case 0x02:  // Read sectors
            {
                uint8_t al = GET_AL();   // Sector count
                uint8_t cl = GET_CL() & 0x3F;  // Sector (bits 0-5)
                uint8_t ch = GET_CH();   // Cylinder low
                uint16_t cyl = ch | ((GET_CL() & 0xC0) << 2);  // Full cylinder
                uint8_t dh = GET_DH();   // Head
                
                uint32_t addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_BX];
                uint32_t lba = disk_chs_to_lba(dl, cyl, dh, cl);
                uint32_t fixed_lba = lba;
                bool use_lba_fallback = false;
                bool use_lba_offset = false;
                disk_geometry_t geom;
                if (disk_get_geometry(dl, &geom) && lba >= geom.total_sectors) {
                    uint32_t base = lba / 512;
                    uint32_t rem = lba % 512;
                    // Only apply fallback if the LBA looks like a byte offset (rem is small)
                    // This prevents masking legitimate "End of Disk" errors where LBA is just slightly above total_sectors
                    if (base < geom.total_sectors && rem < 128) {
                        fixed_lba = base;
                        use_lba_fallback = true;
                        if (rem != 0) {
                            uint32_t adjusted = base + rem;
                            if (adjusted < geom.total_sectors) {
                                fixed_lba = adjusted;
                                use_lba_offset = true;
                            }
                        }
                    }
                }
                
                if (should_log_disk(&disk_read_log_count)) {
                    if (use_lba_offset) {
                        ESP_LOGW(TAG, "INT 13h/02: CHS out of range, treating LBA=%lu as byte offset + sector delta -> sector=%lu",
                                 lba, fixed_lba);
                    } else if (use_lba_fallback) {
                        ESP_LOGW(TAG, "INT 13h/02: CHS out of range, treating LBA=%lu as byte offset -> sector=%lu",
                                 lba, fixed_lba);
                    }
                    ESP_LOGI(TAG, "INT 13h/02: Read %d sectors from C=%d H=%d S=%d (LBA=%lu) to %05lX (DL=%02X)",
                             al, cyl, dh, cl, use_lba_fallback ? fixed_lba : lba, addr, dl);
                }
                
                uint8_t *buffer = mem_get_ptr(addr);
                bool ok = use_lba_fallback
                          ? disk_read_lba(dl, fixed_lba, al, buffer)
                          : disk_read_chs(dl, cyl, dh, cl, al, buffer);
                if (ok) {
                    SET_AH(0);  // AH = success
                    SET_AL(al); // AL = sectors read
                    cpu->flags &= ~FLAG_CF;
                    if (!root_dir_logged && addr == 0x0500 && al == 1) {
                        uint16_t cluster = mem_read_word(addr + 0x1A);
                        uint32_t size = mem_read_dword(addr + 0x1C);
                        ESP_LOGI(TAG, "RootDir[0000:0500]: first entry cluster=%04X size=%lu",
                                 cluster, size);
                        root_dir_logged = true;
                    }
                } else {
                    SET_AH(disk_get_error());
                    cpu->flags |= FLAG_CF;
                    log_disk_failure("02", dl, cyl, dh, cl, al);
                }
            }
            break;
            
        case 0x03:  // Write sectors
            {
                uint8_t al = GET_AL();
                uint8_t cl = GET_CL() & 0x3F;
                uint8_t ch = GET_CH();
                uint16_t cyl = ch | ((GET_CL() & 0xC0) << 2);
                uint8_t dh = GET_DH();
                
                uint32_t addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_BX];
                uint32_t lba = disk_chs_to_lba(dl, cyl, dh, cl);
                
                if (should_log_disk(&disk_write_log_count)) {
                    ESP_LOGI(TAG, "INT 13h/03: Write %d sectors to C=%d H=%d S=%d (LBA=%lu) from %05lX (DL=%02X)",
                             al, cyl, dh, cl, lba, addr, dl);
                }
                
                uint8_t *buffer = mem_get_ptr(addr);
                if (disk_write_chs(dl, cyl, dh, cl, al, buffer)) {
                    SET_AH(0);
                    SET_AL(al);
                    cpu->flags &= ~FLAG_CF;
                } else {
                    SET_AH(disk_get_error());
                    cpu->flags |= FLAG_CF;
                    log_disk_failure("03", dl, cyl, dh, cl, al);
                }
            }
            break;
            
        case 0x04:  // Verify sectors
            SET_AH(0);
            cpu->flags &= ~FLAG_CF;
            break;
            
        case 0x08:  // Get drive parameters
            {
                disk_geometry_t geom;
                if (disk_get_geometry(dl, &geom)) {
                    uint16_t max_cyl = (geom.cylinders > 0) ? (uint16_t)(geom.cylinders - 1) : 0;
                    uint8_t max_head = (geom.heads > 0) ? (uint8_t)(geom.heads - 1) : 0;
                    uint8_t spt = geom.sectors ? geom.sectors : 1;
                    SET_AH(0);  // AH = 0
                    SET_AL(0);  // AL = 0
                    SET_BL((dl < 0x80) ? 0x04 : 0x00);  // BL = drive type
                    SET_CL((spt & 0x3F) | ((max_cyl >> 2) & 0xC0));  // CL: sector + cyl high bits
                    SET_CH(max_cyl & 0xFF);  // CH: cyl low byte
                    SET_DH(max_head);  // DH: max head
                    if (dl < 0x80) {
                        SET_DL(bios_get_floppy_count());
                    } else {
                        SET_DL(bios_get_hdd_count());  // DL = number of hard drives
                    }
                    cpu->flags &= ~FLAG_CF;

                    uint8_t idx = (dl >= 0x80) ? (dl - 0x80) + 2 : dl;
                    if (idx < (uint8_t)(sizeof(disk_params_logged) / sizeof(disk_params_logged[0])) &&
                        !disk_params_logged[idx]) {
                        disk_params_logged[idx] = true;
                        ESP_LOGI(TAG, "INT 13h/08: DL=%02X C=%u H=%u S=%u total=%lu",
                                 dl, geom.cylinders, geom.heads, geom.sectors, geom.total_sectors);
                    }
                } else {
                    SET_AH(0x07);  // Drive parameter error
                    cpu->flags |= FLAG_CF;
                    ESP_LOGE(TAG, "INT 13h/08 failed: DL=%02X err=%02X", dl, disk_get_error());
                }
            }
            break;
            
        case 0x15:  // Get disk type
            if (disk_is_ready(dl)) {
                if (dl < 0x80) {
                    SET_AH(0x02);  // Floppy with change line
                } else {
                    SET_AH(0x03);  // Hard disk
                    disk_geometry_t geom;
                    disk_get_geometry(dl, &geom);
                    cpu->regs16[REG_CX] = (geom.total_sectors >> 16) & 0xFFFF;
                    cpu->regs16[REG_DX] = geom.total_sectors & 0xFFFF;
                }
                cpu->flags &= ~FLAG_CF;
            } else {
                SET_AH(0x00);  // No drive
                cpu->flags |= FLAG_CF;
                ESP_LOGW(TAG, "INT 13h/15: Drive not ready DL=%02X", dl);
            }
            break;

        case 0x16:  // Get disk change status (floppy)
            if (dl < 0x80) {
                // Emulate a change line so DOS invalidates its FAT/dir cache after a swap.
                if (disk_take_media_changed(dl)) {
                    SET_AH(0x06);  // Change line active
                    cpu->flags |= FLAG_CF;
                } else {
                    SET_AH(0x00);  // No change
                    cpu->flags &= ~FLAG_CF;
                }
            } else {
                SET_AH(0x01);  // Invalid command for HDD here
                cpu->flags |= FLAG_CF;
            }
            break;
            
        default:
            ESP_LOGD(TAG, "INT 13h/%02X: Unhandled", ah);
            SET_AH(0x01);  // Invalid command
            cpu->flags |= FLAG_CF;
            break;
    }
    
    return true;
}

/**
 * Handle INT 14h - Serial Port Services (minimal, COM1 only)
 *
 * Enough for common DOS serial mouse drivers that use BIOS calls to init/poll COM.
 */
static bool handle_int14(cpu8086_t *cpu)
{
    const uint8_t ah = GET_AH();
    const uint8_t port_idx = (uint8_t)(cpu->regs16[REG_DX] & 0xFF);
    if (port_idx > 3) {
        cpu->flags |= FLAG_CF;
        return true;
    }

    const uint16_t base = mem_read_word((uint32_t)BDA_COM1_PORT + ((uint32_t)port_idx * 2u));
    if (base == 0) {
        cpu->flags |= FLAG_CF;
        return true;
    }

    switch (ah) {
        case 0x00:  // Initialize port
            // Ignore AL's requested baud/parity; just set a sane default and enable IRQs.
            port_out(base + 1, 0x00);  // IER: disable interrupts
            port_out(base + 3, 0x03);  // LCR: 8N1
            port_out(base + 4, 0x0B);  // MCR: DTR|RTS|OUT2
            cpu->flags &= ~FLAG_CF;
            SET_AH(port_in(base + 5)); // LSR
            SET_AL(port_in(base + 6)); // MSR
            return true;

        case 0x01:  // Transmit character (AL)
            port_out(base + 0, GET_AL());
            cpu->flags &= ~FLAG_CF;
            SET_AH(port_in(base + 5)); // LSR
            SET_AL(port_in(base + 6)); // MSR
            return true;

        case 0x02:  // Receive character
            {
                const uint8_t lsr = port_in(base + 5);
                if (lsr & 0x01) {
                    SET_AL(port_in(base + 0));
                    cpu->flags &= ~FLAG_CF;
                } else {
                    SET_AL(0x00);
                    cpu->flags |= FLAG_CF;
                }
                SET_AH(lsr);
            }
            return true;

        case 0x03:  // Get port status
            cpu->flags &= ~FLAG_CF;
            SET_AH(port_in(base + 5)); // LSR
            SET_AL(port_in(base + 6)); // MSR
            return true;

        default:
            cpu->flags |= FLAG_CF;
            return true;
    }
}

/**
 * Handle INT 16h - Keyboard Services
 */
static bool handle_int16(cpu8086_t *cpu)
{
    uint8_t ah = GET_AH();
    static int64_t int16_poll_log_us = 0;
    
    switch (ah) {
        case 0x00:  // Get keystroke
        case 0x10:  // Get enhanced keystroke
            // Wait for key (in real implementation, this would block)
            if (bios_key_available()) {
                uint16_t key = bios_get_key();
                SET_AL(key & 0xFF);        // AL = ASCII
                SET_AH((key >> 8) & 0xFF); // AH = scancode

                ESP_LOGI(KBD_TAG, "INT 16h/%02X: Returning key %04X", ah, key);

            } else if (auto_enter_remaining > 0) {
                // Auto-accept date/time prompts to reach the DOS prompt.
                auto_enter_remaining--;
                SET_AL(0x0D);  // CR
                SET_AH(0x1C);  // Enter scancode
            } else {
                // No key available - return 0
                cpu->regs16[REG_AX] = 0;
                const int64_t now = esp_timer_get_time();
                if (now - int16_poll_log_us >= 100000) {
                    ESP_LOGI(KBD_TAG, "INT 16h/%02X: No key", ah);
                    int16_poll_log_us = now;
                }
            }
            break;
            
        case 0x01:  // Check keystroke
        case 0x11:  // Check enhanced keystroke
            {
                uint16_t key = 0;
                bool have_key = false;
                portENTER_CRITICAL(&kb_mux);
                have_key = (kb_head != kb_tail);
                if (have_key) {
                    key = kb_buffer[kb_tail];
                }
                portEXIT_CRITICAL(&kb_mux);

                if (have_key) {
                SET_AL(key & 0xFF);
                SET_AH((key >> 8) & 0xFF);
                cpu->flags &= ~FLAG_ZF;  // Key available
                ESP_LOGI(KBD_TAG, "INT 16h/%02X: Check key %04X ZF=0", ah, key);
                } else if (auto_enter_remaining > 0) {
                // Report a pending Enter so DOS advances past date/time prompts.
                SET_AL(0x0D);
                SET_AH(0x1C);
                cpu->flags &= ~FLAG_ZF;
                ESP_LOGI(KBD_TAG, "INT 16h/%02X: Auto-enter pending ZF=0", ah);
                } else {
                cpu->flags |= FLAG_ZF;   // No key
                const int64_t now = esp_timer_get_time();
                if (now - int16_poll_log_us >= 100000) {
                    ESP_LOGI(KBD_TAG, "INT 16h/%02X: No key ZF=1", ah);
                    int16_poll_log_us = now;
                }
                }
            }
            break;
            
        case 0x02:  // Get shift flags
            {
                uint8_t flags = mem_read_byte(BDA_KB_FLAG);
                SET_AL(flags);
                ESP_LOGI(KBD_TAG, "INT 16h/02: flags=%02X shift=%d ctrl=%d alt=%d num=%d",
                         flags,
                         (flags & 0x03) ? 1 : 0,
                         (flags & 0x04) ? 1 : 0,
                         (flags & 0x08) ? 1 : 0,
                         (flags & 0x20) ? 1 : 0);
            }
            break;
            
        case 0x03:  // Set typematic rate
            break;
            
        default:
            ESP_LOGD(TAG, "INT 16h/%02X: Unhandled", ah);
            break;
    }
    
    return true;
}

static bool handle_int15(cpu8086_t *cpu)
{
    uint8_t ah = GET_AH();
    switch (ah) {
        case 0xC2: // Pointing Device BIOS Interface
            switch (GET_AL()) {
                case 0x00: // Enable/Disable
                    // BH: 0=Disable, 1=Enable
                    {
                        uint8_t bh = GET_BH();
                        ESP_LOGI(TAG, "INT 15h/C200: Mouse %s", bh ? "Enable" : "Disable");
                        // Send command to KBC
                        port_out(PORT_KBC_CMD, 0xD4); // Write to Mouse
                        port_out(PORT_KBC_DATA, bh ? 0xF4 : 0xF5); // Enable/Disable Data Reporting
                        SET_AH(0x00); // Success
                        cpu->flags &= ~FLAG_CF;
                    }
                    break;
                case 0x01: // Reset
                    ESP_LOGI(TAG, "INT 15h/C201: Mouse Reset");
                    port_out(PORT_KBC_CMD, 0xD4);
                    // Reset command is 0xFF.
                    // Note: sending 0xFF to mouse expects FA AA 00.
                    port_out(PORT_KBC_DATA, 0xFF);
                    SET_BH(0x00); // Device ID
                    SET_AH(0x00); // Success
                    cpu->flags &= ~FLAG_CF;
                    break;
                case 0x02: // Set Sample Rate
                    // BH = sample rate
                    ESP_LOGI(TAG, "INT 15h/C202: Set Sample Rate %d", GET_BH());
                    port_out(PORT_KBC_CMD, 0xD4);
                    port_out(PORT_KBC_DATA, 0xF3); // Set Sample Rate
                    port_out(PORT_KBC_CMD, 0xD4);
                    port_out(PORT_KBC_DATA, GET_BH());
                    SET_AH(0x00);
                    cpu->flags &= ~FLAG_CF;
                    break;
                case 0x03: // Set Resolution
                    // BH = resolution
                    ESP_LOGI(TAG, "INT 15h/C203: Set Resolution %d", GET_BH());
                    port_out(PORT_KBC_CMD, 0xD4);
                    port_out(PORT_KBC_DATA, 0xE8); // Set Resolution
                    port_out(PORT_KBC_CMD, 0xD4);
                    port_out(PORT_KBC_DATA, GET_BH());
                    SET_AH(0x00);
                    cpu->flags &= ~FLAG_CF;
                    break;
                case 0x04: // Get Device ID
                    ESP_LOGI(TAG, "INT 15h/C204: Get Device ID");
                    SET_BH(0x00); // Standard PS/2
                    SET_AH(0x00);
                    cpu->flags &= ~FLAG_CF;
                    break;
                case 0x05: // Initialize (Set Data Packet Size)
                    // BH = data package size (1-8 bytes)
                    ESP_LOGI(TAG, "INT 15h/C205: Init Packet Size %d", GET_BH());
                    SET_AH(0x00);
                    cpu->flags &= ~FLAG_CF;
                    break;
                case 0x06: // Extended commands
                    ESP_LOGI(TAG, "INT 15h/C206: Extended");
                    SET_AH(0x86);
                    cpu->flags |= FLAG_CF;
                    break;
                case 0x07: // Set Scaling Factor
                    // BH = scaling (1 or 2)
                        ESP_LOGI(TAG, "INT 15h/C207: Set Scaling %d", GET_BH());
                    port_out(PORT_KBC_CMD, 0xD4);
                    port_out(PORT_KBC_DATA, (GET_BH() >= 2) ? 0xE7 : 0xE6); 
                    SET_AH(0x00);
                    cpu->flags &= ~FLAG_CF;
                    break;
                default:
                    ESP_LOGE(TAG, "INT 15h/C2 Unknown Subfn %02X", GET_AL());
                    SET_AH(0x86); // function not supported
                    cpu->flags |= FLAG_CF;
            }
            break;
        
        case 0x88: // Get Extended Memory Size
            cpu->regs16[REG_AX] = 32 * 1024;
            cpu->flags &= ~FLAG_CF;
            break;
        
        case 0x87: // Copy Extended Memory Block (Block Move)
            // This is required for HIMEM.SYS / Windows 3.1
            // Ref: RBIL INT 15h/AH=87h
            // ES:SI points to Global Descriptor Table
            // CX = number of words to copy
            // Return: AH = 0 (success), 1 (RAM parity), 2 (exception), 3 (A20 failed)
            // Implementation: We can just do a memcpy since our memory is flat
            {
                uint32_t count = (uint32_t)cpu->regs16[REG_CX] * 2;
                uint16_t gdt_seg = cpu->sregs[SEG_ES];
                uint16_t gdt_off = cpu->regs16[REG_SI];
                uint32_t gdt_phys = ((uint32_t)gdt_seg << 4) + gdt_off;
                const uint32_t mem_limit = MEM_SIZE + 0x10000;
                uint32_t src_base = 0;
                uint32_t dst_base = 0;
                uint64_t src_end = 0;
                uint64_t dst_end = 0;
                bool range_bad = false;
                bool lowmem_bad = false;
                bool a20_ok = mem_get_a20();
                
                // Read source and dest from GDT
                // Entry 0: Dummy
                // Entry 1: GDT itself? NO here it is:
                // GDT entries are 8 bytes.
                // ES:SI + 10h = Source descriptor
                // ES:SI + 18h = Dest descriptor
                
                // Descriptor:
                // Bytes 0-1: Limit low
                // Bytes 2-4: Base low 24
                // Byte 5: Access
                // Byte 6: Limit high / Flags
                // Byte 7: Base high
                
                uint32_t src_base_desc = gdt_phys + 0x10;
                uint32_t dst_base_desc = gdt_phys + 0x18;
                
                if (count == 0) {
                    cpu->regs16[REG_AX] = 0x0000;
                    cpu->flags &= ~FLAG_CF;
                    break;
                }

                if (gdt_phys + 0x18 + 7 >= mem_limit) {
                    ESP_LOGI(CPU_TRACE_TAG,
                             "INT 15h/AH=87h reject gdt=%05lX count=%lu (gdt out of range)",
                             (unsigned long)gdt_phys, (unsigned long)count);
                    SET_AH(0x02);
                    cpu->flags |= FLAG_CF;
                    break;
                }

                src_base =
                    ((uint32_t)mem_read_byte(src_base_desc + 2)) |
                    ((uint32_t)mem_read_byte(src_base_desc + 3) << 8) |
                    ((uint32_t)mem_read_byte(src_base_desc + 4) << 16) |
                    ((uint32_t)mem_read_byte(src_base_desc + 7) << 24);
                    
                dst_base =
                    ((uint32_t)mem_read_byte(dst_base_desc + 2)) |
                    ((uint32_t)mem_read_byte(dst_base_desc + 3) << 8) |
                    ((uint32_t)mem_read_byte(dst_base_desc + 4) << 16) |
                    ((uint32_t)mem_read_byte(dst_base_desc + 7) << 24);

                src_end = (uint64_t)src_base + count;
                dst_end = (uint64_t)dst_base + count;
                range_bad = (src_end > mem_limit) || (dst_end > mem_limit);
                lowmem_bad = (src_base < 0x0500 && src_end > 0) || (dst_base < 0x0500 && dst_end > 0);

                ESP_LOGI(CPU_TRACE_TAG,
                         "INT 15h/AH=87h gdt=%05lX count=%lu src=%08lX dst=%08lX a20=%u",
                         (unsigned long)gdt_phys, (unsigned long)count,
                         (unsigned long)src_base, (unsigned long)dst_base,
                         a20_ok ? 1 : 0);

                if (!a20_ok) {
                    SET_AH(0x03);
                    cpu->flags |= FLAG_CF;
                    break;
                }

                if (range_bad || lowmem_bad) {
                    ESP_LOGI(CPU_TRACE_TAG,
                             "INT 15h/AH=87h reject src=%08lX dst=%08lX count=%lu",
                             (unsigned long)src_base, (unsigned long)dst_base, (unsigned long)count);
                    SET_AH(0x02);
                    cpu->flags |= FLAG_CF;
                    break;
                }
                    
                // Perform copy
                for (uint32_t i = 0; i < count; i++) {
                    mem_write_byte(dst_base + i, mem_read_byte(src_base + i));
                }
                
                cpu->regs16[REG_AX] = 0x0000; // AH=0 success
                cpu->flags &= ~FLAG_CF;
            }
            break;

        case 0xC0: // Get system configuration parameters
            // ES:BX points to table.
            cpu->flags |= FLAG_CF;
            SET_AH(0x86);
            break;

        default:
            ESP_LOGD(TAG, "INT 15h/%02X: Unhandled", ah);
            cpu->flags |= FLAG_CF;
            SET_AH(0x86); 
            break;
    }
    return true;
}

/**
 * Handle INT 1Ah - Time Services
 */
static bool handle_int1a(cpu8086_t *cpu)
{
    uint8_t ah = GET_AH();
    
    switch (ah) {
        case 0x00:  // Get system time
            cpu->regs16[REG_CX] = (timer_ticks >> 16) & 0xFFFF;
            cpu->regs16[REG_DX] = timer_ticks & 0xFFFF;
            SET_AL(mem_read_byte(BDA_TIMER_OVERFLOW));
            mem_write_byte(BDA_TIMER_OVERFLOW, 0);
            cpu->flags &= ~FLAG_CF;
            break;
            
        case 0x01:  // Set system time
            timer_ticks = ((uint32_t)cpu->regs16[REG_CX] << 16) | cpu->regs16[REG_DX];
            mem_write_dword(BDA_TIMER_COUNT, timer_ticks);
            cpu->flags &= ~FLAG_CF;
            break;
            
        case 0x02:  // Get RTC time
            {
                time_t now = time(NULL);
                struct tm *t = localtime(&now);
                
                // BCD format
                SET_CH(((t->tm_hour / 10) << 4) | (t->tm_hour % 10));  // CH = hours
                SET_CL(((t->tm_min / 10) << 4) | (t->tm_min % 10));    // CL = minutes
                SET_DH(((t->tm_sec / 10) << 4) | (t->tm_sec % 10));    // DH = seconds
                SET_DL(0);  // DL = DST flag
                cpu->flags &= ~FLAG_CF;
            }
            break;
            
        case 0x04:  // Get RTC date
            {
                time_t now = time(NULL);
                struct tm *t = localtime(&now);
                int year = t->tm_year + 1900;
                
                // BCD format
                SET_CH(((year / 100 / 10) << 4) | ((year / 100) % 10));  // CH = century
                SET_CL((((year % 100) / 10) << 4) | (year % 10));        // CL = year
                SET_DH((((t->tm_mon + 1) / 10) << 4) | ((t->tm_mon + 1) % 10));  // DH = month
                SET_DL(((t->tm_mday / 10) << 4) | (t->tm_mday % 10));    // DL = day
                cpu->flags &= ~FLAG_CF;
            }
            break;

        case 0xB1:  // PCI BIOS (not implemented)
            // Many DOS storage drivers probe PCI BIOS via INT 1Ah AX=B1xx.
            // If we "handle" the interrupt without setting CF, they may assume success
            // and then act on garbage return values.
            ESP_LOGD(TAG, "INT 1Ah/PCI BIOS (AX=%04X): not supported", cpu->regs16[REG_AX]);
            SET_AH(0x81);          // Function not supported (common convention)
            cpu->flags |= FLAG_CF; // Signal failure
            break;
            
        default:
            ESP_LOGD(TAG, "INT 1Ah/%02X: Unhandled", ah);
            cpu->flags |= FLAG_CF;
            break;
    }
    
    return true;
}

/**
 * Handle BIOS interrupt
 */
bool bios_interrupt(cpu8086_t *cpu, uint8_t intnum)
{
    switch (intnum) {
        case INT_VIDEO:
            return handle_int10(cpu);
            
        case INT_EQUIPMENT:
            return handle_int11(cpu);
            
        case INT_MEMSIZE:
            return handle_int12(cpu);
            
        case INT_DISK:
            return handle_int13(cpu);

        case INT_SERIAL:
            return handle_int14(cpu);

        case INT_SYSTEM:
            return handle_int15(cpu);
            
        case INT_KEYBOARD:
            return handle_int16(cpu);
            
        case INT_TIME:
            return handle_int1a(cpu);
            
        case INT_BOOTSTRAP:
            {
                uint8_t dl = GET_DL();
                uint8_t boot_sector[512];
                ESP_LOGI(TAG, "INT 19h: Bootstrap from DL=%02X", dl);
                if (!disk_read_chs(dl, 0, 0, 1, 1, boot_sector)) {
                    ESP_LOGE(TAG, "INT 19h: Boot read failed (DL=%02X err=%02X)",
                             dl, disk_get_error());
                    cpu->halted = true;
                    return true;
                }
                mem_write_block(0x7C00, boot_sector, sizeof(boot_sector));
                cpu->sregs[SEG_CS] = 0x0000;
                cpu->ip = 0x7C00;
                cpu->sregs[SEG_DS] = 0x0000;
                cpu->sregs[SEG_ES] = 0x0000;
                cpu->sregs[SEG_SS] = 0x0000;
                cpu->regs16[REG_SP] = 0x7C00;
                cpu->halted = false;
            }
            return true;
            
        case INT_ROM_BASIC:
            ESP_LOGW(TAG, "INT 18h: ROM BASIC invoked");
            return false;

        case 0x20:  // DOS terminate
            ESP_LOGI(TAG, "INT 20h: Program terminated");
            return true;
            
        case INT_DOS:
            // This should be handled by DOS, not BIOS
            // Return false to let the IVT handler take over
            return false;
            
        default:
            ESP_LOGD(TAG, "INT %02Xh: Not handled by BIOS", intnum);
            return false;
    }
}

// Getter functions
void bios_set_video_mode(uint8_t mode)
{
    video_set_mode(mode);
}

uint8_t bios_get_video_mode(void)
{
    return video_get_mode();
}

void bios_set_numlock(bool enabled)
{
    uint8_t flags = mem_read_byte(BDA_KB_FLAG);
    if (enabled) {
        flags |= 0x20;
    } else {
        flags &= (uint8_t)~0x20;
    }
    mem_write_byte(BDA_KB_FLAG, flags);
}

void bios_set_cursor_pos(uint8_t page, uint8_t row, uint8_t col)
{
    video_set_cursor_pos(row, col);
}
