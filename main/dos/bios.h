/**
 * @file bios.h
 * @brief BIOS emulation for DOS
 */

#ifndef BIOS_H
#define BIOS_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu8086.h"

#ifdef __cplusplus
extern "C" {
#endif

// BIOS Data Area addresses
#define BDA_BASE            0x0400
#define BDA_COM1_PORT       0x0400
#define BDA_COM2_PORT       0x0402
#define BDA_COM3_PORT       0x0404
#define BDA_COM4_PORT       0x0406
#define BDA_LPT1_PORT       0x0408
#define BDA_LPT2_PORT       0x040A
#define BDA_LPT3_PORT       0x040C
#define BDA_EQUIP_FLAG      0x0410
#define BDA_MFG_TEST        0x0412
#define BDA_MEM_SIZE        0x0413
#define BDA_KB_FLAG         0x0417
#define BDA_KB_FLAG_1       0x0418
#define BDA_ALT_KEYPAD      0x0419
#define BDA_KB_BUF_HEAD     0x041A
#define BDA_KB_BUF_TAIL     0x041C
#define BDA_KB_BUFFER       0x041E
#define BDA_VIDEO_MODE      0x0449
#define BDA_VIDEO_COLS      0x044A
#define BDA_VIDEO_PAGE_SIZE 0x044C
#define BDA_VIDEO_PAGE_OFF  0x044E
#define BDA_CURSOR_POS      0x0450
#define BDA_CURSOR_TYPE     0x0460
#define BDA_ACTIVE_PAGE     0x0462
#define BDA_CRTC_PORT       0x0463
#define BDA_CRT_MODE_SET    0x0465
#define BDA_CRT_PALETTE     0x0466
#define BDA_TIMER_COUNT     0x046C
#define BDA_TIMER_OVERFLOW  0x0470
#define BDA_BREAK_FLAG      0x0471
#define BDA_RESET_FLAG      0x0472
#define BDA_HD_LAST_OP      0x0474
#define BDA_HD_COUNT        0x0475
#define BDA_HD_CTRL         0x0476
#define BDA_HD_PORT_OFF     0x0477
#define BDA_LPT_TIMEOUT     0x0478
#define BDA_COM_TIMEOUT     0x047C
#define BDA_KB_BUF_START    0x0480
#define BDA_KB_BUF_END      0x0482
#define BDA_VIDEO_ROWS      0x0484
#define BDA_CHAR_HEIGHT     0x0485
#define BDA_VIDEO_CTL       0x0487
#define BDA_VIDEO_SWITCHES  0x0488
#define BDA_VIDEO_FLAGS     0x0489
#define BDA_VGA_DCC         0x048A

// BIOS interrupts
#define INT_VIDEO           0x10
#define INT_EQUIPMENT       0x11
#define INT_MEMSIZE         0x12
#define INT_DISK            0x13
#define INT_SERIAL          0x14
#define INT_SYSTEM          0x15
#define INT_KEYBOARD        0x16
#define INT_PRINTER         0x17
#define INT_ROM_BASIC       0x18
#define INT_BOOTSTRAP       0x19
#define INT_TIME            0x1A
#define INT_CTRL_BREAK      0x1B
#define INT_TIMER_TICK      0x1C
#define INT_VIDEO_PARAMS    0x1D
#define INT_DISKETTE_PARAMS 0x1E
#define INT_VIDEO_GRAPHICS  0x1F
#define INT_DOS             0x21
#define INT_DOS_IDLE        0x28
#define INT_MULTIPLEX       0x2F

// Initialize BIOS
void bios_init(void);

// Update number of floppy drives in BDA equipment flags
void bios_set_floppy_drives(uint8_t count);

// Update number of hard drives in BDA (0040:0075).
void bios_set_hard_drives(uint8_t count);
void bios_set_numlock(bool enabled);

// Handle BIOS interrupt
bool bios_interrupt(cpu8086_t *cpu, uint8_t intnum);

// Timer tick (called ~18.2 times per second)
void bios_timer_tick(void);

// Keyboard functions
void bios_key_press(uint8_t scancode, bool pressed);
void bios_key_enqueue(uint8_t scancode, uint8_t ascii, bool extended);
void bios_kbd_update_flags(uint8_t scancode, bool pressed, bool extended);
bool bios_key_available(void);
uint16_t bios_get_key(void);

// Video mode functions
void bios_set_video_mode(uint8_t mode);
uint8_t bios_get_video_mode(void);

// Cursor functions
void bios_set_cursor_pos(uint8_t page, uint8_t row, uint8_t col);

#ifdef __cplusplus
}
#endif

#endif // BIOS_H
