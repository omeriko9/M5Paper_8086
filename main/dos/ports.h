/**
 * @file ports.h
 * @brief I/O Port emulation for 8086
 */

#ifndef PORTS_H
#define PORTS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Keyboard controller ports
#define PORT_KBC_DATA       0x60
#define PORT_KBC_STATUS     0x64
#define PORT_KBC_CMD        0x64

// PIC (Programmable Interrupt Controller) ports
#define PORT_PIC1_CMD       0x20
#define PORT_PIC1_DATA      0x21
#define PORT_PIC2_CMD       0xA0
#define PORT_PIC2_DATA      0xA1

// PIT (Programmable Interval Timer) ports
#define PORT_PIT_CH0        0x40
#define PORT_PIT_CH1        0x41
#define PORT_PIT_CH2        0x42
#define PORT_PIT_CTRL       0x43

// DMA Controller ports
#define PORT_DMA1_STATUS    0x08
#define PORT_DMA1_CMD       0x08
#define PORT_DMA1_REQ       0x09
#define PORT_DMA1_MASK      0x0A
#define PORT_DMA1_MODE      0x0B
#define PORT_DMA1_CLEAR_FF  0x0C
#define PORT_DMA1_TEMP      0x0D
#define PORT_DMA1_RESET     0x0D
#define PORT_DMA1_CLR_MASK  0x0E
#define PORT_DMA1_ALL_MASK  0x0F
#define PORT_DMA_PAGE_CH2   0x81
#define PORT_DMA_PAGE_CH3   0x82
#define PORT_DMA_PAGE_CH1   0x83

// Floppy controller ports
#define PORT_FDC_DOR        0x3F2    // Digital Output Register
#define PORT_FDC_MSR        0x3F4    // Main Status Register
#define PORT_FDC_DATA       0x3F5    // Data Register
#define PORT_FDC_DIR        0x3F7    // Digital Input Register
#define PORT_FDC_CCR        0x3F7    // Configuration Control Register (write)

// CMOS/RTC ports
#define PORT_CMOS_ADDR      0x70
#define PORT_CMOS_DATA      0x71

// VGA/CGA ports
#define PORT_CGA_MODE       0x3D8
#define PORT_CGA_PALETTE    0x3D9
#define PORT_CGA_STATUS     0x3DA
#define PORT_MDA_MODE       0x3B8
#define PORT_MDA_STATUS     0x3BA

// CRTC ports
#define PORT_CRTC_INDEX     0x3D4
#define PORT_CRTC_DATA      0x3D5
#define PORT_MDA_CRTC_INDEX 0x3B4
#define PORT_MDA_CRTC_DATA  0x3B5

// Serial ports (COM)
#define PORT_COM1_BASE      0x3F8
#define PORT_COM2_BASE      0x2F8
#define PORT_COM3_BASE      0x3E8
#define PORT_COM4_BASE      0x2E8

// Parallel ports (LPT)
#define PORT_LPT1_BASE      0x378
#define PORT_LPT2_BASE      0x278

// Speaker port
#define PORT_SPEAKER        0x61

// Initialize port emulation
void ports_init(void);

// Read from I/O port
uint8_t port_in(uint16_t port);
uint16_t port_in_word(uint16_t port);

// Write to I/O port
void port_out(uint16_t port, uint8_t val);
void port_out_word(uint16_t port, uint16_t val);

// Interrupt request
void port_irq(uint8_t irq);

// Get pending interrupt (-1 if none)
int port_get_interrupt(void);

// Update PIT counters
void ports_tick(void);

// Keyboard controller
void kb_set_scancode(uint8_t scancode);
void kb_set_scancode_ext(uint8_t scancode, bool extended);
bool kb_has_data(void);

// Serial mouse injection (Microsoft serial mouse on COM1).
// dx,dy are relative deltas in screen coordinates: +x=right, +y=down.
// buttons bitmask: bit0=left, bit1=right, bit2=middle.
void mouse_serial_enqueue(int8_t dx, int8_t dy, uint8_t buttons);

// PS/2 mouse injection (via 8042 KBC).
// dx,dy are relative deltas.
// buttons bitmask: bit0=left, bit1=right, bit2=middle.
void mouse_ps2_enqueue(int8_t dx, int8_t dy, uint8_t buttons);

#ifdef __cplusplus
}
#endif

#endif // PORTS_H
