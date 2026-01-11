/**
 * @file interrupts.h
 * @brief Interrupt handling for 8086 emulator
 */

#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu8086.h"

#ifdef __cplusplus
extern "C" {
#endif

// IVT base address
#define IVT_BASE    0x00000

// Hardware IRQ numbers
#define IRQ_TIMER       0   // PIT Timer
#define IRQ_KEYBOARD    1   // Keyboard
#define IRQ_CASCADE     2   // Cascade from PIC2
#define IRQ_COM2        3   // COM2/COM4
#define IRQ_COM1        4   // COM1/COM3
#define IRQ_LPT2        5   // LPT2
#define IRQ_FLOPPY      6   // Floppy disk
#define IRQ_LPT1        7   // LPT1
#define IRQ_RTC         8   // Real-time clock
#define IRQ_ACPI        9   // ACPI
#define IRQ_AVAILABLE1  10  // Available
#define IRQ_AVAILABLE2  11  // Available
#define IRQ_MOUSE       12  // PS/2 Mouse
#define IRQ_FPU         13  // FPU
#define IRQ_ATA_PRI     14  // Primary ATA
#define IRQ_ATA_SEC     15  // Secondary ATA

// Software interrupt vectors
#define INT_DIVIDE_ERROR    0x00
#define INT_SINGLE_STEP     0x01
#define INT_NMI             0x02
#define INT_BREAKPOINT      0x03
#define INT_OVERFLOW        0x04
#define INT_BOUND           0x05
#define INT_INVALID_OP      0x06
#define INT_DEVICE_NA       0x07

// IRQ to interrupt vector mapping
#define IRQ_TO_INT(irq)     ((irq) < 8 ? (irq) + 0x08 : (irq) + 0x68)

// Initialize interrupt system
void interrupts_init(void);

// Set/get IVT entry
void set_ivt_entry(uint8_t vector, uint16_t segment, uint16_t offset);
void get_ivt_entry(uint8_t vector, uint16_t *segment, uint16_t *offset);

// Enable/disable interrupts
void interrupts_set_enabled(bool enabled);
bool interrupts_are_enabled(void);

// Software interrupt call
bool interrupt_call(cpu8086_t *cpu, uint8_t vector);

// Hardware interrupt handling
bool interrupt_hardware(cpu8086_t *cpu);

// Return from interrupt (IRET)
void interrupt_return(cpu8086_t *cpu);

// NMI handling
void interrupt_nmi(cpu8086_t *cpu);
void interrupt_set_nmi(bool enabled);

// Exception handlers
void interrupt_divide_error(cpu8086_t *cpu);
void interrupt_single_step(cpu8086_t *cpu);
void interrupt_breakpoint(cpu8086_t *cpu);
void interrupt_overflow(cpu8086_t *cpu);
void interrupt_bounds(cpu8086_t *cpu);
void interrupt_invalid_opcode(cpu8086_t *cpu);

#ifdef __cplusplus
}
#endif

#endif // INTERRUPTS_H
