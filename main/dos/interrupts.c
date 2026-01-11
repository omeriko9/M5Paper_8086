/**
 * @file interrupts.c
 * @brief Interrupt handling for DOS emulator
 * 
 * Manages the Interrupt Vector Table and interrupt dispatching
 */

#include <string.h>
#include "esp_log.h"
#include "interrupts.h"
#include "memory.h"
#include "bios.h"
#include "ports.h"
#include "xms.h"

static const char *TAG = "INT";
static uint32_t stub_int_log_count[256];
static uint32_t timer_fallback_log_count = 0;

// Interrupt flags
static bool interrupts_enabled = false;
static bool nmi_enabled = true;

// Disk parameter table location in low memory (after BDA at 0x500)
// This must be in segment 0 because MS-DOS boot code uses DS:SI after LES SI,[78h]
#define DISK_PARAM_TABLE_ADDR  0x0522
#define DISK_PARAM_TABLE_SIZE  11

// Disk parameter table for 1.44MB floppy (matches 8086tiny BIOS)
static const uint8_t disk_param_table[DISK_PARAM_TABLE_SIZE] = {
    0xDF,  // Step rate / head unload time
    0x02,  // Head load time / DMA mode
    0x25,  // Motor off delay (ticks)
    0x02,  // Bytes per sector (2 = 512)
    0x12,  // Sectors per track (18)
    0x1B,  // Gap length
    0xFF,  // Data length
    0x6C,  // Gap length for format
    0xF6,  // Fill byte for format
    0x0F,  // Head settle time (ms)
    0x08   // Motor start time (1/8 sec)
};

static bool ivt_entry_is_default_stub(uint8_t vector)
{
    uint16_t seg = 0;
    uint16_t off = 0;
    get_ivt_entry(vector, &seg, &off);
    // Default init points to F000:0000 (IRET stub). Treat null as unhandled too.
    return ((seg == 0xF000 && off == 0x0000) || (seg == 0x0000 && off == 0x0000));
}

static bool handle_multiplex_int(cpu8086_t *cpu)
{
    const uint16_t ax = cpu->regs16[REG_AX];
    switch (ax) {
    case 0x1687:
    case 0x1686:
        // DPMI install/version check: report "not installed".
        ESP_LOGI(TAG, "INT 2Fh AX=%04X: no DPMI host", ax);
        cpu->flags |= FLAG_CF;
        cpu->regs16[REG_AX] = 0x0000;
        cpu->regs16[REG_BX] = 0x0000;
        cpu->regs16[REG_CX] = 0x0000;
        cpu->regs16[REG_DX] = 0x0000;
        cpu->regs16[REG_SI] = 0x0000;
        cpu->regs16[REG_DI] = 0x0000;
        cpu->sregs[SEG_ES] = 0x0000;
        return true;
    case 0x4300:
        if (xms_present()) {
            ESP_LOGI(TAG, "INT 2Fh AX=4300: XMS present");
            cpu->flags &= ~FLAG_CF;
            cpu->regs16[REG_AX] = 0x0080;
        } else {
            ESP_LOGI(TAG, "INT 2Fh AX=4300: no XMS host");
            cpu->flags &= ~FLAG_CF;
            cpu->regs16[REG_AX] = 0x0000;
        }
        return true;
    case 0x4310:
        if (xms_present()) {
            uint16_t seg = 0;
            uint16_t off = 0;
            xms_get_entry(&seg, &off);
            ESP_LOGI(TAG, "INT 2Fh AX=4310: XMS entry %04X:%04X", seg, off);
            cpu->flags &= ~FLAG_CF;
            cpu->regs16[REG_AX] = 0x0000;
            cpu->regs16[REG_BX] = off;
            cpu->sregs[SEG_ES] = seg;
        } else {
            ESP_LOGI(TAG, "INT 2Fh AX=4310: no XMS host");
            cpu->flags &= ~FLAG_CF;
            cpu->regs16[REG_AX] = 0x0000;
            cpu->regs16[REG_BX] = 0x0000;
            cpu->sregs[SEG_ES] = 0x0000;
        }
        return true;
    default:
        return false;
    }
}

static void keyboard_irq_fallback(void)
{
    // Drain the controller data port so the KBC queue doesn't stall
    // when no INT 9 handler is installed.
    (void)port_in(PORT_KBC_DATA);
}

/**
 * Initialize interrupt system
 */
void interrupts_init(void)
{
    ESP_LOGI(TAG, "Initializing interrupt system");

    // Install a safe default handler (IRET) at F000:0000.
    mem_write_byte(ROM_BIOS_START, 0xCF);

    // Point all vectors at the default IRET stub. DOS/BIOS will overwrite entries as needed.
    for (int i = 0; i < 256; i++) {
        mem_write_word(IVT_BASE + i * 4, 0x0000);      // Offset
        mem_write_word(IVT_BASE + i * 4 + 2, 0xF000);  // Segment
    }

    // Copy disk parameter table to low memory (segment 0).
    // MS-DOS boot code does: LES SI,[78h] then LODSB using DS:SI (with DS=0).
    // If INT 1E points to F000:xxxx, ES gets F000 but DS stays 0, so LODSB
    // reads from 0:xxxx instead of F000:xxxx. Fix: put table in segment 0.
    mem_write_block(DISK_PARAM_TABLE_ADDR, disk_param_table, DISK_PARAM_TABLE_SIZE);
    set_ivt_entry(0x1E, 0x0000, DISK_PARAM_TABLE_ADDR);
    ESP_LOGI(TAG, "Disk param table at 0000:%04X", DISK_PARAM_TABLE_ADDR);

    interrupts_enabled = false;
}

/**
 * Set IVT entry
 */
void set_ivt_entry(uint8_t vector, uint16_t segment, uint16_t offset)
{
    uint32_t addr = IVT_BASE + (vector * 4);
    mem_write_word(addr, offset);
    mem_write_word(addr + 2, segment);
}

/**
 * Get IVT entry
 */
void get_ivt_entry(uint8_t vector, uint16_t *segment, uint16_t *offset)
{
    uint32_t addr = IVT_BASE + (vector * 4);
    *offset = mem_read_word(addr);
    *segment = mem_read_word(addr + 2);
}

/**
 * Enable/disable interrupts (CLI/STI)
 */
void interrupts_set_enabled(bool enabled)
{
    interrupts_enabled = enabled;
}

/**
 * Check if interrupts are enabled
 */
bool interrupts_are_enabled(void)
{
    return interrupts_enabled;
}

/**
 * Generate software interrupt
 */
bool interrupt_call(cpu8086_t *cpu, uint8_t vector)
{
    ESP_LOGD(TAG, "INT %02Xh AX=%04X", vector, cpu->regs16[REG_AX]);
    
    // First try BIOS handler
    if (bios_interrupt(cpu, vector)) {
        return true;
    }
    
    // Get IVT entry
    uint16_t seg, off;
    get_ivt_entry(vector, &seg, &off);

    if (vector == INT_MULTIPLEX) {
        if (handle_multiplex_int(cpu)) {
            return true;
        }
    }

    if (ivt_entry_is_default_stub(vector)) {
        uint32_t count = ++stub_int_log_count[vector];
        if (count <= 5 || (count % 256) == 0) {
            ESP_LOGW(TAG, "INT %02Xh -> default IRET stub AX=%04X BX=%04X CX=%04X DX=%04X CS:IP=%04X:%04X",
                     vector,
                     cpu->regs16[REG_AX], cpu->regs16[REG_BX],
                     cpu->regs16[REG_CX], cpu->regs16[REG_DX],
                     cpu->sregs[SEG_CS], cpu->ip);
        }
    }
    
    // Check for null vector
    if (seg == 0 && off == 0) {
        ESP_LOGW(TAG, "INT %02Xh: Null vector", vector);
        return false;
    }

    // Push flags, CS, IP
    cpu->regs16[REG_SP] -= 2;
    mem_write_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP], cpu->flags);
    
    cpu->regs16[REG_SP] -= 2;
    mem_write_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP], cpu->sregs[SEG_CS]);
    
    cpu->regs16[REG_SP] -= 2;
    mem_write_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP], cpu->ip);
    
    // Clear IF and TF
    cpu->flags &= ~(FLAG_IF | FLAG_TF);
    
    // Jump to handler
    cpu->sregs[SEG_CS] = seg;
    cpu->ip = off;
    
    return true;
}

/**
 * Handle hardware interrupt
 */
bool interrupt_hardware(cpu8086_t *cpu)
{
    if (!interrupts_enabled) {
        return false;
    }
    
    // Get pending interrupt from PIC
    int irq = port_get_interrupt();
    if (irq < 0) {
        return false;
    }
    
    ESP_LOGD(TAG, "Hardware IRQ -> INT %02Xh", irq);
    cpu->halted = false;

    if (irq == 0x08 && ivt_entry_is_default_stub(0x08)) {
        uint32_t count = ++timer_fallback_log_count;
        if (count <= 5 || (count % 256) == 0) {
            ESP_LOGW(TAG, "INT 08h fallback: tick + chain INT 1Ch");
        }
        bios_timer_tick();
        if (!ivt_entry_is_default_stub(0x1C)) {
            return interrupt_call(cpu, 0x1C);
        }
        return true;
    }

    if (irq == 0x09 && ivt_entry_is_default_stub(0x09)) {
        keyboard_irq_fallback();
        return true;
    }

    return interrupt_call(cpu, irq);
}

/**
 * Return from interrupt (IRET)
 */
void interrupt_return(cpu8086_t *cpu)
{
    // Pop IP, CS, flags
    cpu->ip = mem_read_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP]);
    cpu->regs16[REG_SP] += 2;
    
    cpu->sregs[SEG_CS] = mem_read_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP]);
    cpu->regs16[REG_SP] += 2;
    
    cpu->flags = mem_read_word((cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP]);
    cpu->regs16[REG_SP] += 2;
    
    // Update interrupt enable state from restored flags
    interrupts_enabled = (cpu->flags & FLAG_IF) != 0;
}

/**
 * Non-maskable interrupt
 */
void interrupt_nmi(cpu8086_t *cpu)
{
    if (nmi_enabled) {
        interrupt_call(cpu, 0x02);
    }
}

/**
 * Enable/disable NMI
 */
void interrupt_set_nmi(bool enabled)
{
    nmi_enabled = enabled;
}

/**
 * Exception handlers
 */
void interrupt_divide_error(cpu8086_t *cpu)
{
    ESP_LOGE(TAG, "Division by zero at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
    interrupt_call(cpu, 0x00);
}

void interrupt_single_step(cpu8086_t *cpu)
{
    if (cpu->flags & FLAG_TF) {
        interrupt_call(cpu, 0x01);
    }
}

void interrupt_breakpoint(cpu8086_t *cpu)
{
    ESP_LOGI(TAG, "Breakpoint at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
    interrupt_call(cpu, 0x03);
}

void interrupt_overflow(cpu8086_t *cpu)
{
    if (cpu->flags & FLAG_OF) {
        interrupt_call(cpu, 0x04);
    }
}

void interrupt_bounds(cpu8086_t *cpu)
{
    ESP_LOGE(TAG, "Bounds check failed at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
    interrupt_call(cpu, 0x05);
}

void interrupt_invalid_opcode(cpu8086_t *cpu)
{
    ESP_LOGE(TAG, "Invalid opcode at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
    interrupt_call(cpu, 0x06);
}
