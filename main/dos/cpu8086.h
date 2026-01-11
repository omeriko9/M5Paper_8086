/**
 * @file cpu8086.h
 * @brief 8086 CPU Emulator for M5Paper DOS
 * 
 * Lightweight 8086 emulator based on 8086tiny concepts
 */

#ifndef CPU8086_H
#define CPU8086_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// CPU Register indices
#define REG_AX 0
#define REG_CX 1
#define REG_DX 2
#define REG_BX 3
#define REG_SP 4
#define REG_BP 5
#define REG_SI 6
#define REG_DI 7

// Segment register indices
#define SEG_ES 0
#define SEG_CS 1
#define SEG_SS 2
#define SEG_DS 3

// Flag bits
#define FLAG_CF     0x0001  // Carry
#define FLAG_PF     0x0004  // Parity
#define FLAG_AF     0x0010  // Auxiliary carry
#define FLAG_ZF     0x0040  // Zero
#define FLAG_SF     0x0080  // Sign
#define FLAG_TF     0x0100  // Trap
#define FLAG_IF     0x0200  // Interrupt enable
#define FLAG_DF     0x0400  // Direction
#define FLAG_OF     0x0800  // Overflow

// Memory size (1MB addressable)
#define MEM_SIZE    (1024 * 1024)

// Video memory base addresses
#define VIDEO_MEM_TEXT  0xB8000
#define VIDEO_MEM_CGA   0xB8000
#define VIDEO_MEM_MDA   0xB0000

// CPU State structure
typedef struct {
    // General purpose registers (can be accessed as 8-bit or 16-bit)
    union {
        uint16_t regs16[8];
        uint8_t regs8[16];  // AL, AH, CL, CH, DL, DH, BL, BH mapped
    };
    
    // Segment registers
    uint16_t sregs[4];
    
    // Instruction pointer
    uint16_t ip;
    
    // Flags register
    uint16_t flags;
    
    // Interrupt flag for pending interrupts
    bool int_pending;
    uint8_t int_num;
    
    // Halt state
    bool halted;
    
    // Cycle counter for timing
    uint64_t cycles;
    
    // Current opcode being executed
    uint8_t opcode;
    
    // ModR/M byte
    uint8_t modrm;
    
    // Segment override (-1 = none)
    int8_t seg_override;
    
    // REP prefix state
    uint8_t rep_mode;  // 0=none, 1=REP/REPE, 2=REPNE
    
} cpu8086_t;

// Initialize CPU
void cpu_init(cpu8086_t *cpu);

// Reset CPU
void cpu_reset(cpu8086_t *cpu);

// Execute one instruction
int cpu_exec_instruction(cpu8086_t *cpu);

// Execute N cycles worth of instructions
int cpu_exec_cycles(cpu8086_t *cpu, int cycles);

// Trigger hardware interrupt
void cpu_interrupt(cpu8086_t *cpu, uint8_t num);

// Get effective address from ModR/M byte
uint32_t cpu_get_ea(cpu8086_t *cpu);

// Read/Write memory (with segment:offset)
uint8_t cpu_read_byte(cpu8086_t *cpu, uint16_t seg, uint16_t off);
uint16_t cpu_read_word(cpu8086_t *cpu, uint16_t seg, uint16_t off);
void cpu_write_byte(cpu8086_t *cpu, uint16_t seg, uint16_t off, uint8_t val);
void cpu_write_word(cpu8086_t *cpu, uint16_t seg, uint16_t off, uint16_t val);

// Stack operations
void cpu_push(cpu8086_t *cpu, uint16_t val);
uint16_t cpu_pop(cpu8086_t *cpu);

// Flag helpers
void cpu_set_flags_add8(cpu8086_t *cpu, uint8_t a, uint8_t b, uint32_t res);
void cpu_set_flags_add16(cpu8086_t *cpu, uint16_t a, uint16_t b, uint32_t res);
void cpu_set_flags_sub8(cpu8086_t *cpu, uint8_t a, uint8_t b, uint32_t res);
void cpu_set_flags_sub16(cpu8086_t *cpu, uint16_t a, uint16_t b, uint32_t res);
void cpu_set_flags_logic8(cpu8086_t *cpu, uint8_t res);
void cpu_set_flags_logic16(cpu8086_t *cpu, uint16_t res);

// Access register by index
static inline uint8_t cpu_get_reg8(cpu8086_t *cpu, int reg) {
    // Map: 0=AL, 1=CL, 2=DL, 3=BL, 4=AH, 5=CH, 6=DH, 7=BH
    if (reg < 4) {
        return cpu->regs16[reg] & 0xFF;
    } else {
        return (cpu->regs16[reg - 4] >> 8) & 0xFF;
    }
}

static inline void cpu_set_reg8(cpu8086_t *cpu, int reg, uint8_t val) {
    if (reg < 4) {
        cpu->regs16[reg] = (cpu->regs16[reg] & 0xFF00) | val;
    } else {
        cpu->regs16[reg - 4] = (cpu->regs16[reg - 4] & 0x00FF) | (val << 8);
    }
}

// Macro for linear address calculation
#define LINEAR_ADDR(seg, off) (((uint32_t)(seg) << 4) + (uint32_t)(off))

#ifdef __cplusplus
}
#endif

#endif // CPU8086_H
