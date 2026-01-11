/**
 * @file cpu8086.c
 * @brief 8086 CPU Emulator for M5Paper DOS
 * 
 * Based on 8086tiny by Adrian Cable, adapted for ESP32.
 * This is a simplified but functional 8086 emulator.
 * 
 * OPTIMIZED VERSION: Removed vTaskDelay from hot loops,
 * added parity lookup table, increased batch size.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "cpu8086.h"
#include "memory.h"
#include "ports.h"
#include "interrupts.h"
#include "bios.h"
#include "xms.h"
#include "embedded_8086tiny_bios.h"
#include "config_defs.h"

static const char *TAG = "CPU";
static const char *CPU_TRACE_TAG = "CPU_TRACE";
static const char *CPU_DIAG_TAG = "CPU_DIAG";
static const char *PORT_TRACE_TAG = "PORT";
static const char *DOS_TAG = "DOS";
static uint32_t int21_read_log_count;
static uint32_t int21_write_log_count;
static uint32_t iret_lowmem_log_count;
static uint32_t iret_lowmem_stack_log_count;
static uint32_t retf_lowmem_log_count;
static uint32_t iret_bad_target_log_count;
static uint32_t far_bad_target_log_count;
static uint32_t diag_prefix66_log_count;
static uint32_t diag_prefix67_log_count;
static uint32_t diag_prefix0f_log_count;
static uint32_t diag_enter_log_count;
static uint32_t diag_farptr_low_log_count;

#define INT21_RETURN_STACK_MAX 8
typedef struct {
    uint16_t cs;
    uint16_t ip;
    uint8_t ah;
    uint8_t al;
} int21_ret_addr_t;

static int21_ret_addr_t int21_ret_stack[INT21_RETURN_STACK_MAX];
static uint8_t int21_ret_depth;

#define CPU_TRACE_LOGI(...) ESP_LOGD(CPU_TRACE_TAG, __VA_ARGS__)
#define CPU_TRACE_LOGW(...) ESP_LOGW(CPU_TRACE_TAG, __VA_ARGS__)
#define CPU_DIAG_LOGI(...) ESP_LOGI(CPU_DIAG_TAG, __VA_ARGS__)
#define CPU_DIAG_LOGW(...) ESP_LOGW(CPU_DIAG_TAG, __VA_ARGS__)

static bool should_rate_log(uint32_t *counter)
{
    uint32_t count = ++(*counter);
    return (count <= 5 || (count % 256) == 0);
}

static bool is_bad_far_target(uint16_t cs, uint16_t ip)
{
    return (ip == 0x0000 && (cs == 0x0000 || cs == 0xF000));
}

static bool port_trace_interesting(uint16_t port)
{
    switch (port) {
        case 0x0061: // speaker
        case 0x0201: // gameport
        // PCI config mechanism #1 (some DOS IDE drivers probe PCI for IDE controllers)
        case 0x0CF8: // CONFIG_ADDRESS
        case 0x0CFC: // CONFIG_DATA (dword, bytes/words via +0..+3)
        case 0x0CFD:
        case 0x0CFE:
        case 0x0CFF:
        // IDE/ATAPI status ports (common CD-ROM drivers poll these in tight loops)
        case 0x01F7: // primary status/command
        case 0x03F6: // primary altstatus/devctrl
        case 0x0177: // secondary status/command
        case 0x0376: // secondary altstatus/devctrl
        case 0x01EF: // tertiary status/command
        case 0x03EE: // tertiary altstatus/devctrl
        case 0x016F: // quaternary status/command
        case 0x036E: // quaternary altstatus/devctrl
        case 0x03D8: // CGA mode control
        case 0x03D9: // CGA palette
        case 0x03DA: // CGA status
        case 0x0040: // PIT ch0
        case 0x0041: // PIT ch1
        case 0x0042: // PIT ch2
        case 0x0043: // PIT ctrl
            return true;
        default:
            return false;
    }
}

static void read_dos_string(uint16_t seg, uint16_t off, char *out, size_t out_cap)
{
    if (!out || out_cap == 0) {
        return;
    }
    const uint32_t linear = ((uint32_t)seg << 4) + off;
    size_t pos = 0;
    for (; pos + 1 < out_cap; pos++) {
        uint8_t ch = mem_read_byte(linear + (uint32_t)pos);
        if (ch == 0) {
            break;
        }
        if (ch < 0x20 || ch > 0x7E) {
            ch = '.';
        }
        out[pos] = (char)ch;
    }
    out[pos] = '\0';
}

static void log_dos_int21(cpu8086_t *cpu)
{
    const uint16_t ax = cpu->regs16[REG_AX];
    const uint8_t ah = (uint8_t)(ax >> 8);
    const uint8_t al = (uint8_t)(ax & 0xFF);
    char path[96];

    if (ah == 0x4C) {
        ESP_LOGI(DOS_TAG, "INT 21h/AH=4Ch exit_code=%02X", al);
        return;
    }
    if (ah != 0x4B) {
        switch (ah) {
            case 0x25: { // SET INTERRUPT VECTOR
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                ESP_LOGI(CPU_TRACE_TAG, "INT 21h/AH=25h int=%02X DS:DX=%04X:%04X", al, ds, dx);
                return;
            }
            case 0x35: // GET INTERRUPT VECTOR
                ESP_LOGI(CPU_TRACE_TAG, "INT 21h/AH=35h int=%02X", al);
                return;
            case 0x3B: { // CHDIR
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=3Bh DS:DX=%04X:%04X \"%s\"", ds, dx, path);
                return;
            }
            case 0x3C: { // CREATE
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=3Ch CX=%04X DS:DX=%04X:%04X \"%s\"",
                         cpu->regs16[REG_CX], ds, dx, path);
                return;
            }
            case 0x3D: { // OPEN
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=3Dh AL=%02X DS:DX=%04X:%04X \"%s\"",
                         al, ds, dx, path);
                return;
            }
            case 0x3E: // CLOSE
                ESP_LOGI(DOS_TAG, "INT 21h/AH=3Eh BX=%04X", cpu->regs16[REG_BX]);
                return;
            case 0x3F: { // READ
                uint32_t count = ++int21_read_log_count;
                if (count <= 20 || (count % 200) == 0) {
                    ESP_LOGI(DOS_TAG, "INT 21h/AH=3Fh BX=%04X CX=%04X DS:DX=%04X:%04X",
                             cpu->regs16[REG_BX], cpu->regs16[REG_CX],
                             cpu->sregs[SEG_DS], cpu->regs16[REG_DX]);
                }
                return;
            }
            case 0x40: { // WRITE
                uint32_t count = ++int21_write_log_count;
                if (count <= 20 || (count % 200) == 0) {
                    ESP_LOGI(DOS_TAG, "INT 21h/AH=40h BX=%04X CX=%04X DS:DX=%04X:%04X",
                             cpu->regs16[REG_BX], cpu->regs16[REG_CX],
                             cpu->sregs[SEG_DS], cpu->regs16[REG_DX]);
                }
                return;
            }
            case 0x41: { // DELETE
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=41h DS:DX=%04X:%04X \"%s\"", ds, dx, path);
                return;
            }
            case 0x42: { // LSEEK
                ESP_LOGI(DOS_TAG, "INT 21h/AH=42h AL=%02X BX=%04X CX:DX=%04X:%04X",
                         al, cpu->regs16[REG_BX], cpu->regs16[REG_CX], cpu->regs16[REG_DX]);
                return;
            }
            case 0x43: { // ATTRIB
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=43h AL=%02X DS:DX=%04X:%04X \"%s\"",
                         al, ds, dx, path);
                return;
            }
            case 0x48: { // ALLOCATE MEMORY
                ESP_LOGI(DOS_TAG, "INT 21h/AH=48h BX=%04X (paras)", cpu->regs16[REG_BX]);
                return;
            }
            case 0x49: { // FREE MEMORY
                ESP_LOGI(DOS_TAG, "INT 21h/AH=49h ES=%04X", cpu->sregs[SEG_ES]);
                return;
            }
            case 0x4A: { // RESIZE MEMORY
                ESP_LOGI(DOS_TAG, "INT 21h/AH=4Ah ES=%04X BX=%04X (paras)",
                         cpu->sregs[SEG_ES], cpu->regs16[REG_BX]);
                return;
            }
            case 0x4E: { // FIND FIRST
                const uint16_t ds = cpu->sregs[SEG_DS];
                const uint16_t dx = cpu->regs16[REG_DX];
                read_dos_string(ds, dx, path, sizeof(path));
                ESP_LOGI(DOS_TAG, "INT 21h/AH=4Eh CX=%04X DS:DX=%04X:%04X \"%s\"",
                         cpu->regs16[REG_CX], ds, dx, path);
                return;
            }
            case 0x4F: // FIND NEXT
                ESP_LOGI(DOS_TAG, "INT 21h/AH=4Fh");
                return;
            default:
                return;
        }
    }

    const uint16_t ds = cpu->sregs[SEG_DS];
    const uint16_t dx = cpu->regs16[REG_DX];
    read_dos_string(ds, dx, path, sizeof(path));

    ESP_LOGI(DOS_TAG, "INT 21h/AH=4Bh AL=%02X DS:DX=%04X:%04X \"%s\"",
             al, ds, dx, path);
}

static void int21_return_push(const cpu8086_t *cpu)
{
    if (int21_ret_depth >= INT21_RETURN_STACK_MAX) {
        return;
    }
    uint16_t ax = cpu->regs16[REG_AX];
    int21_ret_stack[int21_ret_depth].cs = cpu->sregs[SEG_CS];
    int21_ret_stack[int21_ret_depth].ip = cpu->ip;
    int21_ret_stack[int21_ret_depth].ah = (uint8_t)(ax >> 8);
    int21_ret_stack[int21_ret_depth].al = (uint8_t)(ax & 0xFF);
    int21_ret_depth++;
}

static void int21_return_maybe_log(cpu8086_t *cpu, uint16_t ret_cs, uint16_t ret_ip)
{
    if (int21_ret_depth == 0) {
        return;
    }
    uint8_t idx = (uint8_t)(int21_ret_depth - 1);
    int21_ret_addr_t entry = int21_ret_stack[idx];
    if (entry.cs != ret_cs || entry.ip != ret_ip) {
        return;
    }
    int21_ret_depth--;
    ESP_LOGI(DOS_TAG, "INT 21h return CF=%u AX=%04X BX=%04X CX=%04X DX=%04X",
             (cpu->flags & FLAG_CF) ? 1 : 0,
             cpu->regs16[REG_AX], cpu->regs16[REG_BX],
             cpu->regs16[REG_CX], cpu->regs16[REG_DX]);
    if (entry.ah == 0x35) {
        ESP_LOGI(CPU_TRACE_TAG, "INT 21h/AH=35h return int=%02X ES:BX=%04X:%04X",
                 entry.al, cpu->sregs[SEG_ES], cpu->regs16[REG_BX]);
    }
}

static void log_lowmem_return(const char *kind, uint32_t *counter, cpu8086_t *cpu,
                              uint16_t cs, uint16_t ip, uint16_t flags)
{
    if (cs != 0x0000 || ip >= 0x0500) {
        return;
    }
    uint32_t count = ++(*counter);
    if (count <= 5 || (count % 256) == 0) {
        ESP_LOGW(TAG, "%s -> %04X:%04X SS:SP=%04X:%04X FLAGS=%04X",
                 kind, cs, ip, cpu->sregs[SEG_SS], cpu->regs16[REG_SP], flags);
    }
}

static void log_bad_iret_target(cpu8086_t *cpu, uint16_t cur_cs, uint16_t cur_ip,
                                uint16_t ret_cs, uint16_t ret_ip, uint16_t sp_before)
{
    if (!is_bad_far_target(ret_cs, ret_ip)) {
        return;
    }
    if (!should_rate_log(&iret_bad_target_log_count)) {
        return;
    }
    uint32_t sp_addr = ((uint32_t)cpu->sregs[SEG_SS] << 4) + sp_before;
    uint16_t w0 = mem_read_word(sp_addr);
    uint16_t w1 = mem_read_word(sp_addr + 2);
    uint16_t w2 = mem_read_word(sp_addr + 4);
    uint16_t w3 = mem_read_word(sp_addr + 6);
    uint16_t w4 = mem_read_word(sp_addr + 8);
    uint16_t w5 = mem_read_word(sp_addr + 10);
    CPU_TRACE_LOGW("IRET -> %04X:%04X at %04X:%04X SS:SP=%04X:%04X stack=[%04X %04X %04X %04X %04X %04X]",
                   ret_cs, ret_ip, cur_cs, cur_ip, cpu->sregs[SEG_SS], sp_before,
                   w0, w1, w2, w3, w4, w5);
}

static void port_trace_spin(bool is_in, cpu8086_t *cpu, uint16_t ip_before, uint16_t port, uint16_t value, bool is_word)
{
    static uint16_t last_port = 0xFFFF;
    static bool last_is_in = false;
    static uint32_t repeat_count = 0;
    static uint16_t last_cs = 0;
    static uint16_t last_ip = 0;
    static uint16_t last_val = 0;
    static bool last_is_word = false;
    static int64_t last_log_us = 0;

    if (!port_trace_interesting(port) || !cpu) {
        return;
    }

    if (port == last_port && is_in == last_is_in) {
        repeat_count++;
    } else {
        last_port = port;
        last_is_in = is_in;
        repeat_count = 1;
        last_log_us = 0;
    }

    last_cs = cpu->sregs[SEG_CS];
    last_ip = ip_before;
    last_val = value;
    last_is_word = is_word;

    // Log only when it looks like a tight I/O poll loop, and rate-limit heavily.
    if (repeat_count < 20000) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (last_log_us != 0 && (now - last_log_us) < 1000000) { // <= 1Hz
        return;
    }
    last_log_us = now;

    const char *dir = last_is_in ? "IN" : "OUT";
    if (last_is_word) {
        ESP_LOGW(PORT_TRACE_TAG, "I/O spin: %s port %04X x%lu at %04X:%04X last=%04X",
                 dir, (unsigned)last_port, (unsigned long)repeat_count, last_cs, last_ip, (unsigned)last_val);
    } else {
        ESP_LOGW(PORT_TRACE_TAG, "I/O spin: %s port %04X x%lu at %04X:%04X last=%02X",
                 dir, (unsigned)last_port, (unsigned long)repeat_count, last_cs, last_ip, (unsigned)last_val & 0xFFu);
    }
}

// Parity lookup table (1 = even parity, 0 = odd parity)
static const uint8_t parity_table[256] DRAM_ATTR = {
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1
};

// BIOS lookup tables
static uint8_t bios_table_lookup[20][256];
typedef struct {
    uint16_t cs;
    uint16_t ip;
    uint8_t opcode;
    uint8_t b1;
    uint8_t b2;
    uint16_t ax;
    uint16_t bx;
    uint16_t cx;
    uint16_t dx;
    uint16_t si;
    uint16_t di;
    uint16_t bp;
    uint16_t sp;
    uint16_t flags;
    uint16_t ds;
    uint16_t es;
    uint16_t ss;
} trace_entry_t;

#define TRACE_DEPTH 16
static trace_entry_t trace_buf[TRACE_DEPTH];
static uint8_t trace_pos = 0;
static bool trace_full = false;
static uint16_t debug_bp_last = 0xFFFF;
static uint16_t debug_bp4_last = 0xFFFF;
static uint16_t debug_bp6_last = 0xFFFF;
static bool debug_boot_entry_logged = false;
static uint32_t debug_tight_loop_count = 0;
static uint16_t debug_tight_loop_cs = 0xFFFF;
static uint16_t debug_tight_loop_ip = 0xFFFF;
static int64_t debug_tight_loop_last_dump_us = 0;

// Helper macros from 8086tiny
#define REGS_BASE       0xF0000
#define SEGREG(seg, ofs, op) (16 * cpu->sregs[seg] + (uint16_t)(op cpu->regs16[ofs]))
#define GET_REG_ADDR(reg_id) (REGS_BASE + (i_w ? 2 * (reg_id) : 2 * (reg_id) + (reg_id) / 4 & 7))
#define TOP_BIT         (8 * (i_w + 1))
#define SIGN_OF(a)      (1 & (i_w ? (int16_t)(a) : (int8_t)(a)) >> (TOP_BIT - 1))

// Flag update macros
#define SET_FLAG_IF(flag, cond) do { if (cond) cpu->flags |= (flag); else cpu->flags &= ~(flag); } while(0)

// Access helper
#define MEM(addr)       mem_get_ptr(addr)

// Internal state during instruction execution
static uint8_t i_w, i_d, i_reg, i_mod, i_rm;
static uint8_t raw_opcode_id;
static int16_t i_data0, i_data1, i_data2;

/**
 * Initialize BIOS lookup tables
 */
static void init_bios_tables(cpu8086_t *cpu)
{
    uint8_t *regs8 = MEM(REGS_BASE);
    uint16_t *regs16 = (uint16_t *)regs8;
    
    // Load lookup tables from BIOS
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 256; j++) {
            bios_table_lookup[i][j] = regs8[regs16[0x81 + i] + j];
        }
    }
}

static void log_boot_bytes(void)
{
    const uint32_t addr = 0x7C00;
    CPU_TRACE_LOGI("Boot[7C00]: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                   mem_read_byte(addr + 0), mem_read_byte(addr + 1), mem_read_byte(addr + 2), mem_read_byte(addr + 3),
                   mem_read_byte(addr + 4), mem_read_byte(addr + 5), mem_read_byte(addr + 6), mem_read_byte(addr + 7),
                   mem_read_byte(addr + 8), mem_read_byte(addr + 9), mem_read_byte(addr + 10), mem_read_byte(addr + 11),
                   mem_read_byte(addr + 12), mem_read_byte(addr + 13), mem_read_byte(addr + 14), mem_read_byte(addr + 15));
}

static void log_int_opcode(cpu8086_t *cpu, uint8_t intnum, uint16_t ip_before)
{
    CPU_TRACE_LOGI("INT %02Xh opcode at %04X:%04X AX=%04X BX=%04X CX=%04X DX=%04X ES=%04X DS=%04X",
                   intnum, cpu->sregs[SEG_CS], ip_before,
                   cpu->regs16[REG_AX], cpu->regs16[REG_BX], cpu->regs16[REG_CX], cpu->regs16[REG_DX],
                   cpu->sregs[SEG_ES], cpu->sregs[SEG_DS]);
}

static void log_lowmem_exec(cpu8086_t *cpu, uint8_t opcode, uint16_t ip_before)
{
    uint32_t sp_addr = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
    uint16_t ret_ip = mem_read_word(sp_addr);
    uint16_t ret_cs = mem_read_word(sp_addr + 2);
    CPU_TRACE_LOGW("Exec low mem at %04X:%04X op=%02X SS:SP=%04X:%04X ret=%04X:%04X FLAGS=%04X",
                   cpu->sregs[SEG_CS], ip_before, opcode,
                   cpu->sregs[SEG_SS], cpu->regs16[REG_SP], ret_cs, ret_ip, cpu->flags);
}

static void trace_record(cpu8086_t *cpu, uint16_t ip_before)
{
    uint32_t addr = (cpu->sregs[SEG_CS] << 4) + ip_before;
    trace_buf[trace_pos].cs = cpu->sregs[SEG_CS];
    trace_buf[trace_pos].ip = ip_before;
    trace_buf[trace_pos].opcode = mem_read_byte(addr);
    trace_buf[trace_pos].b1 = mem_read_byte(addr + 1);
    trace_buf[trace_pos].b2 = mem_read_byte(addr + 2);
    trace_buf[trace_pos].ax = cpu->regs16[REG_AX];
    trace_buf[trace_pos].bx = cpu->regs16[REG_BX];
    trace_buf[trace_pos].cx = cpu->regs16[REG_CX];
    trace_buf[trace_pos].dx = cpu->regs16[REG_DX];
    trace_buf[trace_pos].si = cpu->regs16[REG_SI];
    trace_buf[trace_pos].di = cpu->regs16[REG_DI];
    trace_buf[trace_pos].bp = cpu->regs16[REG_BP];
    trace_buf[trace_pos].sp = cpu->regs16[REG_SP];
    trace_buf[trace_pos].flags = cpu->flags;
    trace_buf[trace_pos].ds = cpu->sregs[SEG_DS];
    trace_buf[trace_pos].es = cpu->sregs[SEG_ES];
    trace_buf[trace_pos].ss = cpu->sregs[SEG_SS];

    trace_pos = (uint8_t)((trace_pos + 1) % TRACE_DEPTH);
    if (trace_pos == 0) {
        trace_full = true;
    }
}

static void trace_dump(const char *reason)
{
    int count = trace_full ? TRACE_DEPTH : trace_pos;
    int start = trace_full ? trace_pos : 0;

    CPU_TRACE_LOGW("Trace dump (%s), last %d instructions:", reason, count);
    for (int i = 0; i < count; i++) {
        int idx = (start + i) % TRACE_DEPTH;
        CPU_TRACE_LOGW("  %02d: %04X:%04X op=%02X %02X %02X AX=%04X BX=%04X CX=%04X DX=%04X SI=%04X DI=%04X BP=%04X SP=%04X FLAGS=%04X DS=%04X ES=%04X SS=%04X",
                       i,
                       trace_buf[idx].cs,
                       trace_buf[idx].ip,
                       trace_buf[idx].opcode,
                       trace_buf[idx].b1,
                       trace_buf[idx].b2,
                       trace_buf[idx].ax,
                       trace_buf[idx].bx,
                       trace_buf[idx].cx,
                       trace_buf[idx].dx,
                       trace_buf[idx].si,
                       trace_buf[idx].di,
                       trace_buf[idx].bp,
                       trace_buf[idx].sp,
                       trace_buf[idx].flags,
                       trace_buf[idx].ds,
                       trace_buf[idx].es,
                       trace_buf[idx].ss);
    }
}

/**
 * Set flags based on result - OPTIMIZED with parity LUT
 */
static inline void IRAM_ATTR set_pzs_flags(cpu8086_t *cpu, uint16_t result, int width)
{
    // Parity from lookup table (much faster than loop)
    uint8_t pf = parity_table[result & 0xFF];
    
    if (width == 8) {
        cpu->flags = (cpu->flags & ~(FLAG_PF | FLAG_ZF | FLAG_SF)) |
                     (pf ? FLAG_PF : 0) |
                     ((result & 0xFF) == 0 ? FLAG_ZF : 0) |
                     ((result & 0x80) ? FLAG_SF : 0);
    } else {
        cpu->flags = (cpu->flags & ~(FLAG_PF | FLAG_ZF | FLAG_SF)) |
                     (pf ? FLAG_PF : 0) |
                     ((result & 0xFFFF) == 0 ? FLAG_ZF : 0) |
                     ((result & 0x8000) ? FLAG_SF : 0);
    }
}

// REP yield - only yield for very long operations to prevent WDT
// Changed from vTaskDelay(1) which was 10ms minimum, to taskYIELD() which just yields
// to other ready tasks without a mandatory delay. Also increased threshold to 64K iterations.
static inline void IRAM_ATTR rep_yield(uint32_t *iter)
{
    // Yield much less frequently - only every 64K iterations (was 16K)
    // Use taskYIELD() instead of vTaskDelay(1) to avoid 10ms penalty
    if (((*iter)++ & 0xFFFF) == 0) {
        taskYIELD();
    }
}

/**
 * Push word to stack - IRAM for speed
 */
static void IRAM_ATTR push_word(cpu8086_t *cpu, uint16_t value)
{
    uint16_t sp_before = cpu->regs16[REG_SP];
    cpu->regs16[REG_SP] -= 2;
    uint32_t addr = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
    mem_write_word(addr, value);

    if (cpu->sregs[SEG_CS] == 0x0000 &&
        cpu->ip >= 0x7C00 && cpu->ip < 0x7D00) {
        CPU_TRACE_LOGI("STACK push %04X at %04X:%04X SS:SP %04X->%04X",
                       value, cpu->sregs[SEG_CS], cpu->ip, sp_before, cpu->regs16[REG_SP]);
    }
}

/**
 * Pop word from stack - IRAM for speed
 */
static uint16_t IRAM_ATTR pop_word(cpu8086_t *cpu)
{
    uint32_t addr = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
    uint16_t value = mem_read_word(addr);
    uint16_t sp_before = cpu->regs16[REG_SP];
    cpu->regs16[REG_SP] += 2;

    if (cpu->sregs[SEG_CS] == 0x0000 &&
        cpu->ip >= 0x7C00 && cpu->ip < 0x7D00) {
        CPU_TRACE_LOGI("STACK pop %04X at %04X:%04X SS:SP %04X->%04X",
                       value, cpu->sregs[SEG_CS], cpu->ip, sp_before, cpu->regs16[REG_SP]);
    }
    return value;
}

/**
 * Calculate effective address for ModR/M byte - IRAM for speed
 */
static IRAM_ATTR uint32_t calc_ea(cpu8086_t *cpu, uint8_t modrm, int16_t disp, int seg_override)
{
    uint8_t mod = (modrm >> 6) & 3;
    uint8_t rm = modrm & 7;
    uint32_t ea = 0;
    int seg = SEG_DS;

    if (mod == 1) {
        disp = (int8_t)disp;
    } else if (mod == 0 && rm != 6) {
        // No displacement byte for mod=00 unless rm=110 (disp16).
        disp = 0;
    }

    if (mod == 3) {
        // Register mode - return register offset (8-bit mapping needs special handling).
        if (i_w) {
            return REGS_BASE + (2 * rm);
        }
        return REGS_BASE + ((2 * rm + (rm >> 2)) & 7);
    }
    
    switch (rm) {
        case 0: ea = cpu->regs16[REG_BX] + cpu->regs16[REG_SI]; break;
        case 1: ea = cpu->regs16[REG_BX] + cpu->regs16[REG_DI]; break;
        case 2: ea = cpu->regs16[REG_BP] + cpu->regs16[REG_SI]; seg = SEG_SS; break;
        case 3: ea = cpu->regs16[REG_BP] + cpu->regs16[REG_DI]; seg = SEG_SS; break;
        case 4: ea = cpu->regs16[REG_SI]; break;
        case 5: ea = cpu->regs16[REG_DI]; break;
        case 6:
            if (mod == 0) {
                ea = disp;
                disp = 0;
            } else {
                ea = cpu->regs16[REG_BP];
                seg = SEG_SS;
            }
            break;
        case 7: ea = cpu->regs16[REG_BX]; break;
    }
    
    ea = (uint16_t)(ea + disp);
    
    if (seg_override >= 0) {
        seg = seg_override;
    }
    
    return (cpu->sregs[seg] << 4) + ea;
}

static uint32_t calc_ea_seg_off(cpu8086_t *cpu, uint8_t modrm, int16_t disp, int seg_override,
                                uint16_t *out_seg, uint16_t *out_off)
{
    uint8_t mod = (modrm >> 6) & 3;
    uint8_t rm = modrm & 7;
    uint32_t ea = 0;
    int seg = SEG_DS;

    if (mod == 1) {
        disp = (int8_t)disp;
    } else if (mod == 0 && rm != 6) {
        disp = 0;
    }

    if (mod == 3) {
        if (out_seg) {
            *out_seg = 0;
        }
        if (out_off) {
            *out_off = 0;
        }
        if (i_w) {
            return REGS_BASE + (2 * rm);
        }
        return REGS_BASE + ((2 * rm + (rm >> 2)) & 7);
    }

    switch (rm) {
        case 0: ea = cpu->regs16[REG_BX] + cpu->regs16[REG_SI]; break;
        case 1: ea = cpu->regs16[REG_BX] + cpu->regs16[REG_DI]; break;
        case 2: ea = cpu->regs16[REG_BP] + cpu->regs16[REG_SI]; seg = SEG_SS; break;
        case 3: ea = cpu->regs16[REG_BP] + cpu->regs16[REG_DI]; seg = SEG_SS; break;
        case 4: ea = cpu->regs16[REG_SI]; break;
        case 5: ea = cpu->regs16[REG_DI]; break;
        case 6:
            if (mod == 0) {
                ea = disp;
                disp = 0;
            } else {
                ea = cpu->regs16[REG_BP];
                seg = SEG_SS;
            }
            break;
        case 7: ea = cpu->regs16[REG_BX]; break;
    }

    ea = (uint16_t)(ea + disp);

    if (seg_override >= 0) {
        seg = seg_override;
    }

    if (out_seg) {
        *out_seg = cpu->sregs[seg];
    }
    if (out_off) {
        *out_off = (uint16_t)ea;
    }
    return ((uint32_t)cpu->sregs[seg] << 4) + (uint16_t)ea;
}

/**
 * Initialize CPU
 */
void cpu_init(cpu8086_t *cpu)
{
    ESP_LOGI(TAG, "Initializing 8086 CPU");
    
    memset(cpu, 0, sizeof(cpu8086_t));
    
    // Initial register values (as per IBM PC)
    cpu->sregs[SEG_CS] = 0xF000;
    cpu->ip = 0x0100;
    cpu->sregs[SEG_DS] = 0x0000;
    cpu->sregs[SEG_ES] = 0x0000;
    cpu->sregs[SEG_SS] = 0x0000;
    cpu->regs16[REG_SP] = 0x0000;
    cpu->flags = 0xF002;  // Initial flags value
    cpu->seg_override = -1;
    interrupts_set_enabled(false);
    
    // Initialize BIOS tables
    init_bios_tables(cpu);
    
    ESP_LOGI(TAG, "CPU initialized: CS:IP = %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
}

/**
 * Reset CPU
 */
void cpu_reset(cpu8086_t *cpu)
{
    cpu_init(cpu);
}

/**
 * Trigger interrupt
 */
void cpu_interrupt(cpu8086_t *cpu, uint8_t intnum)
{
    if (intnum == 0x21) {
        int21_return_push(cpu);
    }
    // Push flags
    push_word(cpu, cpu->flags);
    
    // Push CS
    push_word(cpu, cpu->sregs[SEG_CS]);
    
    // Push IP
    push_word(cpu, cpu->ip);
    
    // Clear IF and TF
    cpu->flags &= ~(FLAG_IF | FLAG_TF);
    interrupts_set_enabled(false);
    
    // Load new CS:IP from IVT
    uint32_t ivt_addr = intnum * 4;
    cpu->ip = mem_read_word(ivt_addr);
    cpu->sregs[SEG_CS] = mem_read_word(ivt_addr + 2);
}

/**
 * Execute one instruction - CRITICAL PATH, place in IRAM for speed
 */
int cpu_exec_instruction(cpu8086_t *cpu)
{
    if (cpu->halted) {
        return 0;
    }
    
    // Get instruction pointer
    uint32_t ip_addr = (cpu->sregs[SEG_CS] << 4) + cpu->ip;
    uint8_t *opcode_stream = MEM(ip_addr);
    
    // Check for null pointer (shouldn't happen)
    if (!opcode_stream) {
        ESP_LOGE(TAG, "NULL opcode stream at %05lX", ip_addr);
        cpu->halted = true;
        return 0;
    }
    
    // Fetch opcode
    cpu->opcode = *opcode_stream;
    raw_opcode_id = cpu->opcode;
    
    // Handle prefixes
    cpu->seg_override = -1;
    cpu->rep_mode = 0;
    uint16_t prefix_cs = cpu->sregs[SEG_CS];
    uint16_t prefix_ip = cpu->ip;
    bool diag_prefix66 = false;
    bool diag_prefix67 = false;
    
    while (1) {
        switch (cpu->opcode) {
            case 0x26: cpu->seg_override = SEG_ES; break;
            case 0x2E: cpu->seg_override = SEG_CS; break;
            case 0x36: cpu->seg_override = SEG_SS; break;
            case 0x3E: cpu->seg_override = SEG_DS; break;
            case 0x64: break;  // FS segment override (ignored)
            case 0x65: break;  // GS segment override (ignored)
            case 0x67:
                if (should_rate_log(&diag_prefix67_log_count)) {
                    CPU_DIAG_LOGW("Invalid opcode 67 at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
                }
                cpu_interrupt(cpu, 6);
                return 1;
            case 0x66:
                if (should_rate_log(&diag_prefix66_log_count)) {
                    CPU_DIAG_LOGW("Invalid opcode 66 at %04X:%04X", cpu->sregs[SEG_CS], cpu->ip);
                }
                cpu_interrupt(cpu, 6);
                return 1;
            case 0xF0: break;  // LOCK prefix (ignored)
            case 0xF2: cpu->rep_mode = 2; break;  // REPNE
            case 0xF3: cpu->rep_mode = 1; break;  // REP/REPE
            default:
                goto prefix_done;
        }
        cpu->ip++;
        ip_addr++;
        opcode_stream++;
        cpu->opcode = *opcode_stream;
        raw_opcode_id = cpu->opcode;
    }
prefix_done:
    
    uint16_t ip_before = cpu->ip;

    if (diag_prefix66 && should_rate_log(&diag_prefix66_log_count)) {
        CPU_DIAG_LOGI("Prefix 66 at %04X:%04X -> op=%02X %02X %02X",
                      prefix_cs, prefix_ip,
                      cpu->opcode, mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2));
    }
    if (diag_prefix67 && should_rate_log(&diag_prefix67_log_count)) {
        CPU_DIAG_LOGI("Prefix 67 at %04X:%04X -> op=%02X %02X %02X",
                      prefix_cs, prefix_ip,
                      cpu->opcode, mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2));
    }

    mem_set_cpu_context(cpu->sregs[SEG_CS], ip_before, cpu->opcode,
                        mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2));

    if (xms_is_entry(cpu->sregs[SEG_CS], cpu->ip)) {
        xms_handle_call(cpu);
        cpu->cycles++;
        return 1;
    }

    trace_record(cpu, ip_before);

    // Detect tight infinite loops (e.g. "EB FE" => JMP $-0x0) and dump recent trace.
    // This helps diagnose cases where 16-bit drivers intentionally lock up during init.
    if (cpu->opcode == 0xEB) {
        const uint8_t disp = mem_read_byte(ip_addr + 1);
        if (disp == 0xFE) {
            if (cpu->sregs[SEG_CS] == debug_tight_loop_cs && ip_before == debug_tight_loop_ip) {
                debug_tight_loop_count++;
            } else {
                debug_tight_loop_cs = cpu->sregs[SEG_CS];
                debug_tight_loop_ip = ip_before;
                debug_tight_loop_count = 1;
                debug_tight_loop_last_dump_us = 0;
            }

            // Dump immediately on entry so the trace still contains the lead-up.
            if (debug_tight_loop_count == 2) {
                trace_dump("tight_loop_enter");
                CPU_TRACE_LOGW("Tight loop: %04X:%04X addr=%05lX bytes=%02X %02X %02X AX=%04X BX=%04X CX=%04X DX=%04X FLAGS=%04X DS=%04X ES=%04X SS:SP=%04X:%04X",
                               cpu->sregs[SEG_CS], ip_before,
                               (unsigned long)((cpu->sregs[SEG_CS] << 4) + ip_before),
                               cpu->opcode, mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2),
                               cpu->regs16[REG_AX], cpu->regs16[REG_BX], cpu->regs16[REG_CX], cpu->regs16[REG_DX],
                               cpu->flags, cpu->sregs[SEG_DS], cpu->sregs[SEG_ES],
                               cpu->sregs[SEG_SS], cpu->regs16[REG_SP]);
                debug_tight_loop_last_dump_us = esp_timer_get_time();
            } else if (debug_tight_loop_count > 2) {
                const int64_t now = esp_timer_get_time();
                if (debug_tight_loop_last_dump_us != 0 && (now - debug_tight_loop_last_dump_us) > 5000000) { // 0.2 Hz
                    CPU_TRACE_LOGW("Still in tight loop at %04X:%04X (count=%lu)",
                                   cpu->sregs[SEG_CS], ip_before, (unsigned long)debug_tight_loop_count);
                    debug_tight_loop_last_dump_us = now;
                }
            }
        }
    }

    if (!debug_boot_entry_logged && cpu->sregs[SEG_CS] == 0x0000 && cpu->ip == 0x7C00) {
        CPU_TRACE_LOGI("Boot entry at %04X:%04X AX=%04X BX=%04X CX=%04X DX=%04X",
                       cpu->sregs[SEG_CS], cpu->ip,
                       cpu->regs16[REG_AX], cpu->regs16[REG_BX],
                       cpu->regs16[REG_CX], cpu->regs16[REG_DX]);
        log_boot_bytes();
        debug_boot_entry_logged = true;
    }

    if (cpu->sregs[SEG_CS] == 0x0000 && cpu->ip < 0x0500) {
        trace_dump("lowmem");
        log_lowmem_exec(cpu, cpu->opcode, ip_before);
    }

    /*
    if (cpu->sregs[SEG_CS] == 0x0000 &&
        cpu->ip >= 0x7DF0 && cpu->ip <= 0x7E20 &&
        debug_fat_trace_count < 40) {
        uint32_t ds_si = (cpu->sregs[SEG_DS] << 4) + cpu->regs16[REG_SI];
        uint32_t ss_bp = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_BP];
        uint16_t fat_word = mem_read_word(ds_si);
        uint16_t bp_word4 = mem_read_word(ss_bp + 4);
        uint16_t bp_word6 = mem_read_word(ss_bp + 6);
        ESP_LOGI(TAG, "FAT trace %04X:%04X op=%02X %02X %02X AX=%04X BX=%04X CX=%04X DX=%04X SI=%04X DI=%04X BP=%04X FLAGS=%04X DS=%04X ES=%04X [DS:SI]=%04X [SS:BP+4]=%04X [SS:BP+6]=%04X",
                 cpu->sregs[SEG_CS], ip_before,
                 cpu->opcode, mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2),
                 cpu->regs16[REG_AX], cpu->regs16[REG_BX], cpu->regs16[REG_CX], cpu->regs16[REG_DX],
                 cpu->regs16[REG_SI], cpu->regs16[REG_DI], cpu->regs16[REG_BP], cpu->flags,
                 cpu->sregs[SEG_DS], cpu->sregs[SEG_ES], fat_word, bp_word4, bp_word6);
        debug_fat_trace_count++;
    }
    */

    if (cpu->sregs[SEG_CS] == 0x0000 &&
        cpu->ip >= 0x7C00 && cpu->ip <= 0x7E50) {
        uint32_t ss_bp = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_BP];
        uint16_t bp4 = mem_read_word(ss_bp + 4);
        uint16_t bp6 = mem_read_word(ss_bp + 6);
        if (cpu->regs16[REG_BP] != debug_bp_last ||
            bp4 != debug_bp4_last ||
            bp6 != debug_bp6_last) {
            CPU_TRACE_LOGI("BP vars %04X:%04X op=%02X %02X %02X BP=%04X [BP+4]=%04X [BP+6]=%04X",
                           cpu->sregs[SEG_CS], ip_before,
                           cpu->opcode, mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2),
                           cpu->regs16[REG_BP], bp4, bp6);
            debug_bp_last = cpu->regs16[REG_BP];
            debug_bp4_last = bp4;
            debug_bp6_last = bp6;
        }
    }

    if (cpu->sregs[SEG_CS] == 0xF000 && cpu->ip == 0x0000) {
        trace_dump("bios_stub");
        CPU_TRACE_LOGW("Exec BIOS IRET stub at %04X:%04X op=%02X SS:SP=%04X:%04X FLAGS=%04X",
                       cpu->sregs[SEG_CS], cpu->ip, cpu->opcode,
                       cpu->sregs[SEG_SS], cpu->regs16[REG_SP], cpu->flags);
    }

    // Extract common fields
    i_w = raw_opcode_id & 1;
    i_d = (raw_opcode_id >> 1) & 1;
    
    // Fetch additional bytes
    i_data0 = *(int16_t *)(opcode_stream + 1);
    i_data1 = *(int16_t *)(opcode_stream + 2);
    i_data2 = *(int16_t *)(opcode_stream + 3);
    
    // Instruction length
    int inst_len = 1;
    
    // Decode ModR/M if needed
    uint8_t modrm = opcode_stream[1];
    i_mod = (modrm >> 6) & 3;
    i_reg = (modrm >> 3) & 7;
    i_rm = modrm & 7;
    
    // Execute instruction
    switch (cpu->opcode) {
        // ADD r/m, r and ADD r, r/m
        case 0x00: case 0x01: case 0x02: case 0x03: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst + src;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst + src;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, res < dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst + src;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst + src;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, res < dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // ADD AL/AX, imm
        case 0x04: case 0x05: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                uint16_t dst = cpu->regs16[REG_AX];
                uint16_t src = i_data0;
                uint16_t res = dst + src;
                cpu->regs16[REG_AX] = res;
                SET_FLAG_IF(FLAG_CF, res < dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t dst = cpu->regs8[0];  // AL
                uint8_t src = i_data0 & 0xFF;
                uint8_t res = dst + src;
                cpu->regs8[0] = res;
                SET_FLAG_IF(FLAG_CF, res < dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // PUSH ES
        case 0x06:
            push_word(cpu, cpu->sregs[SEG_ES]);
            break;
            
        // POP ES
        case 0x07:
            cpu->sregs[SEG_ES] = pop_word(cpu);
            break;
            
        // OR r/m, r and OR r, r/m
        case 0x08: case 0x09: case 0x0A: case 0x0B: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst | src;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst | src;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst | src;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst | src;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // OR AL/AX, imm
        case 0x0C: case 0x0D: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                cpu->regs16[REG_AX] |= i_data0;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs16[REG_AX], 16);
            } else {
                cpu->regs8[0] |= (i_data0 & 0xFF);
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs8[0], 8);
            }
            break;
        }
            
        // PUSH CS
        case 0x0E:
            push_word(cpu, cpu->sregs[SEG_CS]);
            break;
            
        // 0x0F prefix (286+). Trap as invalid opcode to behave like 8086.
        case 0x0F:
            if (should_rate_log(&diag_prefix0f_log_count)) {
                CPU_DIAG_LOGW("Invalid opcode 0F at %04X:%04X next=%02X %02X",
                              cpu->sregs[SEG_CS], ip_before,
                              mem_read_byte(ip_addr + 1), mem_read_byte(ip_addr + 2));
            }
            cpu_interrupt(cpu, 6);
            inst_len = 0;
            break;

        // ADC r/m, r and ADC r, r/m
        case 0x10: case 0x11: case 0x12: case 0x13: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint8_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;

            if (i_w) {
                uint16_t src, dst, res;
                uint32_t sum;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    sum = (uint32_t)dst + src + carry;
                    res = (uint16_t)sum;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    sum = (uint32_t)dst + src + carry;
                    res = (uint16_t)sum;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, sum > 0xFFFF);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                uint8_t src, dst, res;
                uint16_t sum;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    sum = (uint16_t)dst + src + carry;
                    res = (uint8_t)sum;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    sum = (uint16_t)dst + src + carry;
                    res = (uint8_t)sum;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, sum > 0xFF);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // ADC AL/AX, imm
        case 0x14: case 0x15: {
            inst_len = i_w ? 3 : 2;
            uint8_t carry = (cpu->flags & FLAG_CF) ? 1 : 0;
            if (i_w) {
                uint16_t dst = cpu->regs16[REG_AX];
                uint16_t src = i_data0;
                uint32_t sum = (uint32_t)dst + src + carry;
                uint16_t res = (uint16_t)sum;
                cpu->regs16[REG_AX] = res;
                SET_FLAG_IF(FLAG_CF, sum > 0xFFFF);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t dst = cpu->regs8[0];
                uint8_t src = i_data0 & 0xFF;
                uint16_t sum = (uint16_t)dst + src + carry;
                uint8_t res = (uint8_t)sum;
                cpu->regs8[0] = res;
                SET_FLAG_IF(FLAG_CF, sum > 0xFF);
                SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // PUSH SS
        case 0x16:
            push_word(cpu, cpu->sregs[SEG_SS]);
            break;
            
        // POP SS
        case 0x17:
            cpu->sregs[SEG_SS] = pop_word(cpu);
            break;
            
        // SBB r/m, r and SBB r, r/m
        case 0x18: case 0x19: case 0x1A: case 0x1B: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint8_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;

            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst - src - borrow;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst - src - borrow;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, (uint32_t)src + borrow > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                uint8_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst - src - borrow;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst - src - borrow;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, (uint16_t)src + borrow > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // SBB AL/AX, imm
        case 0x1C: case 0x1D: {
            inst_len = i_w ? 3 : 2;
            uint8_t borrow = (cpu->flags & FLAG_CF) ? 1 : 0;
            if (i_w) {
                uint16_t dst = cpu->regs16[REG_AX];
                uint16_t src = i_data0;
                uint16_t res = dst - src - borrow;
                cpu->regs16[REG_AX] = res;
                SET_FLAG_IF(FLAG_CF, (uint32_t)src + borrow > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t dst = cpu->regs8[0];
                uint8_t src = i_data0 & 0xFF;
                uint8_t res = dst - src - borrow;
                cpu->regs8[0] = res;
                SET_FLAG_IF(FLAG_CF, (uint16_t)src + borrow > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // PUSH DS
        case 0x1E:
            push_word(cpu, cpu->sregs[SEG_DS]);
            break;
            
        // POP DS
        case 0x1F:
            cpu->sregs[SEG_DS] = pop_word(cpu);
            break;
            
        // AND r/m, r and AND r, r/m
        case 0x20: case 0x21: case 0x22: case 0x23: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst & src;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst & src;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst & src;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst & src;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // AND AL/AX, imm
        case 0x24: case 0x25: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                cpu->regs16[REG_AX] &= i_data0;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs16[REG_AX], 16);
            } else {
                cpu->regs8[0] &= (i_data0 & 0xFF);
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs8[0], 8);
            }
            break;
        }

        // DAA
        case 0x27: {
            uint8_t al = cpu->regs8[0];
            bool old_cf = (cpu->flags & FLAG_CF) != 0;
            bool adjust_low = ((al & 0x0F) > 9) || (cpu->flags & FLAG_AF);
            if (adjust_low) {
                al += 0x06;
            }
            SET_FLAG_IF(FLAG_AF, adjust_low);
            bool adjust_high = (al > 0x9F) || old_cf;
            if (adjust_high) {
                al += 0x60;
            }
            SET_FLAG_IF(FLAG_CF, adjust_high);
            cpu->regs8[0] = al;
            set_pzs_flags(cpu, al, 8);
            break;
        }
        
        // SUB r/m, r and SUB r, r/m
        case 0x28: case 0x29: case 0x2A: case 0x2B: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst - src;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst - src;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst - src;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst - src;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // SUB AL/AX, imm
        case 0x2C: case 0x2D: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                uint16_t dst = cpu->regs16[REG_AX];
                uint16_t src = i_data0;
                uint16_t res = dst - src;
                cpu->regs16[REG_AX] = res;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t dst = cpu->regs8[0];
                uint8_t src = i_data0 & 0xFF;
                uint8_t res = dst - src;
                cpu->regs8[0] = res;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // DAS
        case 0x2F: {
            uint8_t al = cpu->regs8[0];
            uint8_t old_al = al;
            bool old_cf = (cpu->flags & FLAG_CF) != 0;
            bool adjust_low = ((al & 0x0F) > 9) || (cpu->flags & FLAG_AF);
            if (adjust_low) {
                al -= 0x06;
            }
            SET_FLAG_IF(FLAG_AF, adjust_low);
            bool adjust_high = (old_al > 0x99) || old_cf;
            if (adjust_high) {
                al -= 0x60;
            }
            SET_FLAG_IF(FLAG_CF, adjust_high);
            cpu->regs8[0] = al;
            set_pzs_flags(cpu, al, 8);
            break;
        }
        
        // XOR r/m, r and XOR r, r/m
        case 0x30: case 0x31: case 0x32: case 0x33: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    res = dst ^ src;
                    cpu->regs16[i_reg] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                    res = dst ^ src;
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    res = dst ^ src;
                    cpu->regs8[ri] = res;
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                    res = dst ^ src;
                    if (i_mod == 3) cpu->regs8[rs] = res;
                    else mem_write_byte(ea, res);
                }
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // XOR AL/AX, imm
        case 0x34: case 0x35: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                cpu->regs16[REG_AX] ^= i_data0;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs16[REG_AX], 16);
            } else {
                cpu->regs8[0] ^= (i_data0 & 0xFF);
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, cpu->regs8[0], 8);
            }
            break;
        }

        // AAA
        case 0x37: {
            uint8_t al = cpu->regs8[0];
            uint8_t ah = cpu->regs8[1];
            bool adjust = ((al & 0x0F) > 9) || (cpu->flags & FLAG_AF);
            if (adjust) {
                al = (uint8_t)(al + 0x06);
                ah = (uint8_t)(ah + 1);
            }
            SET_FLAG_IF(FLAG_AF, adjust);
            SET_FLAG_IF(FLAG_CF, adjust);
            cpu->regs8[0] = al & 0x0F;
            cpu->regs8[1] = ah;
            break;
        }
        
        // CMP r/m, r and CMP r, r/m
        case 0x38: case 0x39: case 0x3A: case 0x3B: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src, dst, res;
                if (i_d) {
                    dst = cpu->regs16[i_reg];
                    src = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                } else {
                    dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    src = cpu->regs16[i_reg];
                }
                res = dst - src;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t src, dst, res;
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                if (i_d) {
                    dst = cpu->regs8[ri];
                    src = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                } else {
                    dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                    src = cpu->regs8[ri];
                }
                res = dst - src;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // CMP AL/AX, imm
        case 0x3C: case 0x3D: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                uint16_t dst = cpu->regs16[REG_AX];
                uint16_t src = i_data0;
                uint16_t res = dst - src;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t dst = cpu->regs8[0];
                uint8_t src = i_data0 & 0xFF;
                uint8_t res = dst - src;
                SET_FLAG_IF(FLAG_CF, src > dst);
                SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }

        // AAS
        case 0x3F: {
            uint8_t al = cpu->regs8[0];
            uint8_t ah = cpu->regs8[1];
            bool adjust = ((al & 0x0F) > 9) || (cpu->flags & FLAG_AF);
            if (adjust) {
                al = (uint8_t)(al - 0x06);
                ah = (uint8_t)(ah - 1);
            }
            SET_FLAG_IF(FLAG_AF, adjust);
            SET_FLAG_IF(FLAG_CF, adjust);
            cpu->regs8[0] = al & 0x0F;
            cpu->regs8[1] = ah;
            break;
        }
        
        // INC reg16
        case 0x40: case 0x41: case 0x42: case 0x43:
        case 0x44: case 0x45: case 0x46: case 0x47: {
            int reg = cpu->opcode & 7;
            uint16_t old = cpu->regs16[reg];
            cpu->regs16[reg]++;
            SET_FLAG_IF(FLAG_OF, old == 0x7FFF);
            set_pzs_flags(cpu, cpu->regs16[reg], 16);
            break;
        }
        
        // DEC reg16
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F: {
            int reg = cpu->opcode & 7;
            uint16_t old = cpu->regs16[reg];
            cpu->regs16[reg]--;
            SET_FLAG_IF(FLAG_OF, old == 0x8000);
            set_pzs_flags(cpu, cpu->regs16[reg], 16);
            break;
        }
        
        // PUSH reg16
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57: {
            int reg = cpu->opcode & 7;
            uint16_t val = cpu->regs16[reg];
            if (reg == REG_SP) {
                // 8086 quirk: PUSH SP pushes SP after decrement.
                val = (uint16_t)(val - 2);
            }
            push_word(cpu, val);
            break;
        }
            
        // POP reg16
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F:
            cpu->regs16[cpu->opcode & 7] = pop_word(cpu);
            break;

        // PUSHA/POPA
        case 0x60: {
            uint16_t sp = cpu->regs16[REG_SP];
            push_word(cpu, cpu->regs16[REG_AX]);
            push_word(cpu, cpu->regs16[REG_CX]);
            push_word(cpu, cpu->regs16[REG_DX]);
            push_word(cpu, cpu->regs16[REG_BX]);
            push_word(cpu, sp);
            push_word(cpu, cpu->regs16[REG_BP]);
            push_word(cpu, cpu->regs16[REG_SI]);
            push_word(cpu, cpu->regs16[REG_DI]);
            break;
        }
        case 0x61:
            cpu->regs16[REG_DI] = pop_word(cpu);
            cpu->regs16[REG_SI] = pop_word(cpu);
            cpu->regs16[REG_BP] = pop_word(cpu);
            pop_word(cpu);  // discard SP
            cpu->regs16[REG_BX] = pop_word(cpu);
            cpu->regs16[REG_DX] = pop_word(cpu);
            cpu->regs16[REG_CX] = pop_word(cpu);
            cpu->regs16[REG_AX] = pop_word(cpu);
            break;

        // PUSH imm16 / PUSH imm8
        case 0x68:
            inst_len = 3;
            push_word(cpu, (uint16_t)i_data0);
            break;
        case 0x6A:
            inst_len = 2;
            push_word(cpu, (uint16_t)(int16_t)(int8_t)(i_data0 & 0xFF));
            break;

        // IMUL r16, r/m16, imm
        case 0x69: case 0x6B: {
            inst_len = (cpu->opcode == 0x69) ? 4 : 3;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int16_t src = (i_mod == 3) ? (int16_t)cpu->regs16[i_rm] : (int16_t)mem_read_word(ea);
            int16_t imm = (cpu->opcode == 0x69)
                ? *(int16_t *)(opcode_stream + (inst_len - 2))
                : (int16_t)(int8_t)(opcode_stream[inst_len - 1]);
            int32_t res = (int32_t)src * imm;
            cpu->regs16[i_reg] = (uint16_t)res;
            SET_FLAG_IF(FLAG_CF, res != (int16_t)res);
            SET_FLAG_IF(FLAG_OF, res != (int16_t)res);
            break;
        }

        // INSB/INSW
        case 0x6C: case 0x6D: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint16_t port = cpu->regs16[REG_DX];
                uint32_t dst_addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_DI];
                if (i_w) {
                    mem_write_word(dst_addr, port_in_word(port));
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    mem_write_byte(dst_addr, port_in(port));
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                }
            }
            break;
        }

        // OUTSB/OUTSW
        case 0x6E: case 0x6F: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint16_t port = cpu->regs16[REG_DX];
                uint32_t src_addr = (cpu->sregs[cpu->seg_override >= 0 ? cpu->seg_override : SEG_DS] << 4) + cpu->regs16[REG_SI];
                if (i_w) {
                    port_out_word(port, mem_read_word(src_addr));
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    port_out(port, mem_read_byte(src_addr));
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                }
            }
            break;
        }

        // BOUND
        case 0x62: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            if (i_mod != 3) {
                uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
                int16_t low = (int16_t)mem_read_word(ea);
                int16_t high = (int16_t)mem_read_word(ea + 2);
                int16_t val = (int16_t)cpu->regs16[i_reg];
                if (val < low || val > high) {
                    cpu_interrupt(cpu, 5);
                    inst_len = 0;
                }
            }
            break;
        }

        // ARPL
        case 0x63: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint16_t src = cpu->regs16[i_reg];
            uint16_t dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(calc_ea(cpu, modrm, i_data1, cpu->seg_override));
            uint16_t new_dst = (dst & ~0x0003) | (src & 0x0003);

            SET_FLAG_IF(FLAG_ZF, new_dst != dst);

            if (i_mod == 3) {
                cpu->regs16[i_rm] = new_dst;
            } else {
                uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
                mem_write_word(ea, new_dst);
            }
            break;
        }
        
        // Conditional jumps
        case 0x70: // JO
            inst_len = 2;
            if (cpu->flags & FLAG_OF) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x71: // JNO
            inst_len = 2;
            if (!(cpu->flags & FLAG_OF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x72: // JB/JC/JNAE
            inst_len = 2;
            if (cpu->flags & FLAG_CF) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x73: // JNB/JNC/JAE
            inst_len = 2;
            if (!(cpu->flags & FLAG_CF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x74: // JE/JZ
            inst_len = 2;
            if (cpu->flags & FLAG_ZF) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x75: // JNE/JNZ
            inst_len = 2;
            if (!(cpu->flags & FLAG_ZF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x76: // JBE/JNA
            inst_len = 2;
            if ((cpu->flags & FLAG_CF) || (cpu->flags & FLAG_ZF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x77: // JNBE/JA
            inst_len = 2;
            if (!(cpu->flags & FLAG_CF) && !(cpu->flags & FLAG_ZF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x78: // JS
            inst_len = 2;
            if (cpu->flags & FLAG_SF) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x79: // JNS
            inst_len = 2;
            if (!(cpu->flags & FLAG_SF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7A: // JP/JPE
            inst_len = 2;
            if (cpu->flags & FLAG_PF) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7B: // JNP/JPO
            inst_len = 2;
            if (!(cpu->flags & FLAG_PF)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7C: // JL/JNGE
            inst_len = 2;
            if (((cpu->flags & FLAG_SF) != 0) != ((cpu->flags & FLAG_OF) != 0)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7D: // JNL/JGE
            inst_len = 2;
            if (((cpu->flags & FLAG_SF) != 0) == ((cpu->flags & FLAG_OF) != 0)) cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7E: // JLE/JNG
            inst_len = 2;
            if ((cpu->flags & FLAG_ZF) || (((cpu->flags & FLAG_SF) != 0) != ((cpu->flags & FLAG_OF) != 0)))
                cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        case 0x7F: // JNLE/JG
            inst_len = 2;
            if (!(cpu->flags & FLAG_ZF) && (((cpu->flags & FLAG_SF) != 0) == ((cpu->flags & FLAG_OF) != 0)))
                cpu->ip += (int8_t)(i_data0 & 0xFF);
            break;
        
        // Immediate Group: ADD/OR/ADC/SBB/AND/SUB/XOR/CMP r/m8, imm8
        case 0x80: case 0x82: {
            inst_len = 3;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint8_t dst = (i_mod == 3) ? cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] : mem_read_byte(ea);
            uint8_t src = (opcode_stream[inst_len - 1]) & 0xFF;
            uint8_t res;
            
            switch (i_reg) {
                case 0: // ADD
                    res = dst + src;
                    SET_FLAG_IF(FLAG_CF, res < dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 1: // OR
                    res = dst | src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 2: // ADC
                    res = dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint16_t)dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0) > 0xFF);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x80);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 3: // SBB
                    res = dst - src - ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint16_t)src + ((cpu->flags & FLAG_CF) ? 1 : 0) > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 4: // AND
                    res = dst & src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 5: // SUB
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 6: // XOR
                    res = dst ^ src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs8[i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1] = res;
                    else mem_write_byte(ea, res);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 7: // CMP
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x80);
                    set_pzs_flags(cpu, res, 8);
                    break;
            }
            break;
        }
        
        // Immediate Group: ADD/OR/ADC/SBB/AND/SUB/XOR/CMP r/m16, imm16
        case 0x81: {
            inst_len = 4;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint16_t dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
            uint16_t src = *(uint16_t *)(opcode_stream + (inst_len - 2));
            uint16_t res;
            
            switch (i_reg) {
                case 0: // ADD
                    res = dst + src;
                    SET_FLAG_IF(FLAG_CF, res < dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 1: // OR
                    res = dst | src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 2: // ADC
                    res = dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint32_t)dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0) > 0xFFFF);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 3: // SBB
                    res = dst - src - ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint32_t)src + ((cpu->flags & FLAG_CF) ? 1 : 0) > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 4: // AND
                    res = dst & src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 5: // SUB
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 6: // XOR
                    res = dst ^ src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 7: // CMP
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    break;
            }
            break;
        }
        
        // Immediate Group: ADD/OR/ADC/SBB/AND/SUB/XOR/CMP r/m16, sign-extended imm8
        case 0x83: {
            inst_len = 3;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint16_t dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
            uint16_t src = (int16_t)(int8_t)(opcode_stream[inst_len - 1]);  // Sign-extend
            uint16_t res;
            
            switch (i_reg) {
                case 0: // ADD
                    res = dst + src;
                    SET_FLAG_IF(FLAG_CF, res < dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 1: // OR
                    res = dst | src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 2: // ADC
                    res = dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint32_t)dst + src + ((cpu->flags & FLAG_CF) ? 1 : 0) > 0xFFFF);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ res) & (src ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 3: // SBB
                    res = dst - src - ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, (uint32_t)src + ((cpu->flags & FLAG_CF) ? 1 : 0) > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 4: // AND
                    res = dst & src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 5: // SUB
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 6: // XOR
                    res = dst ^ src;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 7: // CMP
                    res = dst - src;
                    SET_FLAG_IF(FLAG_CF, src > dst);
                    SET_FLAG_IF(FLAG_OF, ((dst ^ src) & (dst ^ res)) & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    break;
            }
            break;
        }
        
        // TEST r/m, r (0x84, 0x85)
        case 0x84: case 0x85: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t src = cpu->regs16[i_reg];
                uint16_t dst = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                uint16_t res = dst & src;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 16);
            } else {
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                uint8_t src = cpu->regs8[ri];
                uint8_t dst = (i_mod == 3) ? cpu->regs8[rs] : mem_read_byte(ea);
                uint8_t res = dst & src;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // XCHG r/m, r (0x86, 0x87)
        case 0x86: case 0x87: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                uint16_t tmp = cpu->regs16[i_reg];
                if (i_mod == 3) {
                    cpu->regs16[i_reg] = cpu->regs16[i_rm];
                    cpu->regs16[i_rm] = tmp;
                } else {
                    cpu->regs16[i_reg] = mem_read_word(ea);
                    mem_write_word(ea, tmp);
                }
            } else {
                int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                uint8_t tmp = cpu->regs8[ri];
                if (i_mod == 3) {
                    cpu->regs8[ri] = cpu->regs8[rs];
                    cpu->regs8[rs] = tmp;
                } else {
                    cpu->regs8[ri] = mem_read_byte(ea);
                    mem_write_byte(ea, tmp);
                }
            }
            break;
        }
            
        // MOV r/m, r or MOV r, r/m
        case 0x88: case 0x89: case 0x8A: case 0x8B: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            if (i_w) {
                if (i_d) {
                    cpu->regs16[i_reg] = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                } else {
                    if (i_mod == 3) cpu->regs16[i_rm] = cpu->regs16[i_reg];
                    else mem_write_word(ea, cpu->regs16[i_reg]);
                }
            } else {
                if (i_d) {
                    int ri = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                    if (i_mod == 3) {
                        int rs = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                        cpu->regs8[ri] = cpu->regs8[rs];
                    } else {
                        cpu->regs8[ri] = mem_read_byte(ea);
                    }
                } else {
                    int rs = i_reg < 4 ? i_reg * 2 : (i_reg - 4) * 2 + 1;
                    if (i_mod == 3) {
                        int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                        cpu->regs8[ri] = cpu->regs8[rs];
                    } else {
                        mem_write_byte(ea, cpu->regs8[rs]);
                    }
                }
            }
            break;
        }
        
        // MOV seg, r/m16
        case 0x8E: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int sreg = (modrm >> 3) & 3;
            cpu->sregs[sreg] = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
            break;
        }

        // POP r/m16
        case 0x8F: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            if (i_reg == 0) {
                uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
                uint16_t val = pop_word(cpu);
                if (i_mod == 3) cpu->regs16[i_rm] = val;
                else mem_write_word(ea, val);
            }
            break;
        }
        
        // MOV r/m16, seg
        case 0x8C: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int sreg = (modrm >> 3) & 3;
            if (i_mod == 3) cpu->regs16[i_rm] = cpu->sregs[sreg];
            else mem_write_word(ea, cpu->sregs[sreg]);
            break;
        }
        
        // LEA r16, m
        case 0x8D: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            // Calculate effective address without segment
            uint8_t mod = (modrm >> 6) & 3;
            uint8_t rm = modrm & 7;
            uint16_t base = 0;
            int32_t disp = 0;
            
            switch (rm) {
                case 0: base = cpu->regs16[REG_BX] + cpu->regs16[REG_SI]; break;
                case 1: base = cpu->regs16[REG_BX] + cpu->regs16[REG_DI]; break;
                case 2: base = cpu->regs16[REG_BP] + cpu->regs16[REG_SI]; break;
                case 3: base = cpu->regs16[REG_BP] + cpu->regs16[REG_DI]; break;
                case 4: base = cpu->regs16[REG_SI]; break;
                case 5: base = cpu->regs16[REG_DI]; break;
                case 6: base = (mod == 0) ? 0 : cpu->regs16[REG_BP]; break;
                case 7: base = cpu->regs16[REG_BX]; break;
            }
            
            if (mod == 1) {
                disp = (int8_t)(i_data1 & 0xFF);
            } else if (mod == 2) {
                disp = (int16_t)i_data1;
            } else if (mod == 0 && rm == 6) {
                disp = (uint16_t)i_data1;
            }
            
            cpu->regs16[i_reg] = (uint16_t)(base + disp);
            break;
        }
        
        // NOP (XCHG AX, AX)
        case 0x90:
            break;
            
        // XCHG AX, reg
        case 0x91: case 0x92: case 0x93:
        case 0x94: case 0x95: case 0x96: case 0x97: {
            int reg = cpu->opcode & 7;
            uint16_t tmp = cpu->regs16[REG_AX];
            cpu->regs16[REG_AX] = cpu->regs16[reg];
            cpu->regs16[reg] = tmp;
            break;
        }
        
        // CBW
        case 0x98:
            cpu->regs16[REG_AX] = (int16_t)(int8_t)cpu->regs8[0];
            break;
            
        // CWD
        case 0x99:
            cpu->regs16[REG_DX] = (cpu->regs16[REG_AX] & 0x8000) ? 0xFFFF : 0x0000;
            break;
            
        // CALL far ptr
        case 0x9A:
            if (is_bad_far_target(i_data2, i_data0) && should_rate_log(&far_bad_target_log_count)) {
                CPU_TRACE_LOGW("CALL FAR imm -> %04X:%04X from %04X:%04X",
                               i_data2, i_data0, cpu->sregs[SEG_CS], ip_before);
            }
            push_word(cpu, cpu->sregs[SEG_CS]);
            push_word(cpu, cpu->ip + 5);
            cpu->ip = i_data0;
            cpu->sregs[SEG_CS] = i_data2;
            inst_len = 0;
            break;
            
        // PUSHF
        case 0x9C:
            push_word(cpu, cpu->flags | 0xF002);
            break;
            
        // POPF
        case 0x9D:
            cpu->flags = pop_word(cpu) | 0xF002;
            interrupts_set_enabled((cpu->flags & FLAG_IF) != 0);
            break;

        // SAHF
        case 0x9E: {
            uint8_t ah = cpu->regs8[1];
            cpu->flags &= ~(FLAG_SF | FLAG_ZF | FLAG_AF | FLAG_PF | FLAG_CF);
            if (ah & 0x80) cpu->flags |= FLAG_SF;
            if (ah & 0x40) cpu->flags |= FLAG_ZF;
            if (ah & 0x10) cpu->flags |= FLAG_AF;
            if (ah & 0x04) cpu->flags |= FLAG_PF;
            if (ah & 0x01) cpu->flags |= FLAG_CF;
            break;
        }

        // LAHF
        case 0x9F: {
            uint8_t ah = 0x02;
            if (cpu->flags & FLAG_SF) ah |= 0x80;
            if (cpu->flags & FLAG_ZF) ah |= 0x40;
            if (cpu->flags & FLAG_AF) ah |= 0x10;
            if (cpu->flags & FLAG_PF) ah |= 0x04;
            if (cpu->flags & FLAG_CF) ah |= 0x01;
            cpu->regs8[1] = ah;
            break;
        }

        // WAIT
        case 0x9B:
            break;
            
        // MOV AL/AX, [addr]
        case 0xA0: case 0xA1: {
            inst_len = 3;
            uint32_t addr = ((cpu->seg_override >= 0 ? cpu->sregs[cpu->seg_override] : cpu->sregs[SEG_DS]) << 4) + (uint16_t)i_data0;
            if (i_w) {
                cpu->regs16[REG_AX] = mem_read_word(addr);
            } else {
                cpu->regs8[0] = mem_read_byte(addr);
            }
            break;
        }
        
        // MOV [addr], AL/AX
        case 0xA2: case 0xA3: {
            inst_len = 3;
            uint32_t addr = ((cpu->seg_override >= 0 ? cpu->sregs[cpu->seg_override] : cpu->sregs[SEG_DS]) << 4) + (uint16_t)i_data0;
            if (i_w) {
                mem_write_word(addr, cpu->regs16[REG_AX]);
            } else {
                mem_write_byte(addr, cpu->regs8[0]);
            }
            break;
        }
        
        // MOVSB/MOVSW
        case 0xA4: case 0xA5: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint32_t src_addr = (cpu->sregs[cpu->seg_override >= 0 ? cpu->seg_override : SEG_DS] << 4) + cpu->regs16[REG_SI];
                uint32_t dst_addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_DI];
                
                if (i_w) {
                    mem_write_word(dst_addr, mem_read_word(src_addr));
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    mem_write_byte(dst_addr, mem_read_byte(src_addr));
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                }
            }
            break;
        }
        
        // STOSB/STOSW
        case 0xAA: case 0xAB: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint32_t dst_addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_DI];
                
                if (i_w) {
                    mem_write_word(dst_addr, cpu->regs16[REG_AX]);
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    mem_write_byte(dst_addr, cpu->regs8[0]);
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                }
            }
            break;
        }
        
        // LODSB/LODSW
        case 0xAC: case 0xAD: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint32_t src_addr = (cpu->sregs[cpu->seg_override >= 0 ? cpu->seg_override : SEG_DS] << 4) + cpu->regs16[REG_SI];
                if (i_w) {
                    cpu->regs16[REG_AX] = mem_read_word(src_addr);
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    cpu->regs8[0] = mem_read_byte(src_addr);
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                }
            }
            break;
        }
        
        // CMPSB/CMPSW
        case 0xA6: case 0xA7: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint32_t src_addr = (cpu->sregs[cpu->seg_override >= 0 ? cpu->seg_override : SEG_DS] << 4) + cpu->regs16[REG_SI];
                uint32_t dst_addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_DI];
                
                if (i_w) {
                    uint16_t src = mem_read_word(src_addr);
                    uint16_t dst = mem_read_word(dst_addr);
                    uint16_t res = src - dst;
                    SET_FLAG_IF(FLAG_CF, dst > src);
                    SET_FLAG_IF(FLAG_OF, ((src ^ dst) & (src ^ res)) & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    uint8_t src = mem_read_byte(src_addr);
                    uint8_t dst = mem_read_byte(dst_addr);
                    uint8_t res = src - dst;
                    SET_FLAG_IF(FLAG_CF, dst > src);
                    SET_FLAG_IF(FLAG_OF, ((src ^ dst) & (src ^ res)) & 0x80);
                    set_pzs_flags(cpu, res, 8);
                    cpu->regs16[REG_SI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                    // REPE/REPNE logic
                    if (cpu->rep_mode == 1 && !(cpu->flags & FLAG_ZF)) break;  // REPE - stop if not equal
                    if (cpu->rep_mode == 2 && (cpu->flags & FLAG_ZF)) break;   // REPNE - stop if equal
                }
            }
            break;
        }
        
        // TEST AL, imm8 / TEST AX, imm16
        case 0xA8: case 0xA9: {
            inst_len = i_w ? 3 : 2;
            if (i_w) {
                uint16_t res = cpu->regs16[REG_AX] & i_data0;
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 16);
            } else {
                uint8_t res = cpu->regs8[0] & (i_data0 & 0xFF);
                cpu->flags &= ~(FLAG_CF | FLAG_OF);
                set_pzs_flags(cpu, res, 8);
            }
            break;
        }
        
        // SCASB/SCASW
        case 0xAE: case 0xAF: {
            uint32_t rep_iter = 0;
            uint16_t count = cpu->rep_mode ? cpu->regs16[REG_CX] : 1;
            while (count--) {
                uint32_t dst_addr = (cpu->sregs[SEG_ES] << 4) + cpu->regs16[REG_DI];
                
                if (i_w) {
                    uint16_t src = cpu->regs16[REG_AX];
                    uint16_t dst = mem_read_word(dst_addr);
                    uint16_t res = src - dst;
                    SET_FLAG_IF(FLAG_CF, dst > src);
                    SET_FLAG_IF(FLAG_OF, ((src ^ dst) & (src ^ res)) & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -2 : 2;
                } else {
                    uint8_t src = cpu->regs8[0];
                    uint8_t dst = mem_read_byte(dst_addr);
                    uint8_t res = src - dst;
                    SET_FLAG_IF(FLAG_CF, dst > src);
                    SET_FLAG_IF(FLAG_OF, ((src ^ dst) & (src ^ res)) & 0x80);
                    set_pzs_flags(cpu, res, 8);
                    cpu->regs16[REG_DI] += (cpu->flags & FLAG_DF) ? -1 : 1;
                }
                
                if (cpu->rep_mode && cpu->regs16[REG_CX] > 0) {
                    cpu->regs16[REG_CX]--;
                    rep_yield(&rep_iter);
                    if (cpu->rep_mode == 1 && !(cpu->flags & FLAG_ZF)) break;
                    if (cpu->rep_mode == 2 && (cpu->flags & FLAG_ZF)) break;
                }
            }
            break;
        }
        
        // MOV reg8, imm8
        case 0xB0: case 0xB1: case 0xB2: case 0xB3:
        case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            inst_len = 2;
            int reg = cpu->opcode & 7;
            int ri = reg < 4 ? reg * 2 : (reg - 4) * 2 + 1;
            cpu->regs8[ri] = i_data0 & 0xFF;
            break;
        }
        
        // MOV reg16, imm16
        case 0xB8: case 0xB9: case 0xBA: case 0xBB:
        case 0xBC: case 0xBD: case 0xBE: case 0xBF:
            inst_len = 3;
            cpu->regs16[cpu->opcode & 7] = i_data0;
            break;

        // Shift/Rotate r/m8, imm8
        case 0xC0: {
            inst_len = 3;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint8_t count = opcode_stream[inst_len - 1] & 0x1F;
            if (count == 0) break;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
            uint8_t res = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);

            for (uint8_t i = 0; i < count; i++) {
                uint8_t val = res;
                switch (i_reg) {
                    case 0: // ROL
                        res = (val << 1) | (val >> 7);
                        SET_FLAG_IF(FLAG_CF, val & 0x80);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                        break;
                    case 1: // ROR
                        res = (val >> 1) | (val << 7);
                        SET_FLAG_IF(FLAG_CF, val & 0x01);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                        break;
                    case 2: // RCL
                        res = (val << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0);
                        SET_FLAG_IF(FLAG_CF, val & 0x80);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                        break;
                    case 3: // RCR
                        res = (val >> 1) | ((cpu->flags & FLAG_CF) ? 0x80 : 0);
                        SET_FLAG_IF(FLAG_CF, val & 0x01);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                        break;
                    case 4: // SHL/SAL
                        res = val << 1;
                        SET_FLAG_IF(FLAG_CF, val & 0x80);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                        set_pzs_flags(cpu, res, 8);
                        break;
                    case 5: // SHR
                        res = val >> 1;
                        SET_FLAG_IF(FLAG_CF, val & 0x01);
                        SET_FLAG_IF(FLAG_OF, val & 0x80);
                        set_pzs_flags(cpu, res, 8);
                        break;
                    case 7: // SAR
                        res = (val >> 1) | (val & 0x80);
                        SET_FLAG_IF(FLAG_CF, val & 0x01);
                        cpu->flags &= ~FLAG_OF;
                        set_pzs_flags(cpu, res, 8);
                        break;
                    default:
                        break;
                }
            }

            if (i_mod == 3) cpu->regs8[ri] = res;
            else mem_write_byte(ea, res);
            break;
        }

        // Shift/Rotate r/m16, imm8
        case 0xC1: {
            inst_len = 3;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint8_t count = opcode_stream[inst_len - 1] & 0x1F;
            if (count == 0) break;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint16_t res = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);

            for (uint8_t i = 0; i < count; i++) {
                uint16_t val = res;
                switch (i_reg) {
                    case 0: // ROL
                        res = (uint16_t)((val << 1) | (val >> 15));
                        SET_FLAG_IF(FLAG_CF, val & 0x8000);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                        break;
                    case 1: // ROR
                        res = (uint16_t)((val >> 1) | (val << 15));
                        SET_FLAG_IF(FLAG_CF, val & 0x0001);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                        break;
                    case 2: // RCL
                        res = (uint16_t)((val << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0));
                        SET_FLAG_IF(FLAG_CF, val & 0x8000);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                        break;
                    case 3: // RCR
                        res = (uint16_t)((val >> 1) | ((cpu->flags & FLAG_CF) ? 0x8000 : 0));
                        SET_FLAG_IF(FLAG_CF, val & 0x0001);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                        break;
                    case 4: // SHL/SAL
                        res = (uint16_t)(val << 1);
                        SET_FLAG_IF(FLAG_CF, val & 0x8000);
                        SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                        set_pzs_flags(cpu, res, 16);
                        break;
                    case 5: // SHR
                        res = (uint16_t)(val >> 1);
                        SET_FLAG_IF(FLAG_CF, val & 0x0001);
                        SET_FLAG_IF(FLAG_OF, val & 0x8000);
                        set_pzs_flags(cpu, res, 16);
                        break;
                    case 7: // SAR
                        res = (uint16_t)((val >> 1) | (val & 0x8000));
                        SET_FLAG_IF(FLAG_CF, val & 0x0001);
                        cpu->flags &= ~FLAG_OF;
                        set_pzs_flags(cpu, res, 16);
                        break;
                    default:
                        break;
                }
            }

            if (i_mod == 3) cpu->regs16[i_rm] = res;
            else mem_write_word(ea, res);
            break;
        }

        // LES/LDS
        case 0xC4: case 0xC5: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            if (i_mod != 3) {
                uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
                cpu->regs16[i_reg] = mem_read_word(ea);
                if (cpu->opcode == 0xC4) cpu->sregs[SEG_ES] = mem_read_word(ea + 2);
                else cpu->sregs[SEG_DS] = mem_read_word(ea + 2);
            }
            break;
        }

        // MOV r/m, imm
        case 0xC6: case 0xC7: {
            inst_len = (cpu->opcode == 0xC6) ? 3 : 4;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;

            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            if (cpu->opcode == 0xC6) {
                int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
                uint8_t imm = opcode_stream[inst_len - 1];
                if (i_mod == 3) cpu->regs8[ri] = imm;
                else mem_write_byte(ea, imm);
            } else {
                uint16_t imm = *(uint16_t *)(opcode_stream + (inst_len - 2));
                if (i_mod == 3) cpu->regs16[i_rm] = imm;
                else mem_write_word(ea, imm);
            }
            break;
        }

        // ENTER/LEAVE
        case 0xC8: {
            inst_len = 4;
            uint16_t frame = *(uint16_t *)(opcode_stream + 1);
            uint8_t nesting = opcode_stream[3];
            uint16_t sp_before = cpu->regs16[REG_SP];
            uint16_t bp_before = cpu->regs16[REG_BP];
            push_word(cpu, cpu->regs16[REG_BP]);
            cpu->regs16[REG_BP] = cpu->regs16[REG_SP];
            cpu->regs16[REG_SP] -= frame;
            if (should_rate_log(&diag_enter_log_count)) {
                CPU_DIAG_LOGW("ENTER frame=%04X nest=%u at %04X:%04X BP=%04X->%04X SP=%04X->%04X",
                              frame, (unsigned)nesting, cpu->sregs[SEG_CS], ip_before,
                              bp_before, cpu->regs16[REG_BP], sp_before, cpu->regs16[REG_SP]);
            }
            break;
        }
        case 0xC9:
            cpu->regs16[REG_SP] = cpu->regs16[REG_BP];
            cpu->regs16[REG_BP] = pop_word(cpu);
            break;
        
        // Shift/Rotate r/m8, 1
        case 0xD0: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
            uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
            uint8_t res;
            
            switch (i_reg) {
                case 0: // ROL
                    res = (val << 1) | (val >> 7);
                    SET_FLAG_IF(FLAG_CF, val & 0x80);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                    break;
                case 1: // ROR
                    res = (val >> 1) | (val << 7);
                    SET_FLAG_IF(FLAG_CF, val & 0x01);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                    break;
                case 2: // RCL
                    res = (val << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, val & 0x80);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                    break;
                case 3: // RCR
                    res = (val >> 1) | ((cpu->flags & FLAG_CF) ? 0x80 : 0);
                    SET_FLAG_IF(FLAG_CF, val & 0x01);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                    break;
                case 4: // SHL/SAL
                    res = val << 1;
                    SET_FLAG_IF(FLAG_CF, val & 0x80);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x80);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 5: // SHR
                    res = val >> 1;
                    SET_FLAG_IF(FLAG_CF, val & 0x01);
                    SET_FLAG_IF(FLAG_OF, val & 0x80);
                    set_pzs_flags(cpu, res, 8);
                    break;
                case 7: // SAR
                    res = (val >> 1) | (val & 0x80);
                    SET_FLAG_IF(FLAG_CF, val & 0x01);
                    cpu->flags &= ~FLAG_OF;
                    set_pzs_flags(cpu, res, 8);
                    break;
                default:
                    res = val;
                    break;
            }
            if (i_mod == 3) cpu->regs8[ri] = res;
            else mem_write_byte(ea, res);
            break;
        }
        
        // Shift/Rotate r/m16, 1
        case 0xD1: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
            uint16_t res;
            
            switch (i_reg) {
                case 0: // ROL
                    res = (val << 1) | (val >> 15);
                    SET_FLAG_IF(FLAG_CF, val & 0x8000);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                    break;
                case 1: // ROR
                    res = (val >> 1) | (val << 15);
                    SET_FLAG_IF(FLAG_CF, val & 0x0001);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                    break;
                case 2: // RCL
                    res = (val << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0);
                    SET_FLAG_IF(FLAG_CF, val & 0x8000);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                    break;
                case 3: // RCR
                    res = (val >> 1) | ((cpu->flags & FLAG_CF) ? 0x8000 : 0);
                    SET_FLAG_IF(FLAG_CF, val & 0x0001);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                    break;
                case 4: // SHL/SAL
                    res = val << 1;
                    SET_FLAG_IF(FLAG_CF, val & 0x8000);
                    SET_FLAG_IF(FLAG_OF, (res ^ val) & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 5: // SHR
                    res = val >> 1;
                    SET_FLAG_IF(FLAG_CF, val & 0x0001);
                    SET_FLAG_IF(FLAG_OF, val & 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    break;
                case 7: // SAR
                    res = (val >> 1) | (val & 0x8000);
                    SET_FLAG_IF(FLAG_CF, val & 0x0001);
                    cpu->flags &= ~FLAG_OF;
                    set_pzs_flags(cpu, res, 16);
                    break;
                default:
                    res = val;
                    break;
            }
            if (i_mod == 3) cpu->regs16[i_rm] = res;
            else mem_write_word(ea, res);
            break;
        }
        
        // Shift/Rotate r/m8, CL
        case 0xD2: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
            uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
            uint8_t count = cpu->regs8[2] & 0x1F;  // CL, masked to 5 bits
            uint8_t res = val;
            
            for (int c = 0; c < count; c++) {
                switch (i_reg) {
                    case 0: // ROL
                        SET_FLAG_IF(FLAG_CF, res & 0x80);
                        res = (res << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0);
                        break;
                    case 1: // ROR
                        SET_FLAG_IF(FLAG_CF, res & 0x01);
                        res = (res >> 1) | ((cpu->flags & FLAG_CF) ? 0x80 : 0);
                        break;
                    case 2: { // RCL
                        uint8_t cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        SET_FLAG_IF(FLAG_CF, res & 0x80);
                        res = (res << 1) | cf;
                        break;
                    }
                    case 3: { // RCR
                        uint8_t cf = (cpu->flags & FLAG_CF) ? 0x80 : 0;
                        SET_FLAG_IF(FLAG_CF, res & 0x01);
                        res = (res >> 1) | cf;
                        break;
                    }
                    case 4: // SHL/SAL
                        SET_FLAG_IF(FLAG_CF, res & 0x80);
                        res <<= 1;
                        break;
                    case 5: // SHR
                        SET_FLAG_IF(FLAG_CF, res & 0x01);
                        res >>= 1;
                        break;
                    case 7: // SAR
                        SET_FLAG_IF(FLAG_CF, res & 0x01);
                        res = (res >> 1) | (res & 0x80);
                        break;
                }
            }
            if (count > 0) set_pzs_flags(cpu, res, 8);
            if (i_mod == 3) cpu->regs8[ri] = res;
            else mem_write_byte(ea, res);
            break;
        }
        
        // Shift/Rotate r/m16, CL
        case 0xD3: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
            uint8_t count = cpu->regs8[2] & 0x1F;  // CL, masked to 5 bits
            uint16_t res = val;
            
            for (int c = 0; c < count; c++) {
                switch (i_reg) {
                    case 0: // ROL
                        SET_FLAG_IF(FLAG_CF, res & 0x8000);
                        res = (res << 1) | ((cpu->flags & FLAG_CF) ? 1 : 0);
                        break;
                    case 1: // ROR
                        SET_FLAG_IF(FLAG_CF, res & 0x0001);
                        res = (res >> 1) | ((cpu->flags & FLAG_CF) ? 0x8000 : 0);
                        break;
                    case 2: { // RCL
                        uint16_t cf = (cpu->flags & FLAG_CF) ? 1 : 0;
                        SET_FLAG_IF(FLAG_CF, res & 0x8000);
                        res = (res << 1) | cf;
                        break;
                    }
                    case 3: { // RCR
                        uint16_t cf = (cpu->flags & FLAG_CF) ? 0x8000 : 0;
                        SET_FLAG_IF(FLAG_CF, res & 0x0001);
                        res = (res >> 1) | cf;
                        break;
                    }
                    case 4: // SHL/SAL
                        SET_FLAG_IF(FLAG_CF, res & 0x8000);
                        res <<= 1;
                        break;
                    case 5: // SHR
                        SET_FLAG_IF(FLAG_CF, res & 0x0001);
                        res >>= 1;
                        break;
                    case 7: // SAR
                        SET_FLAG_IF(FLAG_CF, res & 0x0001);
                        res = (res >> 1) | (res & 0x8000);
                        break;
                }
            }
            if (count > 0) set_pzs_flags(cpu, res, 16);
            if (i_mod == 3) cpu->regs16[i_rm] = res;
            else mem_write_word(ea, res);
            break;
        }

        // AAM
        case 0xD4: {
            inst_len = 2;
            uint8_t base = opcode_stream[1];
            if (base == 0) {
                cpu_interrupt(cpu, 0);
                inst_len = 0;
                break;
            }
            uint8_t al = cpu->regs8[0];
            cpu->regs8[1] = al / base;
            cpu->regs8[0] = al % base;
            set_pzs_flags(cpu, cpu->regs8[0], 8);
            break;
        }

        // AAD
        case 0xD5: {
            inst_len = 2;
            uint8_t base = opcode_stream[1];
            uint16_t res = (uint16_t)cpu->regs8[1] * base + cpu->regs8[0];
            cpu->regs8[0] = (uint8_t)res;
            cpu->regs8[1] = 0;
            set_pzs_flags(cpu, cpu->regs8[0], 8);
            break;
        }

        // SALC
        case 0xD6:
            cpu->regs8[0] = (cpu->flags & FLAG_CF) ? 0xFF : 0x00;
            break;

        // XLAT
        case 0xD7: {
            uint16_t seg = cpu->sregs[cpu->seg_override >= 0 ? cpu->seg_override : SEG_DS];
            uint32_t addr = (seg << 4) + cpu->regs16[REG_BX] + cpu->regs8[0];
            cpu->regs8[0] = mem_read_byte(addr);
            break;
        }

        // ESC (8087 FPU instructions) - treat as NOP but consume operands
        case 0xD8: case 0xD9: case 0xDA: case 0xDB:
        case 0xDC: case 0xDD: case 0xDE: case 0xDF: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            break;
        }
            
        // RET near
        case 0xC2:
            inst_len = 3;
            cpu->ip = pop_word(cpu);
            cpu->regs16[REG_SP] += (uint16_t)i_data0;
            inst_len = 0;
            break;
        case 0xC3:
            cpu->ip = pop_word(cpu);
            inst_len = 0;
            break;
            
        // RET far
        case 0xCA:
            inst_len = 3;
            cpu->ip = pop_word(cpu);
            cpu->sregs[SEG_CS] = pop_word(cpu);
            cpu->regs16[REG_SP] += (uint16_t)i_data0;
            log_lowmem_return("RETF", &retf_lowmem_log_count, cpu,
                              cpu->sregs[SEG_CS], cpu->ip, cpu->flags);
            inst_len = 0;
            break;
        case 0xCB:
            {
                uint32_t sp_addr = (cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
                uint16_t ret_ip = mem_read_word(sp_addr);
                uint16_t ret_cs = mem_read_word(sp_addr + 2);
                if (cpu->sregs[SEG_CS] == 0x0000 &&
                    cpu->ip >= 0x7C00 && cpu->ip < 0x7D00) {
                    CPU_TRACE_LOGW("RETF at %04X:%04X SS:SP=%04X:%04X ret=%04X:%04X",
                                   cpu->sregs[SEG_CS], ip_before,
                                   cpu->sregs[SEG_SS], cpu->regs16[REG_SP],
                                   ret_cs, ret_ip);
                }
                cpu->ip = pop_word(cpu);
                cpu->sregs[SEG_CS] = pop_word(cpu);
                log_lowmem_return("RETF", &retf_lowmem_log_count, cpu,
                                  cpu->sregs[SEG_CS], cpu->ip, cpu->flags);
            }
            inst_len = 0;
            break;
            
        // INT 3
        case 0xCC:
            cpu->ip++;
            cpu_interrupt(cpu, 3);
            inst_len = 0;
            break;

        // INT 1 (ICEBP)
        case 0xF1:
            cpu->ip++;
            cpu_interrupt(cpu, 1);
            inst_len = 0;
            break;
            
        // INT imm8
        case 0xCD:
            inst_len = 2;
            cpu->ip += inst_len;
            {
                uint8_t intnum = (uint8_t)(i_data0 & 0xFF);
                if (intnum == 0x13 || intnum == 0x18 || intnum == 0x19) {
                    log_int_opcode(cpu, intnum, ip_before);
                }
                if (intnum == 0x21) {
                    log_dos_int21(cpu);
                }
            }
            
            // Check if BIOS handles this interrupt
            if (bios_interrupt(cpu, i_data0 & 0xFF)) {
                inst_len = 0;  // BIOS handled it, don't advance IP again
            } else {
                cpu_interrupt(cpu, i_data0 & 0xFF);
                inst_len = 0;
            }
            break;

        // INTO
        case 0xCE:
            if (cpu->flags & FLAG_OF) {
                cpu->ip++;
                cpu_interrupt(cpu, 4);
                inst_len = 0;
            }
            break;
            
        // IRET
        case 0xCF:
            {
                uint16_t cur_cs = cpu->sregs[SEG_CS];
                uint16_t sp_before = cpu->regs16[REG_SP];
                uint32_t sp_addr = ((uint32_t)cpu->sregs[SEG_SS] << 4) + sp_before;
                uint16_t peek_ip = mem_read_word(sp_addr);
                uint16_t peek_cs = mem_read_word(sp_addr + 2);
                log_bad_iret_target(cpu, cur_cs, ip_before, peek_cs, peek_ip, sp_before);
                uint16_t ret_ip = pop_word(cpu);
                uint16_t ret_cs = pop_word(cpu);
                uint16_t ret_flags = pop_word(cpu) | 0xF002;
                cpu->ip = ret_ip;
                cpu->sregs[SEG_CS] = ret_cs;
                cpu->flags = ret_flags;
                int21_return_maybe_log(cpu, ret_cs, ret_ip);
                if (ret_cs == 0x0000 && ret_ip < 0x0500 &&
                    should_rate_log(&iret_lowmem_stack_log_count)) {
                    uint16_t w0 = mem_read_word(sp_addr);
                    uint16_t w1 = mem_read_word(sp_addr + 2);
                    uint16_t w2 = mem_read_word(sp_addr + 4);
                    uint16_t w3 = mem_read_word(sp_addr + 6);
                    CPU_TRACE_LOGW("IRET lowmem stack ret=%04X:%04X SS:SP=%04X:%04X words=[%04X %04X %04X %04X]",
                                   ret_cs, ret_ip, cpu->sregs[SEG_SS], sp_before,
                                   w0, w1, w2, w3);
                }
                log_lowmem_return("IRET", &iret_lowmem_log_count, cpu,
                                  ret_cs, ret_ip, ret_flags);
            }
            interrupts_set_enabled((cpu->flags & FLAG_IF) != 0);
            inst_len = 0;
            break;
            
        // CALL rel16
        case 0xE8:
            inst_len = 3;
            push_word(cpu, cpu->ip + inst_len);
            cpu->ip += inst_len + i_data0;
            inst_len = 0;
            break;
            
        // JMP rel16
        case 0xE9:
            inst_len = 3;
            cpu->ip += inst_len + i_data0;
            inst_len = 0;
            break;
            
        // JMP far ptr
        case 0xEA:
            if (i_data2 == 0x0000 && i_data0 < 0x0500 &&
                should_rate_log(&diag_farptr_low_log_count)) {
                CPU_DIAG_LOGW("FAR JMP imm lowmem %04X:%04X from %04X:%04X",
                              i_data2, i_data0, cpu->sregs[SEG_CS], ip_before);
            }
            if (is_bad_far_target(i_data2, i_data0) && should_rate_log(&far_bad_target_log_count)) {
                CPU_TRACE_LOGW("JMP FAR imm -> %04X:%04X from %04X:%04X",
                               i_data2, i_data0, cpu->sregs[SEG_CS], ip_before);
            }
            cpu->ip = i_data0;
            cpu->sregs[SEG_CS] = i_data2;
            inst_len = 0;
            break;
            
        // JMP rel8
        case 0xEB:
            inst_len = 2;
            cpu->ip += inst_len + (int8_t)(i_data0 & 0xFF);
            inst_len = 0;
            break;
        
        // LOOPNE/LOOPNZ
        case 0xE0:
            inst_len = 2;
            cpu->regs16[REG_CX]--;
            if (cpu->regs16[REG_CX] != 0 && !(cpu->flags & FLAG_ZF)) {
                cpu->ip += inst_len + (int8_t)(i_data0 & 0xFF);
                inst_len = 0;
            }
            break;
            
        // LOOPE/LOOPZ
        case 0xE1:
            inst_len = 2;
            cpu->regs16[REG_CX]--;
            if (cpu->regs16[REG_CX] != 0 && (cpu->flags & FLAG_ZF)) {
                cpu->ip += inst_len + (int8_t)(i_data0 & 0xFF);
                inst_len = 0;
            }
            break;
            
        // LOOP
        case 0xE2:
            inst_len = 2;
            cpu->regs16[REG_CX]--;
            if (cpu->regs16[REG_CX] != 0) {
                cpu->ip += inst_len + (int8_t)(i_data0 & 0xFF);
                inst_len = 0;
            }
            break;
            
        // JCXZ
        case 0xE3:
            inst_len = 2;
            if (cpu->regs16[REG_CX] == 0) {
                cpu->ip += inst_len + (int8_t)(i_data0 & 0xFF);
                inst_len = 0;
            }
            break;
            
        // IN AL, imm8
        case 0xE4:
            inst_len = 2;
            cpu->regs8[0] = port_in(i_data0 & 0xFF);
            port_trace_spin(true, cpu, ip_before, (uint16_t)(i_data0 & 0xFF), cpu->regs8[0], false);
            break;
            
        // IN AX, imm8
        case 0xE5:
            inst_len = 2;
            cpu->regs16[REG_AX] = port_in_word(i_data0 & 0xFF);
            port_trace_spin(true, cpu, ip_before, (uint16_t)(i_data0 & 0xFF), cpu->regs16[REG_AX], true);
            break;
            
        // OUT imm8, AL
        case 0xE6:
            inst_len = 2;
            port_out(i_data0 & 0xFF, cpu->regs8[0]);
            port_trace_spin(false, cpu, ip_before, (uint16_t)(i_data0 & 0xFF), cpu->regs8[0], false);
            break;
            
        // OUT imm8, AX
        case 0xE7:
            inst_len = 2;
            port_out_word(i_data0 & 0xFF, cpu->regs16[REG_AX]);
            port_trace_spin(false, cpu, ip_before, (uint16_t)(i_data0 & 0xFF), cpu->regs16[REG_AX], true);
            break;
            
        // IN AL, DX
        case 0xEC:
            cpu->regs8[0] = port_in(cpu->regs16[REG_DX]);
            port_trace_spin(true, cpu, ip_before, cpu->regs16[REG_DX], cpu->regs8[0], false);
            break;
            
        // IN AX, DX
        case 0xED:
            cpu->regs16[REG_AX] = port_in_word(cpu->regs16[REG_DX]);
            port_trace_spin(true, cpu, ip_before, cpu->regs16[REG_DX], cpu->regs16[REG_AX], true);
            break;
            
        // OUT DX, AL
        case 0xEE:
            port_out(cpu->regs16[REG_DX], cpu->regs8[0]);
            port_trace_spin(false, cpu, ip_before, cpu->regs16[REG_DX], cpu->regs8[0], false);
            break;
            
        // OUT DX, AX
        case 0xEF:
            port_out_word(cpu->regs16[REG_DX], cpu->regs16[REG_AX]);
            port_trace_spin(false, cpu, ip_before, cpu->regs16[REG_DX], cpu->regs16[REG_AX], true);
            break;
            
        // CLC
        case 0xF8:
            cpu->flags &= ~FLAG_CF;
            break;
            
        // STC
        case 0xF9:
            cpu->flags |= FLAG_CF;
            break;
            
        // CLI
        case 0xFA:
            cpu->flags &= ~FLAG_IF;
            interrupts_set_enabled(false);
            break;
            
        // STI
        case 0xFB:
            cpu->flags |= FLAG_IF;
            interrupts_set_enabled(true);
            break;
            
        // CLD
        case 0xFC:
            cpu->flags &= ~FLAG_DF;
            break;
            
        // STD
        case 0xFD:
            cpu->flags |= FLAG_DF;
            break;

        // CMC
        case 0xF5:
            cpu->flags ^= FLAG_CF;
            break;
              
        // HLT
        case 0xF4:
            CPU_TRACE_LOGW("HLT at %04X:%04X IF=%u FLAGS=%04X",
                           cpu->sregs[SEG_CS], ip_before,
                           (cpu->flags & FLAG_IF) ? 1U : 0U, cpu->flags);
            cpu->halted = true;
            break;
        
        // NOT/NEG/MUL/IMUL/DIV/IDIV r/m8
        case 0xF6: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
            
            switch (i_reg) {
                case 0: // TEST r/m8, imm8
                case 1: {
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    uint8_t imm = opcode_stream[inst_len];
                    inst_len++;
                    uint8_t res = val & imm;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    set_pzs_flags(cpu, res, 8);
                    break;
                }
                case 2: { // NOT r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    val = ~val;
                    if (i_mod == 3) cpu->regs8[ri] = val;
                    else mem_write_byte(ea, val);
                    break;
                }
                case 3: { // NEG r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    uint8_t res = -val;
                    SET_FLAG_IF(FLAG_CF, val != 0);
                    SET_FLAG_IF(FLAG_OF, val == 0x80);
                    set_pzs_flags(cpu, res, 8);
                    if (i_mod == 3) cpu->regs8[ri] = res;
                    else mem_write_byte(ea, res);
                    break;
                }
                case 4: { // MUL r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    uint16_t res = (uint16_t)cpu->regs8[0] * val;
                    cpu->regs16[REG_AX] = res;
                    SET_FLAG_IF(FLAG_CF, (res & 0xFF00) != 0);
                    SET_FLAG_IF(FLAG_OF, (res & 0xFF00) != 0);
                    break;
                }
                case 5: { // IMUL r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    int16_t res = (int8_t)cpu->regs8[0] * (int8_t)val;
                    cpu->regs16[REG_AX] = (uint16_t)res;
                    SET_FLAG_IF(FLAG_CF, res != (int8_t)res);
                    SET_FLAG_IF(FLAG_OF, res != (int8_t)res);
                    break;
                }
                case 6: { // DIV r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    if (val == 0) {
                        cpu_interrupt(cpu, 0);  // Divide by zero
                        inst_len = 0;
                    } else {
                        uint16_t dividend = cpu->regs16[REG_AX];
                        uint16_t quotient = dividend / val;
                        uint16_t remainder = dividend % val;
                        if (quotient > 0xFF) {
                            cpu_interrupt(cpu, 0);  // Divide overflow
                            inst_len = 0;
                        } else {
                            cpu->regs8[0] = (uint8_t)quotient;  // AL = quotient
                            cpu->regs8[1] = (uint8_t)remainder; // AH = remainder
                        }
                    }
                    break;
                }
                case 7: { // IDIV r/m8
                    int8_t val = (i_mod == 3) ? (int8_t)cpu->regs8[ri] : (int8_t)mem_read_byte(ea);
                    if (val == 0) {
                        cpu_interrupt(cpu, 0);
                        inst_len = 0;
                    } else {
                        int16_t dividend = (int16_t)cpu->regs16[REG_AX];
                        int16_t quotient = (int16_t)(dividend / val);
                        int16_t remainder = (int16_t)(dividend % val);
                        if (quotient < -128 || quotient > 127) {
                            cpu_interrupt(cpu, 0);  // Divide overflow
                            inst_len = 0;
                        } else {
                            cpu->regs8[0] = (uint8_t)quotient;
                            cpu->regs8[1] = (uint8_t)remainder;
                        }
                    }
                    break;
                }
            }
            break;
        }
        
        // NOT/NEG/MUL/IMUL/DIV/IDIV r/m16
        case 0xF7: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            switch (i_reg) {
                case 0: // TEST r/m16, imm16
                case 1: {
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    uint16_t imm = *(uint16_t *)(opcode_stream + inst_len);
                    inst_len += 2;
                    uint16_t res = val & imm;
                    cpu->flags &= ~(FLAG_CF | FLAG_OF);
                    set_pzs_flags(cpu, res, 16);
                    break;
                }
                case 2: { // NOT r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    val = ~val;
                    if (i_mod == 3) cpu->regs16[i_rm] = val;
                    else mem_write_word(ea, val);
                    break;
                }
                case 3: { // NEG r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    uint16_t res = -val;
                    SET_FLAG_IF(FLAG_CF, val != 0);
                    SET_FLAG_IF(FLAG_OF, val == 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    break;
                }
                case 4: { // MUL r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    uint32_t res = (uint32_t)cpu->regs16[REG_AX] * val;
                    cpu->regs16[REG_AX] = res & 0xFFFF;
                    cpu->regs16[REG_DX] = (res >> 16) & 0xFFFF;
                    SET_FLAG_IF(FLAG_CF, cpu->regs16[REG_DX] != 0);
                    SET_FLAG_IF(FLAG_OF, cpu->regs16[REG_DX] != 0);
                    break;
                }
                case 5: { // IMUL r/m16
                    int16_t val = (i_mod == 3) ? (int16_t)cpu->regs16[i_rm] : (int16_t)mem_read_word(ea);
                    int32_t res = (int16_t)cpu->regs16[REG_AX] * val;
                    cpu->regs16[REG_AX] = (uint16_t)(res & 0xFFFF);
                    cpu->regs16[REG_DX] = (uint16_t)((res >> 16) & 0xFFFF);
                    SET_FLAG_IF(FLAG_CF, res != (int16_t)res);
                    SET_FLAG_IF(FLAG_OF, res != (int16_t)res);
                    break;
                }
                case 6: { // DIV r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    if (val == 0) {
                        cpu_interrupt(cpu, 0);
                        inst_len = 0;
                    } else {
                        uint32_t dividend = ((uint32_t)cpu->regs16[REG_DX] << 16) | cpu->regs16[REG_AX];
                        uint32_t quotient = dividend / val;
                        uint32_t remainder = dividend % val;
                        if (quotient > 0xFFFF) {
                            cpu_interrupt(cpu, 0);  // Divide overflow
                            inst_len = 0;
                        } else {
                            cpu->regs16[REG_AX] = (uint16_t)quotient;
                            cpu->regs16[REG_DX] = (uint16_t)remainder;
                        }
                    }
                    break;
                }
                case 7: { // IDIV r/m16
                    int16_t val = (i_mod == 3) ? (int16_t)cpu->regs16[i_rm] : (int16_t)mem_read_word(ea);
                    if (val == 0) {
                        cpu_interrupt(cpu, 0);
                        inst_len = 0;
                    } else {
                        int32_t dividend = ((int32_t)cpu->regs16[REG_DX] << 16) | cpu->regs16[REG_AX];
                        int32_t quotient = dividend / val;
                        int32_t remainder = dividend % val;
                        if (quotient < -32768 || quotient > 32767) {
                            cpu_interrupt(cpu, 0);  // Divide overflow
                            inst_len = 0;
                        } else {
                            cpu->regs16[REG_AX] = (uint16_t)quotient;
                            cpu->regs16[REG_DX] = (uint16_t)remainder;
                        }
                    }
                    break;
                }
            }
            break;
        }
        
        // INC/DEC r/m8
        case 0xFE: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            int ri = i_rm < 4 ? i_rm * 2 : (i_rm - 4) * 2 + 1;
            
            switch (i_reg) {
                case 0: { // INC r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    uint8_t res = val + 1;
                    SET_FLAG_IF(FLAG_OF, val == 0x7F);
                    set_pzs_flags(cpu, res, 8);
                    if (i_mod == 3) cpu->regs8[ri] = res;
                    else mem_write_byte(ea, res);
                    break;
                }
                case 1: { // DEC r/m8
                    uint8_t val = (i_mod == 3) ? cpu->regs8[ri] : mem_read_byte(ea);
                    uint8_t res = val - 1;
                    SET_FLAG_IF(FLAG_OF, val == 0x80);
                    set_pzs_flags(cpu, res, 8);
                    if (i_mod == 3) cpu->regs8[ri] = res;
                    else mem_write_byte(ea, res);
                    break;
                }
            }
            break;
        }
        
        // INC/DEC/CALL/JMP/PUSH r/m16
        case 0xFF: {
            inst_len = 2;
            if (i_mod == 0 && i_rm == 6) inst_len += 2;
            else if (i_mod == 1) inst_len += 1;
            else if (i_mod == 2) inst_len += 2;
            
            uint32_t ea = calc_ea(cpu, modrm, i_data1, cpu->seg_override);
            
            switch (i_reg) {
                case 0: { // INC r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    uint16_t res = val + 1;
                    SET_FLAG_IF(FLAG_OF, val == 0x7FFF);
                    set_pzs_flags(cpu, res, 16);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    break;
                }
                case 1: { // DEC r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    uint16_t res = val - 1;
                    SET_FLAG_IF(FLAG_OF, val == 0x8000);
                    set_pzs_flags(cpu, res, 16);
                    if (i_mod == 3) cpu->regs16[i_rm] = res;
                    else mem_write_word(ea, res);
                    break;
                }
                case 2: { // CALL r/m16
                    uint16_t target = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    push_word(cpu, cpu->ip + inst_len);
                    cpu->ip = target;
                    inst_len = 0;
                    break;
                }
                case 3: { // CALL far m16:16
                    if (i_mod != 3) {
                        uint16_t new_ip = mem_read_word(ea);
                        uint16_t new_cs = mem_read_word(ea + 2);
                        if (new_cs == 0x0000 && new_ip < 0x0500 &&
                            should_rate_log(&diag_farptr_low_log_count)) {
                            uint16_t ptr_seg = 0;
                            uint16_t ptr_off = 0;
                            calc_ea_seg_off(cpu, modrm, i_data1, cpu->seg_override, &ptr_seg, &ptr_off);
                            CPU_DIAG_LOGW("FAR CALL lowmem ptr [%04X:%04X]=%04X:%04X from %04X:%04X",
                                          ptr_seg, ptr_off, new_cs, new_ip,
                                          cpu->sregs[SEG_CS], ip_before);
                        }
                        bool log_bad = false;
                        uint16_t ptr_seg = 0;
                        uint16_t ptr_off = 0;
                        uint16_t sp_before = cpu->regs16[REG_SP];
                        uint16_t ret_ip = (uint16_t)(cpu->ip + inst_len);
                        uint16_t ret_cs = cpu->sregs[SEG_CS];
                        if (is_bad_far_target(new_cs, new_ip) && should_rate_log(&far_bad_target_log_count)) {
                            calc_ea_seg_off(cpu, modrm, i_data1, cpu->seg_override, &ptr_seg, &ptr_off);
                            log_bad = true;
                        }
                        push_word(cpu, cpu->sregs[SEG_CS]);
                        push_word(cpu, cpu->ip + inst_len);
                        if (log_bad) {
                            uint16_t sp_after = cpu->regs16[REG_SP];
                            uint32_t sp_addr = ((uint32_t)cpu->sregs[SEG_SS] << 4) + sp_after;
                            uint16_t stk_ip = mem_read_word(sp_addr);
                            uint16_t stk_cs = mem_read_word(sp_addr + 2);
                            CPU_TRACE_LOGW("CALL FAR ptr [%04X:%04X] -> %04X:%04X from %04X:%04X",
                                           ptr_seg, ptr_off, new_cs, new_ip,
                                           cpu->sregs[SEG_CS], ip_before);
                            CPU_TRACE_LOGW("CALL FAR stack SS:SP %04X:%04X->%04X ret=%04X:%04X mem=[%04X %04X]",
                                           cpu->sregs[SEG_SS], sp_before, sp_after,
                                           ret_cs, ret_ip, stk_ip, stk_cs);
                        }
                        cpu->ip = new_ip;
                        cpu->sregs[SEG_CS] = new_cs;
                        inst_len = 0;
                    }
                    break;
                }
                case 4: { // JMP r/m16
                    uint16_t target = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    cpu->ip = target;
                    inst_len = 0;
                    break;
                }
                case 5: { // JMP far m16:16
                    if (i_mod != 3) {
                        uint16_t new_ip = mem_read_word(ea);
                        uint16_t new_cs = mem_read_word(ea + 2);
                        if (new_cs == 0x0000 && new_ip < 0x0500 &&
                            should_rate_log(&diag_farptr_low_log_count)) {
                            uint16_t ptr_seg = 0;
                            uint16_t ptr_off = 0;
                            calc_ea_seg_off(cpu, modrm, i_data1, cpu->seg_override, &ptr_seg, &ptr_off);
                            CPU_DIAG_LOGW("FAR JMP lowmem ptr [%04X:%04X]=%04X:%04X from %04X:%04X",
                                          ptr_seg, ptr_off, new_cs, new_ip,
                                          cpu->sregs[SEG_CS], ip_before);
                        }
                        if (is_bad_far_target(new_cs, new_ip) && should_rate_log(&far_bad_target_log_count)) {
                            uint16_t ptr_seg = 0;
                            uint16_t ptr_off = 0;
                            calc_ea_seg_off(cpu, modrm, i_data1, cpu->seg_override, &ptr_seg, &ptr_off);
                            CPU_TRACE_LOGW("JMP FAR ptr [%04X:%04X] -> %04X:%04X from %04X:%04X",
                                           ptr_seg, ptr_off, new_cs, new_ip,
                                           cpu->sregs[SEG_CS], ip_before);
                        }
                        cpu->ip = new_ip;
                        cpu->sregs[SEG_CS] = new_cs;
                        inst_len = 0;
                    }
                    break;
                }
                case 6: { // PUSH r/m16
                    uint16_t val = (i_mod == 3) ? cpu->regs16[i_rm] : mem_read_word(ea);
                    push_word(cpu, val);
                    break;
                }
            }
            break;
        }
            
        default:
            ESP_LOGW(TAG, "Unhandled opcode: %02X at %04X:%04X", 
                     cpu->opcode, cpu->sregs[SEG_CS], cpu->ip);
            break;
    }
    
    // Advance instruction pointer
    cpu->ip += inst_len;
    cpu->cycles++;
    
    return 1;
}

/**
 * Execute multiple cycles - OPTIMIZED: no vTaskDelay in hot path
 * Yields are handled by the caller (cpu_task) to prevent watchdog timeout
 * This function runs as fast as possible without any delays
 */
int IRAM_ATTR cpu_exec_cycles(cpu8086_t *cpu, int cycles)
{
    int executed = 0;
    
    // Execute in tight loop without any delays
    // WDT and yielding is handled by the cpu_task caller
    while (executed < cycles && !cpu->halted) {
        cpu_exec_instruction(cpu);
        executed++;
    }
    
    return executed;
}
