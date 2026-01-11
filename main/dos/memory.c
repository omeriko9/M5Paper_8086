/**
 * @file memory.c
 * @brief Memory management for 8086 emulator
 * 
 * Manages the 1MB addressable memory space of the 8086.
 * Special handling for video memory regions.
 * 
 * OPTIMIZED: Critical functions marked IRAM_ATTR for speed.
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "memory.h"

static const char *TAG = "MEM";
static const char *CPU_TRACE_TAG = "CPU_TRACE";

// Main memory array (1MB + some extra for register storage)
static uint8_t *mem = NULL;
static bool mem_a20_enabled = false;

// Video memory dirty tracking
static bool video_dirty = false;

static uint16_t mem_ctx_cs = 0;
static uint16_t mem_ctx_ip = 0;
static uint8_t mem_ctx_op = 0;
static uint8_t mem_ctx_b1 = 0;
static uint8_t mem_ctx_b2 = 0;
static bool mem_ctx_valid = false;

static inline IRAM_ATTR bool ivt_watch_addr(uint32_t addr, uint8_t *vec, bool *is_seg, uint16_t *word_addr)
{
    switch (addr) {
        case 0x0000: case 0x0001:
            *vec = 0x00; *is_seg = false; *word_addr = 0x0000; return true;
        case 0x0002: case 0x0003:
            *vec = 0x00; *is_seg = true; *word_addr = 0x0002; return true;
        case 0x0010: case 0x0011:
            *vec = 0x04; *is_seg = false; *word_addr = 0x0010; return true;
        case 0x0012: case 0x0013:
            *vec = 0x04; *is_seg = true; *word_addr = 0x0012; return true;
        case 0x0014: case 0x0015:
            *vec = 0x05; *is_seg = false; *word_addr = 0x0014; return true;
        case 0x0016: case 0x0017:
            *vec = 0x05; *is_seg = true; *word_addr = 0x0016; return true;
        case 0x0018: case 0x0019:
            *vec = 0x06; *is_seg = false; *word_addr = 0x0018; return true;
        case 0x001A: case 0x001B:
            *vec = 0x06; *is_seg = true; *word_addr = 0x001A; return true;
        default:
            return false;
    }
}

/**
 * Initialize memory system
 */
void dos_mem_init(void)
{
    ESP_LOGI(TAG, "Initializing 1MB memory...");
    
    // Try to allocate from SPIRAM first, fallback to internal
    mem = (uint8_t *)heap_caps_malloc(MEM_SIZE + 0x10000, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mem) {
        ESP_LOGW(TAG, "SPIRAM allocation failed, using internal memory");
        mem = (uint8_t *)malloc(MEM_SIZE + 0x10000);
    }
    
    if (!mem) {
        ESP_LOGE(TAG, "Failed to allocate memory!");
        return;
    }
    
    // Clear memory
    memset(mem, 0, MEM_SIZE + 0x10000);
    mem_a20_enabled = false;
    mem_ctx_valid = false;
    
    ESP_LOGI(TAG, "Memory initialized at %p", mem);
}

void IRAM_ATTR mem_set_cpu_context(uint16_t cs, uint16_t ip, uint8_t op, uint8_t b1, uint8_t b2)
{
    mem_ctx_cs = cs;
    mem_ctx_ip = ip;
    mem_ctx_op = op;
    mem_ctx_b1 = b1;
    mem_ctx_b2 = b2;
    mem_ctx_valid = true;
}

/**
 * Free memory system
 */
void dos_mem_free(void)
{
    if (mem) {
        free(mem);
        mem = NULL;
    }
}

/**
 * Read a byte from memory - IRAM for speed
 */
static inline IRAM_ATTR uint32_t mem_mask_addr(uint32_t addr)
{
    if (!mem_a20_enabled) {
        return addr & 0xFFFFF;
    }
    if (addr >= (MEM_SIZE + 0x10000)) {
        return addr & 0xFFFFF;
    }
    return addr;
}

IRAM_ATTR uint8_t mem_read_byte(uint32_t addr)
{
    addr = mem_mask_addr(addr);
    return mem[addr];
}

/**
 * Read a word from memory (little-endian) - IRAM for speed
 */
IRAM_ATTR uint16_t mem_read_word(uint32_t addr)
{
    uint32_t a0 = mem_mask_addr(addr);
    uint32_t a1 = mem_mask_addr(addr + 1);
    return mem[a0] | (mem[a1] << 8);
}

/**
 * Read a dword from memory (little-endian)
 */
uint32_t mem_read_dword(uint32_t addr)
{
    if (!mem) return 0xFFFFFFFF;
    uint32_t a0 = mem_mask_addr(addr);
    uint32_t a1 = mem_mask_addr(addr + 1);
    uint32_t a2 = mem_mask_addr(addr + 2);
    uint32_t a3 = mem_mask_addr(addr + 3);
    return mem[a0] |
           (mem[a1] << 8) |
           (mem[a2] << 16) |
           (mem[a3] << 24);
}

/**
 * Write a byte to memory - IRAM for speed
 */
IRAM_ATTR void mem_write_byte(uint32_t addr, uint8_t val)
{
    addr = mem_mask_addr(addr);

    uint8_t ivt_vec = 0;
    bool ivt_is_seg = false;
    uint16_t ivt_word_addr = 0;
    bool ivt_watch = ivt_watch_addr(addr, &ivt_vec, &ivt_is_seg, &ivt_word_addr);
    uint16_t ivt_old = 0;
    if (ivt_watch) {
        ivt_old = mem_read_word(ivt_word_addr);
    }
    
    // Check if writing to video memory
    if (addr >= VIDEO_RAM_START && addr < VIDEO_RAM_START + VIDEO_RAM_SIZE) {
        video_dirty = true;
    }
    
    mem[addr] = val;

    if (ivt_watch) {
        uint16_t ivt_new = mem_read_word(ivt_word_addr);
        if (ivt_new != ivt_old) {
            if (mem_ctx_valid) {
                ESP_LOGI(CPU_TRACE_TAG,
                         "IVT vec=%02X %s %04X->%04X (byte @%04X) CS:IP=%04X:%04X op=%02X %02X %02X",
                         ivt_vec, ivt_is_seg ? "seg" : "off",
                         ivt_old, ivt_new, (unsigned)addr,
                         mem_ctx_cs, mem_ctx_ip, mem_ctx_op, mem_ctx_b1, mem_ctx_b2);
            } else {
                ESP_LOGI(CPU_TRACE_TAG,
                         "IVT vec=%02X %s %04X->%04X (byte @%04X) CS:IP=----:---- op=-- -- --",
                         ivt_vec, ivt_is_seg ? "seg" : "off",
                         ivt_old, ivt_new, (unsigned)addr);
            }
        }
    }
}

/**
 * Write a word to memory (little-endian) - IRAM for speed
 */
IRAM_ATTR void mem_write_word(uint32_t addr, uint16_t val)
{
    uint32_t a0 = mem_mask_addr(addr);
    uint32_t a1 = mem_mask_addr(addr + 1);

    uint8_t ivt_vec = 0;
    bool ivt_is_seg = false;
    uint16_t ivt_word_addr = 0;
    bool ivt_watch = ivt_watch_addr(a0, &ivt_vec, &ivt_is_seg, &ivt_word_addr);
    if (!ivt_watch) {
        ivt_watch = ivt_watch_addr(a1, &ivt_vec, &ivt_is_seg, &ivt_word_addr);
    }
    uint16_t ivt_old = 0;
    if (ivt_watch) {
        ivt_old = mem_read_word(ivt_word_addr);
    }
    
    // Check if writing to video memory
    if ((a0 >= VIDEO_RAM_START && a0 < VIDEO_RAM_START + VIDEO_RAM_SIZE) ||
        (a1 >= VIDEO_RAM_START && a1 < VIDEO_RAM_START + VIDEO_RAM_SIZE)) {
        video_dirty = true;
    }
    
    mem[a0] = val & 0xFF;
    mem[a1] = (val >> 8) & 0xFF;

    if (ivt_watch) {
        uint16_t ivt_new = mem_read_word(ivt_word_addr);
        if (ivt_new != ivt_old) {
            if (mem_ctx_valid) {
                ESP_LOGI(CPU_TRACE_TAG,
                         "IVT vec=%02X %s %04X->%04X (word @%04X) CS:IP=%04X:%04X op=%02X %02X %02X",
                         ivt_vec, ivt_is_seg ? "seg" : "off",
                         ivt_old, ivt_new, (unsigned)a0,
                         mem_ctx_cs, mem_ctx_ip, mem_ctx_op, mem_ctx_b1, mem_ctx_b2);
            } else {
                ESP_LOGI(CPU_TRACE_TAG,
                         "IVT vec=%02X %s %04X->%04X (word @%04X) CS:IP=----:---- op=-- -- --",
                         ivt_vec, ivt_is_seg ? "seg" : "off",
                         ivt_old, ivt_new, (unsigned)a0);
            }
        }
    }
}

/**
 * Write a dword to memory (little-endian)
 */
void mem_write_dword(uint32_t addr, uint32_t val)
{
    if (!mem) return;
    uint32_t a0 = mem_mask_addr(addr);
    uint32_t a1 = mem_mask_addr(addr + 1);
    uint32_t a2 = mem_mask_addr(addr + 2);
    uint32_t a3 = mem_mask_addr(addr + 3);
    
    // Check if writing to video memory
    if ((a0 >= VIDEO_RAM_START && a0 < VIDEO_RAM_START + VIDEO_RAM_SIZE) ||
        (a1 >= VIDEO_RAM_START && a1 < VIDEO_RAM_START + VIDEO_RAM_SIZE) ||
        (a2 >= VIDEO_RAM_START && a2 < VIDEO_RAM_START + VIDEO_RAM_SIZE) ||
        (a3 >= VIDEO_RAM_START && a3 < VIDEO_RAM_START + VIDEO_RAM_SIZE)) {
        video_dirty = true;
    }
    
    mem[a0] = val & 0xFF;
    mem[a1] = (val >> 8) & 0xFF;
    mem[a2] = (val >> 16) & 0xFF;
    mem[a3] = (val >> 24) & 0xFF;
}

/**
 * Read a block of memory
 */
void mem_read_block(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!mem || !buf) return;
    
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = mem[mem_mask_addr(addr + i)];
    }
}

/**
 * Write a block of memory
 */
void mem_write_block(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!mem || !buf) return;
    
    for (uint32_t i = 0; i < len; i++) {
        uint32_t a = mem_mask_addr(addr + i);
        if (a >= VIDEO_RAM_START && a < VIDEO_RAM_START + VIDEO_RAM_SIZE) {
            video_dirty = true;
        }
        mem[a] = buf[i];
    }
}

/**
 * Get direct pointer to memory - IRAM for speed
 */
uint8_t * mem_get_ptr(uint32_t addr)
{
    if (!mem) return NULL;
    return &mem[mem_mask_addr(addr)];
}

void mem_set_a20(bool enabled)
{
    mem_a20_enabled = enabled;
}

bool mem_get_a20(void)
{
    return mem_a20_enabled;
}

/**
 * Load BIOS into memory
 */
bool mem_load_bios(const uint8_t *bios_data, uint32_t size)
{
    if (!mem || !bios_data) return false;
    
    // BIOS loads at F000:0100 (linear address 0xF0100)
    if (size > ROM_BIOS_SIZE - 0x100) {
        size = ROM_BIOS_SIZE - 0x100;
    }
    
    ESP_LOGI(TAG, "Loading BIOS (%lu bytes) at 0xF0100", size);
    memcpy(mem + ROM_BIOS_START + 0x100, bios_data, size);
    
    return true;
}

/**
 * Check if video memory changed
 */
bool mem_is_video_dirty(void)
{
    return video_dirty;
}

/**
 * Clear video dirty flag
 */
void mem_clear_video_dirty(void)
{
    video_dirty = false;
}

/**
 * Set video dirty flag
 */
void mem_set_video_dirty(uint32_t addr, bool dirty)
{
    if (dirty) {
        video_dirty = true;
    }
}

/**
 * Get video memory pointer
 */
uint8_t *mem_get_video_ptr(void)
{
    if (!mem) return NULL;
    return &mem[TEXT_VIDEO_START];
}
