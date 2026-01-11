/**
 * @file memory.h
 * @brief Memory management for 8086 emulator
 */

#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Memory regions
#define MEM_SIZE            (1024 * 1024)   // 1MB total
#define ROM_BIOS_START      0xF0000         // BIOS ROM start
#define ROM_BIOS_SIZE       0x10000         // 64KB BIOS
#define VIDEO_RAM_START     0xA0000         // Video RAM start
#define VIDEO_RAM_SIZE      0x20000         // 128KB video RAM
#define CONV_MEM_END        0xA0000         // End of conventional memory (640KB)

// Video memory specific
#define VIDEO_BASE          0xB8000         // CGA/VGA text mode base
#define TEXT_VIDEO_START    0xB8000         // CGA/VGA text mode
#define TEXT_VIDEO_SIZE     0x8000          // 32KB text video
#define MDA_VIDEO_START     0xB0000         // MDA text mode
#define MDA_VIDEO_SIZE      0x8000          // 32KB MDA video

// BIOS Data Area
#define BDA_BASE            0x0400
#define BDA_VIDEO_MODE      0x0449
#define BDA_VIDEO_COLS      0x044A
#define BDA_VIDEO_PAGE_SIZE 0x044C
#define BDA_VIDEO_PAGE_OFF  0x044E
#define BDA_CURSOR_POS      0x0450
#define BDA_CURSOR_TYPE     0x0460
#define BDA_ACTIVE_PAGE     0x0462
#define BDA_CRT_PORT        0x0463
#define BDA_TIMER_COUNT     0x046C
#define BDA_VIDEO_ROWS      0x0484
#define BDA_CHAR_HEIGHT     0x0485

// Initialize memory system
void dos_mem_init(void);

// Free memory system
void dos_mem_free(void);

// Read operations
uint8_t mem_read_byte(uint32_t addr);
uint16_t mem_read_word(uint32_t addr);
uint32_t mem_read_dword(uint32_t addr);

// Write operations
void mem_write_byte(uint32_t addr, uint8_t val);
void mem_write_word(uint32_t addr, uint16_t val);
void mem_write_dword(uint32_t addr, uint32_t val);

// Bulk operations
void mem_read_block(uint32_t addr, uint8_t *buf, uint32_t len);
void mem_write_block(uint32_t addr, const uint8_t *buf, uint32_t len);

// Get direct pointer to memory (for video, DMA, etc.)
uint8_t *mem_get_ptr(uint32_t addr);

// A20 gate control
void mem_set_a20(bool enabled);
bool mem_get_a20(void);

// Debug: set current CPU context for memory write tracing
void mem_set_cpu_context(uint16_t cs, uint16_t ip, uint8_t op, uint8_t b1, uint8_t b2);

// Load BIOS into memory
bool mem_load_bios(const uint8_t *bios_data, uint32_t size);

// Video dirty tracking
bool mem_is_video_dirty(void);
void mem_clear_video_dirty(void);
void mem_set_video_dirty(uint32_t addr, bool dirty);

// Get video memory base for current mode
uint8_t *mem_get_video_ptr(void);

#ifdef __cplusplus
}
#endif

#endif // MEMORY_H
