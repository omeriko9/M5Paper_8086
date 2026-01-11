/**
 * @file xms.c
 * @brief XMS (Extended Memory) emulation
 */

#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "xms.h"
#include "memory.h"

static const char *TAG = "XMS";

#define XMS_ENTRY_SEG 0xE000
#define XMS_ENTRY_OFF 0x0000

#define XMS_MAX_HANDLES 32

typedef struct {
    bool in_use;
    uint32_t offset;
    uint32_t size_bytes;
} xms_handle_t;

static uint8_t *xms_pool = NULL;
static uint32_t xms_pool_size_bytes = 0;
static xms_handle_t xms_handles[XMS_MAX_HANDLES];
static bool xms_hma_allocated = false;

static uint16_t xms_free_handle_count(void)
{
    uint16_t free = 0;
    for (int i = 0; i < XMS_MAX_HANDLES; i++) {
        if (!xms_handles[i].in_use) {
            free++;
        }
    }
    return free;
}

static void xms_calc_free_kb(uint16_t *largest_kb, uint16_t *total_kb)
{
    uint32_t pool_bytes = xms_pool_size_bytes;
    uint32_t pos = 0;
    uint32_t total = 0;
    uint32_t largest = 0;

    while (pos < pool_bytes) {
        int next = -1;
        uint32_t next_off = pool_bytes;
        for (int i = 0; i < XMS_MAX_HANDLES; i++) {
            if (!xms_handles[i].in_use) {
                continue;
            }
            if (xms_handles[i].offset >= pos && xms_handles[i].offset < next_off) {
                next_off = xms_handles[i].offset;
                next = i;
            }
        }

        if (next < 0) {
            uint32_t gap = pool_bytes - pos;
            total += gap;
            if (gap > largest) {
                largest = gap;
            }
            break;
        }

        if (next_off > pos) {
            uint32_t gap = next_off - pos;
            total += gap;
            if (gap > largest) {
                largest = gap;
            }
        }

        pos = xms_handles[next].offset + xms_handles[next].size_bytes;
    }

    if (largest_kb) {
        *largest_kb = (uint16_t)(largest / 1024);
    }
    if (total_kb) {
        *total_kb = (uint16_t)(total / 1024);
    }
}

static int xms_find_handle_slot(void)
{
    for (int i = 0; i < XMS_MAX_HANDLES; i++) {
        if (!xms_handles[i].in_use) {
            return i;
        }
    }
    return -1;
}

static bool xms_alloc_block(uint16_t size_kb, uint16_t *out_handle)
{
    if (!xms_pool || size_kb == 0) {
        return false;
    }

    uint32_t size_bytes = (uint32_t)size_kb * 1024u;
    if (size_bytes > xms_pool_size_bytes) {
        return false;
    }

    int slot = xms_find_handle_slot();
    if (slot < 0) {
        return false;
    }

    uint32_t pool_bytes = xms_pool_size_bytes;
    uint32_t pos = 0;

    while (pos <= pool_bytes) {
        int next = -1;
        uint32_t next_off = pool_bytes;
        for (int i = 0; i < XMS_MAX_HANDLES; i++) {
            if (!xms_handles[i].in_use) {
                continue;
            }
            if (xms_handles[i].offset >= pos && xms_handles[i].offset < next_off) {
                next_off = xms_handles[i].offset;
                next = i;
            }
        }

        if (next < 0) {
            if (pool_bytes - pos >= size_bytes) {
                xms_handles[slot].in_use = true;
                xms_handles[slot].offset = pos;
                xms_handles[slot].size_bytes = size_bytes;
                if (out_handle) {
                    *out_handle = (uint16_t)(slot + 1);
                }
                return true;
            }
            return false;
        }

        if (next_off >= pos + size_bytes) {
            xms_handles[slot].in_use = true;
            xms_handles[slot].offset = pos;
            xms_handles[slot].size_bytes = size_bytes;
            if (out_handle) {
                *out_handle = (uint16_t)(slot + 1);
            }
            return true;
        }

        pos = xms_handles[next].offset + xms_handles[next].size_bytes;
    }

    return false;
}

static bool xms_get_handle(uint16_t handle, xms_handle_t **out)
{
    if (handle == 0 || handle > XMS_MAX_HANDLES) {
        return false;
    }
    xms_handle_t *h = &xms_handles[handle - 1];
    if (!h->in_use) {
        return false;
    }
    if (out) {
        *out = h;
    }
    return true;
}

static void xms_set_error(cpu8086_t *cpu, uint8_t error)
{
    cpu->regs16[REG_AX] = 0x0000;
    cpu->regs16[REG_BX] = (cpu->regs16[REG_BX] & 0xFF00) | error;
    cpu->flags |= FLAG_CF;
}

static void xms_set_success(cpu8086_t *cpu, uint16_t ax)
{
    cpu->regs16[REG_AX] = ax;
    cpu->regs16[REG_BX] &= 0xFF00;
    cpu->flags &= (uint16_t)~FLAG_CF;
}

static bool xms_get_conv_ptr(uint32_t linear, uint32_t length, uint8_t **out)
{
    if (linear >= MEM_SIZE || length > (MEM_SIZE - linear)) {
        return false;
    }
    if (out) {
        *out = mem_get_ptr(linear);
    }
    return true;
}

static bool xms_move_block(cpu8086_t *cpu)
{
    const uint32_t desc = ((uint32_t)cpu->sregs[SEG_DS] << 4) + cpu->regs16[REG_SI];
    const uint32_t length = mem_read_dword(desc);
    const uint16_t src_handle = mem_read_word(desc + 4);
    const uint32_t src_off = mem_read_dword(desc + 6);
    const uint16_t dst_handle = mem_read_word(desc + 10);
    const uint32_t dst_off = mem_read_dword(desc + 12);

    if (length == 0) {
        return true;
    }

    uint8_t *src_ptr = NULL;
    uint8_t *dst_ptr = NULL;

    if (src_handle == 0) {
        if (!xms_get_conv_ptr(src_off, length, &src_ptr)) {
            return false;
        }
    } else {
        xms_handle_t *h = NULL;
        if (!xms_get_handle(src_handle, &h)) {
            return false;
        }
        if (src_off + length > h->size_bytes) {
            return false;
        }
        src_ptr = xms_pool + h->offset + src_off;
    }

    if (dst_handle == 0) {
        if (!xms_get_conv_ptr(dst_off, length, &dst_ptr)) {
            return false;
        }
    } else {
        xms_handle_t *h = NULL;
        if (!xms_get_handle(dst_handle, &h)) {
            return false;
        }
        if (dst_off + length > h->size_bytes) {
            return false;
        }
        dst_ptr = xms_pool + h->offset + dst_off;
    }

    if (src_ptr && dst_ptr) {
        memmove(dst_ptr, src_ptr, length);
        return true;
    }

    return false;
}

static void xms_retf(cpu8086_t *cpu)
{
    uint32_t sp_addr = ((uint32_t)cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
    uint16_t ret_ip = mem_read_word(sp_addr);
    cpu->regs16[REG_SP] += 2;
    sp_addr = ((uint32_t)cpu->sregs[SEG_SS] << 4) + cpu->regs16[REG_SP];
    uint16_t ret_cs = mem_read_word(sp_addr);
    cpu->regs16[REG_SP] += 2;
    cpu->ip = ret_ip;
    cpu->sregs[SEG_CS] = ret_cs;
}

bool xms_init(void)
{
    static const uint32_t sizes[] = {
        4 * 1024 * 1024,
        2 * 1024 * 1024,
        1 * 1024 * 1024,
        512 * 1024,
        256 * 1024,
    };

    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
        uint8_t *buf = (uint8_t *)heap_caps_malloc(sizes[i], MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf) {
            xms_pool = buf;
            xms_pool_size_bytes = sizes[i];
            memset(xms_handles, 0, sizeof(xms_handles));
            ESP_LOGI(TAG, "XMS pool=%lu KB", (unsigned long)(xms_pool_size_bytes / 1024));
            return true;
        }
    }

    ESP_LOGW(TAG, "XMS disabled (no PSRAM pool)");
    return false;
}

bool xms_present(void)
{
    return xms_pool != NULL && xms_pool_size_bytes >= 1024;
}

void xms_get_entry(uint16_t *seg, uint16_t *off)
{
    if (seg) {
        *seg = XMS_ENTRY_SEG;
    }
    if (off) {
        *off = XMS_ENTRY_OFF;
    }
}

bool xms_is_entry(uint16_t seg, uint16_t off)
{
    return xms_present() && seg == XMS_ENTRY_SEG && off == XMS_ENTRY_OFF;
}

void xms_handle_call(cpu8086_t *cpu)
{
    if (!xms_present()) {
        xms_set_error(cpu, 0x80);
        xms_retf(cpu);
        return;
    }

    const uint8_t ah = (uint8_t)(cpu->regs16[REG_AX] >> 8);
    ESP_LOGI(TAG, "XMS call AH=%02X AX=%04X BX=%04X CX=%04X DX=%04X DS:SI=%04X:%04X",
             ah,
             cpu->regs16[REG_AX],
             cpu->regs16[REG_BX],
             cpu->regs16[REG_CX],
             cpu->regs16[REG_DX],
             cpu->sregs[SEG_DS],
             cpu->regs16[REG_SI]);

    switch (ah) {
        case 0x00:  // Get XMS version
            cpu->regs16[REG_AX] = 0x0200;
            cpu->regs16[REG_BX] = 0x0200;
            cpu->regs16[REG_DX] = 0x0000;
            cpu->flags &= (uint16_t)~FLAG_CF;
            break;
        case 0x01:  // Request HMA
            if (!xms_hma_allocated) {
                xms_hma_allocated = true;
                mem_set_a20(true);
                xms_set_success(cpu, 0x0001);
            } else {
                xms_set_error(cpu, 0x91);
            }
            break;
        case 0x02:  // Release HMA
            xms_hma_allocated = false;
            xms_set_success(cpu, 0x0001);
            break;
        case 0x03:  // Global enable A20
            mem_set_a20(true);
            xms_set_success(cpu, 0x0001);
            break;
        case 0x04:  // Global disable A20
            mem_set_a20(false);
            xms_set_success(cpu, 0x0001);
            break;
        case 0x05:  // Local enable A20
            mem_set_a20(true);
            xms_set_success(cpu, 0x0001);
            break;
        case 0x06:  // Local disable A20
            mem_set_a20(false);
            xms_set_success(cpu, 0x0001);
            break;
        case 0x07:  // Query A20 state
            cpu->regs16[REG_AX] = mem_get_a20() ? 0x0001 : 0x0000;
            cpu->regs16[REG_BX] &= 0xFF00;
            cpu->flags &= (uint16_t)~FLAG_CF;
            break;
        case 0x08: { // Query free extended memory
            uint16_t largest = 0;
            uint16_t total = 0;
            xms_calc_free_kb(&largest, &total);
            cpu->regs16[REG_AX] = largest;
            cpu->regs16[REG_DX] = total;
            cpu->flags &= (uint16_t)~FLAG_CF;
            break;
        }
        case 0x09: { // Allocate extended memory block
            uint16_t size_kb = cpu->regs16[REG_DX];
            uint16_t handle = 0;
            if (xms_alloc_block(size_kb, &handle)) {
                cpu->regs16[REG_DX] = handle;
                xms_set_success(cpu, 0x0001);
            } else if (xms_free_handle_count() == 0) {
                xms_set_error(cpu, 0xA1);
            } else {
                xms_set_error(cpu, 0xA0);
            }
            break;
        }
        case 0x0A: { // Free extended memory block
            uint16_t handle = cpu->regs16[REG_DX];
            xms_handle_t *h = NULL;
            if (!xms_get_handle(handle, &h)) {
                xms_set_error(cpu, 0xA2);
                break;
            }
            h->in_use = false;
            xms_set_success(cpu, 0x0001);
            break;
        }
        case 0x0B:  // Move extended memory block
            if (xms_move_block(cpu)) {
                xms_set_success(cpu, 0x0001);
            } else {
                xms_set_error(cpu, 0xA3);
            }
            break;
        case 0x0E: { // Get handle information
            uint16_t handle = cpu->regs16[REG_DX];
            xms_handle_t *h = NULL;
            if (!xms_get_handle(handle, &h)) {
                xms_set_error(cpu, 0xA2);
                break;
            }
            cpu->regs16[REG_DX] = (uint16_t)(h->size_bytes / 1024);
            cpu->regs16[REG_BX] = (uint16_t)((xms_free_handle_count() & 0xFF) | 0x0000);
            cpu->flags &= (uint16_t)~FLAG_CF;
            cpu->regs16[REG_AX] = 0x0001;
            break;
        }
        case 0x0F: { // Reallocate extended memory block
            uint16_t handle = cpu->regs16[REG_DX];
            uint16_t new_kb = cpu->regs16[REG_BX];
            xms_handle_t *h = NULL;
            if (!xms_get_handle(handle, &h)) {
                xms_set_error(cpu, 0xA2);
                break;
            }
            uint32_t new_bytes = (uint32_t)new_kb * 1024u;
            if (new_bytes <= h->size_bytes) {
                h->size_bytes = new_bytes;
                xms_set_success(cpu, 0x0001);
                break;
            }
            uint32_t next_off = xms_pool_size_bytes;
            for (int i = 0; i < XMS_MAX_HANDLES; i++) {
                if (!xms_handles[i].in_use || &xms_handles[i] == h) {
                    continue;
                }
                if (xms_handles[i].offset >= h->offset + h->size_bytes &&
                    xms_handles[i].offset < next_off) {
                    next_off = xms_handles[i].offset;
                }
            }
            if (h->offset + new_bytes <= next_off) {
                h->size_bytes = new_bytes;
                xms_set_success(cpu, 0x0001);
            } else {
                xms_set_error(cpu, 0xA0);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandled XMS function AH=%02X", ah);
            xms_set_error(cpu, 0x80);
            break;
    }

    ESP_LOGI(TAG, "XMS ret CF=%d AX=%04X BX=%04X DX=%04X",
             (cpu->flags & FLAG_CF) ? 1 : 0,
             cpu->regs16[REG_AX],
             cpu->regs16[REG_BX],
             cpu->regs16[REG_DX]);
    xms_retf(cpu);
}
