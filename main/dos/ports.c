/**
 * @file ports.c
 * @brief I/O Port emulation for DOS
 * 
 * Emulates PC I/O ports (PIC, PIT, keyboard controller, etc.)
 * Includes PC speaker emulation via PIT channel 2 and port 0x61.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "ports.h"
#include "memory.h"
#include "bios.h"
#include "interrupts.h"
#include "video.h"
#include "speaker.h"
#include "config_defs.h"
#include "settings.h"

static const char *TAG = "PORTS";
static const char *PORT_TRACE_TAG = "PORT";
static const char *KBD_TAG = "KBD";
static const char *SPK_TAG = "SPK";

// Trace logging macros
#define TRACE_LOGI(...) ESP_LOGI(PORT_TRACE_TAG, __VA_ARGS__)
#define TRACE_LOGW(...) ESP_LOGW(PORT_TRACE_TAG, __VA_ARGS__)

// Only trace specific interesting ports to avoid flooding
static bool should_trace_io(uint16_t port) {
    return (port == 0x60 || port == 0x64 || 
            (port >= 0x3F8 && port <= 0x3FF) || 
            (port >= 0x2F8 && port <= 0x2FF) ||
            port == 0x20 || port == 0x21 || port == 0xA0 || port == 0xA1);
}

// Port 0x61 state (keyboard controller output port / speaker control)
static uint8_t port61_state = 0x00;

// CGA registers (very simplified)
static uint8_t cga_mode_control = 0x29; // 80x25 text, video enabled (approx)
static uint8_t cga_color_select = 0x00;
static uint8_t cga_status = 0x00;

// Gameport (joystick) 0x201 (very simplified, time-based axis discharge)
static struct {
    uint64_t start_us;
    uint32_t axis_us[4];     // X1,Y1,X2,Y2 discharge durations
    uint8_t buttons_mask;    // bits 0-3, 1=not pressed
} gameport = {
    .start_us = 0,
    .axis_us = { 2000, 2000, 2000, 2000 },
    .buttons_mask = 0x0F,
};

// PIC (8259) state
static struct {
    uint8_t imr;        // Interrupt Mask Register
    uint8_t irr;        // Interrupt Request Register
    uint8_t isr;        // In-Service Register
    uint8_t icw1;
    uint8_t icw2;
    uint8_t icw3;
    uint8_t icw4;
    uint8_t init_state; // 0=ready, 1-4=waiting for ICWx
    uint8_t vector_base;
} pic1, pic2;

// PIT (8254) state
static struct {
    uint16_t counter[3];
    uint16_t reload[3];
    uint8_t control[3];
    uint8_t mode[3];
    uint16_t latch[3];
    bool latched[3];
    uint8_t read_state[3];  // 0=LSB, 1=MSB
    uint8_t write_state[3];
    uint32_t phase[3];
    bool output[3];
} pit;

#define PIT_FREQUENCY 1193182

static uint64_t pit_last_update_us = 0;
static uint64_t pit_tick_accum = 0;

static uint8_t cga_derive_video_mode(uint8_t mode_control)
{
    // CGA Mode Control (port 0x3D8) bits (approx):
    // - Bit 1: graphics/text select (1=graphics, 0=text)
    // - Bit 2: BW select (1=BW, 0=color) in 320x200 graphics
    const bool graphics = (mode_control & 0x02) != 0;
    const bool bw = (mode_control & 0x04) != 0;

    if (!graphics) {
        // Only 80x25 color text mode is currently rendered correctly.
        return 0x03;
    }

    return bw ? 0x05 : 0x04;
}

static void cga_apply_mode_control(uint8_t mode_control)
{
    const uint8_t derived = cga_derive_video_mode(mode_control);
    const uint8_t current = (uint8_t)(video_get_mode() & 0x7F);
    if (derived != current) {
        // Direct CGA register programming should not clear VRAM; preserve contents.
        video_set_mode((uint8_t)(derived | 0x80));
    }
}

static void pit_sync(void)
{
    uint64_t now = esp_timer_get_time();
    if (pit_last_update_us == 0) {
        pit_last_update_us = now;
        return;
    }

    uint64_t delta_us = now - pit_last_update_us;
    if (delta_us == 0) {
        return;
    }
    pit_last_update_us = now;

    uint64_t total_val = (delta_us * 1193182ULL) + pit_tick_accum;
    uint64_t ticks = total_val / 1000000ULL;
    pit_tick_accum = total_val % 1000000ULL;

    if (ticks == 0) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        // Gate disabled for Channel 2: timer stops, output stays high.
        if (i == 2 && (port61_state & 0x01) == 0) {
            pit.output[2] = true;
            continue;
        }

        uint32_t reload = pit.reload[i];
        if (reload == 0) reload = 65536;

        uint32_t phase = pit.phase[i];
        uint64_t channel_ticks = ticks;
        
        uint64_t new_phase_long = phase + channel_ticks;
        uint64_t wraps = new_phase_long / reload;
        phase = (uint32_t)(new_phase_long % reload);

        pit.phase[i] = phase;
        pit.counter[i] = (uint16_t)((reload - phase) & 0xFFFF);

        // IRQ0 generation (System Timer)
        if (i == 0 && wraps > 0) {
            port_irq(0);
        }

        uint8_t mode = pit.mode[i];
        if (mode == 2) {
            pit.output[i] = (phase != (reload - 1));
        } else if (mode == 3) {
            uint32_t high_ticks = (reload + 1U) / 2U;
            pit.output[i] = (phase < high_ticks);
        } else {
            pit.output[i] = true;
        }
    }
}

// Keyboard controller (8042) state
static struct {
    uint8_t status;
    uint8_t output_buffer;
    uint8_t input_buffer;
    uint8_t command;
    bool output_full;
    uint8_t ccb;          // Controller Command Byte
    bool expecting_ccb;   // Waiting for CCB write (command 0x60)
    bool expecting_mouse_byte; // Waiting for mouse command byte (command 0xD4)
    
    // PS/2 Mouse state
    bool mouse_enabled;
    uint8_t mouse_resolution;
    uint8_t mouse_sample_rate;
    bool mouse_scaling_1_2; // 2:1 scaling
    bool mouse_stream_mode;
    uint8_t mouse_packet[3]; // 3-byte packet buffer
    uint8_t mouse_packet_idx;
} kbc;

// Keyboard controller output queue (scancodes / mouse data)
// High byte: 0x00 = Keyboard, 0x01 = Mouse (AUX)
#define KBC_QUEUE_SIZE 64
static uint16_t kbc_queue[KBC_QUEUE_SIZE];
static uint8_t kbc_head = 0;
static uint8_t kbc_tail = 0;
static bool kb_scanning_enabled = true;
static uint8_t kb_pending_cmd = 0;
static portMUX_TYPE kbc_mux = portMUX_INITIALIZER_UNLOCKED;

// UART 8250 (minimal) for COM1. Used for Microsoft serial mouse emulation.
#define COM1_RX_QUEUE_SIZE 128
static struct {
    uint8_t ier;   // Interrupt Enable Register
    uint8_t lcr;   // Line Control Register
    uint8_t mcr;   // Modem Control Register
    uint8_t scr;   // Scratch Register
    uint8_t fcr;   // FIFO Control Register (write-only)
    uint16_t divisor; // Divisor latch (DLL/DLM)

    uint8_t rx[COM1_RX_QUEUE_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;

    bool mouse_id_sent;
    uint32_t mouse_probe_polls;
} com1;
static portMUX_TYPE com1_mux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t com1_rx_count_nolock(void)
{
    if (com1.rx_head >= com1.rx_tail) {
        return (uint16_t)(com1.rx_head - com1.rx_tail);
    }
    return (uint16_t)(COM1_RX_QUEUE_SIZE - (com1.rx_tail - com1.rx_head));
}

static bool com1_rx_push_byte_nolock(uint8_t b)
{
    const uint16_t next = (uint16_t)((com1.rx_head + 1u) % COM1_RX_QUEUE_SIZE);
    if (next == com1.rx_tail) {
        return false;
    }
    com1.rx[com1.rx_head] = b;
    com1.rx_head = next;
    return true;
}

static bool com1_rx_pop_byte_nolock(uint8_t *out)
{
    if (com1.rx_head == com1.rx_tail) {
        return false;
    }
    if (out) {
        *out = com1.rx[com1.rx_tail];
    }
    com1.rx_tail = (uint16_t)((com1.rx_tail + 1u) % COM1_RX_QUEUE_SIZE);
    return true;
}

static void com1_raise_irq_if_needed_nolock(void)
{
    const bool rx_ready = (com1.rx_head != com1.rx_tail);
    const bool rx_irq_enabled = (com1.ier & 0x01) != 0;
    const bool out2_enabled = (com1.mcr & 0x08) != 0;
    if (rx_ready && rx_irq_enabled && out2_enabled) {
        port_irq(IRQ_COM1);
    }
}

static void com1_send_mouse_id_nolock(void)
{
    if (com1.mouse_id_sent) {
        return;
    }
    // Keep the ID sequence separate from movement packets.
    if (com1.rx_head != com1.rx_tail) {
        return;
    }

    // Microsoft-compatible serial mouse ID.
    // Common DOS drivers accept 'M' (2-button). 'M3' causes issues with some drivers.
    if (!com1_rx_push_byte_nolock('M')) {
        return;
    }
    // (void)com1_rx_push_byte_nolock('3'); // Commented out to improve compatibility
    com1.mouse_id_sent = true;
    com1_raise_irq_if_needed_nolock();
}

static void com1_maybe_send_mouse_id_nolock(void)
{
    // Many DOS mouse drivers (including MS MOUSE.COM) probe by asserting both DTR+RTS
    // and then polling for the ID byte ('M').
    const bool dtr_and_rts = (com1.mcr & 0x03) == 0x03;
    if (dtr_and_rts && !com1.mouse_id_sent) {
        com1_send_mouse_id_nolock();
    }
}

static void com1_maybe_send_mouse_id_on_poll_nolock(void)
{
    if (com1.mouse_id_sent) {
        return;
    }
    // Avoid injecting an ID unless the port is actually "opened" (DTR+RTS asserted),
    // otherwise some probe sequences will consume it too early and then fail later.
    if ((com1.mcr & 0x03) != 0x03) {
        return;
    }
    // Some drivers poll LSR for a long time before (or without) touching MCR.
    // After enough polls with an empty RX queue, provide the standard 'M' ID byte.
    if (com1.mouse_probe_polls < 8) {
        return;
    }
    if (com1.rx_head != com1.rx_tail) {
        return;
    }
    com1_send_mouse_id_nolock();
}

static uint8_t com1_read_msr_nolock(void)
{
    // Modem Status Register bits:
    // 7 DCD, 6 RI, 5 DSR, 4 CTS, 3-0 delta bits.
    // In loopback mode (MCR bit4), bits 4-7 reflect MCR bits 0-3 (per 8250 behavior).
    if (com1.mcr & 0x10) {
        uint8_t msr = 0x00;
        if (com1.mcr & 0x02) msr |= 0x10; // RTS -> CTS
        if (com1.mcr & 0x01) msr |= 0x20; // DTR -> DSR
        if (com1.mcr & 0x04) msr |= 0x40; // OUT1 -> RI
        if (com1.mcr & 0x08) msr |= 0x80; // OUT2 -> DCD
        return msr;
    }

    // Default: report modem inputs asserted (mouse present).
    return 0xB0; // CTS|DSR|DCD
}

// Minimal ATA/ATAPI I/O stub.
// Some DOS CD-ROM drivers can busy-wait forever if an IDE status port reads
// as 0xFF (BSY set). Provide deterministic "no device" behavior that never
// reports BSY, and that completes any polled DRQ transfer.
typedef struct {
    uint8_t status;                 // Status / AltStatus
    uint8_t error;                  // Error register
    uint16_t data_words_remaining;  // Remaining words for a polled DATA transfer
    uint8_t data_byte_phase;        // 0=low byte next, 1=high byte next (byte I/O)
    uint8_t packet_words_expected;  // Remaining words for an ATAPI PACKET write
    uint8_t packet_byte_phase;      // 0=low byte next, 1=high byte next (byte I/O)
} ata_chan_t;

static ata_chan_t ata_ch[4];
static const uint16_t ata_base[4] = { 0x1F0, 0x170, 0x1E8, 0x168 };
static const uint16_t ata_ctrl[4] = { 0x3F6, 0x376, 0x3EE, 0x36E };

#define PCI_CFG_ADDR_PORT 0x0CF8
#define PCI_CFG_DATA_PORT 0x0CFC

static uint32_t pci_cfg_addr = 0;

static uint32_t pci_cfg_read_dword(uint32_t addr)
{
    // Always report "no device present" (Vendor ID 0xFFFF).
    // Many DOS probes treat 0xFFFFFFFF as "no PCI device" and fall back to legacy I/O.
    (void)addr;
    return 0xFFFFFFFFu;
}

static bool pci_is_cfg_port(uint16_t port)
{
    return port >= PCI_CFG_ADDR_PORT && port <= (uint16_t)(PCI_CFG_DATA_PORT + 3u);
}

static bool pci_is_cfg_port_word(uint16_t port)
{
    return port == PCI_CFG_ADDR_PORT || port == (uint16_t)(PCI_CFG_ADDR_PORT + 2u) ||
           port == PCI_CFG_DATA_PORT || port == (uint16_t)(PCI_CFG_DATA_PORT + 2u);
}

static bool pci_port_in(uint16_t port, uint8_t *out)
{
    if (!pci_is_cfg_port(port)) {
        return false;
    }

    const uint16_t off = (uint16_t)(port - PCI_CFG_ADDR_PORT);
    uint32_t v = 0xFFFFFFFFu;
    if (off < 4) {
        v = pci_cfg_addr;
    } else {
        v = pci_cfg_read_dword(pci_cfg_addr);
    }

    if (out) {
        *out = (uint8_t)((v >> (8u * (off & 3u))) & 0xFFu);
    }

    ESP_LOGD(PORT_TRACE_TAG, "PCI IN port=%04X addr=%08lX -> %02X",
                 (unsigned)port, (unsigned long)pci_cfg_addr, out ? (unsigned)*out : 0u);

    return true;
}

static bool pci_port_out(uint16_t port, uint8_t value)
{
    if (!pci_is_cfg_port(port)) {
        return false;
    }

    const uint16_t off = (uint16_t)(port - PCI_CFG_ADDR_PORT);
    if (off < 4) {
        const uint32_t shift = 8u * (uint32_t)(off & 3u);
        pci_cfg_addr = (pci_cfg_addr & ~(0xFFu << shift)) | ((uint32_t)value << shift);
    }

    ESP_LOGD(PORT_TRACE_TAG, "PCI OUT port=%04X val=%02X addr=%08lX",
                 (unsigned)port, (unsigned)value, (unsigned long)pci_cfg_addr);

    return true;
}

static void ata_reset_chan(ata_chan_t *ch)
{
    if (!ch) return;
    ch->status = 0x00; // "no device" and never busy
    ch->error = 0x00;
    ch->data_words_remaining = 0;
    ch->data_byte_phase = 0;
    ch->packet_words_expected = 0;
    ch->packet_byte_phase = 0;
}

static int ata_decode_port(uint16_t port, uint8_t *out_reg)
{
    for (int i = 0; i < 4; i++) {
        if (port >= ata_base[i] && port <= (uint16_t)(ata_base[i] + 7u)) {
            if (out_reg) *out_reg = (uint8_t)(port - ata_base[i]);
            return i;
        }
        if (port == ata_ctrl[i]) {
            if (out_reg) *out_reg = 8; // AltStatus / DeviceControl
            return i;
        }
        if (port == (uint16_t)(ata_ctrl[i] + 1u)) {
            if (out_reg) *out_reg = 9; // Drive Address (read-only)
            return i;
        }
    }
    return -1;
}

static bool ata_is_data_port(uint16_t port, int *out_chan)
{
    for (int i = 0; i < 4; i++) {
        if (port == ata_base[i]) {
            if (out_chan) *out_chan = i;
            return true;
        }
    }
    return false;
}

static void ata_finish_data_if_done(ata_chan_t *ch)
{
    if (!ch) return;
    if (ch->data_words_remaining == 0) {
        ch->status = 0x41; // DRDY|ERR (abort)
        ch->error = 0x04;  // ABRT
        ch->data_byte_phase = 0;
    }
}

static uint16_t ata_data_read_word(ata_chan_t *ch)
{
    if (!ch) return 0;
    if (ch->data_words_remaining) {
        ch->data_words_remaining--;
        ata_finish_data_if_done(ch);
    }
    return 0;
}

static void ata_data_write_word(ata_chan_t *ch, uint16_t value)
{
    (void)value;
    if (!ch) return;
    if (ch->packet_words_expected) {
        ch->packet_words_expected--;
        if (ch->packet_words_expected == 0) {
            ch->status = 0x41; // DRDY|ERR (abort)
            ch->error = 0x04;
            ch->packet_byte_phase = 0;
        }
    }
}

static void ata_handle_command(ata_chan_t *ch, uint8_t cmd)
{
    if (!ch) return;

    ch->data_words_remaining = 0;
    ch->packet_words_expected = 0;
    ch->data_byte_phase = 0;
    ch->packet_byte_phase = 0;

    switch (cmd) {
        case 0xEC: // IDENTIFY DEVICE
        case 0xA1: // IDENTIFY PACKET DEVICE
            ch->status = 0x48;               // DRDY|DRQ (polled transfer)
            ch->error = 0x00;
            ch->data_words_remaining = 256;  // 512 bytes
            break;
        case 0xA0: // PACKET (ATAPI)
            ch->status = 0x48;               // DRDY|DRQ (ready to accept packet)
            ch->error = 0x00;
            ch->packet_words_expected = 6;   // 12-byte packet
            break;
        default:
            ch->status = 0x41; // DRDY|ERR (abort)
            ch->error = 0x04;  // ABRT
            break;
    }
}

static bool ata_port_in(uint16_t port, uint8_t *out)
{
    uint8_t reg = 0;
    const int ch_i = ata_decode_port(port, &reg);
    if (ch_i < 0) return false;

    ata_chan_t *ch = &ata_ch[ch_i];
    ESP_LOGD(PORT_TRACE_TAG, "ATA IN ch=%d port=%04X reg=%u", ch_i, (unsigned)port, (unsigned)reg);
    switch (reg) {
        case 0: // DATA (byte)
            if (out) {
                if (ch->data_words_remaining) {
                    ch->data_byte_phase ^= 1u;
                    if (ch->data_byte_phase == 0) {
                        ch->data_words_remaining--;
                        ata_finish_data_if_done(ch);
                    }
                }
                *out = 0x00;
            }
            return true;
        case 1: // ERROR
            if (out) *out = ch->error;
            return true;
        case 7: // STATUS
        case 8: // ALTSTATUS
            ESP_LOGD(PORT_TRACE_TAG, "ATA STATUS ch=%d port=%04X => %02X (err=%02X drq_words=%u pkt_words=%u)",
                         ch_i, (unsigned)port, (unsigned)ch->status, (unsigned)ch->error,
                         (unsigned)ch->data_words_remaining, (unsigned)ch->packet_words_expected);
            if (out) *out = ch->status;
            return true;
        case 9: // Drive Address
            if (out) *out = 0x00;
            return true;
        default:
            if (out) *out = 0x00;
            return true;
    }
}

static bool ata_port_out(uint16_t port, uint8_t value)
{
    uint8_t reg = 0;
    const int ch_i = ata_decode_port(port, &reg);
    if (ch_i < 0) return false;

    ata_chan_t *ch = &ata_ch[ch_i];
    ESP_LOGD(PORT_TRACE_TAG, "ATA OUT ch=%d port=%04X reg=%u val=%02X", ch_i, (unsigned)port, (unsigned)reg, (unsigned)value);
    switch (reg) {
        case 0: // DATA (byte)
            if (ch->packet_words_expected) {
                ch->packet_byte_phase ^= 1u;
                if (ch->packet_byte_phase == 0) {
                    if (ch->packet_words_expected) {
                        ch->packet_words_expected--;
                    }
                    if (ch->packet_words_expected == 0) {
                        ch->status = 0x41; // DRDY|ERR (abort)
                        ch->error = 0x04;
                    }
                }
            }
            return true;
        case 7: // COMMAND
            ESP_LOGD(PORT_TRACE_TAG, "ATA CMD ch=%d port=%04X cmd=%02X", ch_i, (unsigned)port, (unsigned)value);
            ata_handle_command(ch, value);
            return true;
        case 8: // Device Control (write)
            // SRST (bit2) resets the channel; do not ever report BSY to avoid hangs.
            if (value & 0x04) {
                ata_reset_chan(ch);
            }
            return true;
        default:
            (void)value;
            return true;
    }
}

static bool kbc_queue_push_raw(uint16_t val)
{
    uint8_t next = (uint8_t)((kbc_head + 1) % KBC_QUEUE_SIZE);
    if (next == kbc_tail) {
        return false;
    }
    kbc_queue[kbc_head] = val;
    kbc_head = next;
    return true;
}

static bool kbc_queue_push(uint8_t scancode)
{
    return kbc_queue_push_raw((uint16_t)scancode);
}

static bool kbc_queue_pop(uint16_t *val)
{
    if (kbc_head == kbc_tail) {
        return false;
    }
    if (val) {
        *val = kbc_queue[kbc_tail];
    }
    kbc_tail = (uint8_t)((kbc_tail + 1) % KBC_QUEUE_SIZE);
    return true;
}

static uint8_t kbc_queue_free(void)
{
    uint8_t used;
    if (kbc_head >= kbc_tail) {
        used = (uint8_t)(kbc_head - kbc_tail);
    } else {
        used = (uint8_t)(KBC_QUEUE_SIZE - (kbc_tail - kbc_head));
    }
    return (uint8_t)(KBC_QUEUE_SIZE - 1 - used);
}

static bool kbc_queue_push_bytes(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) {
        return false;
    }
    if (kbc_queue_free() < len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        kbc_queue_push(data[i]);
    }
    return true;
}

static void kbc_flush_output(void)
{
    kbc.output_full = false;
    kbc.status &= (uint8_t)~0x01;
    kbc_head = 0;
    kbc_tail = 0;
}

static void kbc_load_next(bool raise_irq);
static void kbc_send_response(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) {
        return;
    }
    kbc_flush_output();
    kbc_queue_push_bytes(data, len);
    kbc_load_next(true);
}

static void kbc_send_mouse_response(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return;
    kbc_flush_output();
    for(int i=0; i<len; i++) {
        kbc_queue_push_raw(0x0100 | data[i]);
    }
    kbc_load_next(true);
}

static void kbc_load_next(bool raise_irq)
{
    uint16_t val = 0;
    if (kbc_queue_pop(&val)) {
        uint8_t data = (uint8_t)(val & 0xFF);
        bool is_aux = (val & 0x0100) != 0;
        
        kbc.output_buffer = data;
        kbc.output_full = true;
        kbc.status |= 0x01; // OBF
        
        if (is_aux) {
            kbc.status |= 0x20; // AUX_OBF
            if (raise_irq && (kbc.ccb & 0x02)) {
                port_irq(12);
            }
        } else {
            kbc.status &= ~0x20; // Clear AUX_OBF
            if (raise_irq && (kbc.ccb & 0x01)) {
                port_irq(1);
            }
        }
    }
}

// CMOS/RTC state
static struct {
    uint8_t index;
    uint8_t data[128];
} cmos;

// VGA register state (simplified)
static struct {
    uint8_t misc_output;
    uint8_t seq_index;
    uint8_t seq_data[8];
    uint8_t crtc_index;
    uint8_t crtc_data[32];
    uint8_t gc_index;
    uint8_t gc_data[16];
    uint8_t attr_index;
    uint8_t attr_data[32];
    bool attr_flip_flop;
} vga;

// DMA controller state (simplified)
static uint8_t dma_page[8];

/**
 * Initialize I/O port emulation
 */
void ports_init(void)
{
    ESP_LOGI(TAG, "Initializing I/O ports");
    
    // Initialize PIC1 (master)
    memset(&pic1, 0, sizeof(pic1));
    // Unmask timer (IRQ0), keyboard (IRQ1), Cascade (IRQ2), and COM1 (IRQ4).
    // IRQ 6 (Floppy) is masked (1).
    // 0xEC = 1110 1100 -> Bit 0,1,4=0 (Enabled). Bit 2=1 (Masked). 
    // We MUST unmask Bit 2 (Cascade) to allow IRQs 8-15 (PIC2).
    // New IMR: 0xE8 (1110 1000)
    pic1.imr = 0xE8; 
    pic1.vector_base = 0x08;
    
    // Initialize PIC2 (slave)
    memset(&pic2, 0, sizeof(pic2));
    // Unmask PS/2 Mouse (IRQ 12).
    // 0xFF = 1111 1111 -> All masked.
    // IRQ 12 is Bit 4 on PIC2. 
    // New IMR: 0xEF (1110 1111)
    pic2.imr = 0xEF;
    pic2.vector_base = 0x70;
    
    // Initialize PIT
    memset(&pit, 0, sizeof(pit));
    for (int i = 0; i < 3; i++) {
        pit.counter[i] = 0xFFFF;
        pit.reload[i] = 0xFFFF;
        pit.output[i] = true;
        pit.phase[i] = 0;
        pit.mode[i] = 0;
    }
    pit_last_update_us = esp_timer_get_time();
    pit_tick_accum = 0;
    
    // Initialize keyboard controller
    memset(&kbc, 0, sizeof(kbc));
    kbc.status = 0x14;  // Bit 4=Unlocked, Bit 2=System Flag
    kbc.ccb = 0x61;     // IRQ1 enabled, Mouse disabled, Translate
    kbc_head = 0;
    kbc_tail = 0;
    kb_scanning_enabled = true;
    kb_pending_cmd = 0;
    kbc.expecting_ccb = false;

    // Initialize COM1 UART state (minimal 8250).
    memset(&com1, 0, sizeof(com1));
    com1.lcr = 0x03;      // 8N1
    com1.divisor = 1;     // Arbitrary non-zero divisor
    com1.rx_head = 0;
    com1.rx_tail = 0;
    com1.mouse_id_sent = false;
    com1.mouse_probe_polls = 0;

    // Initialize ATA channels (stubbed "no device" behavior).
    for (int i = 0; i < 4; i++) {
        ata_reset_chan(&ata_ch[i]);
    }
    
    // Initialize port 0x61 (speaker control)
    port61_state = 0x00;
    
    // Initialize CMOS
    memset(&cmos, 0, sizeof(cmos));
    cmos.data[0x0E] = 0x00;  // Diagnostic status
    cmos.data[0x10] = 0x40;  // Floppy drive type (1.44MB)
    cmos.data[0x14] = 0x0D;  // Equipment byte
    cmos.data[0x15] = 0x80;  // Base memory low (640K)
    cmos.data[0x16] = 0x02;  // Base memory high
    cmos.data[0x17] = 0x00;  // Extended memory low
    cmos.data[0x18] = 0x3C;  // Extended memory high (15MB)
    
    // Initialize VGA
    memset(&vga, 0, sizeof(vga));
    vga.misc_output = 0x67;
    
    // Initialize speaker emulation
    speaker_init();
}

/**
 * Read from PIC
 */
static uint8_t pic_read(bool master, uint8_t port)
{
    struct { uint8_t imr, irr, isr, icw1, icw2, icw3, icw4, init_state, vector_base; } *pic = master ? (void*)&pic1 : (void*)&pic2;
    
    if (port == 0) {
        return pic->irr;
    } else {
        return pic->imr;
    }
}

/**
 * Write to PIC
 */
static void pic_write(bool master, uint8_t port, uint8_t value)
{
    struct { uint8_t imr, irr, isr, icw1, icw2, icw3, icw4, init_state, vector_base; } *pic = master ? (void*)&pic1 : (void*)&pic2;
    
    if (port == 0) {
        if (value & 0x10) {
            // ICW1
            pic->icw1 = value;
            pic->init_state = 1;
        } else if (value & 0x08) {
            // OCW3
            // Read register command - ignore for now
        } else {
            // OCW2 (EOI commands)
            if (value == 0x20) {
                // Non-specific EOI
                pic->isr = 0;
            }
        }
    } else {
        if (pic->init_state > 0) {
            switch (pic->init_state) {
                case 1:
                    pic->icw2 = value;
                    pic->vector_base = value & 0xF8;
                    pic->init_state = 2;
                    break;
                case 2:
                    pic->icw3 = value;
                    pic->init_state = (pic->icw1 & 0x01) ? 3 : 0;
                    break;
                case 3:
                    pic->icw4 = value;
                    pic->init_state = 0;
                    break;
            }
        } else {
            // OCW1 - IMR
            pic->imr = value;
        }
    }
}

/**
 * Read from PIT
 */
static uint8_t pit_read(uint8_t channel)
{
    if (channel > 2) return 0xFF;

    pit_sync();
    
    uint16_t value = pit.latched[channel] ? pit.latch[channel] : pit.counter[channel];
    
    uint8_t mode = (pit.control[channel] >> 4) & 0x03;
    uint8_t result;
    
    switch (mode) {
        case 0:  // Counter latch
        case 3:  // LSB then MSB
            if (pit.read_state[channel] == 0) {
                result = value & 0xFF;
                pit.read_state[channel] = 1;
            } else {
                result = (value >> 8) & 0xFF;
                pit.read_state[channel] = 0;
                pit.latched[channel] = false;
            }
            break;
        case 1:  // MSB only
            result = (value >> 8) & 0xFF;
            break;
        case 2:  // LSB only
        default:
            result = value & 0xFF;
            break;
    }
    
    return result;
}

/**
 * Write to PIT
 */
static void pit_write(uint8_t port, uint8_t value)
{
    if (port == 3) {
        // Control register
        uint8_t channel = (value >> 6) & 0x03;
        if (channel == 2) {
            ESP_LOGD(SPK_TAG, "PIT CTRL write for CH2: 0x%02X", value);
        }
        if (channel < 3) {
            if ((value & 0x30) == 0) {
                // Counter latch command
                pit.latch[channel] = pit.counter[channel];
                pit.latched[channel] = true;
            } else {
                pit_sync();
                pit.control[channel] = value;
                pit.mode[channel] = (value >> 1) & 0x07;
                if (pit.mode[channel] >= 6) {
                    pit.mode[channel] -= 4;
                }
                pit.read_state[channel] = 0;
                pit.write_state[channel] = 0;
                pit.phase[channel] = 0;
                pit.output[channel] = true;
            }
        }
    } else if (port < 3) {
        uint8_t mode = (pit.control[port] >> 4) & 0x03;
        
        if (port == 2) {
            ESP_LOGD(SPK_TAG, "PIT CH2 DATA write: 0x%02X (mode: %d, write_state: %d)", value, mode, pit.write_state[port]);
        }
        
        switch (mode) {
            case 3:  // LSB then MSB
                if (pit.write_state[port] == 0) {
                    pit.reload[port] = (pit.reload[port] & 0xFF00) | value;
                    pit.write_state[port] = 1;
                } else {
                    pit_sync();
                    pit.reload[port] = (pit.reload[port] & 0x00FF) | (value << 8);
                    pit.counter[port] = pit.reload[port];
                    pit.write_state[port] = 0;
                    pit.phase[port] = 0;
                    pit.output[port] = true;
                }
                break;
            case 1:  // MSB only
                pit_sync();
                pit.reload[port] = value << 8;
                pit.counter[port] = pit.reload[port];
                pit.phase[port] = 0;
                pit.output[port] = true;
                break;
            case 2:  // LSB only
            default:
                pit_sync();
                pit.reload[port] = value;
                pit.counter[port] = pit.reload[port];
                pit.phase[port] = 0;
                pit.output[port] = true;
                break;
        }
    }
}

/**
 * Read from I/O port (Internal Implementation)
 */
static uint8_t port_in_impl(uint16_t port)
{
    uint8_t pci_val = 0;
    if (pci_port_in(port, &pci_val)) {
        return pci_val;
    }
    uint8_t ata_val = 0;
    if (ata_port_in(port, &ata_val)) {
        return ata_val;
    }
    switch (port) {
        // PIC1 (Master)
        case PORT_PIC1_CMD:
        case PORT_PIC1_DATA:
            return pic_read(true, port & 1);
            
        // PIC2 (Slave)
        case PORT_PIC2_CMD:
        case PORT_PIC2_DATA:
            return pic_read(false, port & 1);
            
        // PIT
        case PORT_PIT_CH0:
        case PORT_PIT_CH1:
        case PORT_PIT_CH2:
            return pit_read(port & 3);
            
        // Keyboard controller / speaker control
        case PORT_KBC_DATA:
            {
                portENTER_CRITICAL(&kbc_mux);
                uint8_t value = kbc.output_buffer;
                kbc.output_full = false;
                kbc.status &= ~0x01;  // Clear output buffer full
                kbc_load_next(true);
                uint8_t status_after = kbc.status;
                portEXIT_CRITICAL(&kbc_mux);
                ESP_LOGI(KBD_TAG, "KBC DATA IN: %02X (status: %02X)", value, status_after);
                return value;
            }
            
        case PORT_KBC_STATUS:
            {
                portENTER_CRITICAL(&kbc_mux);
                uint8_t status = kbc.status;
                portEXIT_CRITICAL(&kbc_mux);
                // Only log status if it's interesting (e.g. output full) or occasionally
                if (status & 0x01) {
                    ESP_LOGI(KBD_TAG, "KBC STATUS IN: %02X", status);
                }
                return status;
            }
        
        // Port 0x61: Speaker control / misc system control
        case 0x61:
            pit_sync();
            {
                uint8_t value = (uint8_t)(port61_state & (uint8_t)~0x20);
                if (pit.output[2]) {
                    value |= 0x20;
                }
                return value;
            }

        // Gameport (joystick)
        case 0x0201:
            {
                if (!app_settings_gameport_enabled()) {
                    return 0xFF;
                }
                const uint64_t now = esp_timer_get_time();
                uint8_t axis_bits = 0x00;
                if (gameport.start_us == 0) {
                    // Before the first strobe, lines read high on real hardware.
                    axis_bits = 0xF0;
                } else {
                    const uint64_t elapsed = now - gameport.start_us;
                    for (int i = 0; i < 4; i++) {
                        if (elapsed < (uint64_t)gameport.axis_us[i]) {
                            axis_bits |= (uint8_t)(1u << (4 + i));
                        }
                    }
                }
                return (uint8_t)(gameport.buttons_mask | axis_bits);
            }

        // COM1 (8250 UART) - minimal implementation for serial mouse
        case 0x3F8:  // RBR / DLL
            {
                portENTER_CRITICAL(&com1_mux);
                const bool dlab = (com1.lcr & 0x80) != 0;
                uint8_t value = 0x00;
                if (dlab) {
                    value = (uint8_t)(com1.divisor & 0xFF);
                } else {
                    com1_maybe_send_mouse_id_nolock();
                    bool popped = com1_rx_pop_byte_nolock(&value);
                    if (popped) {
                        com1.mouse_probe_polls = 0;
                    } else {
                        // Some drivers poll RBR directly without checking LSR/IIR.
                        com1.mouse_probe_polls++;
                        com1_maybe_send_mouse_id_on_poll_nolock();
                        (void)com1_rx_pop_byte_nolock(&value);
                    }
                    com1_raise_irq_if_needed_nolock();
                }
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
        case 0x3F9:  // IER / DLM
            {
                portENTER_CRITICAL(&com1_mux);
                const bool dlab = (com1.lcr & 0x80) != 0;
                uint8_t value = dlab ? (uint8_t)((com1.divisor >> 8) & 0xFF) : com1.ier;
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
        case 0x3FA:  // IIR
            {
                portENTER_CRITICAL(&com1_mux);
                if (com1.rx_head == com1.rx_tail) {
                    com1.mouse_probe_polls++;
                }
                com1_maybe_send_mouse_id_on_poll_nolock();
                const bool rx_ready = (com1.rx_head != com1.rx_tail);
                const bool pending = rx_ready && ((com1.ier & 0x01) != 0) && ((com1.mcr & 0x08) != 0);
                uint8_t value = pending ? 0x04 : 0x01; // 0x04=RX available, bit0=0 when pending
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
        case 0x3FB:  // LCR
            portENTER_CRITICAL(&com1_mux);
            {
                uint8_t value = com1.lcr;
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
        case 0x3FC:  // MCR
            portENTER_CRITICAL(&com1_mux);
            {
                uint8_t value = com1.mcr;
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
        case 0x3FD:  // LSR
            {
                portENTER_CRITICAL(&com1_mux);
                if (com1.rx_head == com1.rx_tail) {
                    com1.mouse_probe_polls++;
                    com1_maybe_send_mouse_id_on_poll_nolock();
                } else {
                    com1.mouse_probe_polls = 0;
                }
                const bool rx_ready = (com1.rx_head != com1.rx_tail);
                uint8_t lsr = 0x60; // THR empty + transmitter empty
                if (rx_ready) {
                    lsr |= 0x01; // data ready
                }
                portEXIT_CRITICAL(&com1_mux);
                return lsr;
            }
        case 0x3FE:  // MSR
            {
                portENTER_CRITICAL(&com1_mux);
                if (com1.rx_head == com1.rx_tail) {
                    com1.mouse_probe_polls++;
                    com1_maybe_send_mouse_id_on_poll_nolock();
                }
                const uint8_t msr = com1_read_msr_nolock();
                portEXIT_CRITICAL(&com1_mux);
                return msr;
            }
        case 0x3FF:  // SCR
            portENTER_CRITICAL(&com1_mux);
            {
                uint8_t value = com1.scr;
                portEXIT_CRITICAL(&com1_mux);
                return value;
            }
             
        // CMOS/RTC
        case PORT_CMOS_DATA:
            if (cmos.index < sizeof(cmos.data)) {
                return cmos.data[cmos.index];
            }
            return 0xFF;
            
        // VGA ports
        case 0x3C2:  // Input Status 0
            return 0x00;
            
        case 0x3CC:  // Misc Output Read
            return vga.misc_output;
            
        case 0x3D8:  // CGA mode control (readback)
            return cga_mode_control;

        case 0x3D9:  // CGA color select (readback)
            return cga_color_select;

        case 0x3DA:  // Input Status 1 (CGA/VGA)
            vga.attr_flip_flop = false;
            // Many DOS programs busy-wait on bits 0/3 (display enable / vertical retrace).
            // Toggle both to ensure these loops make progress (matches 8086tiny behavior).
            cga_status ^= 0x09;
            return cga_status;
            
        case 0x3C4:  // Sequencer Index
            return vga.seq_index;
            
        case 0x3C5:  // Sequencer Data
            return vga.seq_data[vga.seq_index & 0x07];
            
        case 0x3D4:  // CRTC Index
            return vga.crtc_index;
            
        case 0x3D5:  // CRTC Data
            return vga.crtc_data[vga.crtc_index & 0x1F];
            
        // DMA page registers
        case 0x81:
            return dma_page[2];
        case 0x82:
            return dma_page[3];
        case 0x83:
            return dma_page[1];
        case 0x87:
            return dma_page[0];
            
        // Port 0x92 - A20 gate
        case 0x92:
            return mem_get_a20() ? 0x02 : 0x00;
            
        default:
            ESP_LOGD(TAG, "IN port %04X (unhandled)", port);
            return 0xFF;
    }
}

/**
 * Read from I/O port (Wrapper with Tracing)
 */
uint8_t port_in(uint16_t port)
{
    uint8_t val = port_in_impl(port);
    if (should_trace_io(port)) {
        TRACE_LOGI("IN  %04X, Val=%02X", port, val);
    }
    return val;
}

/**
 * Write to I/O port
 */
void port_out(uint16_t port, uint8_t value)
{
    if (should_trace_io(port)) {
        TRACE_LOGI("OUT %04X, Val=%02X", port, value);
    }

    if (pci_port_out(port, value)) {
        return;
    }
    if (ata_port_out(port, value)) {
        return;
    }
    switch (port) {
        // PIC1 (Master)
        case PORT_PIC1_CMD:
        case PORT_PIC1_DATA:
            pic_write(true, port & 1, value);
            break;
            
        // PIC2 (Slave)
        case PORT_PIC2_CMD:
        case PORT_PIC2_DATA:
            pic_write(false, port & 1, value);
            break;
            
        // PIT
        case PORT_PIT_CH0:
        case PORT_PIT_CH1:
        case PORT_PIT_CH2:
        case PORT_PIT_CTRL:
            {
                uint8_t pit_port = port & 3;
                 ESP_LOGD(SPK_TAG, "PIT OUT port %04X: 0x%02X", port, value);
                 ESP_LOGD(SPK_TAG, "PIT Write State CH0: %d, CH1: %d, CH2: %d", pit.write_state[0], pit.write_state[1], pit.write_state[2]);
                // If this is a control word write (port 0x43) for channel 2, 
                // notify speaker about the mode change
                if (pit_port == 3) {
                    uint8_t channel = (value >> 6) & 0x03;
                    if (channel == 2 && (value & 0x30) != 0) {
                        // Not a latch command, it's setting mode
                        uint8_t mode = (value >> 1) & 0x07;
                        speaker_set_pit_control(mode);
                    }
                }
                
                pit_write(pit_port, value);
                
                // If writing to PIT channel 2 data port, update speaker when complete
                // Only update if the write cycle is complete (write_state == 0 means 
                // we just finished writing the full 16-bit value for lobyte/hibyte mode)
                if (pit_port == 2 && pit.write_state[2] == 0) {
                    speaker_set_counter(pit.reload[2], pit.mode[2]);
                }
            }
            break;
            
        // Keyboard controller
        case PORT_KBC_DATA:
            ESP_LOGD(KBD_TAG, "KBC DATA OUT: %02X", value);
            portENTER_CRITICAL(&kbc_mux);
            kbc.input_buffer = value;
            kbc.status |= 0x02;  // Input buffer full

            if (kbc.expecting_ccb) {
                kbc.ccb = value;
                kbc.expecting_ccb = false;
                // If interrupts enabled in new CCB and data available, trigger IRQ
                if ((kbc.ccb & 0x01) && kbc.output_full) {
                    port_irq(1);
                }
                portEXIT_CRITICAL(&kbc_mux);
                break;
            }

            if (kbc.expecting_mouse_byte) {
                kbc.expecting_mouse_byte = false;
                // Handle mouse command
                uint8_t ack = 0xFA;
                kbc_send_mouse_response(&ack, 1);
                
                switch (value) {
                    case 0xFF: // Reset
                        {
                            kbc.mouse_enabled = true;
                            kbc.mouse_sample_rate = 100;
                            kbc.mouse_resolution = 2;
                            kbc.mouse_scaling_1_2 = false;
                            kbc.mouse_stream_mode = false;
                            uint8_t resp[2] = {0xAA, 0x00};
                            kbc_send_mouse_response(resp, 2);
                        }
                        break;
                    case 0xF6: // Set Defaults
                        kbc.mouse_sample_rate = 100;
                        kbc.mouse_resolution = 2;
                        kbc.mouse_scaling_1_2 = false;
                        kbc.mouse_stream_mode = false;
                        break;
                    case 0xF5: // Disable Data Reporting
                        kbc.mouse_enabled = false;
                        break;
                    case 0xF4: // Enable Data Reporting
                        kbc.mouse_enabled = true;
                        break;
                    case 0xF3: // Set Sample Rate
                        // Next byte is rate
                        // We ignore it for now, or add state to expect it
                        break;
                    case 0xF2: // Get Device ID
                        {
                            uint8_t id = 0x00; // Standard PS/2 mouse
                            kbc_send_mouse_response(&id, 1);
                        }
                        break;
                    case 0xE8: // Set Resolution
                        // Next byte is resolution
                        break;
                    case 0xE6: // Set Scaling 1:1
                        kbc.mouse_scaling_1_2 = false;
                        break;
                    case 0xE7: // Set Scaling 2:1
                        kbc.mouse_scaling_1_2 = true;
                        break;
                    default:
                        break;
                }
                portEXIT_CRITICAL(&kbc_mux);
                break;
            }

            if (kb_pending_cmd != 0) {
                uint8_t ack = 0xFA;
                kbc_send_response(&ack, 1);
                kb_pending_cmd = 0;
                portEXIT_CRITICAL(&kbc_mux);
                break;
            }

            switch (value) {
                case 0xED:  // Set LEDs (expects a byte)
                case 0xF0:  // Set scancode set (expects a byte)
                    {
                        uint8_t ack = 0xFA;
                        kbc_send_response(&ack, 1);
                        kb_pending_cmd = value;
                    }
                    break;
                case 0xF2:  // Identify keyboard
                    {
                        uint8_t resp[3] = {0xFA, 0xAB, 0x83};
                        kbc_send_response(resp, sizeof(resp));
                    }
                    break;
                case 0xF4:  // Enable scanning
                    {
                        uint8_t ack = 0xFA;
                        kb_scanning_enabled = true;
                        kbc_send_response(&ack, 1);
                    }
                    break;
                case 0xF5:  // Disable scanning
                    {
                        uint8_t ack = 0xFA;
                        kb_scanning_enabled = false;
                        kbc_send_response(&ack, 1);
                    }
                    break;
                case 0xF6:  // Set defaults
                    {
                        uint8_t ack = 0xFA;
                        kb_scanning_enabled = true;
                        kbc_send_response(&ack, 1);
                    }
                    break;
                case 0xFF:  // Reset
                    {
                        uint8_t resp[2] = {0xFA, 0xAA};
                        kb_scanning_enabled = true;
                        kbc_send_response(resp, sizeof(resp));
                    }
                    break;
                default:
                    {
                        uint8_t ack = 0xFA;
                        kbc_send_response(&ack, 1);
                    }
                    break;
            }
            portEXIT_CRITICAL(&kbc_mux);
            break;
            
        case PORT_KBC_CMD:
            ESP_LOGD(KBD_TAG, "KBC CMD OUT: %02X", value);
            portENTER_CRITICAL(&kbc_mux);
            kbc.command = value;
            switch (value) {
                case 0x20:  // Read Command Byte
                    kbc.output_buffer = kbc.ccb;
                    kbc.status |= 0x01;
                    break;
                case 0x60:  // Write Command Byte
                    kbc.expecting_ccb = true;
                    break;
                case 0xA8:  // Enable Mouse Interface
                    kbc.ccb &= ~0x20; // Clear Bit 5 (Mouse Disabled)
                    break;
                case 0xA7:  // Disable Mouse Interface
                    kbc.ccb |= 0x20; // Set Bit 5 (Mouse Disabled)
                    break;
                case 0xAA:  // Self-test
                    kbc.output_buffer = 0x55;  // Test passed
                    kbc.status |= 0x01;
                    break;
                case 0xAB:  // Interface test
                    kbc.output_buffer = 0x00;  // No error
                    kbc.status |= 0x01;
                    break;
                case 0xAD:  // Disable keyboard
                    kbc.status &= ~0x10; // Clear Bit 4 (Locked)
                    kbc.ccb |= 0x10;     // Set Bit 4 in CCB (Disabled)
                    break;
                case 0xAE:  // Enable keyboard
                    kbc.status |= 0x10;  // Set Bit 4 (Unlocked)
                    kbc.ccb &= ~0x10;    // Clear Bit 4 in CCB (Enabled)
                    break;
                case 0xD0:  // Read output port
                    kbc.output_buffer = 0x02;  // A20 enabled
                    kbc.status |= 0x01;
                    break;
                case 0xD1:  // Write output port
                    // Next byte goes to output port
                    break;
                case 0xD4:  // Write to Mouse
                    kbc.expecting_mouse_byte = true;
                    break;
            }
            portEXIT_CRITICAL(&kbc_mux);
            break;
        
        // Port 0x61: Speaker control / misc system control
        // Bit 0: PIT channel 2 gate (timer enable)
        // Bit 1: Speaker enable
        case 0x61:
            ESP_LOGD(SPK_TAG, "OUT port 0061: 0x%02X (gate=%d enable=%d)", value, (value & 0x01) ? 1 : 0, (value & 0x02) ? 1 : 0);
            port61_state = value;
            speaker_update_port61(value);
            break;

        // CGA registers (direct video mode switching used by some games)
        case PORT_CGA_MODE:
            cga_mode_control = value;
            cga_apply_mode_control(value);
            break;

        case PORT_CGA_PALETTE:
            cga_color_select = value;
            break;

        // Gameport (joystick): writing starts the discharge timing window.
        case 0x0201:
            (void)value;
            if (!app_settings_gameport_enabled()) {
                gameport.start_us = 0;
                break;
            }
            gameport.start_us = esp_timer_get_time();
            break;
            
        // CMOS/RTC
        case PORT_CMOS_ADDR:
            cmos.index = value & 0x7F;
            break;
            
        case PORT_CMOS_DATA:
            if (cmos.index < sizeof(cmos.data)) {
                cmos.data[cmos.index] = value;
            }
            break;
            
        // VGA ports
        case 0x3C2:  // Misc Output Write
            vga.misc_output = value;
            break;
            
        case 0x3C4:  // Sequencer Index
            vga.seq_index = value;
            break;
            
        case 0x3C5:  // Sequencer Data
            vga.seq_data[vga.seq_index & 0x07] = value;
            break;
            
        case 0x3D4:  // CRTC Index
            vga.crtc_index = value;
            break;
            
        case 0x3D5:  // CRTC Data
            vga.crtc_data[vga.crtc_index & 0x1F] = value;
            break;
            
        case 0x3CE:  // Graphics Controller Index
            vga.gc_index = value;
            break;
            
        case 0x3CF:  // Graphics Controller Data
            vga.gc_data[vga.gc_index & 0x0F] = value;
            break;
            
        case 0x3C0:  // Attribute Controller
            if (!vga.attr_flip_flop) {
                vga.attr_index = value;
            } else {
                vga.attr_data[vga.attr_index & 0x1F] = value;
            }
            vga.attr_flip_flop = !vga.attr_flip_flop;
            break;

        case 0x3C8:  // DAC write index
            video_vga_set_dac_index(value);
            break;

        case 0x3C9:  // DAC data
            video_vga_write_dac(value);
            break;
            
        // DMA page registers
        case 0x81:
            dma_page[2] = value;
            break;
        case 0x82:
            dma_page[3] = value;
            break;
        case 0x83:
            dma_page[1] = value;
            break;
        case 0x87:
            dma_page[0] = value;
            break;
            
        // Port 0x92 - A20 gate
        case 0x92:
            mem_set_a20((value & 0x02) != 0);
            break;

        // COM1 (8250 UART) - minimal implementation for serial mouse
        case 0x3F8:  // THR / DLL
            portENTER_CRITICAL(&com1_mux);
            if ((com1.lcr & 0x80) != 0) {
                com1.divisor = (uint16_t)((com1.divisor & 0xFF00) | value);
            } else {
                // In loopback mode, echo written bytes back to RX (common UART probe).
                if (com1.mcr & 0x10) {
                    (void)com1_rx_push_byte_nolock(value);
                    com1_raise_irq_if_needed_nolock();
                }
            }
            portEXIT_CRITICAL(&com1_mux);
            break;
        case 0x3F9:  // IER / DLM
            portENTER_CRITICAL(&com1_mux);
            if ((com1.lcr & 0x80) != 0) {
                com1.divisor = (uint16_t)((com1.divisor & 0x00FF) | ((uint16_t)value << 8));
            } else {
                com1.ier = value;
                com1_raise_irq_if_needed_nolock();
            }
            portEXIT_CRITICAL(&com1_mux);
            break;
        case 0x3FA:  // FCR (write-only)
            portENTER_CRITICAL(&com1_mux);
            com1.fcr = value;
            portEXIT_CRITICAL(&com1_mux);
            break;
        case 0x3FB:  // LCR
            portENTER_CRITICAL(&com1_mux);
            com1.lcr = value;
            portEXIT_CRITICAL(&com1_mux);
            break;
        case 0x3FC:  // MCR
            portENTER_CRITICAL(&com1_mux);
            {
                const uint8_t prev = com1.mcr;
                com1.mcr = value;
                // Drivers probe serial mice by toggling DTR/RTS and expecting the ID byte
                // after asserting them. Treat any change on these bits as a new probe window.
                const uint8_t prev_lines = (uint8_t)(prev & 0x03);
                const uint8_t new_lines = (uint8_t)(value & 0x03);
                if (prev_lines != new_lines) {
                    com1.mouse_id_sent = false;
                    com1.mouse_probe_polls = 0;
                    // Flush any stale RX data so fresh ID can be sent.
                    com1.rx_head = com1.rx_tail;
                }
                com1_maybe_send_mouse_id_nolock();
                com1_raise_irq_if_needed_nolock();
            }
            portEXIT_CRITICAL(&com1_mux);
            break;
        case 0x3FF:  // SCR
            portENTER_CRITICAL(&com1_mux);
            com1.scr = value;
            portEXIT_CRITICAL(&com1_mux);
            break;
            
        default:
            ESP_LOGD(TAG, "OUT port %04X = %02X (unhandled)", port, value);
            break;
    }
}

/**
 * Read word from I/O port
 */
uint16_t port_in_word(uint16_t port)
{
    if (pci_is_cfg_port_word(port)) {
        const uint32_t v = (port >= PCI_CFG_DATA_PORT) ? pci_cfg_read_dword(pci_cfg_addr) : pci_cfg_addr;
        const uint16_t off = (uint16_t)(port - PCI_CFG_ADDR_PORT);
        return (uint16_t)((v >> (8u * (off & 2u))) & 0xFFFFu);
    }
    int ch_i = -1;
    if (ata_is_data_port(port, &ch_i)) {
        return ata_data_read_word(&ata_ch[ch_i]);
    }
    return port_in(port) | (port_in(port + 1) << 8);
}

/**
 * Write word to I/O port
 */
void port_out_word(uint16_t port, uint16_t value)
{
    if (pci_is_cfg_port_word(port)) {
        const uint16_t off = (uint16_t)(port - PCI_CFG_ADDR_PORT);
        if (off < 4) {
            const uint32_t shift = 8u * (uint32_t)(off & 2u);
            pci_cfg_addr = (pci_cfg_addr & ~(0xFFFFu << shift)) | ((uint32_t)value << shift);
        }
        return;
    }
    int ch_i = -1;
    if (ata_is_data_port(port, &ch_i)) {
        ata_data_write_word(&ata_ch[ch_i], value);
        return;
    }
    port_out(port, value & 0xFF);
    port_out(port + 1, (value >> 8) & 0xFF);
}

/**
 * Trigger interrupt request
 */
void port_irq(uint8_t irq)
{
    if (irq < 8) {
        pic1.irr |= (1 << irq);
    } else {
        pic2.irr |= (1 << (irq - 8));
        pic1.irr |= (1 << 2);  // Cascade on IRQ2
    }
}

/**
 * Get pending interrupt (or -1 if none)
 */
int port_get_interrupt(void)
{
    // Check PIC1
    uint8_t pending1 = pic1.irr & ~pic1.imr;
    if (pending1) {
        for (int i = 0; i < 8; i++) {
            if (pending1 & (1 << i)) {
                if (i == 2) {
                    // Check PIC2
                    uint8_t pending2 = pic2.irr & ~pic2.imr;
                    if (pending2) {
                        for (int j = 0; j < 8; j++) {
                            if (pending2 & (1 << j)) {
                                pic2.irr &= ~(1 << j);
                                pic2.isr |= (1 << j);
                                return pic2.vector_base + j;
                            }
                        }
                    }
                } else {
                    pic1.irr &= ~(1 << i);
                    pic1.isr |= (1 << i);
                    return pic1.vector_base + i;
                }
            }
        }
    }
    
    return -1;
}

/**
 * Update PIT counters (call periodically)
 */
void ports_tick(void)
{
    // Keep PIT time-based (not CPU-speed-based). Games (esp. graphics mode) often busy-wait
    // on timer-related state; if this is tied to emulation throughput, enabling WiFi (or any
    // extra workload) can make the PIT "run slower" and stall the program.
    pit_sync();
}

void kb_set_scancode(uint8_t scancode)
{
    kb_set_scancode_ext(scancode, false);
}

void kb_set_scancode_ext(uint8_t scancode, bool extended)
{
    uint8_t seq[2];
    uint8_t len = 0;
    bool dropped = false;
    uint8_t status = 0;
    bool scanning_enabled = false;
    bool locked = false;
    bool queued = false;

    // Check if scanning enabled and keyboard not locked (Bit 4=1 means Unlocked)
    portENTER_CRITICAL(&kbc_mux);
    scanning_enabled = kb_scanning_enabled;
    status = kbc.status;
    locked = ((status & 0x10) == 0);
    if (!scanning_enabled || locked) {
        dropped = true;
        portEXIT_CRITICAL(&kbc_mux);
        goto kb_log_and_out;
    }

    if (extended) {
        seq[len++] = 0xE0;
    }
    seq[len++] = scancode;

    if (!kbc_queue_push_bytes(seq, len)) {
        portEXIT_CRITICAL(&kbc_mux);
        dropped = true;
        goto kb_log_and_out;
    }
    kbc_load_next(true);
    portEXIT_CRITICAL(&kbc_mux);
    queued = true;

kb_log_and_out:
    if (dropped) {
        ESP_LOGW(KBD_TAG, "KBC DROP SCANCODE %02X (ext=%d): scanning=%d status=%02X",
                 scancode, extended, scanning_enabled ? 1 : 0, status);
    } else if (queued) {
        ESP_LOGI(KBD_TAG, "KBC QUEUE SCANCODE: %02X (ext=%d)", scancode, extended);
    }
}

bool kb_has_data(void)
{
    portENTER_CRITICAL(&kbc_mux);
    bool have = kbc.output_full || (kbc_head != kbc_tail);
    portEXIT_CRITICAL(&kbc_mux);
    return have;
}

void mouse_serial_enqueue(int8_t dx, int8_t dy, uint8_t buttons)
{
    // Microsoft serial mouse protocol (3-byte packets, 2-button).
    // dx,dy are passed in screen coordinates; serial mouse uses +Y=up.
    portENTER_CRITICAL(&com1_mux);

    const bool port_open = ((com1.mcr & 0x03) != 0); // DTR or RTS
    if (!port_open) {
        portEXIT_CRITICAL(&com1_mux);
        return;
    }

    // Prioritize the identification sequence over any injected movement packets.
    if (!com1.mouse_id_sent) {
        com1.rx_tail = com1.rx_head;
        com1_send_mouse_id_nolock();
    }

    int8_t my = (int8_t)(-dy);
    const uint8_t dxu = (uint8_t)dx;
    const uint8_t dyu = (uint8_t)my;

    uint8_t b1 = 0x40;
    if (buttons & 0x01) b1 |= 0x20; // left
    if (buttons & 0x02) b1 |= 0x10; // right
    b1 |= (uint8_t)(((dyu >> 6) & 0x03) << 2);
    b1 |= (uint8_t)((dxu >> 6) & 0x03);

    const uint8_t b2 = (uint8_t)(dxu & 0x3F);
    const uint8_t b3 = (uint8_t)(dyu & 0x3F);

    // If there's not enough room for a full packet, drop it.
    uint16_t count = com1_rx_count_nolock();
    const uint16_t free_slots = (uint16_t)(COM1_RX_QUEUE_SIZE - 1u - count);
    if (free_slots < 3) {
        portEXIT_CRITICAL(&com1_mux);
        return;
    }

    (void)com1_rx_push_byte_nolock(b1);
    (void)com1_rx_push_byte_nolock(b2);
    (void)com1_rx_push_byte_nolock(b3);
    com1_raise_irq_if_needed_nolock();

    portEXIT_CRITICAL(&com1_mux);
}

void mouse_ps2_enqueue(int8_t dx, int8_t dy, uint8_t buttons)
{
    portENTER_CRITICAL(&kbc_mux);
    
    if (!kbc.mouse_enabled) {
        portEXIT_CRITICAL(&kbc_mux);
        return;
    }

    // PS/2 Mouse Packet (Standard 3-byte)
    // Byte 1: Yovfl Xovfl Ysign Xsign 1 M R L
    // Byte 2: X movement
    // Byte 3: Y movement
    
    uint8_t b1 = 0x08; // Bit 3 always 1
    if (buttons & 0x01) b1 |= 0x01; // Left
    if (buttons & 0x02) b1 |= 0x02; // Right
    if (buttons & 0x04) b1 |= 0x04; // Middle
    
    if (dx < 0) b1 |= 0x10; // X sign
    if (dy > 0) b1 |= 0x20; // Y sign (dy is screen coords +Y=down, PS/2 +Y=up. So if dy>0 (down), it's negative Y in PS/2)
    
    uint8_t b2 = (uint8_t)dx;
    uint8_t b3 = (uint8_t)(-dy);

    if (kbc_queue_free() >= 3) {
        kbc_queue_push_raw(0x0100 | b1);
        kbc_queue_push_raw(0x0100 | b2);
        kbc_queue_push_raw(0x0100 | b3);
        kbc_load_next(true);
    }
    
    portEXIT_CRITICAL(&kbc_mux);
}
