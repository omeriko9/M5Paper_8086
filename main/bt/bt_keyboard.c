/**
 * @file bt_keyboard.c
 * @brief Bluetooth HID keyboard host bridge to DOS BIOS
 */

#include <string.h>
#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"

#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_hidh_gattc.h"
#endif
#if CONFIG_BT_HID_HOST_ENABLED
#include "esp_gap_bt_api.h"
#endif

#include "config_defs.h"
#include "dos/bios.h"
#include "dos/memory.h"
#include "dos/ports.h"

static const char *TAG = "BT_KBD";

#define SCAN_DURATION_SECONDS 5
#define SCAN_RETRY_DELAY_MS 2000
#define BOND_RETRY_DELAY_MS 15000

#define NVS_KBD_NAMESPACE "bt_kbd"
#define NVS_KBD_KEY_LAST  "last_peer"
#define KBD_PEER_MAGIC    0xA5

// HID modifier bits
#define HID_MOD_LCTRL  0x01
#define HID_MOD_LSHIFT 0x02
#define HID_MOD_LALT   0x04
#define HID_MOD_LGUI   0x08
#define HID_MOD_RCTRL  0x10
#define HID_MOD_RSHIFT 0x20
#define HID_MOD_RALT   0x40
#define HID_MOD_RGUI   0x80

// HID key usage codes (subset)
#define HID_KEY_A          0x04
#define HID_KEY_B          0x05
#define HID_KEY_C          0x06
#define HID_KEY_D          0x07
#define HID_KEY_E          0x08
#define HID_KEY_F          0x09
#define HID_KEY_G          0x0A
#define HID_KEY_H          0x0B
#define HID_KEY_I          0x0C
#define HID_KEY_J          0x0D
#define HID_KEY_K          0x0E
#define HID_KEY_L          0x0F
#define HID_KEY_M          0x10
#define HID_KEY_N          0x11
#define HID_KEY_O          0x12
#define HID_KEY_P          0x13
#define HID_KEY_Q          0x14
#define HID_KEY_R          0x15
#define HID_KEY_S          0x16
#define HID_KEY_T          0x17
#define HID_KEY_U          0x18
#define HID_KEY_V          0x19
#define HID_KEY_W          0x1A
#define HID_KEY_X          0x1B
#define HID_KEY_Y          0x1C
#define HID_KEY_Z          0x1D
#define HID_KEY_1          0x1E
#define HID_KEY_2          0x1F
#define HID_KEY_3          0x20
#define HID_KEY_4          0x21
#define HID_KEY_5          0x22
#define HID_KEY_6          0x23
#define HID_KEY_7          0x24
#define HID_KEY_8          0x25
#define HID_KEY_9          0x26
#define HID_KEY_0          0x27
#define HID_KEY_ENTER      0x28
#define HID_KEY_ESC        0x29
#define HID_KEY_BACKSPACE  0x2A
#define HID_KEY_TAB        0x2B
#define HID_KEY_SPACE      0x2C
#define HID_KEY_MINUS      0x2D
#define HID_KEY_EQUAL      0x2E
#define HID_KEY_LEFTBRACE  0x2F
#define HID_KEY_RIGHTBRACE 0x30
#define HID_KEY_BACKSLASH  0x31
#define HID_KEY_SEMICOLON  0x33
#define HID_KEY_APOSTROPHE 0x34
#define HID_KEY_GRAVE      0x35
#define HID_KEY_COMMA      0x36
#define HID_KEY_DOT        0x37
#define HID_KEY_SLASH      0x38
#define HID_KEY_CAPSLOCK   0x39
#define HID_KEY_F1         0x3A
#define HID_KEY_F2         0x3B
#define HID_KEY_F3         0x3C
#define HID_KEY_F4         0x3D
#define HID_KEY_F5         0x3E
#define HID_KEY_F6         0x3F
#define HID_KEY_F7         0x40
#define HID_KEY_F8         0x41
#define HID_KEY_F9         0x42
#define HID_KEY_F10        0x43
#define HID_KEY_F11        0x44
#define HID_KEY_F12        0x45
#define HID_KEY_INSERT     0x49
#define HID_KEY_HOME       0x4A
#define HID_KEY_PAGEUP     0x4B
#define HID_KEY_DELETE     0x4C
#define HID_KEY_END        0x4D
#define HID_KEY_PAGEDOWN   0x4E
#define HID_KEY_RIGHT      0x4F
#define HID_KEY_LEFT       0x50
#define HID_KEY_DOWN       0x51
#define HID_KEY_UP         0x52
#define HID_KEY_KEYPAD_NUMLOCK 0x53
#define HID_KEY_KEYPAD_DIVIDE  0x54
#define HID_KEY_KEYPAD_MULTIPLY 0x55
#define HID_KEY_KEYPAD_SUBTRACT 0x56
#define HID_KEY_KEYPAD_ADD      0x57
#define HID_KEY_KEYPAD_ENTER    0x58
#define HID_KEY_KEYPAD_1        0x59
#define HID_KEY_KEYPAD_2        0x5A
#define HID_KEY_KEYPAD_3        0x5B
#define HID_KEY_KEYPAD_4        0x5C
#define HID_KEY_KEYPAD_5        0x5D
#define HID_KEY_KEYPAD_6        0x5E
#define HID_KEY_KEYPAD_7        0x5F
#define HID_KEY_KEYPAD_8        0x60
#define HID_KEY_KEYPAD_9        0x61
#define HID_KEY_KEYPAD_0        0x62
#define HID_KEY_KEYPAD_DOT      0x63

static esp_hidh_dev_t *s_active_dev = NULL;
static bool s_connecting = false;
static volatile bool s_keyboard_connected = false;
static esp_hidh_dev_t *s_mouse_dev = NULL;
static bool s_mouse_connecting = false;
static uint8_t s_pending_mouse_bda[6];
static bool s_pending_mouse_valid = false;
static uint8_t s_last_keys[6];
static bool s_caps_lock = false;
static uint8_t s_last_modifiers = 0;
static uint32_t s_last_bond_attempt_ms = 0;
static uint32_t s_key_log_count = 0;

typedef struct {
    uint8_t magic;
    uint8_t transport;
    uint8_t addr_type;
    uint8_t bda[6];
} bt_kbd_peer_t;

static const char *bda_to_str(const uint8_t *bda, char *out, size_t out_len)
{
    if (!bda || !out || out_len < 18) {
        return "";
    }
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return out;
}

static uint8_t apply_letter_case(char lower, bool shift, bool caps)
{
    bool upper = caps ^ shift;
    return (uint8_t)(upper ? (lower - 'a' + 'A') : lower);
}

static char ascii_preview(uint8_t ascii)
{
    if (ascii >= 0x20 && ascii <= 0x7E) {
        return (char)ascii;
    }
    return '.';
}

static void update_bda_kb_flags(uint8_t modifiers)
{
    uint8_t flags = 0;
    uint8_t flags2 = 0;

    if (modifiers & HID_MOD_RSHIFT) flags |= 0x01;
    if (modifiers & HID_MOD_LSHIFT) flags |= 0x02;
    if (modifiers & (HID_MOD_LCTRL | HID_MOD_RCTRL)) flags |= 0x04;
    if (modifiers & (HID_MOD_LALT | HID_MOD_RALT)) flags |= 0x08;
    if (s_caps_lock) flags |= 0x40;

    if (modifiers & HID_MOD_LCTRL) flags2 |= 0x01;
    if (modifiers & HID_MOD_LALT) flags2 |= 0x02;
    if (s_caps_lock) flags2 |= 0x40;

    mem_write_byte(BDA_KB_FLAG, flags);
    mem_write_byte(BDA_KB_FLAG_1, flags2);
}

static bool map_hid_key(uint8_t hid, bool shift, bool caps,
                        uint8_t *scancode, uint8_t *ascii, bool *extended)
{
    if (extended) {
        *extended = false;
    }
    switch (hid) {
        case HID_KEY_A: *scancode = 0x1E; *ascii = apply_letter_case('a', shift, caps); return true;
        case HID_KEY_B: *scancode = 0x30; *ascii = apply_letter_case('b', shift, caps); return true;
        case HID_KEY_C: *scancode = 0x2E; *ascii = apply_letter_case('c', shift, caps); return true;
        case HID_KEY_D: *scancode = 0x20; *ascii = apply_letter_case('d', shift, caps); return true;
        case HID_KEY_E: *scancode = 0x12; *ascii = apply_letter_case('e', shift, caps); return true;
        case HID_KEY_F: *scancode = 0x21; *ascii = apply_letter_case('f', shift, caps); return true;
        case HID_KEY_G: *scancode = 0x22; *ascii = apply_letter_case('g', shift, caps); return true;
        case HID_KEY_H: *scancode = 0x23; *ascii = apply_letter_case('h', shift, caps); return true;
        case HID_KEY_I: *scancode = 0x17; *ascii = apply_letter_case('i', shift, caps); return true;
        case HID_KEY_J: *scancode = 0x24; *ascii = apply_letter_case('j', shift, caps); return true;
        case HID_KEY_K: *scancode = 0x25; *ascii = apply_letter_case('k', shift, caps); return true;
        case HID_KEY_L: *scancode = 0x26; *ascii = apply_letter_case('l', shift, caps); return true;
        case HID_KEY_M: *scancode = 0x32; *ascii = apply_letter_case('m', shift, caps); return true;
        case HID_KEY_N: *scancode = 0x31; *ascii = apply_letter_case('n', shift, caps); return true;
        case HID_KEY_O: *scancode = 0x18; *ascii = apply_letter_case('o', shift, caps); return true;
        case HID_KEY_P: *scancode = 0x19; *ascii = apply_letter_case('p', shift, caps); return true;
        case HID_KEY_Q: *scancode = 0x10; *ascii = apply_letter_case('q', shift, caps); return true;
        case HID_KEY_R: *scancode = 0x13; *ascii = apply_letter_case('r', shift, caps); return true;
        case HID_KEY_S: *scancode = 0x1F; *ascii = apply_letter_case('s', shift, caps); return true;
        case HID_KEY_T: *scancode = 0x14; *ascii = apply_letter_case('t', shift, caps); return true;
        case HID_KEY_U: *scancode = 0x16; *ascii = apply_letter_case('u', shift, caps); return true;
        case HID_KEY_V: *scancode = 0x2F; *ascii = apply_letter_case('v', shift, caps); return true;
        case HID_KEY_W: *scancode = 0x11; *ascii = apply_letter_case('w', shift, caps); return true;
        case HID_KEY_X: *scancode = 0x2D; *ascii = apply_letter_case('x', shift, caps); return true;
        case HID_KEY_Y: *scancode = 0x15; *ascii = apply_letter_case('y', shift, caps); return true;
        case HID_KEY_Z: *scancode = 0x2C; *ascii = apply_letter_case('z', shift, caps); return true;
        case HID_KEY_1: *scancode = 0x02; *ascii = (uint8_t)(shift ? '!' : '1'); return true;
        case HID_KEY_2: *scancode = 0x03; *ascii = (uint8_t)(shift ? '@' : '2'); return true;
        case HID_KEY_3: *scancode = 0x04; *ascii = (uint8_t)(shift ? '#' : '3'); return true;
        case HID_KEY_4: *scancode = 0x05; *ascii = (uint8_t)(shift ? '$' : '4'); return true;
        case HID_KEY_5: *scancode = 0x06; *ascii = (uint8_t)(shift ? '%' : '5'); return true;
        case HID_KEY_6: *scancode = 0x07; *ascii = (uint8_t)(shift ? '^' : '6'); return true;
        case HID_KEY_7: *scancode = 0x08; *ascii = (uint8_t)(shift ? '&' : '7'); return true;
        case HID_KEY_8: *scancode = 0x09; *ascii = (uint8_t)(shift ? '*' : '8'); return true;
        case HID_KEY_9: *scancode = 0x0A; *ascii = (uint8_t)(shift ? '(' : '9'); return true;
        case HID_KEY_0: *scancode = 0x0B; *ascii = (uint8_t)(shift ? ')' : '0'); return true;
        case HID_KEY_ENTER: *scancode = 0x1C; *ascii = 0x0D; return true;
        case HID_KEY_ESC: *scancode = 0x01; *ascii = 0x1B; return true;
        case HID_KEY_BACKSPACE: *scancode = 0x0E; *ascii = 0x08; return true;
        case HID_KEY_TAB: *scancode = 0x0F; *ascii = 0x09; return true;
        case HID_KEY_SPACE: *scancode = 0x39; *ascii = (uint8_t)' '; return true;
        case HID_KEY_MINUS: *scancode = 0x0C; *ascii = (uint8_t)(shift ? '_' : '-'); return true;
        case HID_KEY_EQUAL: *scancode = 0x0D; *ascii = (uint8_t)(shift ? '+' : '='); return true;
        case HID_KEY_LEFTBRACE: *scancode = 0x1A; *ascii = (uint8_t)(shift ? '{' : '['); return true;
        case HID_KEY_RIGHTBRACE: *scancode = 0x1B; *ascii = (uint8_t)(shift ? '}' : ']'); return true;
        case HID_KEY_BACKSLASH: *scancode = 0x2B; *ascii = (uint8_t)(shift ? '|' : '\\'); return true;
        case HID_KEY_SEMICOLON: *scancode = 0x27; *ascii = (uint8_t)(shift ? ':' : ';'); return true;
        case HID_KEY_APOSTROPHE: *scancode = 0x28; *ascii = (uint8_t)(shift ? '\"' : '\''); return true;
        case HID_KEY_GRAVE: *scancode = 0x29; *ascii = (uint8_t)(shift ? '~' : '`'); return true;
        case HID_KEY_COMMA: *scancode = 0x33; *ascii = (uint8_t)(shift ? '<' : ','); return true;
        case HID_KEY_DOT: *scancode = 0x34; *ascii = (uint8_t)(shift ? '>' : '.'); return true;
        case HID_KEY_SLASH: *scancode = 0x35; *ascii = (uint8_t)(shift ? '?' : '/'); return true;
        case HID_KEY_CAPSLOCK: *scancode = 0x3A; *ascii = 0x00; return true;
        case HID_KEY_F1: *scancode = 0x3B; *ascii = 0x00; return true;
        case HID_KEY_F2: *scancode = 0x3C; *ascii = 0x00; return true;
        case HID_KEY_F3: *scancode = 0x3D; *ascii = 0x00; return true;
        case HID_KEY_F4: *scancode = 0x3E; *ascii = 0x00; return true;
        case HID_KEY_F5: *scancode = 0x3F; *ascii = 0x00; return true;
        case HID_KEY_F6: *scancode = 0x40; *ascii = 0x00; return true;
        case HID_KEY_F7: *scancode = 0x41; *ascii = 0x00; return true;
        case HID_KEY_F8: *scancode = 0x42; *ascii = 0x00; return true;
        case HID_KEY_F9: *scancode = 0x43; *ascii = 0x00; return true;
        case HID_KEY_F10: *scancode = 0x44; *ascii = 0x00; return true;
        case HID_KEY_F11: *scancode = 0x57; *ascii = 0x00; return true;
        case HID_KEY_F12: *scancode = 0x58; *ascii = 0x00; return true;

        // Navigation Keys (Standard) - Force Extended=True for Arrow/Nav keys
        case HID_KEY_INSERT:   *scancode = 0x52; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_HOME:     *scancode = 0x47; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_PAGEUP:   *scancode = 0x49; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_DELETE:   *scancode = 0x53; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_END:      *scancode = 0x4F; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_PAGEDOWN: *scancode = 0x51; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_RIGHT:    *scancode = 0x4D; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_LEFT:     *scancode = 0x4B; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_DOWN:     *scancode = 0x50; *ascii = 0x00; if (extended) *extended = true; return true;
        case HID_KEY_UP:       *scancode = 0x48; *ascii = 0x00; if (extended) *extended = true; return true;

        // Numpad Support - Map to Extended Navigation to simulate NumLock OFF behavior perfectly
        case HID_KEY_KEYPAD_8: *scancode = 0x48; *ascii = 0x00; if (extended) *extended = true; return true; // Up
        case HID_KEY_KEYPAD_2: *scancode = 0x50; *ascii = 0x00; if (extended) *extended = true; return true; // Down
        case HID_KEY_KEYPAD_4: *scancode = 0x4B; *ascii = 0x00; if (extended) *extended = true; return true; // Left
        case HID_KEY_KEYPAD_6: *scancode = 0x4D; *ascii = 0x00; if (extended) *extended = true; return true; // Right
        case HID_KEY_KEYPAD_7: *scancode = 0x47; *ascii = 0x00; if (extended) *extended = true; return true; // Home
        case HID_KEY_KEYPAD_1: *scancode = 0x4F; *ascii = 0x00; if (extended) *extended = true; return true; // End
        case HID_KEY_KEYPAD_9: *scancode = 0x49; *ascii = 0x00; if (extended) *extended = true; return true; // PgUp
        case HID_KEY_KEYPAD_3: *scancode = 0x51; *ascii = 0x00; if (extended) *extended = true; return true; // PgDn
        case HID_KEY_KEYPAD_0: *scancode = 0x52; *ascii = 0x00; if (extended) *extended = true; return true; // Ins
        case HID_KEY_KEYPAD_DOT: *scancode = 0x53; *ascii = 0x00; if (extended) *extended = true; return true; // Del

        // Other Numpad
        case HID_KEY_KEYPAD_5: *scancode = 0x4C; *ascii = 0x00; if (extended) *extended = false; return true; // 5 (Center) - No extended equivalent
        case HID_KEY_KEYPAD_ENTER: *scancode = 0x1C; *ascii = 0x0D; if (extended) *extended = true; return true; // Enter (Extended)
        case HID_KEY_KEYPAD_DIVIDE: *scancode = 0x35; *ascii = '/'; if (extended) *extended = true; return true; // / (Extended)
        case HID_KEY_KEYPAD_MULTIPLY: *scancode = 0x37; *ascii = '*'; return true; // * (Not extended usually)
        case HID_KEY_KEYPAD_SUBTRACT: *scancode = 0x4A; *ascii = '-'; return true; // - (Not extended)
        case HID_KEY_KEYPAD_ADD: *scancode = 0x4E; *ascii = '+'; return true; // + (Not extended)
        default:
            ESP_LOGW(TAG, "Unmapped key: usage=%02X", hid);
            return false;
    }
}

static bool load_last_peer(bt_kbd_peer_t *peer)
{
    nvs_handle_t nvs;
    size_t len = sizeof(*peer);

    if (!peer) {
        return false;
    }

    if (nvs_open(NVS_KBD_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }

    esp_err_t err = nvs_get_blob(nvs, NVS_KBD_KEY_LAST, peer, &len);
    nvs_close(nvs);

    return (err == ESP_OK && len == sizeof(*peer) && peer->magic == KBD_PEER_MAGIC);
}

static void save_last_peer(const esp_hidh_dev_t *dev)
{
    if (!dev) {
        return;
    }

    bt_kbd_peer_t peer = {0};
    peer.magic = KBD_PEER_MAGIC;
    peer.transport = (uint8_t)esp_hidh_dev_transport_get((esp_hidh_dev_t *)dev);
    memcpy(peer.bda, esp_hidh_dev_bda_get((esp_hidh_dev_t *)dev), sizeof(peer.bda));

#if CONFIG_BT_BLE_ENABLED
    if (peer.transport == ESP_HID_TRANSPORT_BLE) {
        int dev_num = esp_ble_get_bond_device_num();
        if (dev_num > 0) {
            esp_ble_bond_dev_t *list = (esp_ble_bond_dev_t *)calloc(dev_num, sizeof(*list));
            if (list) {
                if (esp_ble_get_bond_device_list(&dev_num, list) == ESP_OK) {
                    for (int i = 0; i < dev_num; i++) {
                        if (memcmp(list[i].bd_addr, peer.bda, sizeof(peer.bda)) == 0) {
                            peer.addr_type = (uint8_t)list[i].bd_addr_type;
                            break;
                        }
                    }
                }
                free(list);
            }
        }
    }
#endif

    nvs_handle_t nvs;
    if (nvs_open(NVS_KBD_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }
    if (nvs_set_blob(nvs, NVS_KBD_KEY_LAST, &peer, sizeof(peer)) == ESP_OK) {
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

static bool try_open_peer(const uint8_t *bda, esp_hid_transport_t transport, uint8_t addr_type, const char *label)
{
    char bda_str[18] = {0};
    ESP_LOGI(TAG, "Connecting to %s keyboard: %s (transport=%s)",
             label ? label : "bonded",
             bda_to_str(bda, bda_str, sizeof(bda_str)),
             (transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT");

    if (!esp_hidh_dev_open((uint8_t *)bda, transport, addr_type)) {
        ESP_LOGW(TAG, "esp_hidh_dev_open failed for %s", bda_str);
        return false;
    }

    s_connecting = true;
    return true;
}

#if CONFIG_BT_BLE_ENABLED
static bool find_ble_bonded_addr_type(const uint8_t *bda, uint8_t *addr_type)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num <= 0) {
        return false;
    }

    esp_ble_bond_dev_t *list = (esp_ble_bond_dev_t *)calloc(dev_num, sizeof(*list));
    if (!list) {
        return false;
    }

    bool found = false;
    if (esp_ble_get_bond_device_list(&dev_num, list) == ESP_OK) {
        for (int i = 0; i < dev_num; i++) {
            if (memcmp(list[i].bd_addr, bda, 6) == 0) {
                if (addr_type) {
                    *addr_type = (uint8_t)list[i].bd_addr_type;
                }
                found = true;
                break;
            }
        }
    }

    free(list);
    return found;
}

static bool try_connect_bonded_ble(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num <= 0) {
        return false;
    }

    esp_ble_bond_dev_t *list = (esp_ble_bond_dev_t *)calloc(dev_num, sizeof(*list));
    if (!list) {
        return false;
    }

    bool started = false;
    if (esp_ble_get_bond_device_list(&dev_num, list) == ESP_OK) {
        for (int i = 0; i < dev_num; i++) {
            if (try_open_peer(list[i].bd_addr, ESP_HID_TRANSPORT_BLE,
                              (uint8_t)list[i].bd_addr_type, "bonded BLE")) {
                started = true;
                break;
            }
        }
    }

    free(list);
    return started;
}
#endif

#if CONFIG_BT_HID_HOST_ENABLED
static bool try_connect_bonded_bt(void)
{
    int dev_num = esp_bt_gap_get_bond_device_num();
    if (dev_num <= 0) {
        return false;
    }

    esp_bd_addr_t *list = (esp_bd_addr_t *)calloc(dev_num, sizeof(*list));
    if (!list) {
        return false;
    }

    bool started = false;
    if (esp_bt_gap_get_bond_device_list(&dev_num, list) == ESP_OK) {
        for (int i = 0; i < dev_num; i++) {
            if (try_open_peer(list[i], ESP_HID_TRANSPORT_BT, 0, "bonded BT")) {
                started = true;
                break;
            }
        }
    }

    free(list);
    return started;
}
#endif

static bool try_connect_bonded(void)
{
    bt_kbd_peer_t peer = {0};
    if (load_last_peer(&peer)) {
        if (peer.transport == ESP_HID_TRANSPORT_BLE) {
            uint8_t addr_type = peer.addr_type;
#if CONFIG_BT_BLE_ENABLED
            if (!find_ble_bonded_addr_type(peer.bda, &addr_type)) {
                addr_type = BLE_ADDR_TYPE_PUBLIC;
            }
            if (try_open_peer(peer.bda, ESP_HID_TRANSPORT_BLE, addr_type, "last")) {
                return true;
            }
#endif
        } else if (peer.transport == ESP_HID_TRANSPORT_BT) {
#if CONFIG_BT_HID_HOST_ENABLED
            if (try_open_peer(peer.bda, ESP_HID_TRANSPORT_BT, 0, "last")) {
                return true;
            }
#endif
        }
    }

#if CONFIG_BT_BLE_ENABLED
    if (try_connect_bonded_ble()) {
        return true;
    }
#endif
#if CONFIG_BT_HID_HOST_ENABLED
    if (try_connect_bonded_bt()) {
        return true;
    }
#endif

    return false;
}

static bool key_in_report(uint8_t key, const uint8_t *keys)
{
    for (int i = 0; i < 6; i++) {
        if (keys[i] == key) {
            return true;
        }
    }
    return false;
}

static void enqueue_scancode(uint8_t scancode, bool pressed, bool extended)
{
    uint8_t code = pressed ? scancode : (uint8_t)(scancode | 0x80);
    ESP_LOGD(TAG, "BT ENQUEUE: sc=%02X pressed=%d ext=%d", scancode, pressed, extended);
    kb_set_scancode_ext(code, extended);
}

static void handle_modifier(uint8_t bit, uint8_t scancode, uint8_t modifiers, bool extended)
{
    if ((modifiers & bit) != (s_last_modifiers & bit)) {
        enqueue_scancode(scancode, (modifiers & bit) != 0, extended);
    }
}

static void handle_keyboard_report(const uint8_t *data, size_t length, uint16_t report_id)
{
    size_t offset = 0;
    if (length >= 9 && report_id != 0 && data[0] == (uint8_t)report_id) {
        offset = 1;
    }
    if (length < offset + 8) {
        ESP_LOGD(TAG, "Short HID report (len=%u, id=%u)", (unsigned)length, report_id);
        return;
    }

    uint8_t modifiers = data[offset];
    const uint8_t *keys = data + offset + 2;
    bool shift = (modifiers & (HID_MOD_LSHIFT | HID_MOD_RSHIFT)) != 0;

    handle_modifier(HID_MOD_LSHIFT, 0x2A, modifiers, false);
    handle_modifier(HID_MOD_RSHIFT, 0x36, modifiers, false);
    handle_modifier(HID_MOD_LCTRL, 0x1D, modifiers, false);
    handle_modifier(HID_MOD_RCTRL, 0x1D, modifiers, true);
    handle_modifier(HID_MOD_LALT, 0x38, modifiers, false);
    handle_modifier(HID_MOD_RALT, 0x38, modifiers, true);

    for (int i = 0; i < 6; i++) {
        uint8_t key = s_last_keys[i];
        if (key == 0) {
            continue;
        }
        if (!key_in_report(key, keys)) {
            uint8_t scancode = 0;
            uint8_t ascii = 0;
            bool extended = false;
            if (map_hid_key(key, false, s_caps_lock, &scancode, &ascii, &extended)) {
                enqueue_scancode(scancode, false, extended);
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        uint8_t key = keys[i];
        if (key == 0 || key < HID_KEY_A) {
            continue;
        }
        if (key_in_report(key, s_last_keys)) {
            continue;
        }

        if (key == HID_KEY_CAPSLOCK) {
            s_caps_lock = !s_caps_lock;
        }

        uint8_t scancode = 0;
        uint8_t ascii = 0;
        bool extended = false;
        if (map_hid_key(key, shift, s_caps_lock, &scancode, &ascii, &extended)) {
            ESP_LOGD(TAG, "BT KEY PRESS: hid=%02X sc=%02X ascii=%02X ext=%d", key, scancode, ascii, extended);
            bios_key_enqueue(scancode, ascii, extended);
            enqueue_scancode(scancode, true, extended);
            if (s_key_log_count < 50) {
                ESP_LOGI(TAG, "Key: hid=%02X sc=%02X ascii=%02X('%c') ext=%d mod=%02X",
                         key, scancode, ascii, ascii_preview(ascii),
                         extended ? 1 : 0, modifiers);
                s_key_log_count++;
            }
        }
    }

    memcpy(s_last_keys, keys, sizeof(s_last_keys));
    s_last_modifiers = modifiers;
    update_bda_kb_flags(modifiers);
}

static void handle_mouse_report(const uint8_t *data, size_t length, uint16_t report_id)
{
    size_t offset = 0;
    if (length >= 4 && report_id != 0 && data[0] == (uint8_t)report_id) {
        offset = 1;
    }
    if (length < offset + 3) {
        ESP_LOGD(TAG, "Short HID mouse report (len=%u, id=%u)", (unsigned)length, report_id);
        return;
    }

    const uint8_t buttons = (uint8_t)(data[offset] & 0x07);
    const int8_t dx = (int8_t)data[offset + 1];
    const int8_t dy = (int8_t)data[offset + 2];

    if (dx != 0 || dy != 0 || buttons != 0) {
        mouse_serial_enqueue(dx, dy, buttons);
    }
}

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDH_OPEN_EVENT:
            {
                char bda_str[18] = {0};
                const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
                const char *name = esp_hidh_dev_name_get(param->open.dev);
                const bool pending_mouse = s_pending_mouse_valid && (memcmp(bda, s_pending_mouse_bda, 6) == 0);

                if (pending_mouse) {
                    s_mouse_connecting = false;
                    s_pending_mouse_valid = false;
                    if (param->open.status == ESP_OK) {
                        s_mouse_dev = param->open.dev;
                        ESP_LOGI(TAG, "Mouse connected: %s (%s)",
                                 name ? name : "unknown", bda_to_str(bda, bda_str, sizeof(bda_str)));
                    } else {
                        ESP_LOGW(TAG, "Mouse connect failed");
                        s_mouse_dev = NULL;
                    }
                    break;
                }

                s_connecting = false;
                if (param->open.status == ESP_OK) {
                    s_active_dev = param->open.dev;
                    s_keyboard_connected = true;
                    memset(s_last_keys, 0, sizeof(s_last_keys));
                    s_caps_lock = false;
                    s_last_modifiers = 0;
                    ESP_LOGI(TAG, "Keyboard connected: %s (%s)",
                             name ? name : "unknown", bda_to_str(bda, bda_str, sizeof(bda_str)));
                    save_last_peer(param->open.dev);
                } else {
                    ESP_LOGW(TAG, "Keyboard connect failed");
                    s_active_dev = NULL;
                    s_keyboard_connected = false;
                }
            }
            break;

        case ESP_HIDH_CLOSE_EVENT:
            if (param->close.dev == s_mouse_dev) {
                char bda_str[18] = {0};
                const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
                const char *name = esp_hidh_dev_name_get(param->close.dev);
                ESP_LOGI(TAG, "Mouse disconnected: %s (%s)",
                         name ? name : "unknown", bda_to_str(bda, bda_str, sizeof(bda_str)));
                s_mouse_dev = NULL;
                s_mouse_connecting = false;
                s_pending_mouse_valid = false;
                break;
            }

            {
                char bda_str[18] = {0};
                const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
                const char *name = esp_hidh_dev_name_get(param->close.dev);
                ESP_LOGI(TAG, "Keyboard disconnected: %s (%s)",
                         name ? name : "unknown", bda_to_str(bda, bda_str, sizeof(bda_str)));
            }
            s_active_dev = NULL;
            s_keyboard_connected = false;
            s_connecting = false;
            memset(s_last_keys, 0, sizeof(s_last_keys));
            s_last_modifiers = 0;
            break;

        case ESP_HIDH_INPUT_EVENT:
            if (param->input.usage == ESP_HID_USAGE_KEYBOARD) {
                handle_keyboard_report(param->input.data, param->input.length, param->input.report_id);
            } else if (param->input.usage == ESP_HID_USAGE_MOUSE) {
                handle_mouse_report(param->input.data, param->input.length, param->input.report_id);
            }
            break;

        default:
            break;
    }
}

static void bt_keyboard_scan_task(void *pvParameters)
{
    (void)pvParameters;
    while (true) {
        const bool need_kb = (!s_active_dev && !s_connecting);
        const bool need_mouse = (!s_mouse_dev && !s_mouse_connecting);
        if (need_kb || need_mouse) {
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (need_kb && now_ms - s_last_bond_attempt_ms >= BOND_RETRY_DELAY_MS) {
                s_last_bond_attempt_ms = now_ms;
                if (try_connect_bonded()) {
                    vTaskDelay(pdMS_TO_TICKS(SCAN_RETRY_DELAY_MS));
                    continue;
                }
            }

            size_t results_len = 0;
            esp_hid_scan_result_t *results = NULL;

            ESP_LOGI(TAG, "Scanning for Bluetooth HID devices...");
            if (esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results) == ESP_OK) {
                ESP_LOGI(TAG, "Scan complete (%u results)", (unsigned)results_len);
                esp_hid_scan_result_t *best_kb = NULL;
                int best_kb_rssi = -127;
                esp_hid_scan_result_t *best_mouse = NULL;
                int best_mouse_rssi = -127;
                unsigned keyboard_count = 0;
                unsigned mouse_count = 0;

                for (esp_hid_scan_result_t *r = results; r != NULL; r = r->next) {
                    bool is_keyboard = (r->usage == ESP_HID_USAGE_KEYBOARD);
                    if (!is_keyboard && r->transport == ESP_HID_TRANSPORT_BLE) {
                        // 0x03C1 is Keyboard
                        if (r->ble.appearance == 0x03C1) {
                            is_keyboard = true;
                        }
                    }

                    bool is_mouse = (r->usage == ESP_HID_USAGE_MOUSE);
                    if (!is_mouse && r->transport == ESP_HID_TRANSPORT_BLE) {
                        // 0x03C2 is Mouse
                        if (r->ble.appearance == 0x03C2) {
                            is_mouse = true;
                        }
                    }

                    if (!is_keyboard && !is_mouse) {
                        ESP_LOGD(TAG, "Skipping non-keyboard/mouse HID device: %s (usage=%d, appearance=0x%04x)",
                                 r->name ? r->name : "unknown", r->usage, 
                                 (r->transport == ESP_HID_TRANSPORT_BLE) ? r->ble.appearance : 0);
                        continue;
                    }

                    if (is_keyboard) {
                        keyboard_count++;
                        ESP_LOGI(TAG, "Keyboard candidate: %s RSSI=%d transport=%s",
                                 r->name ? r->name : "unknown",
                                 r->rssi,
                                 (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT");
                        if (!best_kb || r->rssi > best_kb_rssi) {
                            best_kb = r;
                            best_kb_rssi = r->rssi;
                        }
                    }

                    if (is_mouse) {
                        mouse_count++;
                        ESP_LOGI(TAG, "Mouse candidate: %s RSSI=%d transport=%s",
                                 r->name ? r->name : "unknown",
                                 r->rssi,
                                 (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT");
                        if (!best_mouse || r->rssi > best_mouse_rssi) {
                            best_mouse = r;
                            best_mouse_rssi = r->rssi;
                        }
                    }
                }

                if (need_kb && best_kb) {
                    s_connecting = true;
                    const char *name = best_kb->name ? best_kb->name : "(unknown)";
                    ESP_LOGI(TAG, "Connecting to keyboard: %s (RSSI %d)", name, best_kb->rssi);
#if CONFIG_BT_BLE_ENABLED
                    uint8_t addr_type = (best_kb->transport == ESP_HID_TRANSPORT_BLE) ? best_kb->ble.addr_type : 0;
#else
                    uint8_t addr_type = 0;
#endif
                    if (!esp_hidh_dev_open(best_kb->bda, best_kb->transport, addr_type)) {
                        ESP_LOGW(TAG, "esp_hidh_dev_open failed");
                        s_connecting = false;
                    }
                } else if (need_mouse && best_mouse) {
                    s_mouse_connecting = true;
                    s_pending_mouse_valid = true;
                    memcpy(s_pending_mouse_bda, best_mouse->bda, sizeof(s_pending_mouse_bda));
                    const char *name = best_mouse->name ? best_mouse->name : "(unknown)";
                    ESP_LOGI(TAG, "Connecting to mouse: %s (RSSI %d)", name, best_mouse->rssi);
#if CONFIG_BT_BLE_ENABLED
                    uint8_t addr_type = (best_mouse->transport == ESP_HID_TRANSPORT_BLE) ? best_mouse->ble.addr_type : 0;
#else
                    uint8_t addr_type = 0;
#endif
                    if (!esp_hidh_dev_open(best_mouse->bda, best_mouse->transport, addr_type)) {
                        ESP_LOGW(TAG, "esp_hidh_dev_open failed");
                        s_mouse_connecting = false;
                        s_pending_mouse_valid = false;
                    }
                } else {
                    if (need_kb) {
                        ESP_LOGI(TAG, "No keyboard found");
                    }
                    if (need_mouse) {
                        ESP_LOGI(TAG, "No mouse found");
                    }
                }

                esp_hid_scan_results_free(results);
            } else {
                ESP_LOGW(TAG, "HID scan failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_RETRY_DELAY_MS));
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

#if CONFIG_BT_BLE_ENABLED
static void configure_ble_security(void)
{
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t key_size = 16;

    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1));
}
#endif

void bt_keyboard_init(void)
{
    if (HID_HOST_MODE == HIDH_IDLE_MODE) {
        ESP_LOGW(TAG, "Bluetooth HID host not enabled in config");
        return;
    }

    ESP_LOGI(TAG, "Initializing Bluetooth HID keyboard host");
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NVS: %d", (int)ret);
        return;
    }

    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));
    ESP_LOGI(TAG, "Bluetooth controller initialized (mode=%d)", HID_HOST_MODE);

#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
    configure_ble_security();
#endif

    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    xTaskCreate(bt_keyboard_scan_task, "bt_kbd_scan", 6 * 1024, NULL, 2, NULL);
}

bool bt_keyboard_is_connected(void)
{
    return s_keyboard_connected;
}
