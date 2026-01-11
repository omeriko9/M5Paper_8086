/**
 * @file bt_keyboard.h
 * @brief Bluetooth HID keyboard host bridge to DOS BIOS
 */

#ifndef BT_KEYBOARD_H
#define BT_KEYBOARD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void bt_keyboard_init(void);
bool bt_keyboard_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // BT_KEYBOARD_H
