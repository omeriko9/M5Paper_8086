/**
 * @file xms.h
 * @brief XMS (Extended Memory) handler for DOS emulator
 */

#ifndef XMS_H
#define XMS_H

#include <stdint.h>
#include <stdbool.h>
#include "cpu8086.h"

#ifdef __cplusplus
extern "C" {
#endif

bool xms_init(void);
bool xms_present(void);
void xms_get_entry(uint16_t *seg, uint16_t *off);
bool xms_is_entry(uint16_t seg, uint16_t off);
void xms_handle_call(cpu8086_t *cpu);

#ifdef __cplusplus
}
#endif

#endif // XMS_H
