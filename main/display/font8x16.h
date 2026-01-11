/**
 * @file font8x16.h
 * @brief 8x16 IBM PC compatible font
 */

#ifndef FONT8X16_H
#define FONT8X16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get glyph data for a character
 * @param c Character code (0-255)
 * @return Pointer to 16 bytes of glyph data (8 pixels wide, 16 rows)
 */
const uint8_t *font8x16_get_glyph(uint8_t c);

#ifdef __cplusplus
}
#endif

#endif // FONT8X16_H
