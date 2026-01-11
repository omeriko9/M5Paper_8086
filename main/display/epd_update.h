/**
 * @file epd_update.h
 * @brief EPD update planner for efficient partial refreshes
 */

#ifndef EPD_UPDATE_H
#define EPD_UPDATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EPD_UPDATE_TEXT = 0,
    EPD_UPDATE_GRAPHICS = 1,
    EPD_UPDATE_FULL = 2,
} epd_update_kind_t;

/**
 * Begin a new update batch.
 * @param kind Update intent (text/graphics/full).
 * @param allow_partial Whether partial refreshes are allowed.
 */
void epd_update_begin(epd_update_kind_t kind, bool allow_partial);

/**
 * Add a rectangle (in panel pixel coordinates) to the update batch.
 */
void epd_update_add_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/**
 * Force a full refresh when the batch is committed.
 */
void epd_update_force_full(void);

/**
 * Commit the batch (performs the refresh).
 */
void epd_update_commit(void);

#ifdef __cplusplus
}
#endif

#endif // EPD_UPDATE_H
