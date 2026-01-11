/**
 * @file epd_update.c
 * @brief EPD update planner for efficient partial refreshes
 */

#include <string.h>
#include "epd_update.h"
#include "epd_driver.h"

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} epd_rect_t;

#define EPD_UPDATE_MAX_RECTS            64
#define EPD_UPDATE_MERGE_PAD            4
#define EPD_UPDATE_MAX_RECTS_COMMIT     24

static epd_rect_t s_rects[EPD_UPDATE_MAX_RECTS];
static uint16_t s_rect_count = 0;
static bool s_force_full = false;
static bool s_allow_partial = true;
static epd_update_kind_t s_kind = EPD_UPDATE_TEXT;

static inline uint16_t rect_right(const epd_rect_t *r)
{
    return (uint16_t)(r->x + r->w);
}

static inline uint16_t rect_bottom(const epd_rect_t *r)
{
    return (uint16_t)(r->y + r->h);
}

static bool rect_align_and_clip(epd_rect_t *r)
{
    if (r->w == 0 || r->h == 0) {
        return false;
    }

    if (r->x >= EPD_WIDTH || r->y >= EPD_HEIGHT) {
        return false;
    }

    uint16_t x1 = (uint16_t)((r->x / 4) * 4);
    uint16_t y1 = (uint16_t)((r->y / 4) * 4);
    uint16_t x2 = (uint16_t)(((r->x + r->w + 3) / 4) * 4);
    uint16_t y2 = (uint16_t)(((r->y + r->h + 3) / 4) * 4);

    if (x2 <= x1 || y2 <= y1) {
        return false;
    }

    if (x1 >= EPD_WIDTH || y1 >= EPD_HEIGHT) {
        return false;
    }

    if (x2 > EPD_WIDTH) x2 = EPD_WIDTH;
    if (y2 > EPD_HEIGHT) y2 = EPD_HEIGHT;

    r->x = x1;
    r->y = y1;
    r->w = (uint16_t)(x2 - x1);
    r->h = (uint16_t)(y2 - y1);

    return (r->w != 0 && r->h != 0);
}

static bool rect_can_merge(const epd_rect_t *a, const epd_rect_t *b)
{
    const uint16_t pad = EPD_UPDATE_MERGE_PAD;
    if (rect_right(a) + pad < b->x || rect_right(b) + pad < a->x) return false;
    if (rect_bottom(a) + pad < b->y || rect_bottom(b) + pad < a->y) return false;
    return true;
}

static epd_rect_t rect_union(const epd_rect_t *a, const epd_rect_t *b)
{
    epd_rect_t out;
    uint16_t x1 = (a->x < b->x) ? a->x : b->x;
    uint16_t y1 = (a->y < b->y) ? a->y : b->y;
    uint16_t x2 = (rect_right(a) > rect_right(b)) ? rect_right(a) : rect_right(b);
    uint16_t y2 = (rect_bottom(a) > rect_bottom(b)) ? rect_bottom(a) : rect_bottom(b);
    out.x = x1;
    out.y = y1;
    out.w = (uint16_t)(x2 - x1);
    out.h = (uint16_t)(y2 - y1);
    return out;
}

void epd_update_begin(epd_update_kind_t kind, bool allow_partial)
{
    s_rect_count = 0;
    s_force_full = false;
    s_allow_partial = allow_partial;
    s_kind = kind;
}

void epd_update_force_full(void)
{
    s_force_full = true;
    s_rect_count = 0;
}

void epd_update_add_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if (s_force_full) {
        return;
    }

    epd_rect_t rect = { x, y, w, h };
    if (!rect_align_and_clip(&rect)) {
        return;
    }

    bool merged;
    do {
        merged = false;
        for (uint16_t i = 0; i < s_rect_count; i++) {
            if (rect_can_merge(&s_rects[i], &rect)) {
                rect = rect_union(&s_rects[i], &rect);
                s_rects[i] = s_rects[s_rect_count - 1];
                s_rect_count--;
                merged = true;
                break;
            }
        }
    } while (merged);

    if (s_rect_count >= EPD_UPDATE_MAX_RECTS) {
        s_force_full = true;
        s_rect_count = 0;
        return;
    }

    s_rects[s_rect_count++] = rect;
}

static void commit_partial(epd_refresh_mode_t mode)
{
    epd_set_refresh_mode(mode);

    if (s_rect_count == 0) {
        return;
    }

    if (s_rect_count > EPD_UPDATE_MAX_RECTS_COMMIT) {
        epd_rect_t bounds = s_rects[0];
        for (uint16_t i = 1; i < s_rect_count; i++) {
            bounds = rect_union(&bounds, &s_rects[i]);
        }
        epd_refresh_partial(bounds.x, bounds.y, bounds.w, bounds.h);
        return;
    }

    for (uint16_t i = 0; i < s_rect_count; i++) {
        epd_refresh_partial(s_rects[i].x, s_rects[i].y, s_rects[i].w, s_rects[i].h);
    }
}

void epd_update_commit(void)
{
    if (!s_allow_partial || s_force_full || s_kind == EPD_UPDATE_FULL) {
        epd_set_refresh_mode(EPD_REFRESH_FULL);
        epd_refresh_full();
        return;
    }

    uint32_t area = 0;
    for (uint16_t i = 0; i < s_rect_count; i++) {
        area += (uint32_t)s_rects[i].w * (uint32_t)s_rects[i].h;
    }

    const uint32_t screen_area = (uint32_t)EPD_WIDTH * (uint32_t)EPD_HEIGHT;
    const uint32_t fast_area_text = screen_area / 50u;     // ~2%
    const uint32_t fast_area_graphics = screen_area / 20u; // ~5%

    epd_refresh_mode_t mode = EPD_REFRESH_PARTIAL;
    if (s_kind == EPD_UPDATE_TEXT) {
        if (area > 0 && area <= fast_area_text) {
            mode = EPD_REFRESH_FAST;
        }
    } else if (s_kind == EPD_UPDATE_GRAPHICS) {
        if (area > 0 && area <= fast_area_graphics) {
            mode = EPD_REFRESH_FAST;
        }
    }

    commit_partial(mode);
}
