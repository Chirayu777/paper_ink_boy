/**
 * @file display_fb.c
 * @brief display_fb implementation -- the framebuffer, its mirror of the glass, and the drawing
 * calls that write into it.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "display_fb.h"

#include "display_port.h"

#include "esp_log.h"

#include <string.h>

/* Private definitions ---------------------------------------------------------------------------*/

#define DISPLAY_FB_BITS_PER_BYTE (8U)
/* A pixel's quick-mode budget is a 4-bit counter, held as four bit planes laid out like the
 * framebuffer, so eight pixels are set, tested and counted down with a few byte operations. */
#define DISPLAY_FB_QUICK_PLANES  (4U)
#define DISPLAY_FB_QUICK_MAX     ((1U << DISPLAY_FB_QUICK_PLANES) - 1U)
/* Scans the still black and white pixels of the area a quick refresh was asked for get toward
 * their own shade, which wipes what earlier refreshes left there; 0 is off. Can be set from
 * bringup.cmake, and at run time with display_fb_quick_touch_scans(). */
#ifndef DISPLAY_QUICK_TOUCH_SCANS
#define DISPLAY_QUICK_TOUCH_SCANS (0U)
#endif

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "display_fb";

static uint8_t *s_framebuffer = NULL;
static uint8_t *s_prev_framebuffer = NULL;
static uint16_t s_width = 0U;
static uint16_t s_height = 0U;
static uint32_t s_stride = 0U;

/* Quick mode: budget bit planes, the budgets a change starts with, and the area -- in bytes along
 * X, rows along Y, end exclusive -- of the pixels the last staged scan left with budget. */
static uint8_t *s_quick_plane[DISPLAY_FB_QUICK_PLANES];
static uint8_t s_quick_budget_black = 0U;
static uint8_t s_quick_budget_white = 0U;

/* Grey. s_grey marks the framebuffer pixels drawn grey -- a black pixel with its bit set is meant
 * grey -- and s_glass_grey is its mirror: the pixels the glass shows grey. s_quick_dir is the way
 * each pixel with budget is being driven to black or white, 1 toward white, and s_quick_to_grey the
 * pixels with budget being driven to grey instead. s_grey_on is set when the panel has quick mode,
 * the only way grey can be shown; without it grey pixels go out black. */
static uint8_t *s_grey = NULL;
static uint8_t *s_glass_grey = NULL;
static uint8_t *s_quick_dir = NULL;
static bool s_grey_on = false;
static uint8_t *s_quick_to_grey = NULL;

/* Touch wipe: the area a quick refresh was asked for, for its first scan only, and how many scans
 * its still black and white pixels are driven toward their own shade; 0 is off. */
static bool s_touch_on = false;
static epd_rect_t s_touch;
static uint8_t s_touch_scans = DISPLAY_QUICK_TOUCH_SCANS;
static bool s_quick_any = false;
/* Whether the scan being staged drives any pixel grey: a scan that does not, the panel runs
 * without the grey row's rest and grey frames. */
static bool s_quick_has_grey = false;
static uint16_t s_quick_x0 = 0U;
static uint16_t s_quick_x1 = 0U;
static uint16_t s_quick_y0 = 0U;
static uint16_t s_quick_y1 = 0U;

/* Private function declarations -----------------------------------------------------------------*/

static void stage_frame_row(uint8_t *p_dst, uint32_t offset, uint16_t w_bytes);
static void stage_quick_old_row(uint8_t *p_dst,
                                uint32_t offset,
                                uint16_t x_byte,
                                uint16_t y,
                                uint16_t w_bytes);
static void stage_quick_new_row(uint8_t *p_dst,
                                uint32_t offset,
                                uint16_t x_byte,
                                uint16_t y,
                                uint16_t w_bytes);
static void quick_assign(uint32_t offset);
static void quick_touch(uint32_t offset, uint16_t col, uint16_t y);
static void quick_set(uint32_t offset, uint8_t mask, uint8_t budget);
static uint8_t quick_live(uint32_t offset);
static void quick_count_down(uint32_t offset, uint8_t mask);
static void set_framebuffer_pixel(uint16_t x, uint16_t y, bool white);
static bool epd_in_circle(int dx, int dy, int r);
static bool epd_in_rounded_rect(uint16_t px,
                                uint16_t py,
                                const epd_rect_t *p_rect,
                                uint16_t radius);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Allocate the framebuffer and the mirror, both white.
 *
 * @return esp_err_t
 */
esp_err_t display_fb_init(epd_size_t size)
{
    if ((size.w == 0U) || (size.h == 0U) || ((size.w % DISPLAY_FB_BITS_PER_BYTE) != 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytes = ((size_t)size.w / DISPLAY_FB_BITS_PER_BYTE) * size.h;
    s_framebuffer = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
    if (s_framebuffer == NULL) {
        ESP_LOGE(TAG, "Framebuffer allocation failed: %u bytes", (unsigned)bytes);
        return ESP_ERR_NO_MEM;
    }
    memset(s_framebuffer, 0xFF, bytes);

    /* Mirrors what is actually on the glass -- partial refresh's old data is sourced from here,
     * not from any device-persisted reference. White matches the real post-cold-boot glass state,
     * and is kept in sync after every successful refresh of any mode. */
    s_prev_framebuffer = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
    if (s_prev_framebuffer == NULL) {
        ESP_LOGE(TAG, "Mirror allocation failed: %u bytes", (unsigned)bytes);
        return ESP_ERR_NO_MEM;
    }
    memset(s_prev_framebuffer, 0xFF, bytes);

    s_grey = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
    s_glass_grey = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
    if ((s_grey == NULL) || (s_glass_grey == NULL)) {
        ESP_LOGE(TAG, "Grey plane allocation failed: %u bytes", (unsigned)bytes);
        return ESP_ERR_NO_MEM;
    }
    memset(s_grey, 0, bytes);
    memset(s_glass_grey, 0, bytes);

    s_width = size.w;
    s_height = size.h;
    s_stride = size.w / DISPLAY_FB_BITS_PER_BYTE;
    return ESP_OK;
}

/**
 * @brief Whether display_fb_init() has succeeded.
 *
 * @return bool
 */
bool display_fb_is_ready(void)
{
    return (s_framebuffer != NULL) && (s_prev_framebuffer != NULL) && (s_width != 0U);
}

/**
 * @brief Copy rows of an image source into a tile, packed back to back.
 */
void display_fb_stage_rows(display_src_t src,
                           uint8_t *p_tile,
                           uint16_t x_byte,
                           uint16_t y,
                           uint16_t rows,
                           uint16_t w_bytes)
{
    for (uint16_t r = 0U; r < rows; r++) {
        uint8_t *dst = &p_tile[(uint32_t)r * w_bytes];
        if (src == DISPLAY_SRC_ZERO) {
            memset(dst, 0, w_bytes);
            continue;
        }

        uint32_t offset = ((uint32_t)(y + r) * s_stride) + x_byte;
        if (src == DISPLAY_SRC_PREV) {
            memcpy(dst, &s_prev_framebuffer[offset], w_bytes);
        } else if (src == DISPLAY_SRC_QUICK_OLD) {
            stage_quick_old_row(dst, offset, x_byte, (uint16_t)(y + r), w_bytes);
        } else if (src == DISPLAY_SRC_QUICK_NEW) {
            stage_quick_new_row(dst, offset, x_byte, (uint16_t)(y + r), w_bytes);
        } else {
            stage_frame_row(dst, offset, w_bytes);
        }
    }
}

/**
 * @brief Mark pixels of a rectangle as grey, or as not grey.
 */
void display_set_grey(const epd_rect_t *p_rect, const uint8_t *p_mask)
{
    uint16_t w_bytes = (uint16_t)((p_rect->w + 7U) / 8U);

    for (uint16_t row = 0U; (row < p_rect->h) && ((p_rect->y + row) < s_height); row++) {
        for (uint16_t col = 0U; (col < p_rect->w) && ((p_rect->x + col) < s_width); col++) {
            uint16_t px = (uint16_t)(p_rect->x + col);
            uint32_t byte_pos = ((uint32_t)(p_rect->y + row) * s_stride) + (px / 8U);
            uint8_t bit_mask = (uint8_t)(0x80U >> (px % 8U));
            bool grey = (p_mask == NULL) ||
                        (((p_mask[((uint32_t)row * w_bytes) + (col / 8U)] >> (7U - (col % 8U))) &
                          1U) != 0U);

            if (grey) {
                s_grey[byte_pos] |= bit_mask;
            } else {
                s_grey[byte_pos] &= (uint8_t)~bit_mask;
            }
        }
    }
}

/**
 * @brief Turn grey on: the panel has quick mode, so grey pixels can be shown.
 */
void display_fb_grey_enable(void)
{
    s_grey_on = true;
}

/**
 * @brief The area of the pixels the framebuffer has grey that the glass does not show grey.
 *
 * @return bool
 */
bool display_fb_grey_pending(epd_rect_t *p_box)
{
    if (s_grey_on == false) {
        return false;
    }

    bool any = false;
    uint16_t x0 = 0U;
    uint16_t x1 = 0U;
    uint16_t y0 = 0U;
    uint16_t y1 = 0U;
    for (uint16_t y = 0U; y < s_height; y++) {
        for (uint16_t b = 0U; b < s_stride; b++) {
            uint32_t at = ((uint32_t)y * s_stride) + b;
            uint8_t grey = (uint8_t)(s_grey[at] & (uint8_t)~s_framebuffer[at]);
            if ((uint8_t)(grey & (uint8_t)~s_glass_grey[at]) == 0U) {
                continue;
            }
            if (any == false) {
                x0 = b;
                x1 = (uint16_t)(b + 1U);
                y0 = y;
                any = true;
            }
            x0 = (b < x0) ? b : x0;
            x1 = (b >= x1) ? (uint16_t)(b + 1U) : x1;
            y1 = (uint16_t)(y + 1U);
        }
    }
    if (any == false) {
        return false;
    }
    p_box->x = (uint16_t)(x0 * DISPLAY_FB_BITS_PER_BYTE);
    p_box->y = y0;
    p_box->w = (uint16_t)((x1 - x0) * DISPLAY_FB_BITS_PER_BYTE);
    p_box->h = (uint16_t)(y1 - y0);
    return true;
}

/**
 * @brief Allocate the quick-mode budgets on first use and set the budgets a change starts with.
 *
 * @return esp_err_t
 */
esp_err_t display_fb_quick_init(uint8_t budget_black, uint8_t budget_white)
{
    if ((budget_black == 0U) ||
        (budget_white == 0U) ||
        (budget_black > DISPLAY_FB_QUICK_MAX) ||
        (budget_white > DISPLAY_FB_QUICK_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytes = s_stride * s_height;
    for (uint8_t i = 0U; i < DISPLAY_FB_QUICK_PLANES; i++) {
        if (s_quick_plane[i] != NULL) {
            continue;
        }
        s_quick_plane[i] = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
        if (s_quick_plane[i] == NULL) {
            ESP_LOGE(TAG, "Quick budget allocation failed: %u bytes", (unsigned)bytes);
            return ESP_ERR_NO_MEM;
        }
        memset(s_quick_plane[i], 0, bytes);
    }
    if (s_quick_dir == NULL) {
        s_quick_dir = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
        if (s_quick_dir == NULL) {
            ESP_LOGE(TAG, "Quick direction allocation failed: %u bytes", (unsigned)bytes);
            return ESP_ERR_NO_MEM;
        }
        memset(s_quick_dir, 0, bytes);
    }
    if (s_quick_to_grey == NULL) {
        s_quick_to_grey = display_port_alloc(DISPLAY_ALLOC_FRAMEBUFFER, bytes);
        if (s_quick_to_grey == NULL) {
            ESP_LOGE(TAG, "Quick grey-drive allocation failed: %u bytes", (unsigned)bytes);
            return ESP_ERR_NO_MEM;
        }
        memset(s_quick_to_grey, 0, bytes);
    }

    s_quick_budget_black = budget_black;
    s_quick_budget_white = budget_white;
    return ESP_OK;
}

/**
 * @brief Drop every pixel's remaining budget.
 */
void display_fb_quick_reset(void)
{
    for (uint8_t i = 0U; i < DISPLAY_FB_QUICK_PLANES; i++) {
        if (s_quick_plane[i] != NULL) {
            memset(s_quick_plane[i], 0, s_stride * s_height);
        }
    }
    if (s_quick_to_grey != NULL) {
        memset(s_quick_to_grey, 0, s_stride * s_height);
    }
    s_quick_any = false;
    s_touch_on = false;
}

/**
 * @brief Start collecting the area of the pixels a scan leaves with budget.
 */
void display_fb_quick_scan_start(void)
{
    s_quick_has_grey = false;
    s_quick_any = false;
    s_touch_on = false;
}

/**
 * @brief Wipe the still pixels of an area in the next scan.
 */
void display_fb_quick_touch(const epd_rect_t *p_rect)
{
    s_touch = *p_rect;
    s_touch_on = (s_touch_scans != 0U);
}

/**
 * @brief Set how many scans the touch wipe drives the still pixels for; 0 turns it off.
 */
void display_fb_quick_touch_scans(uint8_t scans)
{
    s_touch_scans = (scans > DISPLAY_FB_QUICK_MAX) ? (uint8_t)DISPLAY_FB_QUICK_MAX : scans;
}

/**
 * @brief Whether the scan just staged drives any pixel grey.
 *
 * @return bool
 */
bool display_fb_quick_scan_has_grey(void)
{
    return s_quick_has_grey;
}

/**
 * @brief The area of the pixels the last staged scan left with budget.
 *
 * @return bool
 */
bool display_fb_quick_pending(epd_rect_t *p_box)
{
    if (s_quick_any == false) {
        return false;
    }
    p_box->x = (uint16_t)(s_quick_x0 * DISPLAY_FB_BITS_PER_BYTE);
    p_box->y = s_quick_y0;
    p_box->w = (uint16_t)((s_quick_x1 - s_quick_x0) * DISPLAY_FB_BITS_PER_BYTE);
    p_box->h = (uint16_t)(s_quick_y1 - s_quick_y0);
    return true;
}

/**
 * @brief Set one pixel in the framebuffer; a pixel outside the panel is ignored.
 *
 * @param[in] x landscape X.
 * @param[in] y landscape Y.
 * @param[in] black true sets the pixel black, false sets it white.
 */
void display_set_pixel(uint16_t x, uint16_t y, bool black)
{
    if ((x >= s_width) || (y >= s_height)) {
        return;
    }
    set_framebuffer_pixel(x, y, !black);
}

/**
 * @brief Fill a rectangle in the framebuffer with a solid color.
 *
 * @param[in] p_rect rectangle to fill.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_rect(const epd_rect_t *p_rect, bool black)
{
    for (uint16_t py = p_rect->y; (py < (p_rect->y + p_rect->h)) && (py < s_height); py++) {
        for (uint16_t px = p_rect->x; (px < (p_rect->x + p_rect->w)) && (px < s_width);
             px++) {
            set_framebuffer_pixel(px, py, !black);
        }
    }
}

/**
 * @brief Invert every pixel in a rectangle of the framebuffer.
 *
 * @param[in] p_rect rectangle to invert.
 */
void display_invert_rect(const epd_rect_t *p_rect)
{
    for (uint16_t py = p_rect->y; (py < (p_rect->y + p_rect->h)) && (py < s_height); py++) {
        for (uint16_t px = p_rect->x; (px < (p_rect->x + p_rect->w)) && (px < s_width);
             px++) {
            uint32_t byte_pos = ((uint32_t)py * s_stride) + (px / 8U);
            uint8_t bit_mask = (uint8_t)(0x80U >> (px % 8U));
            s_framebuffer[byte_pos] ^= bit_mask;
        }
    }
}

/**
 * @brief Fill a rectangle with rounded corners in the framebuffer with a solid color.
 *
 * @param[in] p_rect rectangle to fill.
 * @param[in] radius corner radius in pixels.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_rounded_rect(const epd_rect_t *p_rect, uint16_t radius, bool black)
{
    for (uint16_t py = p_rect->y; (py < (p_rect->y + p_rect->h)) && (py < s_height); py++) {
        for (uint16_t px = p_rect->x; (px < (p_rect->x + p_rect->w)) && (px < s_width);
             px++) {
            if (epd_in_rounded_rect(px, py, p_rect, radius)) {
                set_framebuffer_pixel(px, py, !black);
            }
        }
    }
}

/**
 * @brief Invert every pixel inside a rectangle with rounded corners in the framebuffer.
 */
void display_invert_rounded_rect(const epd_rect_t *p_rect, uint16_t radius)
{
    for (uint16_t py = p_rect->y; (py < (p_rect->y + p_rect->h)) && (py < s_height); py++) {
        for (uint16_t px = p_rect->x; (px < (p_rect->x + p_rect->w)) && (px < s_width);
             px++) {
            if (epd_in_rounded_rect(px, py, p_rect, radius)) {
                uint32_t byte_pos = ((uint32_t)py * s_stride) + (px / 8U);
                uint8_t bit_mask = (uint8_t)(0x80U >> (px % 8U));
                s_framebuffer[byte_pos] ^= bit_mask;
            }
        }
    }
}

/**
 * @brief Copy a 1bpp MSB-first bitmap into the framebuffer.
 *
 * Only the w columns of the image are written; pixels either side of it keep their value, whatever
 * the alignment of x.
 *
 * @param[in] p_rect where the image goes: its top-left corner, and its size.
 * @param[in] p_image source bitmap, 1 = black, 1bpp MSB-first, row-major, ceil(w/8) bytes per row.
 */
void display_draw_image(const epd_rect_t *p_rect, const uint8_t *p_image)
{
    uint16_t x = p_rect->x;
    uint16_t y = p_rect->y;
    uint16_t w = p_rect->w;
    uint16_t h = p_rect->h;

    if ((w == 0U) ||
        (h == 0U) ||
        ((x + w) > s_width) ||
        ((y + h) > s_height)) {
        ESP_LOGE(TAG, "display_draw_image: invalid parameters");
        return;
    }

    uint16_t w_bytes = (uint16_t)((w + 7U) / 8U);
    uint16_t first_byte = (uint16_t)(x / 8U);
    uint16_t last_byte = (uint16_t)((x + w - 1U) / 8U);

    for (uint16_t row = 0U; row < h; row++) {
        const uint8_t *src = &p_image[(uint32_t)row * w_bytes];
        uint8_t *dst = &s_framebuffer[((uint32_t)(y + row) * s_stride)];
        uint8_t *grey = &s_grey[((uint32_t)(y + row) * s_stride)];

        for (uint16_t b = first_byte; b <= last_byte; b++) {
            int first_px = ((int)b * 8) - (int)x;
            uint8_t window = 0U;
            if (first_px < 0) {
                window = (uint8_t)(src[0] >> (uint8_t)(-first_px));
            } else {
                uint8_t shift = (uint8_t)(first_px % 8);
                uint16_t idx = (uint16_t)(first_px / 8);
                window = (uint8_t)(src[idx] << shift);
                if ((shift != 0U) && ((idx + 1U) < w_bytes)) {
                    window |= (uint8_t)(src[idx + 1U] >> (8U - shift));
                }
            }

            uint8_t mask = 0xFFU;
            if (b == first_byte) {
                mask &= (uint8_t)(0xFFU >> (x % 8U));
            }
            if (b == last_byte) {
                mask &= (uint8_t)(0xFFU << (7U - ((x + w - 1U) % 8U)));
            }
            dst[b] = (uint8_t)((dst[b] & (uint8_t)~mask) | ((uint8_t)~window & mask));
            grey[b] &= (uint8_t)~mask;
        }
    }
}

/**
 * @brief Copy a rectangle of the framebuffer out as a 1bpp bitmap.
 *
 * @param[in] p_rect the rectangle to read, in landscape coordinates.
 * @param[out] p_image receives the bitmap, ceil(w/8) * h bytes, 1 = black.
 */
void display_read_image(const epd_rect_t *p_rect, uint8_t *p_image)
{
    if ((p_rect == NULL) ||
        (p_image == NULL) ||
        (p_rect->w == 0U) ||
        (p_rect->h == 0U) ||
        ((p_rect->x + p_rect->w) > s_width) ||
        ((p_rect->y + p_rect->h) > s_height)) {
        ESP_LOGE(TAG, "display_read_image: invalid parameters");
        return;
    }

    uint16_t w_bytes = (uint16_t)((p_rect->w + 7U) / 8U);

    memset(p_image, 0, (size_t)w_bytes * p_rect->h);
    for (uint16_t row = 0U; row < p_rect->h; row++) {
        const uint8_t *src = &s_framebuffer[(uint32_t)(p_rect->y + row) * s_stride];

        for (uint16_t col = 0U; col < p_rect->w; col++) {
            uint16_t px = (uint16_t)(p_rect->x + col);
            bool black = (((src[px / 8U] >> (7U - (px % 8U))) & 1U) == 0U);

            if (black == true) {
                p_image[((uint32_t)row * w_bytes) + (col / 8U)] |= (uint8_t)(0x80U >> (col % 8U));
            }
        }
    }
}

/**
 * @brief Fill a circle in the framebuffer with a solid color.
 *
 * @param[in] cx center landscape X.
 * @param[in] cy center landscape Y.
 * @param[in] radius radius in pixels.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_circle(uint16_t cx, uint16_t cy, uint16_t radius, bool black)
{
    int r = (int)radius;

    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int px = (int)cx + dx;
            int py = (int)cy + dy;
            bool in_circle = epd_in_circle(dx, dy, r);
            bool in_bounds = (px >= 0) &&
                             (py >= 0) &&
                             (px < (int)s_width) &&
                             (py < (int)s_height);
            if (in_circle && in_bounds) {
                set_framebuffer_pixel((uint16_t)px, (uint16_t)py, !black);
            }
        }
    }
}

/**
 * @brief Invert every pixel inside a circle in the framebuffer.
 *
 * Same circular mask as display_fill_circle() -- a rectangular invert over a circle's bounding
 * box would reach into its corners, outside the true circle, and mark pixels that were never
 * part of the filled circle in the first place.
 *
 * @param[in] cx center landscape X.
 * @param[in] cy center landscape Y.
 * @param[in] radius radius in pixels.
 */
void display_invert_circle(uint16_t cx, uint16_t cy, uint16_t radius)
{
    int r = (int)radius;

    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int px = (int)cx + dx;
            int py = (int)cy + dy;
            bool in_circle = epd_in_circle(dx, dy, r);
            bool in_bounds = (px >= 0) &&
                             (py >= 0) &&
                             (px < (int)s_width) &&
                             (py < (int)s_height);
            if (in_circle && in_bounds) {
                uint32_t byte_pos = ((uint32_t)py * s_stride) + ((uint32_t)px / 8U);
                uint8_t bit_mask = (uint8_t)(0x80U >> ((uint32_t)px % 8U));
                s_framebuffer[byte_pos] ^= bit_mask;
            }
        }
    }
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Stage one row of a whole-screen image. With grey on, a grey pixel goes out white -- a
 * whole-screen waveform can only show black or white -- and a quick pass brings it to grey after.
 * The mirror takes what was sent; the glass then shows no grey in the row.
 *
 * @param[out] p_dst the row in the tile, w_bytes long.
 * @param[in] offset framebuffer offset of the row's first byte.
 * @param[in] w_bytes width of the row, in bytes.
 */
static void stage_frame_row(uint8_t *p_dst, uint32_t offset, uint16_t w_bytes)
{
    for (uint16_t i = 0U; i < w_bytes; i++) {
        uint32_t at = offset + i;
        uint8_t sent = s_framebuffer[at];
        if (s_grey_on == true) {
            sent |= (uint8_t)(s_grey[at] & (uint8_t)~sent);
        }
        p_dst[i] = sent;
        s_prev_framebuffer[at] = sent;
        s_glass_grey[at] = 0U;
    }
}

/**
 * @brief Stage one row of a quick scan's old data.
 *
 * A pixel the framebuffer changed since the mirror was taken gets its budget and its drive first
 * (quick_assign()). The old and new bits of a pixel pick one of the four rows of the quick LUT:
 *
 *   old 1, new 1: held    old 0, new 1: driven white
 *   old 1, new 0: driven black    old 0, new 0: driven grey (white, then the grey frames)
 *
 * so a pixel without budget goes out as 1, 1, whatever its shade.
 *
 * @param[out] p_dst the row in the tile, w_bytes long.
 * @param[in] offset framebuffer offset of the row's first byte.
 * @param[in] x_byte byte column of the row's first byte.
 * @param[in] y the row.
 * @param[in] w_bytes width of the row, in bytes.
 */
static void stage_quick_old_row(uint8_t *p_dst,
                                uint32_t offset,
                                uint16_t x_byte,
                                uint16_t y,
                                uint16_t w_bytes)
{
    for (uint16_t i = 0U; i < w_bytes; i++) {
        uint32_t at = offset + i;
        quick_assign(at);
        if (s_touch_on == true) {
            quick_touch(at, (uint16_t)(x_byte + i), y);
        }

        uint8_t drive = quick_live(at);
        uint8_t to_black =
            (uint8_t)(drive & (uint8_t)~s_quick_dir[at] & (uint8_t)~s_quick_to_grey[at]);
        p_dst[i] = (uint8_t)(~drive | to_black);
        if ((uint8_t)(drive & s_quick_to_grey[at]) != 0U) {
            s_quick_has_grey = true;
        }
    }
}

/**
 * @brief Stage one row of a quick scan's new data -- the second bit of each pixel's LUT row, see
 * stage_quick_old_row() -- and spend one scan of the row's budgets.
 *
 * @param[out] p_dst the row in the tile, w_bytes long.
 * @param[in] offset framebuffer offset of the row's first byte.
 * @param[in] x_byte byte column of the row's first byte.
 * @param[in] y the row.
 * @param[in] w_bytes width of the row, in bytes.
 */
static void stage_quick_new_row(uint8_t *p_dst,
                                uint32_t offset,
                                uint16_t x_byte,
                                uint16_t y,
                                uint16_t w_bytes)
{
    for (uint16_t i = 0U; i < w_bytes; i++) {
        uint32_t at = offset + i;
        uint8_t drive = quick_live(at);
        uint8_t to_white = (uint8_t)(drive & s_quick_dir[at] & (uint8_t)~s_quick_to_grey[at]);
        p_dst[i] = (uint8_t)(~drive | to_white);
        if (drive == 0U) {
            continue;
        }

        quick_count_down(at, drive);
        uint8_t live = quick_live(at);
        s_quick_to_grey[at] &= live;
        if (live == 0U) {
            continue;
        }

        uint16_t col = (uint16_t)(x_byte + i);
        if (s_quick_any == false) {
            s_quick_x0 = col;
            s_quick_x1 = (uint16_t)(col + 1U);
            s_quick_y0 = y;
            s_quick_y1 = (uint16_t)(y + 1U);
            s_quick_any = true;
            continue;
        }
        if (col < s_quick_x0) {
            s_quick_x0 = col;
        }
        if (col >= s_quick_x1) {
            s_quick_x1 = (uint16_t)(col + 1U);
        }
        if (y >= s_quick_y1) {
            s_quick_y1 = (uint16_t)(y + 1U);
        }
    }
}

/**
 * @brief Give the pixels of a byte whose shade the framebuffer changed since the mirror was taken
 * their budget and drive, then bring the mirror up to the target.
 *
 * A pixel is white, grey or black, both in the framebuffer (its bit, and its grey bit when black)
 * and on the glass (the mirror, and the glass grey bit). Black and white are driven for the
 * panel's whole budget from wherever the pixel is -- a little past is not seen, short of it is. A
 * pixel going grey, from black or white, is driven grey: one scan of the LUT's grey row, which
 * drives it white and then the grey frames toward black within the one refresh. Every grey then
 * has the same history and the same shade, and it lands with the rest of the change.
 *
 * @param[in] offset byte offset into the planes.
 */
static void quick_assign(uint32_t offset)
{
    uint8_t to_white = s_framebuffer[offset];
    uint8_t grey = (s_grey_on == true) ? s_grey[offset] : 0U;
    uint8_t to_grey = (uint8_t)(~to_white & grey);
    uint8_t to_black = (uint8_t)(~to_white & (uint8_t)~grey);
    uint8_t is_white = s_prev_framebuffer[offset];
    uint8_t is_grey = (uint8_t)(~is_white & s_glass_grey[offset]);
    uint8_t changed = (uint8_t)((to_white ^ is_white) | (to_grey ^ is_grey));

    if (changed == 0U) {
        return;
    }

    quick_set(offset, (uint8_t)(changed & to_black), s_quick_budget_black);
    quick_set(offset, (uint8_t)(changed & to_white), s_quick_budget_white);
    quick_set(offset, (uint8_t)(changed & to_grey), 1U);

    s_quick_dir[offset] =
        (uint8_t)((s_quick_dir[offset] & (uint8_t)~changed) | (changed & to_white));
    s_quick_to_grey[offset] =
        (uint8_t)((s_quick_to_grey[offset] & (uint8_t)~changed) | (changed & to_grey));
    s_prev_framebuffer[offset] = to_white;
    s_glass_grey[offset] = grey;
}

/**
 * @brief Touch wipe: give the still black and white pixels of a byte inside the area the refresh
 * was asked for a few scans toward their own shade. Pixels being driven, and grey ones, are left
 * alone -- a grey is only reached from white.
 *
 * @param[in] offset byte offset into the planes.
 * @param[in] col byte column of the byte.
 * @param[in] y its row.
 */
static void quick_touch(uint32_t offset, uint16_t col, uint16_t y)
{
    if ((y < s_touch.y) || (y >= (uint32_t)s_touch.y + s_touch.h)) {
        return;
    }

    uint8_t inside = 0U;
    for (uint8_t bit = 0U; bit < DISPLAY_FB_BITS_PER_BYTE; bit++) {
        uint32_t x = ((uint32_t)col * DISPLAY_FB_BITS_PER_BYTE) + bit;
        if ((x >= s_touch.x) && (x < ((uint32_t)s_touch.x + s_touch.w))) {
            inside |= (uint8_t)(0x80U >> bit);
        }
    }

    uint8_t is_white = s_prev_framebuffer[offset];
    uint8_t is_grey = (uint8_t)(~is_white & s_glass_grey[offset]);
    uint8_t still = (uint8_t)(inside & (uint8_t)~quick_live(offset) & (uint8_t)~is_grey);
    if (still == 0U) {
        return;
    }

    quick_set(offset, still, s_touch_scans);
    s_quick_dir[offset] = (uint8_t)((s_quick_dir[offset] & (uint8_t)~still) | (still & is_white));
    s_quick_to_grey[offset] &= (uint8_t)~still;
}

/**
 * @brief Set the budget of the pixels in a byte's mask.
 *
 * @param[in] offset byte offset into the planes.
 * @param[in] mask the pixels to set.
 * @param[in] budget the budget they get.
 */
static void quick_set(uint32_t offset, uint8_t mask, uint8_t budget)
{
    if (mask == 0U) {
        return;
    }
    for (uint8_t i = 0U; i < DISPLAY_FB_QUICK_PLANES; i++) {
        uint8_t bits = (((budget >> i) & 1U) != 0U) ? mask : 0U;
        s_quick_plane[i][offset] = (uint8_t)((s_quick_plane[i][offset] & (uint8_t)~mask) | bits);
    }
}

/**
 * @brief The pixels of a byte that still have budget.
 *
 * @param[in] offset byte offset into the planes.
 * @return uint8_t one bit per pixel.
 */
static uint8_t quick_live(uint32_t offset)
{
    uint8_t live = 0U;
    for (uint8_t i = 0U; i < DISPLAY_FB_QUICK_PLANES; i++) {
        live |= s_quick_plane[i][offset];
    }
    return live;
}

/**
 * @brief Take one off the budget of the pixels in a byte's mask: a subtraction of 1, bit plane by
 * bit plane, with the borrow carried up only where a plane's bit was 0.
 *
 * @param[in] offset byte offset into the planes.
 * @param[in] mask the pixels to count down; each must have budget.
 */
static void quick_count_down(uint32_t offset, uint8_t mask)
{
    uint8_t borrow = mask;
    for (uint8_t i = 0U; (i < DISPLAY_FB_QUICK_PLANES) && (borrow != 0U); i++) {
        uint8_t bits = s_quick_plane[i][offset];
        s_quick_plane[i][offset] = (uint8_t)(bits ^ borrow);
        borrow = (uint8_t)(borrow & (uint8_t)~bits);
    }
}

/**
 * @brief Set or clear one pixel in the framebuffer.
 *
 * @param[in] x landscape X.
 * @param[in] y landscape Y.
 * @param[in] white true sets the bit (white), false clears it (black).
 */
static void set_framebuffer_pixel(uint16_t x, uint16_t y, bool white)
{
    uint32_t byte_pos = ((uint32_t)y * s_stride) + (x / 8U);
    uint8_t bit_mask = (uint8_t)(0x80U >> (x % 8U));
    if (white) {
        s_framebuffer[byte_pos] |= bit_mask;
    } else {
        s_framebuffer[byte_pos] &= (uint8_t)~bit_mask;
    }
    s_grey[byte_pos] &= (uint8_t)~bit_mask;
}

/**
 * @brief Check whether a pixel falls inside a circle, given its offset from the circle's centre.
 *
 * A pixel is in when its centre is within radius + 1/2 of the circle's centre, that is when
 * dx^2 + dy^2 <= r^2 + r. Testing against radius itself instead leaves a single pixel sticking out
 * at each of the four extremes, because those pixels sit exactly on the radius; this way the
 * extremes are short flat runs and the edge reads as round. The bounding box stays 2r + 1 square.
 *
 * @param[in] dx pixel X offset from the centre.
 * @param[in] dy pixel Y offset from the centre.
 * @param[in] r circle radius in pixels.
 * @return true if the pixel is inside the circle.
 */
static bool epd_in_circle(int dx, int dy, int r)
{
    return ((dx * dx) + (dy * dy)) <= ((r * r) + r);
}

/**
 * @brief Check whether (px, py) falls inside a rectangle with its four corners rounded off.
 *
 * @param[in] px point X.
 * @param[in] py point Y.
 * @param[in] p_rect the rectangle.
 * @param[in] radius corner radius in pixels.
 * @return true if the point is inside the rounded rectangle.
 */
static bool epd_in_rounded_rect(uint16_t px, uint16_t py, const epd_rect_t *p_rect, uint16_t radius)
{
    uint16_t x = p_rect->x;
    uint16_t y = p_rect->y;
    uint16_t w = p_rect->w;
    uint16_t h = p_rect->h;
    bool in_left = px < (x + radius);
    bool in_right = px >= (x + w - radius);
    bool in_top = py < (y + radius);
    bool in_bottom = py >= (y + h - radius);
    bool in_corner = (in_left && in_top) ||
                     (in_right && in_top) ||
                     (in_left && in_bottom) ||
                     (in_right && in_bottom);

    if (in_corner == false) {
        return true;
    }

    uint16_t cx = in_left ? (uint16_t)(x + radius) : (uint16_t)(x + w - radius - 1U);
    uint16_t cy = in_top ? (uint16_t)(y + radius) : (uint16_t)(y + h - radius - 1U);
    int dx = (int)px - (int)cx;
    int dy = (int)py - (int)cy;
    int r = (int)radius;
    return ((dx * dx) + (dy * dy)) <= (r * r);
}
