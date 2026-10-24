/**
 * @file display_fb.h
 * @brief Public API for the display's framebuffer and the drawing calls that write into it.
 *
 * The framebuffer is 1 bit per pixel, row after row, most significant bit first, 1 = white. It is
 * the panel's native (landscape) orientation. Drawing never touches the panel: a refresh through
 * display.h sends whatever the framebuffer holds. Alongside it is a mirror of what the glass shows,
 * kept up to date as images are sent, which a partial refresh needs as the controller's old data.
 *
 * The drawing calls do not depend on the panel, only on its size, so they are the same for every
 * panel and may be called from any task.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "epd_geom.h"

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

/* Public types ----------------------------------------------------------------------------------*/

/**
 * @brief Where the bytes of an image sent to a panel come from.
 */
typedef enum {
    DISPLAY_SRC_ZERO, /**< All zero bytes */
    DISPLAY_SRC_PREV, /**< The mirror of what the glass shows */
    DISPLAY_SRC_FRAME, /**< The framebuffer; also copied into the mirror as it is taken */
    /** A quick scan's old data: the first bit of each pixel's LUT row -- held, driven white,
     * black or grey. First gives every pixel that changed since the mirror was taken its budget
     * and drive, and brings the mirror up to the framebuffer. */
    DISPLAY_SRC_QUICK_OLD,
    /** A quick scan's new data: the second bit of each pixel's LUT row. Spends one scan of every
     * pixel's budget. Follows DISPLAY_SRC_QUICK_OLD over the same window. */
    DISPLAY_SRC_QUICK_NEW,
} display_src_t;

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Allocate the framebuffer and the mirror, both white.
 *
 * @param[in] size panel size in pixels, landscape.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for an empty size or a width that is not a multiple of 8, or
 * ESP_ERR_NO_MEM.
 * @note Call once, from display_init(). Not thread-safe.
 */
esp_err_t display_fb_init(epd_size_t size);

/**
 * @brief Whether display_fb_init() has succeeded.
 *
 * @return true once the framebuffer exists.
 */
bool display_fb_is_ready(void);

/**
 * @brief Copy rows of an image source into a tile, packed back to back.
 *
 * With DISPLAY_SRC_FRAME the rows are also copied into the mirror, so the mirror matches what the
 * panel was given even if the owner draws while the image is still going out.
 *
 * @param[in] src where the bytes come from.
 * @param[out] p_tile destination, rows * w_bytes long.
 * @param[in] x_byte left edge of the window, in bytes.
 * @param[in] y first row of the window to copy.
 * @param[in] rows number of rows.
 * @param[in] w_bytes width of the window, in bytes.
 */
void display_fb_stage_rows(display_src_t src,
                           uint8_t *p_tile,
                           uint16_t x_byte,
                           uint16_t y,
                           uint16_t rows,
                           uint16_t w_bytes);

/**
 * @brief Allocate the quick-mode drive budgets on first use, all zero, and set how many scans a
 * pixel that changes is driven for.
 *
 * A pixel gets its budget when a quick scan's old data is staged and the framebuffer no longer
 * matches the mirror there -- a pixel that changes again while it is still being driven starts
 * over. Each staged scan spends one.
 *
 * @param[in] budget_black scans for a pixel turning black, 1 to 15.
 * @param[in] budget_white scans for a pixel turning white, 1 to 15.
 * @return ESP_OK, ESP_ERR_INVALID_ARG for a budget out of range, or ESP_ERR_NO_MEM.
 */
esp_err_t display_fb_quick_init(uint8_t budget_black, uint8_t budget_white);

/**
 * @brief Drop every pixel's remaining budget: a whole-screen refresh has redrawn them all.
 */
void display_fb_quick_reset(void);

/**
 * @brief Start collecting the area of the pixels a scan leaves with budget.
 */
void display_fb_quick_scan_start(void);

/**
 * @brief Wipe the still pixels of an area in the next scan: its black and white pixels that are
 * not being driven get the touch-wipe scans toward their own shade. Does nothing while the touch
 * wipe is off (display_fb_quick_touch_scans()). Applies to one scan;
 * display_fb_quick_scan_start() drops it.
 *
 * @param[in] p_rect the area, in pixels.
 */
void display_fb_quick_touch(const epd_rect_t *p_rect);

/**
 * @brief Set how many scans the touch wipe drives still pixels for.
 *
 * @param[in] scans scans, up to 15; 0 turns the touch wipe off. Starts as
 * DISPLAY_QUICK_TOUCH_SCANS.
 */
void display_fb_quick_touch_scans(uint8_t scans);

/**
 * @brief Whether the quick scan whose data was just staged drives any pixel grey. A scan that does
 * not can run without the grey row's extra phases.
 *
 * @return true if a pixel in it is driven grey.
 */
bool display_fb_quick_scan_has_grey(void);

/**
 * @brief The area of the pixels the last staged scan left with budget, which the next scan has to
 * cover.
 *
 * @param[out] p_box byte-aligned area, in pixels; left untouched when nothing is left.
 * @return true if any pixel still has budget.
 */
bool display_fb_quick_pending(epd_rect_t *p_box);

/**
 * @brief Turn grey on, for a panel with quick mode: whole-screen images then send grey pixels
 * white, and quick scans drive them to grey. Off, grey pixels go out black.
 *
 * @note Call once, from display_init().
 */
void display_fb_grey_enable(void);

/**
 * @brief The area of the pixels the framebuffer has grey that the glass does not show grey, as a
 * whole-screen refresh leaves them.
 *
 * @param[out] p_box byte-aligned area, in pixels; left untouched when there is none.
 * @return true if there are any, and grey is on.
 */
bool display_fb_grey_pending(epd_rect_t *p_box);

/**
 * @brief Mark pixels of a rectangle as grey, or as not grey. A black pixel marked grey is shown
 * grey: driven white, then PANEL_QUICK_GREY_FRAMES toward black, in one quick refresh. A
 * white pixel stays white, so inverting a grey area and inverting it back gives the same grey.
 * Every other drawing call clears the mark of the pixels it writes.
 *
 * Grey needs a panel driver built with quick mode; without it a grey pixel is plain black.
 *
 * @param[in] p_rect rectangle, in landscape coordinates.
 * @param[in] p_mask which pixels are grey, laid out as display_draw_image()'s bitmap (1 = grey,
 * ceil(w/8) bytes per row); NULL marks the whole rectangle.
 */
void display_set_grey(const epd_rect_t *p_rect, const uint8_t *p_mask);

/**
 * @brief Set one pixel in the framebuffer. A pixel outside the panel is ignored.
 *
 * @param[in] x landscape X.
 * @param[in] y landscape Y.
 * @param[in] black true sets the pixel black, false sets it white.
 */
void display_set_pixel(uint16_t x, uint16_t y, bool black);

/**
 * @brief Fill a rectangle in the framebuffer with a solid color.
 *
 * @param[in] p_rect rectangle to fill, in landscape coordinates.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_rect(const epd_rect_t *p_rect, bool black);

/**
 * @brief Invert every pixel in a rectangle of the framebuffer.
 *
 * Pure framebuffer XOR, no panel I/O -- a second call over the same region restores the original
 * content exactly, since XOR is its own inverse. Does not touch the panel; a refresh through
 * display.h pushes the change.
 *
 * @param[in] p_rect rectangle to invert, in landscape coordinates.
 */
void display_invert_rect(const epd_rect_t *p_rect);

/**
 * @brief Fill a rectangle with rounded corners in the framebuffer with a solid color.
 *
 * @param[in] p_rect rectangle to fill, in landscape coordinates.
 * @param[in] radius corner radius in pixels; should not exceed half of w or h.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_rounded_rect(const epd_rect_t *p_rect, uint16_t radius, bool black);

/**
 * @brief Invert every pixel inside a rectangle with rounded corners in the framebuffer.
 *
 * Same rounded mask as display_fill_rounded_rect() -- a rectangular invert over the same box would
 * reach into the corners, outside the true rounded shape. Pure framebuffer XOR, no panel I/O -- a
 * second call over the same region restores the original content exactly.
 *
 * @param[in] p_rect rectangle to invert, in landscape coordinates.
 * @param[in] radius corner radius in pixels; should not exceed half of w or h.
 */
void display_invert_rounded_rect(const epd_rect_t *p_rect, uint16_t radius);

/**
 * @brief Copy a 1bpp MSB-first bitmap into the framebuffer.
 *
 * Only the w columns of the image are written; pixels either side of it keep their value, whatever
 * the alignment of x.
 *
 * @param[in] p_rect where the image goes: its top-left corner in landscape coordinates, and its
 * size in pixels.
 * @param[in] p_image source bitmap, 1 = black, 1bpp MSB-first, row-major, ceil(w/8) bytes per row.
 */
void display_draw_image(const epd_rect_t *p_rect, const uint8_t *p_image);

/**
 * @brief Copy a rectangle of the framebuffer out as a 1bpp bitmap: the inverse of
 * display_draw_image(), so drawing the result back leaves the framebuffer as it was.
 *
 * @param[in] p_rect the rectangle to read, in landscape coordinates.
 * @param[out] p_image receives the bitmap, 1 = black, 1bpp MSB-first, row-major, ceil(w/8) bytes
 * per row; it must hold ceil(w/8) * h bytes. Padding bits at the end of a row are 0.
 */
void display_read_image(const epd_rect_t *p_rect, uint8_t *p_image);

/**
 * @brief Fill a circle in the framebuffer with a solid color.
 *
 * A pixel is inside when its centre is within radius + 1/2 of the circle's centre, so the circle
 * spans 2 * radius + 1 pixels and its four extremes are short flat runs, not single pixels.
 *
 * @param[in] cx center landscape X.
 * @param[in] cy center landscape Y.
 * @param[in] radius radius in pixels.
 * @param[in] black true fills black, false fills white.
 */
void display_fill_circle(uint16_t cx, uint16_t cy, uint16_t radius, bool black);

/**
 * @brief Invert every pixel inside a circle in the framebuffer.
 *
 * Same circular mask as display_fill_circle() -- pure framebuffer XOR, no panel I/O. A second call
 * over the same circle restores the original content exactly.
 *
 * @param[in] cx center landscape X.
 * @param[in] cy center landscape Y.
 * @param[in] radius radius in pixels.
 */
void display_invert_circle(uint16_t cx, uint16_t cy, uint16_t radius);
