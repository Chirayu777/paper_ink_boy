/**
 * @file renderer.h
 * @brief Draws Game Boy frames into the display's framebuffer: 3x scale with ordered dithering of
 * the 4 shades, rotated into the panel's landscape layout, only where the frame changed.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "epd_geom.h"

#include <stdbool.h>
#include <stdint.h>

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Build the lookup tables and set the shown frame to all white.
 */
void renderer_init(void);

/**
 * @brief Draw a frame into the framebuffer where it differs from the last frame drawn.
 *
 * @param[in] p_frame GB_EMU_WIDTH * GB_EMU_HEIGHT shades, 0 = white to 3 = black.
 * @param[in] whole true draws the whole game area whatever changed (before a whole-screen
 * refresh).
 * @param[out] p_dirty landscape rectangle that was redrawn, for the refresh.
 * @return true if anything was drawn.
 * @note Display owner task only.
 */
bool renderer_draw(const uint8_t *p_frame, bool whole, epd_rect_t *p_dirty);
