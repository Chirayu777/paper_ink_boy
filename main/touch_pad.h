/**
 * @file touch_pad.h
 * @brief The on-screen Game Boy controls: drawing them into the framebuffer, and turning touch
 * points into a button mask.
 *
 * Layout (portrait screen pixels, see screen.h): the D-pad at the lower left, B and A at the lower
 * right, SELECT and START below them, like the original Game Boy.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "hal_touch.h"

#include <stdint.h>

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Clear the framebuffer and draw the static screen: the frame of the game area, the title
 * strip and the controls.
 *
 * @param[in] p_title text for the title strip.
 */
void touch_pad_draw(const char *p_title);

/**
 * @brief Clear the framebuffer and draw a message screen instead of the game (no ROM, errors).
 *
 * @param[in] pp_lines lines of text, upper case.
 * @param[in] count number of lines.
 */
void touch_pad_draw_message(const char *const *pp_lines, uint32_t count);

/**
 * @brief Which buttons a set of touch points holds.
 *
 * @param[in] p_points touch points, landscape coordinates.
 * @param[in] count number of points.
 * @return GB_BTN_* mask.
 */
uint8_t touch_pad_buttons(const hal_touch_point_t *p_points, int count);
