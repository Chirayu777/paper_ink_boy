/**
 * @file screen.h
 * @brief Portrait screen geometry: the device is held upright like a Game Boy (480 wide, 800
 * tall), while the framebuffer and the touch controller are landscape (800 wide, 480 tall).
 *
 * Every layout in this app is written in screen coordinates and converted here, the same mapping
 * the Atlas factory UI uses: screen x = landscape y, screen y = 799 - landscape x. If the picture
 * comes out upside down on your unit, build with SCREEN_FLIP=1.
 *
 * Layout, in screen pixels:
 *   y   0..431  the Game Boy screen at 3x (160x144 -> 480x432), exactly the panel's width; at
 *                GAME_SCALE 2 or 1 a smaller picture centred in this area
 *   y 432..799  title strip and the touch controls
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "board_config.h"
#include "epd_geom.h"

#include <stdint.h>

/* Public macros ---------------------------------------------------------------------------------*/

#ifndef SCREEN_FLIP
#define SCREEN_FLIP (0)
#endif

#define LAND_W   ((uint16_t)EPD_PANEL_WIDTH)
#define LAND_H   ((uint16_t)EPD_PANEL_HEIGHT)
#define SCREEN_W LAND_H
#define SCREEN_H LAND_W

/* Game Boy pixels per screen pixel: 3 fills the width (shades as 3x3 dot cells); 2 and 1 draw a
 * smaller picture (shades as a 2x2 ordered dither), which sends less data per refresh. Set with
 * idf.py -DPIB_GAME_SCALE=2 reconfigure build. */
#ifndef GAME_SCALE
#define GAME_SCALE (3U)
#endif
#define GAME_W      (160U * GAME_SCALE)
#define GAME_H      (144U * GAME_SCALE)
/* The game is centred in the area above the controls. */
#define GAME_AREA_H (432U)
#define GAME_X      ((SCREEN_W - GAME_W) / 2U)
#define GAME_Y      ((GAME_AREA_H - GAME_H) / 2U)

/* Public functions ------------------------------------------------------------------------------*/

/**
 * @brief Screen point to landscape point.
 */
static inline void screen_to_land(uint16_t sx, uint16_t sy, uint16_t *p_lx, uint16_t *p_ly)
{
#if SCREEN_FLIP
    *p_lx = sy;
    *p_ly = (uint16_t)(LAND_H - 1U - sx);
#else
    *p_lx = (uint16_t)(LAND_W - 1U - sy);
    *p_ly = sx;
#endif
}

/**
 * @brief Landscape point (a framebuffer pixel or a touch) to screen point.
 */
static inline void land_to_screen(uint16_t lx, uint16_t ly, uint16_t *p_sx, uint16_t *p_sy)
{
#if SCREEN_FLIP
    *p_sx = (uint16_t)(LAND_H - 1U - ly);
    *p_sy = lx;
#else
    *p_sx = ly;
    *p_sy = (uint16_t)(LAND_W - 1U - lx);
#endif
}

/**
 * @brief Screen rectangle to landscape rectangle.
 */
static inline epd_rect_t screen_to_land_rect(const epd_rect_t *p_s)
{
#if SCREEN_FLIP
    const epd_rect_t land = {
        .x = p_s->y,
        .y = (uint16_t)(LAND_H - (p_s->x + p_s->w)),
        .w = p_s->h,
        .h = p_s->w,
    };
#else
    const epd_rect_t land = {
        .x = (uint16_t)(LAND_W - (p_s->y + p_s->h)),
        .y = p_s->x,
        .w = p_s->h,
        .h = p_s->w,
    };
#endif
    return land;
}
