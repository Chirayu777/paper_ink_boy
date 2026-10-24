/**
 * @file epd_geom.h
 * @brief The geometry types the panel's drawing and layout code shares.
 *
 * Nearly everything drawn on the panel is a rectangle, and nearly every call that draws, marks or
 * hit-tests one takes its four numbers as separate arguments. epd_rect_t carries them as one, so a
 * function that works on a rectangle takes a single pointer and a layout can be written as data.
 * epd_point_t does the same for a position, such as where text starts or where a finger touched,
 * and epd_size_t for an extent, such as the panel or an image.
 *
 * The types say nothing about which coordinates they hold. The framebuffer is landscape (800 wide,
 * 480 tall) and the screen the user holds is portrait (480 wide, 800 tall); each API names the one
 * it expects, as it does for any bare x and y.
 *
 * @version 0.1
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include <stdint.h>

/* Public types ----------------------------------------------------------------------------------*/

/**
 * @brief A position in pixels.
 */
typedef struct {
    uint16_t x; /**< Along x */
    uint16_t y; /**< Along y */
} epd_point_t;

/**
 * @brief An extent in pixels.
 */
typedef struct {
    uint16_t w; /**< Width, along x */
    uint16_t h; /**< Height, along y */
} epd_size_t;

/**
 * @brief A rectangle: its top-left corner and its size, in pixels.
 */
typedef struct {
    uint16_t x; /**< Left edge */
    uint16_t y; /**< Top edge */
    uint16_t w; /**< Width, along x */
    uint16_t h; /**< Height, along y */
} epd_rect_t;
