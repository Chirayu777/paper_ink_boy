/**
 * @file panel_ssd1677.h
 * @brief Panel driver for the SSD1677 controller as fitted in the 4.26" GDEQ0426T82-T01C
 * (800 x 480).
 *
 * Holds everything specific to this panel: its command sequences, waveform choices, timings and
 * size, and how it is wired and clocked on this board. It talks to the hardware through
 * display_port.h and is run by the display core through the display_panel_t it exports.
 *
 * Building this driver in is what selects it: it defines display_select_panel(), which the display
 * core calls, and display's CMakeLists.txt names it when CONFIG_PIB_PANEL_GDEQ0426T82 is set.
 *
 * @version 0.1
 * @date 2026-10-31
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "display_panel.h"

/* Public variables ------------------------------------------------------------------------------*/

/** The GDEQ0426T82-T01C 4.26" panel on an SSD1677 controller. */
extern const display_panel_t g_panel_ssd1677_gdeq0426t82;
