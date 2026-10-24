/**
 * @file panel_uc8179c.h
 * @brief Panel driver for the UC8179C controller as fitted in the 7.5" GDEY075T7-T01 (800 x 480).
 *
 * Holds everything specific to this panel: its command sequences, waveform choices, timings and
 * size, and how it is wired and clocked on this board. It talks to the hardware through
 * display_port.h and is run by the display core through the display_panel_t it exports.
 *
 * Building this driver in is what selects it: it defines display_select_panel(), which the display
 * core calls, and display's CMakeLists.txt names it. A different panel replaces it there.
 *
 * Another size of the same controller is another display_panel_t with its own size and timings,
 * not another driver.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "display_panel.h"

/* Public variables ------------------------------------------------------------------------------*/

/** The GDEY075T7-T01 7.5" panel on a UC8179C controller. */
extern const display_panel_t g_panel_uc8179c_gdey075t7;
