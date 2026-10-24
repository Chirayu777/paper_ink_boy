/**
 * @file gb_emu.h
 * @brief The Game Boy emulator: Peanut-GB running the ROM from the "rom" flash partition, at full
 * speed in its own task, publishing every drawn frame for the display.
 *
 * The emulator never waits for the display. It renders into a back buffer and, at the end of each
 * drawn frame, copies it into a front buffer under a mutex and sets a notification bit on the
 * display task. The display takes the newest frame whenever the panel is free, so frames the panel
 * is too slow to show are simply dropped.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>

/* Public macros ---------------------------------------------------------------------------------*/

#define GB_EMU_WIDTH  (160U)
#define GB_EMU_HEIGHT (144U)

/* Buttons, 1 = pressed. Same bit order as the Game Boy joypad register. */
#define GB_BTN_A      (0x01U)
#define GB_BTN_B      (0x02U)
#define GB_BTN_SELECT (0x04U)
#define GB_BTN_START  (0x08U)
#define GB_BTN_RIGHT  (0x10U)
#define GB_BTN_LEFT   (0x20U)
#define GB_BTN_UP     (0x40U)
#define GB_BTN_DOWN   (0x80U)

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Load the ROM from the "rom" partition, its save from "saves", and set up the core.
 *
 * @return ESP_OK; ESP_ERR_NOT_FOUND if a partition is missing; ESP_ERR_INVALID_STATE if the rom
 * partition holds no valid Game Boy ROM (blank, or a bad header checksum); ESP_ERR_NOT_SUPPORTED
 * for a cartridge type the core does not support; ESP_ERR_NO_MEM.
 * @note Call once, before gb_emu_start().
 */
esp_err_t gb_emu_init(void);

/**
 * @brief The ROM's title from its header, upper case. Empty before gb_emu_init() succeeds.
 *
 * @return title string, at most 16 characters.
 */
const char *gb_emu_title(void);

/**
 * @brief Start the emulator task.
 *
 * @param[in] notify_task task to notify each time a new frame is published.
 * @param[in] frame_bit notification bit to set on it (eSetBits).
 * @return ESP_OK, or ESP_ERR_NO_MEM if the task could not be created.
 */
esp_err_t gb_emu_start(TaskHandle_t notify_task, uint32_t frame_bit);

/**
 * @brief Set which buttons are held. Takes effect from the next emulated frame.
 *
 * @param[in] pressed GB_BTN_* mask, 1 = pressed.
 * @note Any task.
 */
void gb_emu_set_buttons(uint8_t pressed);

/**
 * @brief Copy the newest published frame, if it is newer than the one the caller has.
 *
 * @param[out] p_dst GB_EMU_WIDTH * GB_EMU_HEIGHT bytes, one shade per pixel, 0 = white to
 * 3 = black, row after row.
 * @param[in,out] p_seq sequence number of the caller's frame; updated when a newer one is copied.
 * @return true if a newer frame was copied.
 * @note Any task.
 */
bool gb_emu_take_frame(uint8_t *p_dst, uint32_t *p_seq);

/**
 * @brief Ask the emulator task to write the cartridge RAM to flash now, if it changed.
 *
 * @note Any task.
 */
void gb_emu_request_save(void);
