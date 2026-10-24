/**
 * @file hal_spi.h
 * @brief Public API for the board's shared SPI bus -- one place that brings it up.
 *
 * The display and the SD card sit on the same bus, each with its own chip select. Neither owns it:
 * each calls hal_spi_init() before it adds its device, and whichever call comes first creates the
 * bus. The bus is never torn down.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "board_config.h"

#include "driver/spi_master.h"
#include "esp_err.h"

/* Public macros ---------------------------------------------------------------------------------*/

/** The SPI host the bus runs on. */
#define HAL_SPI_HOST (SPI2_HOST)

/** Largest single transfer the bus is set up for, in bytes. */
#define HAL_SPI_MAX_TRANSFER_BYTES (SPI_MAX_TRANSFER_BYTES)

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Bring up the shared SPI bus with DMA, on the pins and transfer size board_config gives.
 *
 * @return ESP_OK when the bus is up, including when it already was; otherwise why it could not be
 * brought up.
 * @note Safe to call from every driver that uses the bus. Not ISR-safe.
 */
esp_err_t hal_spi_init(void);
