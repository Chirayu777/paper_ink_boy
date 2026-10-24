/**
 * @file display_port.h
 * @brief The display port -- everything the display core and a panel driver need from the hardware.
 *
 * The core and the panel drivers never touch a bus, a pin or an allocator themselves. They call
 * these functions, and one port implementation (display_port_idf for the ESP32 family) is linked
 * in to provide them. The implementation knows the board's wiring; nothing above it does. Moving
 * the display to another MCU means writing another implementation of this file; no panel driver
 * changes.
 *
 * A command is one transfer with the data/command line low; each data byte is a transfer of its
 * own with it high. An image goes out as a stream: the bus is opened once, tiles are queued for
 * DMA, finished tiles are collected as they leave, and the bus is closed after the last one. The
 * implementation wakes the owner task, with the notification set by display_port_set_notify(),
 * each time a tile has left.
 *
 * @version 0.1
 * @date 2026-10-12
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
#include <stddef.h>
#include <stdint.h>

/* Public macros ---------------------------------------------------------------------------------*/

/** Number of tiles a stream can have queued at once. */
#define DISPLAY_PORT_TILE_COUNT (2U)

/* Public types ----------------------------------------------------------------------------------*/

/**
 * @brief How the display is wired and what its controller needs of the bus. Filled in by the panel
 * driver, which is the one place that knows; the port only carries it out.
 */
typedef struct {
    uint8_t pin_cs; /**< Chip select, output */
    uint8_t pin_dc; /**< Data/command, output: low is a command */
    uint8_t pin_rst; /**< Reset, output: low holds the panel in reset */
    uint8_t pin_busy; /**< BUSY, input, pulled up */
    uint8_t busy_ready; /**< The level BUSY has when the panel is ready: 0 or 1 */
    uint32_t spi_clock_hz; /**< The fastest bus clock the controller accepts */
    uint32_t busy_debounce_us; /**< Time between BUSY samples; two ready readings are needed */
} display_port_config_t;

/**
 * @brief What an allocation is for, because the right memory differs: the framebuffer is large and
 * may live in slow RAM, a DMA tile must be in memory the DMA can read.
 */
typedef enum {
    DISPLAY_ALLOC_FRAMEBUFFER, /**< Large, read and written by the CPU */
    DISPLAY_ALLOC_DMA, /**< Read by the DMA while a stream runs */
} display_alloc_t;

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Bring up the pins and the bus device. Touches no panel register.
 *
 * @param[in] p_config the wiring and the controller's bus needs; copied.
 * @return ESP_OK, or why the hardware could not be set up.
 * @note Call once, from the panel driver's init(). Not ISR-safe.
 */
esp_err_t display_port_init(const display_port_config_t *p_config);

/**
 * @brief The BUSY sampling interval the panel driver asked for.
 *
 * @return Interval in microseconds.
 */
uint32_t display_port_busy_debounce_us(void);

/**
 * @brief Allocate memory that suits its use.
 *
 * @param[in] kind what the memory is for.
 * @param[in] bytes size to allocate.
 * @return Pointer to the memory, or NULL if it could not be had or is not fit for the use.
 */
void *display_port_alloc(display_alloc_t kind, size_t bytes);

/**
 * @brief The largest tile a stream can queue.
 *
 * @return Size in bytes.
 */
uint32_t display_port_tile_bytes(void);

/**
 * @brief Register the task and notification bit set when a queued tile has left.
 *
 * @param[in] task owner task.
 * @param[in] bit notification bit, set with eSetBits.
 * @note Call before the first stream. The bit is set from interrupt context.
 */
void display_port_set_notify(TaskHandle_t task, uint32_t bit);

/**
 * @brief Send a command byte: data/command line low, chip select asserted for the transfer only.
 *
 * @param[in] cmd command byte.
 * @return ESP_OK, or why the transfer failed.
 */
esp_err_t display_port_cmd(uint8_t cmd);

/**
 * @brief Send a data byte: data/command line high, chip select asserted for the transfer only.
 *
 * @param[in] data data byte.
 * @return ESP_OK, or why the transfer failed.
 */
esp_err_t display_port_data(uint8_t data);

/**
 * @brief Drive the panel's reset line.
 *
 * @param[in] high true releases reset, false holds the panel in reset.
 */
void display_port_rst(bool high);

/**
 * @brief Sample the panel's BUSY line once.
 *
 * @return true if the panel reports ready, false if it reports busy.
 * @note Safe from the timer task. The line is noisy: the caller debounces it.
 */
bool display_port_busy_ready(void);

/**
 * @brief Open a stream: collect results left by an aborted one, take the bus and assert chip select
 * with the data line high.
 *
 * @return ESP_OK, or why the bus could not be taken.
 */
esp_err_t display_port_stream_open(void);

/**
 * @brief Queue one tile of the stream for DMA.
 *
 * @param[in] p_tile the tile; it must stay valid and unchanged until it has been collected.
 * @param[in] bytes length of the tile.
 * @param[in] slot which of the DISPLAY_PORT_TILE_COUNT queue entries carries it.
 * @return ESP_OK, or why it could not be queued.
 */
esp_err_t display_port_stream_queue(const uint8_t *p_tile, size_t bytes, uint8_t slot);

/**
 * @brief Collect one tile that has left.
 *
 * @param[in] wait_ms how long to wait for one; 0 does not wait.
 * @return ESP_OK if a tile was collected, ESP_ERR_TIMEOUT if none had left, otherwise the error.
 */
esp_err_t display_port_stream_reap(uint32_t wait_ms);

/**
 * @brief Close a stream: release chip select and the bus.
 */
void display_port_stream_close(void);
