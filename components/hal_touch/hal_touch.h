/**
 * @file hal_touch.h
 * @brief Public API for the capacitive touch controller driver -- raw coordinates only. Implemented
 * by hal_touch.c for the GT911 (7.5" module) or hal_touch_ft6336.c for the FT6336U (4.26" module),
 * as menuconfig's panel selects.
 *
 * No chess-square mapping, no hit-region classification -- that belongs to tsk_input
 * (atlas-architecture-presentation.html section iv). This is the HAL layer under it: init the
 * controller and read one touch point.
 *
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include <stdint.h>

/* Public types ----------------------------------------------------------------------------------*/

/**
 * @brief What one hal_touch_read() saw. Kept distinct on purpose: a caller deciding whether a
 * finger has lifted must tell "nothing reported" from "a report with no usable point" -- GT911
 * produces the latter transiently while a finger is still down.
 */
typedef enum {
    /* No INT within wait_ticks. */
    HAL_TOUCH_TIMEOUT,
    /* INT fired but no valid point: 0-point report, status not ready, or I2C error. */
    HAL_TOUCH_NO_POINT,
    /* x, y valid. */
    HAL_TOUCH_POINT,
} hal_touch_result_t;

/** Most points the GT911 reports at once. */
#define HAL_TOUCH_MAX_POINTS (5U)

/**
 * @brief One touch point of a multi-touch report, in landscape framebuffer coordinates.
 */
typedef struct {
    uint16_t x; /**< Landscape X */
    uint16_t y; /**< Landscape Y */
} hal_touch_point_t;

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Hardware reset the GT911, bring up I2C, and arm the INT-pin ISR.
 *
 * @return esp_err_t
 * @note Safe to call again after hal_touch_stop() or a failed attempt: it first releases whatever a
 * previous call acquired. Not thread-safe, not ISR-safe.
 */
esp_err_t hal_touch_init(void);

/**
 * @brief Stop the INT-pin ISR from notifying any task.
 *
 * Call before deleting the task that called hal_touch_init(), so the ISR never notifies a task
 * that no longer exists. The I2C bus is released by the next hal_touch_init().
 *
 * @note Does no I2C traffic, so it is safe even if that task is stuck inside the bus driver.
 * Thread-safe. Not ISR-safe.
 */
void hal_touch_stop(void);

/**
 * @brief Block for one touch report, up to wait_ticks, and classify it.
 *
 * @param[out] x X coordinate, set only on HAL_TOUCH_POINT.
 * @param[out] y Y coordinate, set only on HAL_TOUCH_POINT.
 * @param[in] wait_ticks maximum ticks to block waiting for the INT ISR.
 * @return HAL_TOUCH_POINT, HAL_TOUCH_NO_POINT or HAL_TOUCH_TIMEOUT.
 * @note Call only from the task that called hal_touch_init() (the INT ISR notifies that task).
 */
hal_touch_result_t hal_touch_read(uint16_t *x, uint16_t *y, TickType_t wait_ticks);

/**
 * @brief Wait up to wait_ticks for the INT ISR, then read every point of the latest report.
 *
 * Reads the status register whether or not INT fired, so a missed edge is caught up on the next
 * call. A report with 0 points is the controller saying every finger has lifted.
 *
 * @param[out] p_points receives up to max_points points.
 * @param[in] max_points size of p_points.
 * @param[in] wait_ticks maximum ticks to block waiting for the INT ISR.
 * @return number of points (0 = all lifted) on a fresh report, or -1 when there is no fresh report
 * (status not ready, or an I2C error).
 * @note Call only from the task that called hal_touch_init() (the INT ISR notifies that task).
 */
int hal_touch_read_points(hal_touch_point_t *p_points, uint8_t max_points, TickType_t wait_ticks);
