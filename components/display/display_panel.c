/**
 * @file display_panel.c
 * @brief display_panel implementation -- the helpers a panel driver answers advance() with.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "display_panel.h"

#include <string.h>

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Answer advance() with a fixed delay.
 *
 * @return esp_err_t
 */
esp_err_t display_wait_delay(display_wait_t *p_wait, uint8_t next, uint32_t delay_ms)
{
    memset(p_wait, 0, sizeof(*p_wait));
    p_wait->kind = DISPLAY_WAIT_DELAY;
    p_wait->next = next;
    p_wait->delay_ms = delay_ms;
    return ESP_OK;
}

/**
 * @brief Answer advance() with a wait for the BUSY line.
 *
 * @return esp_err_t
 */
esp_err_t display_wait_busy(display_wait_t *p_wait,
                            uint8_t next,
                            uint32_t lead_ms,
                            uint32_t timeout_ms)
{
    memset(p_wait, 0, sizeof(*p_wait));
    p_wait->kind = DISPLAY_WAIT_BUSY;
    p_wait->next = next;
    p_wait->delay_ms = lead_ms;
    p_wait->timeout_ms = timeout_ms;
    return ESP_OK;
}

/**
 * @brief Answer advance() with an image to send.
 *
 * @return esp_err_t
 */
esp_err_t display_wait_stream(display_wait_t *p_wait,
                              uint8_t next,
                              display_src_t src,
                              uint16_t x_byte,
                              uint16_t y,
                              uint16_t rows,
                              uint16_t w_bytes)
{
    memset(p_wait, 0, sizeof(*p_wait));
    p_wait->kind = DISPLAY_WAIT_STREAM;
    p_wait->next = next;
    p_wait->src = src;
    p_wait->x_byte = x_byte;
    p_wait->y = y;
    p_wait->rows = rows;
    p_wait->w_bytes = w_bytes;
    return ESP_OK;
}

/**
 * @brief Answer advance() with the end of the operation.
 *
 * @return esp_err_t
 */
esp_err_t display_wait_done(display_wait_t *p_wait)
{
    memset(p_wait, 0, sizeof(*p_wait));
    p_wait->kind = DISPLAY_WAIT_DONE;
    return ESP_OK;
}
