/**
 * @file display.h
 * @brief Public API for the display -- start it, refresh it, and keep it asleep when idle. Drawing
 * is in display_fb.h.
 *
 * The application knows nothing of the hardware behind it: no panel, no pins, no bus. The display
 * picks the board's panel driver itself and the driver brings up the hardware. Drawing calls write
 * into a framebuffer the display owns; a refresh pushes it to the glass.
 *
 * Refreshes never block the caller (ADR-17, C8). display_begin_full/fast/partial() starts one and
 * returns; the display sets the bit registered with display_set_notify() each time a wait step
 * (reset, power-on, refresh BUSY) finishes, and the owner calls display_service() on that bit to
 * run the next step, until it returns ESP_OK (done) or an error (aborted). At most one refresh is
 * in flight. Drawing into the framebuffer while one is in flight is safe: the data already sent is
 * what the glass shows, and the next refresh sends whatever changed since.
 *
 * After a refresh the panel is put to sleep once it has been idle for a while, which panel makers
 * say its life depends on. The sleep is steps of the same kind: the display sets the bit, the
 * owner calls display_service(), and display_busy() is true until it ends. The next refresh wakes
 * the panel itself. A sleep never fails a caller: if it does not finish it is only logged.
 *
 * @note Every function except the drawing calls must be called from the one owner task.
 *
 * @version 0.3
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "epd_geom.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdint.h>

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief Bring up the hardware, allocate the framebuffers and the staging tiles, create the timers.
 * Touches no panel register -- the first refresh must be a whole-screen one (display_begin_fast()
 * or _full()), which runs the reset and power-on.
 *
 * @return ESP_OK, or why something could not be set up.
 * @note Call once. Not thread-safe, not ISR-safe.
 */
esp_err_t display_init(void);

/**
 * @brief The size of the panel, in landscape pixels.
 *
 * @return The size, or 0 by 0 before display_init() has succeeded.
 */
epd_size_t display_get_size(void);

/**
 * @brief Register the task and notification bit the display sets when a wait step finishes.
 *
 * @param[in] task owner task -- the only task allowed to call begin/service.
 * @param[in] bit notification bit set with eSetBits; must be non-zero.
 * @return ESP_OK, or ESP_ERR_INVALID_ARG.
 * @note Call once, before the first begin.
 */
esp_err_t display_set_notify(TaskHandle_t task, uint32_t bit);

/**
 * @brief Begin a full-quality refresh of the whole framebuffer: reset, power on, data, refresh.
 * Clears ghosting, but is the slowest mode and, on some panel units, leaves blacks grey -- use
 * display_begin_fast() to draw a page.
 *
 * @return ESP_OK if started, ESP_ERR_INVALID_STATE if not initialized, no owner registered, or a
 * refresh is already in flight.
 */
esp_err_t display_begin_full(void);

/**
 * @brief Begin a fast refresh of the whole framebuffer. Faster than full, does not clear ghosting.
 * Draws solid black on every panel unit bench-tested, so this is the mode for page draws.
 *
 * @return same as display_begin_full().
 */
esp_err_t display_begin_fast(void);

/**
 * @brief Begin a windowed partial refresh -- the fastest mode, only the region changes. The left
 * edge is rounded down and the width up to the nearest 8px internally. Repeated partials over the
 * same area build up ghosting that only a full or fast refresh clears.
 *
 * @param[in] p_rect region to refresh, in landscape coordinates.
 * @return ESP_OK if started; ESP_ERR_INVALID_STATE as for display_begin_full(), and also before
 * any whole-screen refresh or after a failed one (glass state unknown -- refresh the screen first);
 * ESP_ERR_INVALID_ARG for an empty or off-panel region.
 */
esp_err_t display_begin_partial(const epd_rect_t *p_rect);

/**
 * @brief Begin a quick refresh of a region: a run of short scans that drive only the pixels that
 * changed, each for the panel's drive budget, and hold every other pixel still. The run ends by
 * itself when no pixel has budget left; display_service() reports it as one refresh. Pixels drawn
 * inside the area still being scanned while the run goes on are picked up by its next scan.
 *
 * Quick mode trades depth of black and white for speed, and adds no ghosting clean-up of its own.
 *
 * @param[in] p_rect region to refresh, in landscape coordinates.
 * @return ESP_OK if started; ESP_ERR_NOT_SUPPORTED if the panel has no quick mode (use
 * display_begin_partial()); otherwise as display_begin_partial(), and ESP_ERR_NO_MEM if the drive
 * budgets could not be allocated.
 */
esp_err_t display_begin_quick(const epd_rect_t *p_rect);

/**
 * @brief Begin a quick refresh with drive budgets of the caller's own in place of the panel's.
 *
 * A budget shorter than the panel's stops a changing pixel part of the way to its new shade, which
 * leaves it grey. The framebuffer and the mirror still hold it as black or white, so the next
 * change of that pixel is driven as if it had got all the way.
 *
 * @param[in] p_rect region to refresh, in landscape coordinates.
 * @param[in] budget_black scans for a pixel turning black, 1 to 15; 0 for the panel's own.
 * @param[in] budget_white scans for a pixel turning white, 1 to 15; 0 for the panel's own.
 * @return as display_begin_quick(), and ESP_ERR_INVALID_ARG for a budget above 15.
 */
esp_err_t display_begin_quick_budget(const epd_rect_t *p_rect,
                                     uint8_t budget_black,
                                     uint8_t budget_white);

/**
 * @brief Add an area to the quick refresh in flight, for streaming: draw the newest image into the
 * framebuffer, then call this with the area that changed. The run's next scan covers it, so those
 * pixels get their drive budget and start moving while pixels from earlier changes finish theirs,
 * and the run goes on for as long as areas keep being added.
 *
 * @param[in] p_rect area to add, in landscape coordinates.
 * @return ESP_OK; ESP_ERR_INVALID_STATE when no quick refresh is in flight (begin one instead);
 * ESP_ERR_INVALID_ARG for an empty or off-panel area.
 * @note Owner task only.
 */
esp_err_t display_quick_extend(const epd_rect_t *p_rect);

/**
 * @brief How many quick scans have run since display_init() -- each one is a visible update of
 * the glass, however many images joined it.
 *
 * @return the count; wraps at 2^32.
 * @note Owner task only.
 */
uint32_t display_quick_scan_count(void);

/**
 * @brief Run the in-flight refresh's next step. Call each time the registered bit is set.
 *
 * @return ESP_ERR_NOT_FINISHED while still in flight; ESP_OK when this call completed it (or when
 * nothing was in flight); ESP_ERR_TIMEOUT or a bus/timer error when it failed -- the refresh is
 * aborted and the next one must be whole-screen (fast or full).
 * @note Also runs the panel's idle sleep when it is due, so call it on every notification bit.
 */
esp_err_t display_service(void);

/**
 * @brief Whether a refresh is in flight.
 *
 * @return true between a successful begin and service() returning anything but
 * ESP_ERR_NOT_FINISHED.
 */
bool display_busy(void);

/**
 * @brief Refresh the whole panel from the framebuffer at once, blocking, for the terminal fault
 * screen.
 *
 * Abandons any refresh in flight, resets the panel and runs a fast refresh to the end by polling.
 * It does not depend on the state the display was left in, so it works after the owner task hung
 * mid-refresh.
 *
 * @return ESP_OK when the refresh finished, ESP_ERR_TIMEOUT if it did not within the panel's
 * limit, ESP_ERR_INVALID_STATE if the display was never initialized.
 *
 * @note Call only after the owner task has been suspended: it uses the bus and the framebuffer
 * without the owner's protection. Blocks for about 2 s. Not ISR-safe.
 */
esp_err_t display_emergency_refresh(void);

/**
 * @brief Switch on the panel's power rails, for a panel that has them.
 *
 * @return ESP_OK on success, or if the panel has no rails to switch; ESP_ERR_INVALID_STATE if the
 * display is not initialized or a refresh is in flight; otherwise the panel driver's error.
 * @note Owner task only. Not ISR-safe.
 */
esp_err_t display_power_on(void);

/**
 * @brief Switch off the panel's power rails, for a panel that has them.
 *
 * @return same as display_power_on().
 * @note Owner task only. Not ISR-safe.
 */
esp_err_t display_power_off(void);

/**
 * @brief Shut the panel down in the fewest steps that leave it safe, for a brownout.
 *
 * Touches no display state and checks nothing, so it works whatever the display was doing.
 * Best effort; a panel with no such shutdown, or a display not yet initialized, is not an error.
 *
 * @return ESP_OK, or the panel driver's result.
 * @note Safe from interrupt context: no logging, no allocation, nothing that blocks.
 */
esp_err_t display_safe_shutdown(void);
