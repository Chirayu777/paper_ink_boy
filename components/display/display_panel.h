/**
 * @file display_panel.h
 * @brief The panel interface -- what a panel driver supplies so the display core can refresh it.
 *
 * A refresh is a script of steps. The core calls the panel's advance() with the operation and the
 * step it is on; the panel sends whatever commands that step needs, through the display port, and
 * answers with the next thing the refresh has to wait for: a fixed delay, the BUSY line, an image
 * going out, or the end. The core does the waiting without blocking, with its timers, its BUSY
 * debounce, its guards and its notification of the owner task, then calls advance() again with the
 * step the panel asked for. Everything that is the same for every panel is therefore written
 * once, in the core; a panel driver holds only its command sequences, timings and size.
 *
 * Everything electrical is the driver's side of this interface: the panel brings up its own bus
 * settings in init(), through the display port, and the core never sees a clock, a pin or a level.
 *
 * To add a panel, write a driver that defines one display_panel_t and display_select_panel(), add it
 * to the panel choice in display's Kconfig, and list it in display's CMakeLists.txt; the driver
 * compiles its source only when its panel is the one selected.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Includes --------------------------------------------------------------------------------------*/
#include "display_fb.h"

#include "epd_geom.h"

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

/* Public types ----------------------------------------------------------------------------------*/

/**
 * @brief The operations the core asks a panel to run.
 */
typedef enum {
    DISPLAY_OP_NONE, /**< Nothing, or: the state of the glass is not known */
    DISPLAY_OP_FULL, /**< Whole screen, full-quality waveform; clears ghosting */
    DISPLAY_OP_FAST, /**< Whole screen, fast waveform */
    DISPLAY_OP_PARTIAL, /**< A window of the screen */
    DISPLAY_OP_SLEEP, /**< Power the panel down after it has been idle */
    DISPLAY_OP_QUICK, /**< One short scan of a window: only pixels with drive budget left move */
} display_op_t;

/**
 * @brief What a refresh has to wait for next.
 */
typedef enum {
    DISPLAY_WAIT_DELAY, /**< A fixed time */
    DISPLAY_WAIT_BUSY, /**< The BUSY line to report ready, within a timeout */
    DISPLAY_WAIT_STREAM, /**< An image to go out to the panel */
    DISPLAY_WAIT_DONE, /**< Nothing: the operation is complete */
} display_wait_kind_t;

/**
 * @brief The panel's answer to advance(): what to wait for, and where to go on from there.
 */
typedef struct {
    display_wait_kind_t kind; /**< What to wait for */
    uint8_t next; /**< The step to pass to advance() once the wait is over */
    uint32_t delay_ms; /**< DELAY: the time. BUSY: how long to leave BUSY unsampled first */
    uint32_t timeout_ms; /**< BUSY: how long to wait for ready, counted from now */
    display_src_t src; /**< STREAM: where the image comes from */
    uint16_t x_byte; /**< STREAM: left edge of the window, in bytes */
    uint16_t y; /**< STREAM: top row of the window */
    uint16_t rows; /**< STREAM: height of the window, in rows */
    uint16_t w_bytes; /**< STREAM: width of the window, in bytes */
} display_wait_t;

/**
 * @brief What the core knows that the panel needs in order to choose how an operation starts.
 */
typedef struct {
    display_op_t last_mode; /**< Mode of the last refresh that completed; NONE if unknown */
    bool asleep; /**< The panel is powered down and wakes only on a reset */
    epd_rect_t region; /**< PARTIAL, QUICK: the window to refresh, inside the panel */
    /** QUICK, once the scan's data is out: it drives some pixel grey, so it needs the grey row's
     * rest and grey phases */
    bool quick_grey;
} display_request_t;

/**
 * @brief A panel driver.
 */
typedef struct {
    const char *name; /**< Panel and controller, for logs */
    epd_size_t size; /**< Native size in pixels; the width is a multiple of 8 */
    uint32_t emergency_limit_ms; /**< Longest an emergency (fast) refresh may take */
    uint32_t sleep_idle_ms; /**< Idle time after a refresh before the panel is put to sleep */
    /** QUICK: scans a pixel turning black is driven for; 0 if the panel has no quick mode */
    uint8_t quick_budget_black;
    /** QUICK: scans a pixel turning white is driven for; 0 if the panel has no quick mode */
    uint8_t quick_budget_white;

    /**
     * Bring up the hardware this panel needs, through the display port. Touches no panel register.
     */
    esp_err_t (*init)(void);

    /**
     * Run one step of an operation.
     *
     * Step 0 starts the operation. Send the step's commands through display_port_cmd() and
     * display_port_data(), then fill p_wait with what has to be waited for and the step to come
     * back with. A step that does not wait for anything more answers DISPLAY_WAIT_DONE.
     *
     * Return an error to abort the operation; the core then treats the glass as unknown.
     */
    esp_err_t (*advance)(display_op_t op,
                         uint8_t step,
                         const display_request_t *p_request,
                         display_wait_t *p_wait);

    /**
     * Optional, NULL if the panel has none: switch on the panel's power rails, in order. A panel
     * that powers itself through its own commands, as the UC8179C does, leaves this NULL.
     */
    esp_err_t (*power_on)(void);

    /**
     * Optional, NULL if the panel has none: switch the rails off again, in reverse order.
     */
    esp_err_t (*power_off)(void);

    /**
     * Optional, NULL if the panel has none: shut the panel down in the fewest steps that leave it
     * safe, for a brownout. It may run in interrupt context, so it must be IRAM-safe: no logging,
     * no allocation, nothing that blocks. Best effort.
     */
    esp_err_t (*safe_shutdown)(void);
} display_panel_t;

/* Public function declarations ------------------------------------------------------------------*/

/**
 * @brief The panel this board has, for the display core to run.
 *
 * Defined by the panel driver that is built in, so which driver is linked is what selects the
 * panel; the core names none of them.
 *
 * @return The panel driver.
 */
const display_panel_t *display_select_panel(void);

/**
 * @brief Answer advance() with a fixed delay.
 *
 * @param[out] p_wait the answer to fill.
 * @param[in] next step to come back with.
 * @param[in] delay_ms how long to wait.
 * @return ESP_OK.
 */
esp_err_t display_wait_delay(display_wait_t *p_wait, uint8_t next, uint32_t delay_ms);

/**
 * @brief Answer advance() with a wait for the BUSY line.
 *
 * @param[out] p_wait the answer to fill.
 * @param[in] next step to come back with.
 * @param[in] lead_ms how long to leave BUSY unsampled first; 0 starts sampling at once.
 * @param[in] timeout_ms how long to wait for ready, counted from now.
 * @return ESP_OK.
 */
esp_err_t display_wait_busy(display_wait_t *p_wait,
                            uint8_t next,
                            uint32_t lead_ms,
                            uint32_t timeout_ms);

/**
 * @brief Answer advance() with an image to send.
 *
 * The data command that precedes the image has already been sent by the panel.
 *
 * @param[out] p_wait the answer to fill.
 * @param[in] next step to come back with once the image has gone out.
 * @param[in] src where the image comes from.
 * @param[in] x_byte left edge of the window of the framebuffer to send, in bytes.
 * @param[in] y top row of the window.
 * @param[in] rows height of the window, in rows.
 * @param[in] w_bytes width of the window, in bytes.
 * @return ESP_OK.
 */
esp_err_t display_wait_stream(display_wait_t *p_wait,
                              uint8_t next,
                              display_src_t src,
                              uint16_t x_byte,
                              uint16_t y,
                              uint16_t rows,
                              uint16_t w_bytes);

/**
 * @brief Answer advance() with the end of the operation.
 *
 * @param[out] p_wait the answer to fill.
 * @return ESP_OK.
 */
esp_err_t display_wait_done(display_wait_t *p_wait);
