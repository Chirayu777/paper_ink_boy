/**
 * @file panel_ssd1677.c
 * @brief panel_ssd1677 implementation -- the SSD1677's refresh sequences as steps for the display
 * core.
 *
 * Ported from Good Display's GDEQ0426T82-T01C Arduino demo (A32-GDEQ0426T82-T01C). The controller's
 * RAM is addressed in the panel's native landscape layout: X runs along the 800 pixel side, in
 * pixels, Y along the 480 gate lines, and an image goes in row after row, which is the
 * framebuffer's own layout, with Y running backwards to mirror it the right way round on this board
 * (see PANEL_RAM_Y()). RAM bit 1 is white, as in the framebuffer, so no data is inverted.
 *
 * Full mode runs the factory OTP waveform (update control 0xF7). The demo's full refresh instead
 * reads the controller's temperature sensor back over SPI to pick one of five register LUTs; the
 * display port only writes, so the OTP waveform, which is temperature-compensated by the controller
 * itself, takes its place.
 *
 * Fast mode follows the demo's EPD_HW_Init_Fast() then EPD_Update_Fast(): the OTP is loaded at a
 * forced temperature, then the demo's fast register LUT (WS_80_127) and its drive voltages replace
 * it, and the update (0xC7) runs from the registers.
 *
 * Partial mode runs the OTP's display-mode-2 waveform (0xFC), which drives only the pixels whose
 * old data (RAM 0x26) and new data (RAM 0x24) differ -- over the whole panel, not only the RAM
 * window. Full and fast refreshes therefore write the image into both RAMs, as the demo's
 * EPD_SetRAMValue_BaseMap() does: left uninitialised, the old RAM made every partial refresh drive
 * the whole screen through noise (seen on this board, 2026-10-31). Both RAMs are written for every
 * window too, the old from the display core's mirror of the glass, so nothing depends on the
 * controller copying one RAM into the other. 0xFC leaves the analog supply on, so repeated partial
 * refreshes skip both the reset and the power-up; the sleep step switches it off. A partial
 * refresh after any other mode, or after sleep, starts with RST, as every partial refresh in the
 * demo does: it puts the registers fast mode changed back.
 *
 * Quick mode runs display mode 2 from a register LUT of its own (0xCC: no OTP load, and the analog
 * supply stays on between scans). A pixel's old bit (RAM 0x26) and new bit (RAM 0x24) pick its LUT
 * row, row = old * 2 + new, which is what the demo's LUTs show (rows 0 and 2 alike, 1 and 3 alike,
 * new bit 0 = black). The display core's quick sources encode held as 1,1, driven white as 0,1,
 * driven black as 1,0 and driven grey as 0,0, so: LUT3 holds, LUT1 drives VSL (toward white), LUT2
 * drives VSH1 (toward black, the polarity the demo's LUTs end each black row on), and LUT0 drives
 * white, rests, then drives black for the grey frames. Mode 2 drives every pixel of the panel, not
 * only the RAM window, so the RAM outside the window must hold 1,1: the first scan after any other
 * mode writes the whole panel, and every scan after it also covers the previous scan's window, whose
 * pixels with no budget left the quick sources send as 1,1. The timings are bench tunables (see
 * CMakeLists.txt).
 *
 * BUSY is high while the controller is busy. None of this panel's refresh times have been measured
 * on this board yet, so BUSY is sampled from the trigger and every timeout is a stuck-panel guard;
 * measure with idf.py -DPIB_DISPLAY_TIMING=1 and add pre-delays as panel_uc8179c has.
 *
 * @version 0.1
 * @date 2026-10-31
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "panel_ssd1677.h"

#include "board_config.h"
#include "display_port.h"

/* Private definitions ---------------------------------------------------------------------------*/

#define PANEL_WIDTH         (800U)
#define PANEL_HEIGHT        (480U)
#define PANEL_WIDTH_BYTES   (PANEL_WIDTH / 8U)
#define PANEL_FIRST_STEP    (0U)

/* Timeouts are stuck-panel guards, not measured times (see the file comment). */
#define PANEL_BUSY_BOOT_TIMEOUT_MS      (1000U) /* after RST, and after SWRESET */
#define PANEL_BUSY_FULL_TIMEOUT_MS      (5000U)
#define PANEL_BUSY_FAST_TIMEOUT_MS      (3000U)
#define PANEL_BUSY_PARTIAL_TIMEOUT_MS   (2000U)
#define PANEL_BUSY_LOAD_TIMEOUT_MS      (1000U) /* OTP load at forced temperature, LUT write */
#define PANEL_BUSY_POWER_OFF_TIMEOUT_MS (1000U)
/* BUSY rises a moment after a command; it is not sampled before this. */
#define PANEL_BUSY_LEAD_MS       (10U)
#define PANEL_RST_PULSE_MS       (10U) /* demo: at least 10 ms */
#define PANEL_RST_SETTLE_MS      (10U) /* demo: at least 10 ms before BUSY is read */
/* The emergency refresh is a fast refresh; this is its overall limit. */
#define PANEL_EMERGENCY_LIMIT_MS (4000U)
/* How long the panel stays powered after a refresh with nothing following it. */
#define PANEL_SLEEP_IDLE_MS      (3000U)

/* The SSD1677's minimum write clock cycle is 50 ns, so 20 MHz is within its limit; 10 MHz is what
 * this board's display connector has been run at with the 7.5" panel. */
#define PANEL_SPI_CLOCK_HZ       (10U * 1000U * 1000U)
/* BUSY is high while the panel is busy and low when it is ready. */
#define PANEL_BUSY_READY_LEVEL   (0U)
/* Same connector and BUSY trace as the 7.5" panel, but no glitch has been filtered on this panel
 * (2026-10-31, timing monitor), and every quick scan pays the debounce twice, so it is shorter than
 * panel_uc8179c's 5 ms. */
#define PANEL_BUSY_DEBOUNCE_US   (1000U)

/* Quick mode, see the file comment. A scan drives for PANEL_QUICK_FRAMES frames; a scan that
 * drives a pixel grey then rests PANEL_QUICK_REST_FRAMES and drives PANEL_QUICK_GREY_FRAMES toward
 * black. PANEL_QUICK_FR is the frame rate setting of the LUT (the demo's is 2). A changed pixel is
 * driven for its budget of scans, which the application sets. */
#ifndef PANEL_QUICK_FRAMES
#define PANEL_QUICK_FRAMES      (3U)
#endif
#ifndef PANEL_QUICK_REST_FRAMES
#define PANEL_QUICK_REST_FRAMES (2U)
#endif
#ifndef PANEL_QUICK_GREY_FRAMES
#define PANEL_QUICK_GREY_FRAMES (1U)
#endif
#ifndef PANEL_QUICK_FR
#define PANEL_QUICK_FR          (2U)
#endif
#define PANEL_QUICK_BUDGET      (1U) /* the application passes its own */
#define PANEL_BUSY_QUICK_TIMEOUT_MS (1500U)
/* LUT layout: 5 rows of 10 level bytes (one per group, 2 bits per phase A-D, A in bits 7-6), then
 * 10 groups of 5 timing bytes (TPA, TPB, TPC, TPD, repeat), then 5 frame rate bytes (a nibble per
 * group). Quick mode uses group 0. */
#define PANEL_LUT_ROWS          (5U)
#define PANEL_LUT_GROUPS        (10U)
#define PANEL_LUT_GROUP_BYTES   (5U)
#define PANEL_LUT_TIMING        (PANEL_LUT_ROWS * PANEL_LUT_GROUPS)
#define PANEL_LUT_FR            (PANEL_LUT_TIMING + (PANEL_LUT_GROUPS * PANEL_LUT_GROUP_BYTES))
#define PANEL_LEVEL_VSH1        (0x1U) /* drives toward black */
#define PANEL_LEVEL_VSL         (0x2U) /* drives toward white */
#define PANEL_LEVEL(a, c)       ((uint8_t)(((a) << 6) | ((c) << 2)))
/* The quick LUT the registers hold: with the grey phases or not, or not known yet. */
#define PANEL_QUICK_LUT_PLAIN   (0U)
#define PANEL_QUICK_LUT_GREY    (1U)
#define PANEL_QUICK_LUT_UNKNOWN (0xFFU)

/* SSD1677 command bytes. */
#define PANEL_CMD_DRIVER_OUTPUT     (0x01U)
#define PANEL_CMD_GATE_VOLTAGE      (0x03U)
#define PANEL_CMD_SOURCE_VOLTAGE    (0x04U)
#define PANEL_CMD_BOOSTER_SOFT_START (0x0CU)
#define PANEL_CMD_DEEP_SLEEP        (0x10U)
#define PANEL_CMD_DATA_ENTRY_MODE   (0x11U)
#define PANEL_CMD_SW_RESET          (0x12U)
#define PANEL_CMD_TEMP_SENSOR       (0x18U)
#define PANEL_CMD_TEMP_WRITE        (0x1AU)
#define PANEL_CMD_MASTER_ACTIVATION (0x20U)
#define PANEL_CMD_UPDATE_CONTROL_2  (0x22U)
#define PANEL_CMD_WRITE_RAM_BW      (0x24U)
#define PANEL_CMD_WRITE_RAM_RED     (0x26U)
#define PANEL_CMD_VCOM              (0x2CU)
#define PANEL_CMD_WRITE_LUT         (0x32U)
#define PANEL_CMD_BORDER_WAVEFORM   (0x3CU)
#define PANEL_CMD_RAM_X_RANGE       (0x44U)
#define PANEL_CMD_RAM_Y_RANGE       (0x45U)
#define PANEL_CMD_RAM_X_COUNTER     (0x4EU)
#define PANEL_CMD_RAM_Y_COUNTER     (0x4FU)

/* Update control 2 sequences. */
#define PANEL_UPDATE_FULL      (0xF7U) /* clock, analog, temperature, OTP LUT, mode 1, power down */
#define PANEL_UPDATE_FAST      (0xC7U) /* clock, analog, mode 1 from the register LUT, power down */
#define PANEL_UPDATE_PARTIAL   (0xFCU) /* clock, analog, temperature, OTP LUT, mode 2, stays on */
#define PANEL_UPDATE_QUICK     (0xCCU) /* clock, analog, mode 2 from the register LUT, stays on */
#define PANEL_UPDATE_LOAD_OTP  (0x91U) /* clock, load the OTP LUT, oscillator off */
#define PANEL_UPDATE_POWER_OFF (0x83U) /* clock, then analog and oscillator off */

/* Forced temperature for fast mode's OTP load (demo: 0x5A). */
#define PANEL_FAST_TEMPERATURE (0x5AU)
/* Border waveform: full and fast modes follow LUT 1, partial mode holds the border at VCOM. */
#define PANEL_BORDER_FULL      (0x01U)
#define PANEL_BORDER_PARTIAL   (0x80U)

/* Bytes of the waveform register (0x32) in a demo LUT; the bytes after them are the gate voltage,
 * the 3 source voltages and VCOM. */
#define PANEL_LUT_BYTES (105U)

/* The demo's layout (gate scan 0x02, X and Y incrementing) showed the framebuffer mirrored along Y
 * on this board (2026-10-31). Reversing the gate scan (TB) mirrored it but also shifted it by the
 * 200 gates the SSD1677 has beyond this panel's 480, so the mirror is done in RAM instead: Y
 * decrements, and framebuffer row y lands in RAM row PANEL_RAM_Y(y). */
#define PANEL_GATE_SCAN       (0x02U)
#define PANEL_DATA_ENTRY_MODE (0x01U) /* X incrementing, Y decrementing */
#define PANEL_RAM_Y(y)        ((uint16_t)(PANEL_HEIGHT - 1U - (y)))

/* Command scripts, in the form panel_run() takes: command, number of data bytes, the data bytes. */
/* Internal temperature sensor, booster soft start, 480 gate lines, data entry mode, border. */
#define PANEL_SCRIPT_INIT(border)                                                               \
    PANEL_CMD_TEMP_SENSOR, 1U, 0x80U, PANEL_CMD_BOOSTER_SOFT_START, 5U, 0xAEU, 0xC7U, 0xC3U,    \
        0xC0U, 0x80U, PANEL_CMD_DRIVER_OUTPUT, 3U, (uint8_t)((PANEL_HEIGHT - 1U) % 256U),       \
        (uint8_t)((PANEL_HEIGHT - 1U) / 256U), PANEL_GATE_SCAN, PANEL_CMD_DATA_ENTRY_MODE, 1U,  \
        PANEL_DATA_ENTRY_MODE, PANEL_CMD_BORDER_WAVEFORM, 1U, (border)
#define PANEL_SCRIPT_SW_RESET   PANEL_CMD_SW_RESET, 0U
#define PANEL_SCRIPT_UPDATE(seq) PANEL_CMD_UPDATE_CONTROL_2, 1U, (seq), PANEL_CMD_MASTER_ACTIVATION, 0U
#define PANEL_SCRIPT_FAST_LOAD \
    PANEL_CMD_TEMP_WRITE, 1U, PANEL_FAST_TEMPERATURE, PANEL_SCRIPT_UPDATE(PANEL_UPDATE_LOAD_OTP)
#define PANEL_SCRIPT_DEEP_SLEEP PANEL_CMD_DEEP_SLEEP, 1U, 0x01U

/* How long a wait for BUSY leaves it unsampled, then waits for it, as the two arguments the helpers
 * take. */
#define PANEL_BOOT_BUSY      PANEL_RST_SETTLE_MS, PANEL_BUSY_BOOT_TIMEOUT_MS
#define PANEL_SW_RESET_BUSY  PANEL_BUSY_LEAD_MS, PANEL_BUSY_BOOT_TIMEOUT_MS
#define PANEL_LOAD_BUSY      PANEL_BUSY_LEAD_MS, PANEL_BUSY_LOAD_TIMEOUT_MS
#define PANEL_FULL_BUSY      PANEL_BUSY_LEAD_MS, PANEL_BUSY_FULL_TIMEOUT_MS
#define PANEL_FAST_BUSY      PANEL_BUSY_LEAD_MS, PANEL_BUSY_FAST_TIMEOUT_MS
#define PANEL_PARTIAL_BUSY   PANEL_BUSY_LEAD_MS, PANEL_BUSY_PARTIAL_TIMEOUT_MS
#define PANEL_QUICK_BUSY     0U, PANEL_BUSY_QUICK_TIMEOUT_MS
#define PANEL_POWER_OFF_BUSY PANEL_BUSY_LEAD_MS, PANEL_BUSY_POWER_OFF_TIMEOUT_MS

/* Private types ---------------------------------------------------------------------------------*/

/**
 * @brief The steps of a full refresh; each is the one advance() is called with.
 */
typedef enum {
    FULL_BEGIN = PANEL_FIRST_STEP,
    FULL_RELEASE,
    FULL_SW_RESET,
    FULL_DATA,
    FULL_DATA_OLD,
    FULL_REFRESH,
    FULL_DONE,
} full_step_t;

/**
 * @brief The steps of a fast refresh.
 */
typedef enum {
    FAST_BEGIN = PANEL_FIRST_STEP,
    FAST_RELEASE,
    FAST_SW_RESET,
    FAST_LOAD,
    FAST_LUT,
    FAST_DATA,
    FAST_DATA_OLD,
    FAST_REFRESH,
    FAST_DONE,
} fast_step_t;

/**
 * @brief The steps of a partial refresh.
 */
typedef enum {
    PART_BEGIN = PANEL_FIRST_STEP,
    PART_RELEASE,
    PART_INIT,
    PART_DATA_NEW,
    PART_REFRESH,
    PART_DONE,
} part_step_t;

/**
 * @brief The steps of one quick scan.
 */
typedef enum {
    QUICK_BEGIN = PANEL_FIRST_STEP,
    QUICK_RELEASE,
    QUICK_INIT,
    QUICK_DATA_NEW,
    QUICK_REFRESH,
    QUICK_DONE,
} quick_step_t;

/**
 * @brief The steps of the sleep.
 */
typedef enum {
    SLEEP_POWER_OFF = PANEL_FIRST_STEP,
    SLEEP_DEEP,
} sleep_step_t;

/* Private function declarations -----------------------------------------------------------------*/

static esp_err_t panel_init(void);
static esp_err_t panel_advance(display_op_t op,
                               uint8_t step,
                               const display_request_t *p_request,
                               display_wait_t *p_wait);
static esp_err_t panel_run(const uint8_t *p_script, size_t len);
static esp_err_t panel_run_busy(const uint8_t *p_script,
                                size_t len,
                                display_wait_t *p_wait,
                                uint8_t next,
                                uint32_t lead_ms,
                                uint32_t timeout_ms);
static esp_err_t panel_full(uint8_t step, display_wait_t *p_wait);
static esp_err_t panel_fast(uint8_t step, display_wait_t *p_wait);
static esp_err_t panel_partial(uint8_t step,
                               const display_request_t *p_request,
                               display_wait_t *p_wait);
static esp_err_t panel_quick(uint8_t step,
                             const display_request_t *p_request,
                             display_wait_t *p_wait);
static esp_err_t panel_quick_lut(bool grey);
static esp_err_t panel_sleep(uint8_t step, display_wait_t *p_wait);
static epd_rect_t panel_rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b);
static esp_err_t panel_area(const epd_rect_t *p_rect);
static esp_err_t panel_ram(const epd_rect_t *p_rect,
                           uint8_t ram_cmd,
                           display_src_t src,
                           uint8_t next,
                           display_wait_t *p_wait);
static void panel_window_bytes(const epd_rect_t *p_rect, uint16_t *p_x_byte, uint16_t *p_w_bytes);

/* Private variables -----------------------------------------------------------------------------*/

/* The whole panel, as a window. */
static const epd_rect_t s_whole = {0U, 0U, PANEL_WIDTH, PANEL_HEIGHT};

/* The demo's fast waveform (WS_80_127) as a script: the waveform register, then the gate voltage,
 * source voltages and VCOM that follow it in the demo's table. */
static const uint8_t s_fast_lut[] = {
    PANEL_CMD_WRITE_LUT, PANEL_LUT_BYTES,
    0xA8U, 0x00U, 0x55U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x54U, 0x00U, 0xAAU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0xA8U, 0x00U, 0x55U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x54U, 0x00U, 0xAAU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x0CU, 0x0DU, 0x0BU, 0x01U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x0AU, 0x0AU, 0x05U, 0x0BU, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x01U, 0x01U,
    0x22U, 0x22U, 0x22U, 0x22U, 0x22U,
};
/* Also quick mode's drive voltages. */
static const uint8_t s_fast_voltages[] = {
    PANEL_CMD_GATE_VOLTAGE, 1U, 0x17U,
    PANEL_CMD_SOURCE_VOLTAGE, 3U, 0x41U, 0xA8U, 0x32U,
    PANEL_CMD_VCOM, 1U, 0x30U,
};

/* Which quick LUT the registers hold, so it is uploaded only when a scan needs the other one. */
static uint8_t s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
/* The window of the scan in progress, and of the last scan: the RAM outside it holds 1,1. */
static epd_rect_t s_quick_window;
static epd_rect_t s_quick_written;

/* Public variables ------------------------------------------------------------------------------*/

const display_panel_t g_panel_ssd1677_gdeq0426t82 = {
    .name = "SSD1677 GDEQ0426T82-T01C",
    .size = {PANEL_WIDTH, PANEL_HEIGHT},
    .emergency_limit_ms = PANEL_EMERGENCY_LIMIT_MS,
    .sleep_idle_ms = PANEL_SLEEP_IDLE_MS,
    .quick_budget_black = PANEL_QUICK_BUDGET,
    .quick_budget_white = PANEL_QUICK_BUDGET,
    .init = panel_init,
    .advance = panel_advance,
};

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief The panel this board has, for the display core to run.
 *
 * Every panel driver defines this function, so linking one panel driver into the build is what
 * selects it; linking two is a duplicate-symbol error.
 *
 * @return const display_panel_t *
 */
const display_panel_t *display_select_panel(void)
{
    return &g_panel_ssd1677_gdeq0426t82;
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Bring up the display's hardware: the board's pins, and what this controller needs of the
 * bus and the BUSY line.
 *
 * @return esp_err_t
 */
static esp_err_t panel_init(void)
{
    const display_port_config_t config = {
        .pin_cs = EPD_PIN_CS,
        .pin_dc = EPD_PIN_DC,
        .pin_rst = EPD_PIN_RES,
        .pin_busy = EPD_PIN_BUSY,
        .busy_ready = PANEL_BUSY_READY_LEVEL,
        .spi_clock_hz = PANEL_SPI_CLOCK_HZ,
        .busy_debounce_us = PANEL_BUSY_DEBOUNCE_US,
    };

    return display_port_init(&config);
}

/**
 * @brief Run one step of an operation.
 *
 * @param[in] op the operation.
 * @param[in] step the step to run.
 * @param[in] p_request what the core knows about the panel and the window.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_advance(display_op_t op,
                               uint8_t step,
                               const display_request_t *p_request,
                               display_wait_t *p_wait)
{
    switch (op) {
    case DISPLAY_OP_FULL:
        return panel_full(step, p_wait);

    case DISPLAY_OP_FAST:
        return panel_fast(step, p_wait);

    case DISPLAY_OP_PARTIAL:
        return panel_partial(step, p_request, p_wait);

    case DISPLAY_OP_SLEEP:
        return panel_sleep(step, p_wait);

    case DISPLAY_OP_QUICK:
        return panel_quick(step, p_request, p_wait);

    case DISPLAY_OP_NONE:
    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief Send a script of commands: for each, the command byte, the number of data bytes that
 * follow it, then those bytes. Stops at the first transfer that fails.
 *
 * @param[in] p_script the script.
 * @param[in] len length of the script in bytes.
 * @return esp_err_t
 */
static esp_err_t panel_run(const uint8_t *p_script, size_t len)
{
    size_t i = 0U;
    while (i < len) {
        esp_err_t ret = display_port_cmd(p_script[i]);
        if (ret != ESP_OK) {
            return ret;
        }

        size_t count = p_script[i + 1U];
        for (size_t d = 0U; d < count; d++) {
            ret = display_port_data(p_script[i + 2U + d]);
            if (ret != ESP_OK) {
                return ret;
            }
        }
        i += 2U + count;
    }
    return ESP_OK;
}

/**
 * @brief Send a script, then wait for BUSY.
 *
 * @param[in] p_script the script.
 * @param[in] len length of the script in bytes.
 * @param[out] p_wait what to wait for next.
 * @param[in] next the step to come back with.
 * @param[in] lead_ms how long to leave BUSY unsampled first.
 * @param[in] timeout_ms how long to wait for ready.
 * @return esp_err_t
 */
static esp_err_t panel_run_busy(const uint8_t *p_script,
                                size_t len,
                                display_wait_t *p_wait,
                                uint8_t next,
                                uint32_t lead_ms,
                                uint32_t timeout_ms)
{
    esp_err_t ret = panel_run(p_script, len);
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_busy(p_wait, next, lead_ms, timeout_ms);
}

/**
 * @brief A full refresh, from the demo's EPD_HW_Init(), with the OTP waveform (see the file
 * comment).
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_full(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t sw_reset[] = {PANEL_SCRIPT_SW_RESET};
    const uint8_t init[] = {PANEL_SCRIPT_INIT(PANEL_BORDER_FULL)};
    const uint8_t refresh[] = {PANEL_SCRIPT_UPDATE(PANEL_UPDATE_FULL)};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case FULL_BEGIN:
        display_port_rst(false);
        return display_wait_delay(p_wait, FULL_RELEASE, PANEL_RST_PULSE_MS);

    case FULL_RELEASE:
        display_port_rst(true);
        return display_wait_busy(p_wait, FULL_SW_RESET, PANEL_BOOT_BUSY);

    case FULL_SW_RESET:
        return panel_run_busy(sw_reset, sizeof(sw_reset), p_wait, FULL_DATA, PANEL_SW_RESET_BUSY);

    case FULL_DATA:
        ret = panel_run(init, sizeof(init));
        if (ret == ESP_OK) {
            ret = panel_area(&s_whole);
        }
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_ram(&s_whole, PANEL_CMD_WRITE_RAM_BW, DISPLAY_SRC_FRAME, FULL_DATA_OLD, p_wait);

    case FULL_DATA_OLD:
        return panel_ram(&s_whole, PANEL_CMD_WRITE_RAM_RED, DISPLAY_SRC_PREV, FULL_REFRESH, p_wait);

    case FULL_REFRESH:
        return panel_run_busy(refresh, sizeof(refresh), p_wait, FULL_DONE, PANEL_FULL_BUSY);

    case FULL_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief A fast refresh, from the demo's EPD_HW_Init_Fast() then EPD_Update_Fast(). The image goes
 * into both RAMs, so the partial refreshes after it find old and new equal outside their window.
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_fast(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t sw_reset[] = {PANEL_SCRIPT_SW_RESET};
    const uint8_t init[] = {PANEL_SCRIPT_INIT(PANEL_BORDER_FULL)};
    const uint8_t load[] = {PANEL_SCRIPT_FAST_LOAD};
    const uint8_t refresh[] = {PANEL_SCRIPT_UPDATE(PANEL_UPDATE_FAST)};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case FAST_BEGIN:
        display_port_rst(false);
        return display_wait_delay(p_wait, FAST_RELEASE, PANEL_RST_PULSE_MS);

    case FAST_RELEASE:
        display_port_rst(true);
        return display_wait_busy(p_wait, FAST_SW_RESET, PANEL_BOOT_BUSY);

    case FAST_SW_RESET:
        return panel_run_busy(sw_reset, sizeof(sw_reset), p_wait, FAST_LOAD, PANEL_SW_RESET_BUSY);

    case FAST_LOAD:
        ret = panel_run(init, sizeof(init));
        if (ret == ESP_OK) {
            ret = panel_area(&s_whole);
        }
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_run_busy(load, sizeof(load), p_wait, FAST_LUT, PANEL_LOAD_BUSY);

    case FAST_LUT:
        return panel_run_busy(s_fast_lut, sizeof(s_fast_lut), p_wait, FAST_DATA, PANEL_LOAD_BUSY);

    case FAST_DATA:
        ret = panel_run(s_fast_voltages, sizeof(s_fast_voltages));
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_ram(&s_whole, PANEL_CMD_WRITE_RAM_BW, DISPLAY_SRC_FRAME, FAST_DATA_OLD, p_wait);

    case FAST_DATA_OLD:
        return panel_ram(&s_whole, PANEL_CMD_WRITE_RAM_RED, DISPLAY_SRC_PREV, FAST_REFRESH, p_wait);

    case FAST_REFRESH:
        return panel_run_busy(refresh, sizeof(refresh), p_wait, FAST_DONE, PANEL_FAST_BUSY);

    case FAST_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief A partial refresh, from the demo's EPD_Dis_Part(). Resets first unless the previous
 * refresh was partial too and the panel has not slept since (see the file comment).
 *
 * @param[in] step the step to run.
 * @param[in] p_request the window, and how the panel was left.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_partial(uint8_t step,
                               const display_request_t *p_request,
                               display_wait_t *p_wait)
{
    const uint8_t init[] = {PANEL_SCRIPT_INIT(PANEL_BORDER_PARTIAL)};
    const uint8_t refresh[] = {PANEL_SCRIPT_UPDATE(PANEL_UPDATE_PARTIAL)};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case PART_BEGIN:
        if ((p_request->asleep == false) && (p_request->last_mode == DISPLAY_OP_PARTIAL)) {
            ret = panel_area(&p_request->region);
            if (ret != ESP_OK) {
                return ret;
            }
            return panel_ram(&p_request->region,
                             PANEL_CMD_WRITE_RAM_RED,
                             DISPLAY_SRC_PREV,
                             PART_DATA_NEW,
                             p_wait);
        }
        display_port_rst(false);
        return display_wait_delay(p_wait, PART_RELEASE, PANEL_RST_PULSE_MS);

    case PART_RELEASE:
        display_port_rst(true);
        return display_wait_busy(p_wait, PART_INIT, PANEL_BOOT_BUSY);

    case PART_INIT:
        ret = panel_run(init, sizeof(init));
        if (ret == ESP_OK) {
            ret = panel_area(&p_request->region);
        }
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_ram(&p_request->region,
                         PANEL_CMD_WRITE_RAM_RED,
                         DISPLAY_SRC_PREV,
                         PART_DATA_NEW,
                         p_wait);

    case PART_DATA_NEW:
        return panel_ram(&p_request->region,
                         PANEL_CMD_WRITE_RAM_BW,
                         DISPLAY_SRC_FRAME,
                         PART_REFRESH,
                         p_wait);

    case PART_REFRESH:
        return panel_run_busy(refresh, sizeof(refresh), p_wait, PART_DONE, PANEL_PARTIAL_BUSY);

    case PART_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief One quick scan. The first scan after another mode, or after sleep, resets, sets the panel
 * up and writes the whole panel; the scans after it go straight to their window (see the file
 * comment).
 *
 * @param[in] step the step to run.
 * @param[in] p_request the window, and how the panel was left.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_quick(uint8_t step,
                             const display_request_t *p_request,
                             display_wait_t *p_wait)
{
    const uint8_t init[] = {PANEL_SCRIPT_INIT(PANEL_BORDER_PARTIAL)};
    const uint8_t refresh[] = {PANEL_SCRIPT_UPDATE(PANEL_UPDATE_QUICK)};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case QUICK_BEGIN:
        if ((p_request->asleep == false) && (p_request->last_mode == DISPLAY_OP_QUICK)) {
            s_quick_window = panel_rect_union(&p_request->region, &s_quick_written);
            ret = panel_area(&s_quick_window);
            if (ret != ESP_OK) {
                return ret;
            }
            return panel_ram(&s_quick_window,
                             PANEL_CMD_WRITE_RAM_RED,
                             DISPLAY_SRC_QUICK_OLD,
                             QUICK_DATA_NEW,
                             p_wait);
        }
        display_port_rst(false);
        return display_wait_delay(p_wait, QUICK_RELEASE, PANEL_RST_PULSE_MS);

    case QUICK_RELEASE:
        display_port_rst(true);
        return display_wait_busy(p_wait, QUICK_INIT, PANEL_BOOT_BUSY);

    case QUICK_INIT:
        s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
        s_quick_window = s_whole;
        ret = panel_run(init, sizeof(init));
        if (ret == ESP_OK) {
            ret = panel_run(s_fast_voltages, sizeof(s_fast_voltages));
        }
        if (ret == ESP_OK) {
            ret = panel_area(&s_quick_window);
        }
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_ram(&s_quick_window,
                         PANEL_CMD_WRITE_RAM_RED,
                         DISPLAY_SRC_QUICK_OLD,
                         QUICK_DATA_NEW,
                         p_wait);

    case QUICK_DATA_NEW:
        return panel_ram(&s_quick_window,
                         PANEL_CMD_WRITE_RAM_BW,
                         DISPLAY_SRC_QUICK_NEW,
                         QUICK_REFRESH,
                         p_wait);

    case QUICK_REFRESH:
        /* The pixels of the window outside the region had no budget, so they went out as 1,1 */
        s_quick_written = p_request->region;
        ret = panel_quick_lut(p_request->quick_grey);
        if (ret == ESP_OK) {
            ret = panel_run(refresh, sizeof(refresh));
        }
        if (ret != ESP_OK) {
            return ret;
        }
        return display_wait_busy(p_wait, QUICK_DONE, PANEL_QUICK_BUSY);

    case QUICK_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief Make the LUT register hold the quick LUT a scan needs, uploading it only when it holds
 * something else. A scan with no pixel driven grey gets the plain LUT, phase A only, and does not
 * wait out the rest and grey frames.
 *
 * @param[in] grey whether the scan drives any pixel grey.
 * @return esp_err_t
 */
static esp_err_t panel_quick_lut(bool grey)
{
    const uint8_t want = (grey == true) ? PANEL_QUICK_LUT_GREY : PANEL_QUICK_LUT_PLAIN;
    if (s_quick_lut == want) {
        return ESP_OK;
    }

    uint8_t lut[2U + PANEL_LUT_BYTES] = {PANEL_CMD_WRITE_LUT, PANEL_LUT_BYTES};
    uint8_t *p_lut = &lut[2U];
    /* Row = old * 2 + new: grey, driven white, driven black, held; row 4 (VCOM) stays 0 */
    p_lut[0U * PANEL_LUT_GROUPS] = PANEL_LEVEL(PANEL_LEVEL_VSL, PANEL_LEVEL_VSH1);
    p_lut[1U * PANEL_LUT_GROUPS] = PANEL_LEVEL(PANEL_LEVEL_VSL, 0U);
    p_lut[2U * PANEL_LUT_GROUPS] = PANEL_LEVEL(PANEL_LEVEL_VSH1, 0U);
    p_lut[PANEL_LUT_TIMING] = PANEL_QUICK_FRAMES;
    if (grey == true) {
        p_lut[PANEL_LUT_TIMING + 1U] = PANEL_QUICK_REST_FRAMES;
        p_lut[PANEL_LUT_TIMING + 2U] = PANEL_QUICK_GREY_FRAMES;
    }
    for (uint32_t i = 0U; i < (PANEL_LUT_BYTES - PANEL_LUT_FR); i++) {
        p_lut[PANEL_LUT_FR + i] = (uint8_t)((PANEL_QUICK_FR << 4) | PANEL_QUICK_FR);
    }

    s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
    esp_err_t ret = panel_run(lut, sizeof(lut));
    if (ret == ESP_OK) {
        s_quick_lut = want;
    }
    return ret;
}

/**
 * @brief The sleep: analog supply and oscillator off, then deep sleep mode 1, which keeps the RAM.
 * Only a reset wakes the panel from it.
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_sleep(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t power_off[] = {PANEL_SCRIPT_UPDATE(PANEL_UPDATE_POWER_OFF)};
    const uint8_t deep_sleep[] = {PANEL_SCRIPT_DEEP_SLEEP};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case SLEEP_POWER_OFF:
        return panel_run_busy(power_off,
                              sizeof(power_off),
                              p_wait,
                              SLEEP_DEEP,
                              PANEL_POWER_OFF_BUSY);

    case SLEEP_DEEP:
        ret = panel_run(deep_sleep, sizeof(deep_sleep));
        if (ret != ESP_OK) {
            return ret;
        }
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief The smallest rectangle holding two rectangles.
 *
 * @param[in] p_a one rectangle.
 * @param[in] p_b the other.
 * @return epd_rect_t
 */
static epd_rect_t panel_rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b)
{
    const uint32_t x0 = (p_a->x < p_b->x) ? p_a->x : p_b->x;
    const uint32_t y0 = (p_a->y < p_b->y) ? p_a->y : p_b->y;
    const uint32_t ax1 = (uint32_t)p_a->x + p_a->w;
    const uint32_t bx1 = (uint32_t)p_b->x + p_b->w;
    const uint32_t ay1 = (uint32_t)p_a->y + p_a->h;
    const uint32_t by1 = (uint32_t)p_b->y + p_b->h;
    const epd_rect_t u = {
        .x = (uint16_t)x0,
        .y = (uint16_t)y0,
        .w = (uint16_t)(((ax1 > bx1) ? ax1 : bx1) - x0),
        .h = (uint16_t)(((ay1 > by1) ? ay1 : by1) - y0),
    };
    return u;
}

/**
 * @brief Set the RAM window to a region, byte-aligned along X. The controller takes X in pixels.
 * Y decrements, so the window runs from the region's top row's RAM row down to its bottom row's.
 *
 * @param[in] p_rect the region.
 * @return esp_err_t
 */
static esp_err_t panel_area(const epd_rect_t *p_rect)
{
    uint16_t x_byte = 0U;
    uint16_t w_bytes = 0U;
    panel_window_bytes(p_rect, &x_byte, &w_bytes);

    uint16_t x_start = (uint16_t)(x_byte * 8U);
    uint16_t x_end = (uint16_t)(x_start + (w_bytes * 8U) - 1U);
    uint16_t y_start = PANEL_RAM_Y(p_rect->y);
    uint16_t y_end = PANEL_RAM_Y(p_rect->y + p_rect->h - 1U);
    const uint8_t area[] = {PANEL_CMD_RAM_X_RANGE,
                            4U,
                            (uint8_t)(x_start % 256U),
                            (uint8_t)(x_start / 256U),
                            (uint8_t)(x_end % 256U),
                            (uint8_t)(x_end / 256U),
                            PANEL_CMD_RAM_Y_RANGE,
                            4U,
                            (uint8_t)(y_start % 256U),
                            (uint8_t)(y_start / 256U),
                            (uint8_t)(y_end % 256U),
                            (uint8_t)(y_end / 256U)};
    return panel_run(area, sizeof(area));
}

/**
 * @brief Put the RAM address counter at a region's top-left corner, then send the region into one
 * of the RAMs. The counter is set every time because writing one RAM leaves it at the window's
 * end.
 *
 * @param[in] p_rect the region; panel_area() has set the window to it.
 * @param[in] ram_cmd PANEL_CMD_WRITE_RAM_BW (new data) or PANEL_CMD_WRITE_RAM_RED (old data).
 * @param[in] src where the data comes from.
 * @param[in] next the step to come back with once the data has gone out.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_ram(const epd_rect_t *p_rect,
                           uint8_t ram_cmd,
                           display_src_t src,
                           uint8_t next,
                           display_wait_t *p_wait)
{
    uint16_t x_byte = 0U;
    uint16_t w_bytes = 0U;
    panel_window_bytes(p_rect, &x_byte, &w_bytes);

    uint16_t x_start = (uint16_t)(x_byte * 8U);
    uint16_t y_start = PANEL_RAM_Y(p_rect->y);
    const uint8_t ram[] = {PANEL_CMD_RAM_X_COUNTER,
                           2U,
                           (uint8_t)(x_start % 256U),
                           (uint8_t)(x_start / 256U),
                           PANEL_CMD_RAM_Y_COUNTER,
                           2U,
                           (uint8_t)(y_start % 256U),
                           (uint8_t)(y_start / 256U),
                           ram_cmd,
                           0U};
    esp_err_t ret = panel_run(ram, sizeof(ram));
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_stream(p_wait, next, src, x_byte, p_rect->y, p_rect->h, w_bytes);
}

/**
 * @brief Byte-align a region to the framebuffer's row layout.
 *
 * @param[in] p_rect requested region.
 * @param[out] p_x_byte byte column of the aligned left edge.
 * @param[out] p_w_bytes aligned width, in bytes.
 */
static void panel_window_bytes(const epd_rect_t *p_rect, uint16_t *p_x_byte, uint16_t *p_w_bytes)
{
    uint16_t x_aligned = (uint16_t)((p_rect->x / 8U) * 8U);
    uint16_t w_aligned = (uint16_t)((((p_rect->x + p_rect->w + 7U) / 8U) * 8U) - x_aligned);

    *p_x_byte = (uint16_t)(x_aligned / 8U);
    *p_w_bytes = (uint16_t)(w_aligned / 8U);
}
