/**
 * @file panel_uc8179c.c
 * @brief panel_uc8179c implementation -- the UC8179C's refresh sequences as steps for the display
 * core.
 *
 * Ported from the hardware-proven legacy PoC driver (epd_7in5_driver.c; fw/legacy/pre_freeze in
 * git history up to 6142a8e).
 *
 * Full mode pulses RST, which restores the panel's factory OTP LUT after fast/partial overwrote the
 * LUT-select register. Fast mode also resets. Partial mode does not reset, and skips its
 * POWER_ON/LUT entry entirely when the previous refresh was already partial -- the same-mode
 * repeat the legacy driver's move updates relied on.
 *
 * BUSY_N is electrically noisy (~218 spurious rising edges per full refresh, measured -- no
 * external pull-up/filter cap, schematic: EPD_7.5_Touch_C5), so nothing samples it for the first
 * 90% of a refresh's measured budget (ACR-001). POWER_ON waits PANEL_POWER_ON_SETTLE_MS first,
 * because BUSY only goes low after the command.
 *
 * A panel in deep sleep wakes only on a reset, so every refresh after one starts with RST, partial
 * ones included.
 *
 * Quick mode (PANEL_QUICK_ENABLED) runs the controller from waveform registers instead of its OTP:
 * it uploads a one-group LUT of PANEL_QUICK_FRAMES frames into the volatile LUT registers 0x20-0x24,
 * in which a pixel whose old and new data match is held (WW/KK) and the others are driven toward
 * their new shade (KW/WK). The display core then sends each scan's old data with only the pixels
 * that still have drive budget flipped. The waveform's values are Good Display's partial-mode
 * settings, which are under NDA: they live in panel_uc8179c_lut_nda.h, which is not in git (see
 * panel_uc8179c_lut_nda.h.example). Only volatile LUT registers are written -- never the commands
 * that program the controller's OTP. A partial refresh after quick mode starts with RST, which puts
 * the OTP LUT and every register back.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "panel_uc8179c.h"

#include "board_config.h"
#include "display_port.h"

#if PANEL_QUICK_ENABLED
#if !__has_include("panel_uc8179c_lut_nda.h")
#error "PANEL_QUICK_ENABLED needs components/panel_uc8179c/nda/panel_uc8179c_lut_nda.h: copy panel_uc8179c_lut_nda.h.example there and fill it in from Good Display's partial-mode LUT data"
#endif
#include "panel_uc8179c_lut_nda.h"
#endif

/* Private definitions ---------------------------------------------------------------------------*/

#define PANEL_WIDTH         (800U)
#define PANEL_HEIGHT        (480U)
#define PANEL_WIDTH_BYTES   (PANEL_WIDTH / 8U)
#define PANEL_FIRST_STEP    (0U)

#define PANEL_BUSY_READY_TIMEOUT_MS    (3800U) /* panel spec: 3.5s full refresh */
/* Fast timing carried over from measurement on the same panel/board in a prior project --
 * reconfirm against this build with DISPLAY_TIMING_MONITOR_ENABLED, don't re-derive from
 * scratch. */
#define PANEL_BUSY_FAST_TIMEOUT_MS     (1800U) /* prior-project measured: <1.7s */
/* Partial BUSY, measured on this board (2026-09-28, DISPLAY_PARTIAL_BENCH_ENABLED, ~250 refreshes,
 * windows 540 B - 41 KB): busy ~= 368ms + 1.25us per window byte, so a full-panel window
 * (48,000 B) is the physical maximum, ~428ms. EXPECTED sizes the unsampled pre-delay (the
 * smallest window's duration, so no window is sampled late). TIMEOUT is the fault threshold: the
 * physical maximum + 40%, so it fires only for a panel that is actually stuck. */
#define PANEL_BUSY_PARTIAL_EXPECTED_MS (368U)
#define PANEL_BUSY_PARTIAL_TIMEOUT_MS  (600U)
/* POWER_ON's busy duration is a different physical event from a panel update's -- entering
 * fast/partial mode never pulses RST, so it's a resume, not a cold boot, and its duration has no
 * relationship to how long a waveform update takes. This is a safety net against a stuck or
 * disconnected panel, not a measured operation time. */
#define PANEL_BUSY_POWER_ON_TIMEOUT_MS (4000U)
/* Power-off and deep sleep follow the vendor demo (GDEY075T7_Arduino, EPD_DeepSleep()): the VCOM
 * and data interval is set to 0xF7, POWER_OFF is sent and BUSY waited for, then DEEP_SLEEP with the
 * check code 0xA5. The power-off wait is a safety net, not a measured time; BUSY goes low a moment
 * after the command, so it is not sampled for the first PANEL_POWER_OFF_SETTLE_MS. */
#define PANEL_BUSY_POWER_OFF_TIMEOUT_MS (1000U)
#define PANEL_POWER_OFF_SETTLE_MS       (20U)
/* Nothing samples BUSY before this -- 90% of a refresh's measured duration (ACR-001). */
#define PANEL_BUSY_PRE_DELAY_MS(measured_ms) (((measured_ms) * 9U) / 10U)
/* BUSY goes low a moment after POWER_ON is written, so an immediate sample can read "ready" before
 * the controller has started powering up. The vendor reference waits this long before it first
 * looks at BUSY. */
#define PANEL_POWER_ON_SETTLE_MS (100U)
#define PANEL_RST_PULSE_MS       (10U)
#define PANEL_RST_SETTLE_FULL_MS (110U) /* RST release settle + panel internal boot */
#define PANEL_RST_SETTLE_FAST_MS (10U)
/* The emergency refresh is a fast refresh (~1.9 s); this is its overall limit. */
#define PANEL_EMERGENCY_LIMIT_MS (4000U)
/* How long the panel stays powered after a refresh with nothing following it. A refresh that comes
 * sooner finds it still awake, so play does not pay a power-up for every move. */
#define PANEL_SLEEP_IDLE_MS      (3000U)

/* The controller's minimum write clock cycle is 100 ns (UC8179C 4-wire write, GDEY075T7-T01
 * datasheet section 6.4): 10 MHz is its limit, so this must not be raised. */
#define PANEL_SPI_CLOCK_HZ       (10U * 1000U * 1000U)
/* BUSY_N is low while the panel is busy and high when it is ready. */
#define PANEL_BUSY_READY_LEVEL   (1U)
/* BUSY_N has no external pull-up/filter cap and sits next to the SPI lines on the FPC connector
 * (schematic: EPD_7.5_Touch_C5): the interval between samples while BUSY still reads busy, and the
 * settle time between the two ready readings a ready needs. Microseconds, on an esp_timer: a
 * FreeRTOS tick at CONFIG_FREERTOS_HZ=100 would round 5ms down to 0. */
#define PANEL_BUSY_DEBOUNCE_US   (5000U)

/* Partial refresh's BUSY watch. DISPLAY_PARTIAL_BENCH_ENABLED samples from the trigger with a
 * wide limit, to measure the real completion time per window size. */
#if DISPLAY_PARTIAL_BENCH_ENABLED
#if !DISPLAY_TIMING_MONITOR_ENABLED
#error "DISPLAY_PARTIAL_BENCH_ENABLED needs DISPLAY_TIMING_MONITOR_ENABLED to log its results"
#endif
#define PANEL_PARTIAL_WATCH_TIMEOUT_MS (2000U)
#define PANEL_PARTIAL_WATCH_PRE_DELAY_MS (0U)
#else
#define PANEL_PARTIAL_WATCH_TIMEOUT_MS (PANEL_BUSY_PARTIAL_TIMEOUT_MS)
#define PANEL_PARTIAL_WATCH_PRE_DELAY_MS (PANEL_BUSY_PRE_DELAY_MS(PANEL_BUSY_PARTIAL_EXPECTED_MS))
#endif

/* Quick mode. A pixel's old and new bits pick its LUT row: held, driven white, driven black, or
 * driven grey. A scan runs PANEL_QUICK_FRAMES frames, in which the grey row drives white; a scan
 * that drives a pixel grey then rests for PANEL_QUICK_REST_FRAMES, undriven, so a pixel that was
 * black settles at white, and drives the grey row PANEL_QUICK_GREY_FRAMES toward black, which sets
 * the shade of grey. A change to grey so lands in the same refresh as the rest. A changed pixel is
 * driven for its budget of scans. The defaults give every change all its drive in one scan of 16 frames,
 * about the vendor partial waveform's 15: a piece appears in one step, and fewer frames leave the
 * old image showing.
 * PANEL_QUICK_STRONG_DRIVE takes the vendor's full-refresh drive voltages in place of its
 * partial-mode ones, which moves a pixel further per frame. All are bench-tuned and can be set from
 * bringup.cmake. A scan's BUSY is sampled from the trigger, as its duration is not yet measured;
 * the timeout is only a stuck-panel guard. */
#if PANEL_QUICK_ENABLED
#ifndef PANEL_QUICK_FRAMES
#define PANEL_QUICK_FRAMES (16U)
#endif
#ifndef PANEL_QUICK_GREY_FRAMES
#define PANEL_QUICK_GREY_FRAMES (2U)
#endif
#ifndef PANEL_QUICK_REST_FRAMES
#define PANEL_QUICK_REST_FRAMES (4U)
#endif
#if PANEL_QUICK_STRONG_DRIVE
#define PANEL_QUICK_POWER_SETTING PANEL_NDA_POWER_SETTING_FULL
#else
#define PANEL_QUICK_POWER_SETTING PANEL_NDA_POWER_SETTING
#endif
#ifndef PANEL_QUICK_BUDGET_BLACK
#define PANEL_QUICK_BUDGET_BLACK (1U)
#endif
#ifndef PANEL_QUICK_BUDGET_WHITE
#define PANEL_QUICK_BUDGET_WHITE (1U)
#endif
#else
#define PANEL_QUICK_BUDGET_BLACK (0U)
#define PANEL_QUICK_BUDGET_WHITE (0U)
#endif
#define PANEL_BUSY_QUICK_TIMEOUT_MS (1500U)
#define PANEL_QUICK_BUSY            0U, PANEL_BUSY_QUICK_TIMEOUT_MS
/* One LUT register: 7 groups of 6 bytes -- a level byte (2 bits per phase), the frame counts of its
 * 4 phases, and how many times the group repeats. Quick mode uses group 0: phase A drives, B rests
 * and C drives the grey frames. */
#define PANEL_LUT_BYTES      (42U)
#define PANEL_LUT_GROUP_REPS (5U)
/* A level byte with phase A from one vendor row's first phase, phase B undriven, and phase C from
 * another row's first phase. */
#define PANEL_LEVEL(phase_a, phase_c) \
    ((uint8_t)(((phase_a) & 0xC0U) | (((phase_c) & 0xC0U) >> 4)))
/* VCOM keeps one level through all three phases. */
#define PANEL_LEVEL_ALL(level) \
    ((uint8_t)(((level) & 0xC0U) | (((level) & 0xC0U) >> 2) | (((level) & 0xC0U) >> 4)))
/* The quick LUT the registers hold: with the grey phases or not, or not known yet. */
#define PANEL_QUICK_LUT_PLAIN   (0U)
#define PANEL_QUICK_LUT_GREY    (1U)
#define PANEL_QUICK_LUT_UNKNOWN (0xFFU)

/* UC8179C command bytes. */
#define PANEL_CMD_PANEL_SETTING             (0x00U)
#define PANEL_CMD_POWER_SETTING             (0x01U)
#define PANEL_CMD_POWER_OFF                 (0x02U)
#define PANEL_CMD_POWER_ON                  (0x04U)
#define PANEL_CMD_BOOSTER_SOFT_START        (0x06U)
#define PANEL_CMD_DEEP_SLEEP                (0x07U)
#define PANEL_CMD_DATA_START_TRANSMISSION_1 (0x10U)
#define PANEL_CMD_DISPLAY_REFRESH           (0x12U)
#define PANEL_CMD_DATA_START_TRANSMISSION_2 (0x13U)
#define PANEL_CMD_LUT_VCOM                  (0x20U)
#define PANEL_CMD_LUT_WW                    (0x21U)
#define PANEL_CMD_LUT_KW                    (0x22U)
#define PANEL_CMD_LUT_WK                    (0x23U)
#define PANEL_CMD_LUT_KK                    (0x24U)
#define PANEL_CMD_PLL                       (0x30U)
#define PANEL_CMD_VCOM_DATA_INTERVAL        (0x50U)
#define PANEL_CMD_LUT_OPTION                (0x52U)
#define PANEL_CMD_VCOM_DC                   (0x82U)
#define PANEL_CMD_PARTIAL_WINDOW            (0x90U)
#define PANEL_CMD_PARTIAL_IN                (0x91U)
#define PANEL_CMD_FULL_UPDATE               (0x92U)
#define PANEL_CMD_CASCADE_SETTING           (0xE0U)
#define PANEL_CMD_FORCE_TEMPERATURE         (0xE5U)

/* How long a wait for BUSY leaves it unsampled, then waits for it, as the two arguments the helpers
 * take. */
#define PANEL_POWER_ON_BUSY PANEL_POWER_ON_SETTLE_MS, PANEL_BUSY_POWER_ON_TIMEOUT_MS
#define PANEL_FULL_BUSY \
    PANEL_BUSY_PRE_DELAY_MS(PANEL_BUSY_READY_TIMEOUT_MS), PANEL_BUSY_READY_TIMEOUT_MS
#define PANEL_FAST_BUSY \
    PANEL_BUSY_PRE_DELAY_MS(PANEL_BUSY_FAST_TIMEOUT_MS), PANEL_BUSY_FAST_TIMEOUT_MS
#define PANEL_PARTIAL_BUSY PANEL_PARTIAL_WATCH_PRE_DELAY_MS, PANEL_PARTIAL_WATCH_TIMEOUT_MS
#define PANEL_POWER_OFF_BUSY PANEL_POWER_OFF_SETTLE_MS, PANEL_BUSY_POWER_OFF_TIMEOUT_MS

/* The partial window command carries 9 parameter bytes. */
#define PANEL_WINDOW_PARAMS (9U)
/* VCOM and data interval in a window: border floating, data polarity as the framebuffer's. Partial
 * mode also has the controller copy new data to old after the refresh; quick mode sends both
 * itself, so it leaves that off. */
#define PANEL_CDI_PARTIAL (0xA9U)
#define PANEL_CDI_QUICK   (0xA1U)
/* What each mode takes off the window's end coordinates; see panel_window(). */
#define PANEL_END_TRIM_PARTIAL (1U)
#define PANEL_END_TRIM_QUICK   (0U)

/* Command scripts, in the form panel_run() takes: command, number of data bytes, the data bytes. */
#define PANEL_SCRIPT_FULL_POWER_ON PANEL_CMD_PANEL_SETTING, 1U, 0x1FU, PANEL_CMD_POWER_ON, 0U
#define PANEL_SCRIPT_FULL_DATA \
    PANEL_CMD_VCOM_DATA_INTERVAL, 2U, 0x21U, 0x07U, PANEL_CMD_BOOSTER_SOFT_START, 0U, \
        PANEL_CMD_DATA_START_TRANSMISSION_2, 0U
#define PANEL_SCRIPT_FULL_REFRESH PANEL_CMD_FULL_UPDATE, 0U, PANEL_CMD_DISPLAY_REFRESH, 0U
/* VCOM: fast mode setting (image data is pre-inverted at source). */
#define PANEL_SCRIPT_FAST_POWER_ON \
    PANEL_CMD_PANEL_SETTING, 1U, 0x1FU, PANEL_CMD_VCOM_DATA_INTERVAL, 2U, 0x11U, 0x07U, \
        PANEL_CMD_POWER_ON, 0U
/* Enhanced booster -- 4 bytes, fast only; 0x28 is the manufacturer's suggestion for better power
 * delivery. 0x5A selects the fast LUT -- partial mode uses 0x6E instead. */
#define PANEL_SCRIPT_FAST_DATA_OLD \
    PANEL_CMD_BOOSTER_SOFT_START, 4U, 0x27U, 0x27U, 0x28U, 0x17U, PANEL_CMD_CASCADE_SETTING, 1U, \
        0x02U, PANEL_CMD_FORCE_TEMPERATURE, 1U, 0x5AU, PANEL_CMD_DATA_START_TRANSMISSION_1, 0U
#define PANEL_SCRIPT_DATA_NEW PANEL_CMD_DATA_START_TRANSMISSION_2, 0U
#define PANEL_SCRIPT_REFRESH  PANEL_CMD_DISPLAY_REFRESH, 0U
#define PANEL_SCRIPT_PART_POWER_ON \
    PANEL_CMD_PANEL_SETTING, 1U, 0x1FU, PANEL_CMD_VCOM_DATA_INTERVAL, 2U, 0x21U, 0x07U, \
        PANEL_CMD_POWER_ON, 0U
/* 0x6E selects the partial LUT -- fast mode uses 0x5A instead. */
#define PANEL_SCRIPT_PART_LUT \
    PANEL_CMD_CASCADE_SETTING, 1U, 0x02U, PANEL_CMD_FORCE_TEMPERATURE, 1U, 0x6EU
#define PANEL_SCRIPT_POWER_OFF PANEL_CMD_VCOM_DATA_INTERVAL, 1U, 0xF7U, PANEL_CMD_POWER_OFF, 0U
#define PANEL_SCRIPT_DEEP_SLEEP PANEL_CMD_DEEP_SLEEP, 1U, 0xA5U

/* Private types ---------------------------------------------------------------------------------*/

/**
 * @brief The steps of a full refresh; each is the one advance() is called with.
 */
typedef enum {
    FULL_BEGIN = PANEL_FIRST_STEP,
    FULL_RELEASE,
    FULL_POWER_ON,
    FULL_DATA,
    FULL_REFRESH,
    FULL_DONE,
} full_step_t;

/**
 * @brief The steps of a fast refresh.
 */
typedef enum {
    FAST_BEGIN = PANEL_FIRST_STEP,
    FAST_RELEASE,
    FAST_POWER_ON,
    FAST_DATA_OLD,
    FAST_DATA_NEW,
    FAST_REFRESH,
    FAST_DONE,
} fast_step_t;

/**
 * @brief The steps of a partial refresh.
 */
typedef enum {
    PART_BEGIN = PANEL_FIRST_STEP,
    PART_RELEASE,
    PART_POWER_ON,
    PART_WINDOW,
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
    QUICK_POWER_ON,
    QUICK_LUT,
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
static esp_err_t panel_run_frame(const uint8_t *p_script,
                                 size_t len,
                                 display_wait_t *p_wait,
                                 uint8_t next,
                                 display_src_t src);
static esp_err_t panel_full(uint8_t step, display_wait_t *p_wait);
static esp_err_t panel_fast(uint8_t step, display_wait_t *p_wait);
static esp_err_t panel_partial(uint8_t step,
                               const display_request_t *p_request,
                               display_wait_t *p_wait);
static esp_err_t panel_sleep(uint8_t step, display_wait_t *p_wait);
static esp_err_t panel_partial_power_on(display_wait_t *p_wait);
static esp_err_t panel_window(const epd_rect_t *p_rect,
                              uint8_t cdi,
                              uint8_t end_trim,
                              display_src_t src,
                              uint8_t next,
                              display_wait_t *p_wait);
static esp_err_t panel_window_new(const epd_rect_t *p_rect,
                                  display_src_t src,
                                  uint8_t next,
                                  display_wait_t *p_wait);
#if PANEL_QUICK_ENABLED
static esp_err_t panel_quick(uint8_t step,
                             const display_request_t *p_request,
                             display_wait_t *p_wait);
static esp_err_t panel_quick_power_on(display_wait_t *p_wait);
static esp_err_t panel_quick_settings(void);
static esp_err_t panel_quick_lut(bool grey);
static esp_err_t panel_lut(uint8_t cmd, uint8_t level, bool grey);
#endif
static void panel_window_bytes(const epd_rect_t *p_rect, uint16_t *p_x_byte, uint16_t *p_w_bytes);

/* Private variables -----------------------------------------------------------------------------*/

#if PANEL_QUICK_ENABLED
/* Which quick LUT the registers hold, so it is uploaded only when a scan needs the other one. */
static uint8_t s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
#endif

/* Public variables ------------------------------------------------------------------------------*/

const display_panel_t g_panel_uc8179c_gdey075t7 = {
    .name = "UC8179C GDEY075T7-T01",
    .size = {PANEL_WIDTH, PANEL_HEIGHT},
    .emergency_limit_ms = PANEL_EMERGENCY_LIMIT_MS,
    .sleep_idle_ms = PANEL_SLEEP_IDLE_MS,
    .quick_budget_black = PANEL_QUICK_BUDGET_BLACK,
    .quick_budget_white = PANEL_QUICK_BUDGET_WHITE,
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
    return &g_panel_uc8179c_gdey075t7;
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

#if PANEL_QUICK_ENABLED
    case DISPLAY_OP_QUICK:
        return panel_quick(step, p_request, p_wait);
#endif

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
 * @brief Send a script, then send the whole screen.
 *
 * @param[in] p_script the script, ending with the command that starts the image.
 * @param[in] len length of the script in bytes.
 * @param[out] p_wait what to wait for next.
 * @param[in] next the step to come back with once the image has gone out.
 * @param[in] src where the image comes from.
 * @return esp_err_t
 */
static esp_err_t panel_run_frame(const uint8_t *p_script,
                                 size_t len,
                                 display_wait_t *p_wait,
                                 uint8_t next,
                                 display_src_t src)
{
    esp_err_t ret = panel_run(p_script, len);
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_stream(p_wait, next, src, 0U, 0U, PANEL_HEIGHT, PANEL_WIDTH_BYTES);
}

/**
 * @brief A full refresh. RST restores the factory OTP LUT, the only LUT full mode uses.
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_full(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t power_on[] = {PANEL_SCRIPT_FULL_POWER_ON};
    const uint8_t data[] = {PANEL_SCRIPT_FULL_DATA};
    const uint8_t refresh[] = {PANEL_SCRIPT_FULL_REFRESH};

    switch (step) {
    case FULL_BEGIN:
        display_port_rst(false);
        return display_wait_delay(p_wait, FULL_RELEASE, PANEL_RST_PULSE_MS);

    case FULL_RELEASE:
        display_port_rst(true);
        return display_wait_delay(p_wait, FULL_POWER_ON, PANEL_RST_SETTLE_FULL_MS);

    case FULL_POWER_ON:
        return panel_run_busy(power_on, sizeof(power_on), p_wait, FULL_DATA, PANEL_POWER_ON_BUSY);

    case FULL_DATA:
        return panel_run_frame(data, sizeof(data), p_wait, FULL_REFRESH, DISPLAY_SRC_FRAME);

    case FULL_REFRESH:
        return panel_run_busy(refresh, sizeof(refresh), p_wait, FULL_DONE, PANEL_FULL_BUSY);

    case FULL_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief A fast refresh. Follows the legacy firmware's page commit, EPD_Init_Fast() then
 * EPD_Display_Fast() (epd_7in5_driver.c on the dev branch).
 *
 * The old data (DTM1) is all zeros: under the fast LUT's convention that means "all white".
 * Writing 0xFF there instead ghosts the page -- the legacy firmware's basemap did, and it was
 * dropped for this.
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_fast(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t power_on[] = {PANEL_SCRIPT_FAST_POWER_ON};
    const uint8_t data_old[] = {PANEL_SCRIPT_FAST_DATA_OLD};
    const uint8_t data_new[] = {PANEL_SCRIPT_DATA_NEW};
    const uint8_t refresh[] = {PANEL_SCRIPT_REFRESH};

    switch (step) {
    case FAST_BEGIN:
        display_port_rst(false);
        return display_wait_delay(p_wait, FAST_RELEASE, PANEL_RST_PULSE_MS);

    case FAST_RELEASE:
        display_port_rst(true);
        return display_wait_delay(p_wait, FAST_POWER_ON, PANEL_RST_SETTLE_FAST_MS);

    case FAST_POWER_ON:
        return panel_run_busy(power_on,
                              sizeof(power_on),
                              p_wait,
                              FAST_DATA_OLD,
                              PANEL_POWER_ON_BUSY);

    case FAST_DATA_OLD:
        return panel_run_frame(data_old, sizeof(data_old), p_wait, FAST_DATA_NEW, DISPLAY_SRC_ZERO);

    case FAST_DATA_NEW:
        return panel_run_frame(data_new, sizeof(data_new), p_wait, FAST_REFRESH, DISPLAY_SRC_FRAME);

    case FAST_REFRESH:
        return panel_run_busy(refresh, sizeof(refresh), p_wait, FAST_DONE, PANEL_FAST_BUSY);

    case FAST_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief A partial refresh. Ported from the legacy PoC's Partial_Update_init() -- no RST unless the
 * panel was asleep: a reset here visibly dimmed the panel and this mode doesn't need one, but a
 * panel in deep sleep wakes only on a reset (the vendor demo's EPD_Init_Part() starts with one).
 * After quick mode it resets too, which is what puts the OTP LUT and the registers quick mode
 * changed back.
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
    const uint8_t lut[] = {PANEL_SCRIPT_PART_LUT};
    const uint8_t refresh[] = {PANEL_SCRIPT_REFRESH};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case PART_BEGIN:
        if ((p_request->asleep == true) || (p_request->last_mode == DISPLAY_OP_QUICK)) {
            display_port_rst(false);
            return display_wait_delay(p_wait, PART_RELEASE, PANEL_RST_PULSE_MS);
        }
        if (p_request->last_mode == DISPLAY_OP_PARTIAL) {
            return panel_window(&p_request->region,
                                PANEL_CDI_PARTIAL,
                                PANEL_END_TRIM_PARTIAL,
                                DISPLAY_SRC_PREV,
                                PART_DATA_NEW,
                                p_wait);
        }
        return panel_partial_power_on(p_wait);

    case PART_RELEASE:
        display_port_rst(true);
        return display_wait_delay(p_wait, PART_POWER_ON, PANEL_RST_SETTLE_FAST_MS);

    case PART_POWER_ON:
        return panel_partial_power_on(p_wait);

    case PART_WINDOW:
        ret = panel_run(lut, sizeof(lut));
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_window(&p_request->region,
                            PANEL_CDI_PARTIAL,
                            PANEL_END_TRIM_PARTIAL,
                            DISPLAY_SRC_PREV,
                            PART_DATA_NEW,
                            p_wait);

    case PART_DATA_NEW:
        return panel_window_new(&p_request->region, DISPLAY_SRC_FRAME, PART_REFRESH, p_wait);

    case PART_REFRESH:
        ret = panel_run(refresh, sizeof(refresh));
        if (ret != ESP_OK) {
            return ret;
        }
        return display_wait_busy(p_wait, PART_DONE, PANEL_PARTIAL_BUSY);

    case PART_DONE:
        return display_wait_done(p_wait);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief The sleep: VCOM interval and POWER_OFF, then DEEP_SLEEP with its check code once BUSY
 * says power-off is done.
 *
 * @param[in] step the step to run.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_sleep(uint8_t step, display_wait_t *p_wait)
{
    const uint8_t power_off[] = {PANEL_SCRIPT_POWER_OFF};
    const uint8_t deep_sleep[] = {PANEL_SCRIPT_DEEP_SLEEP};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case SLEEP_POWER_OFF:
        ret = panel_run(power_off, sizeof(power_off));
        if (ret != ESP_OK) {
            return ret;
        }
        return display_wait_busy(p_wait, SLEEP_DEEP, PANEL_POWER_OFF_BUSY);

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
 * @brief Enter partial mode: panel setting, VCOM interval and POWER_ON, then wait for BUSY.
 *
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_partial_power_on(display_wait_t *p_wait)
{
    const uint8_t power_on[] = {PANEL_SCRIPT_PART_POWER_ON};
    esp_err_t ret = panel_run(power_on, sizeof(power_on));
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_busy(p_wait, PART_WINDOW, PANEL_POWER_ON_BUSY);
}

/**
 * @brief Window, then the old data going out; the new data and the trigger follow from the next
 * steps.
 *
 * The window is given to the controller in bytes along X, not pixels, so the region is aligned to
 * the framebuffer's 1bpp row layout (multiples of 8) first.
 *
 * @param[in] p_rect the region to refresh.
 * @param[in] cdi first byte of the VCOM and data interval setting.
 * @param[in] end_trim taken off the low byte of each window end. Partial mode keeps the 1 it was
 * ported with (vendor demo and legacy driver), which leaves the window's last row unrefreshed --
 * unseen, as its window ends in a margin. Quick mode sends the true end (0): its windows end
 * inside the board, where the missed row showed as a thin line.
 * @param[in] src where the old data comes from.
 * @param[in] next the step to come back with once the old data has gone out.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_window(const epd_rect_t *p_rect,
                              uint8_t cdi,
                              uint8_t end_trim,
                              display_src_t src,
                              uint8_t next,
                              display_wait_t *p_wait)
{
    uint16_t x_byte = 0U;
    uint16_t w_bytes = 0U;
    panel_window_bytes(p_rect, &x_byte, &w_bytes);

    uint16_t x_start = (uint16_t)(x_byte * 8U);
    uint16_t x_end = (uint16_t)(x_start + (w_bytes * 8U) - 1U);
    uint16_t y_end = (uint16_t)(p_rect->y + p_rect->h - 1U);
    const uint8_t window[] = {PANEL_CMD_VCOM_DATA_INTERVAL,
                              2U,
                              cdi,
                              0x07U,
                              PANEL_CMD_PARTIAL_IN,
                              0U,
                              PANEL_CMD_PARTIAL_WINDOW,
                              PANEL_WINDOW_PARAMS,
                              (uint8_t)(x_start / 256U),
                              (uint8_t)(x_start % 256U),
                              (uint8_t)(x_end / 256U),
                              (uint8_t)((x_end % 256U) - end_trim),
                              (uint8_t)(p_rect->y / 256U),
                              (uint8_t)(p_rect->y % 256U),
                              (uint8_t)(y_end / 256U),
                              (uint8_t)((y_end % 256U) - end_trim),
                              0x01U,
                              PANEL_CMD_DATA_START_TRANSMISSION_1,
                              0U};
    esp_err_t ret = panel_run(window, sizeof(window));
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_stream(p_wait, next, src, x_byte, p_rect->y, p_rect->h, w_bytes);
}

/**
 * @brief The new data of a windowed refresh.
 *
 * @param[in] p_rect the region to refresh.
 * @param[in] src where the new data comes from.
 * @param[in] next the step to come back with once the new data has gone out.
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_window_new(const epd_rect_t *p_rect,
                                  display_src_t src,
                                  uint8_t next,
                                  display_wait_t *p_wait)
{
    const uint8_t data_new[] = {PANEL_SCRIPT_DATA_NEW};
    uint16_t x_byte = 0U;
    uint16_t w_bytes = 0U;
    panel_window_bytes(p_rect, &x_byte, &w_bytes);

    esp_err_t ret = panel_run(data_new, sizeof(data_new));
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_stream(p_wait, next, src, x_byte, p_rect->y, p_rect->h, w_bytes);
}

#if PANEL_QUICK_ENABLED
/**
 * @brief One quick scan. The first scan after another mode powers the panel up and uploads the
 * LUT, with no reset unless the panel was asleep, as partial mode; the scans after it find both in
 * place and go straight to the window.
 *
 * The old and new data come from the display core's quick sources: they differ only for the
 * pixels that still have budget, and the new bit says which way each of those is driven.
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
    const uint8_t refresh[] = {PANEL_SCRIPT_REFRESH};
    esp_err_t ret = ESP_OK;

    switch (step) {
    case QUICK_BEGIN:
        if (p_request->asleep == true) {
            display_port_rst(false);
            return display_wait_delay(p_wait, QUICK_RELEASE, PANEL_RST_PULSE_MS);
        }
        if (p_request->last_mode == DISPLAY_OP_QUICK) {
            return panel_window(&p_request->region,
                                PANEL_CDI_QUICK,
                                PANEL_END_TRIM_QUICK,
                                DISPLAY_SRC_QUICK_OLD,
                                QUICK_DATA_NEW,
                                p_wait);
        }
        return panel_quick_power_on(p_wait);

    case QUICK_RELEASE:
        display_port_rst(true);
        return display_wait_delay(p_wait, QUICK_POWER_ON, PANEL_RST_SETTLE_FAST_MS);

    case QUICK_POWER_ON:
        return panel_quick_power_on(p_wait);

    case QUICK_LUT:
        ret = panel_quick_settings();
        if (ret != ESP_OK) {
            return ret;
        }
        return panel_window(&p_request->region,
                            PANEL_CDI_QUICK,
                            PANEL_END_TRIM_QUICK,
                            DISPLAY_SRC_QUICK_OLD,
                            QUICK_DATA_NEW,
                            p_wait);

    case QUICK_DATA_NEW:
        return panel_window_new(&p_request->region,
                                DISPLAY_SRC_QUICK_NEW,
                                QUICK_REFRESH,
                                p_wait);

    case QUICK_REFRESH:
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
 * @brief Enter quick mode: booster, drive voltages, and the panel setting that takes the waveform
 * from the LUT registers, then POWER_ON and a wait for BUSY. Follows the vendor's partial-mode
 * register-LUT init.
 *
 * @param[out] p_wait what to wait for next.
 * @return esp_err_t
 */
static esp_err_t panel_quick_power_on(display_wait_t *p_wait)
{
    const uint8_t power_on[] = {PANEL_CMD_BOOSTER_SOFT_START,
                                4U,
                                PANEL_NDA_BOOSTER,
                                PANEL_CMD_POWER_SETTING,
                                4U,
                                PANEL_QUICK_POWER_SETTING,
                                PANEL_CMD_PANEL_SETTING,
                                1U,
                                PANEL_NDA_PANEL_SETTING,
                                PANEL_CMD_POWER_ON,
                                0U};
    esp_err_t ret = panel_run(power_on, sizeof(power_on));
    if (ret != ESP_OK) {
        return ret;
    }
    return display_wait_busy(p_wait, QUICK_LUT, PANEL_POWER_ON_BUSY);
}

/**
 * @brief Set up the quick waveform: VCOM DC, frame rate, and the LUT option that lets register
 * LUTs override the OTP. The LUT registers themselves go in before each refresh, as the scan needs
 * (panel_quick_lut()).
 *
 * @return esp_err_t
 */
static esp_err_t panel_quick_settings(void)
{
    const uint8_t settings[] = {PANEL_CMD_VCOM_DC,
                                1U,
                                PANEL_NDA_VCOM_DC,
                                PANEL_CMD_PLL,
                                1U,
#ifdef PANEL_QUICK_PLL
                                PANEL_QUICK_PLL,
#else
                                PANEL_NDA_PLL,
#endif
                                PANEL_CMD_LUT_OPTION,
                                1U,
                                PANEL_NDA_LUT_OPTION};

    s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
    return panel_run(settings, sizeof(settings));
}

/**
 * @brief Make the LUT registers hold the quick LUT a scan needs, uploading it only when they hold
 * the other one.
 *
 * Each row's phase A comes from a vendor partial-mode row -- held (WW), driven white (KW), driven
 * black (WK), and driven grey (KK, which drives KW's white) -- phase B rests, and phase C drives
 * the grey row WK's black. A scan with no pixel driven grey gets the plain LUT, phase A only, and
 * does not wait out the rest and grey frames.
 *
 * @param[in] grey whether the scan drives any pixel grey.
 * @return esp_err_t
 */
static esp_err_t panel_quick_lut(bool grey)
{
    uint8_t want = (grey == true) ? PANEL_QUICK_LUT_GREY : PANEL_QUICK_LUT_PLAIN;
    if (s_quick_lut == want) {
        return ESP_OK;
    }

    const uint8_t luts[][2] = {
        {PANEL_CMD_LUT_VCOM, PANEL_LEVEL_ALL(PANEL_NDA_LEVEL_VCOM)},
        {PANEL_CMD_LUT_WW, PANEL_LEVEL(PANEL_NDA_LEVEL_WW, PANEL_NDA_LEVEL_WW)},
        {PANEL_CMD_LUT_KW, PANEL_LEVEL(PANEL_NDA_LEVEL_KW, PANEL_NDA_LEVEL_WW)},
        {PANEL_CMD_LUT_WK, PANEL_LEVEL(PANEL_NDA_LEVEL_WK, PANEL_NDA_LEVEL_WW)},
        {PANEL_CMD_LUT_KK, PANEL_LEVEL(PANEL_NDA_LEVEL_KW, PANEL_NDA_LEVEL_WK)},
    };

    s_quick_lut = PANEL_QUICK_LUT_UNKNOWN;
    esp_err_t ret = ESP_OK;
    for (size_t i = 0U; (ret == ESP_OK) && (i < (sizeof(luts) / sizeof(luts[0]))); i++) {
        ret = panel_lut(luts[i][0], luts[i][1], grey);
    }
    if (ret == ESP_OK) {
        s_quick_lut = want;
    }
    return ret;
}

/**
 * @brief Send one LUT register: group 0 runs phase A for PANEL_QUICK_FRAMES frames and, with grey,
 * phase B for PANEL_QUICK_REST_FRAMES and phase C for PANEL_QUICK_GREY_FRAMES, once, at the given
 * levels; every other group is empty.
 *
 * @param[in] cmd the LUT register.
 * @param[in] level the level byte of group 0, phase A in bits 7-6, B in 5-4 and C in 3-2.
 * @param[in] grey whether to run phases B and C.
 * @return esp_err_t
 */
static esp_err_t panel_lut(uint8_t cmd, uint8_t level, bool grey)
{
    uint8_t lut[2U + PANEL_LUT_BYTES] = {cmd, PANEL_LUT_BYTES, level, PANEL_QUICK_FRAMES};
    if (grey == true) {
        lut[4U] = PANEL_QUICK_REST_FRAMES;
        lut[5U] = PANEL_QUICK_GREY_FRAMES;
    }
    lut[2U + PANEL_LUT_GROUP_REPS] = 1U;
    return panel_run(lut, sizeof(lut));
}
#endif

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
