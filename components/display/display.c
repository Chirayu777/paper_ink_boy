/**
 * @file display.c
 * @brief display implementation -- the refresh engine: waits, image streams, idle sleep and the
 * emergency refresh, for whichever panel driver the application hands it.
 *
 * Refreshes are non-blocking (ADR-17, C8): display_begin_*() does the first step's work and
 * returns; every wait in between -- reset pulse, settle, POWER_ON BUSY, refresh BUSY -- is an
 * esp_timer one-shot whose callback only samples BUSY and sets the owner's notification bit. The
 * owner calls display_service() on that bit to run the next step. All bus traffic and framebuffer
 * reads happen in the owner's task, never in the timer callback.
 *
 * What a step does is the panel driver's: it sends the commands and says what to wait for next.
 * This file does the waiting and the image streaming, the same for every panel.
 *
 * Image data is streamed by DMA (ADR-18). The rows of the window being sent are copied, by the
 * owner, into two internal-RAM tiles; one tile is on the wire while the owner fills the other. The
 * port wakes the owner, on the same bit, each time a tile has left, so no step of a refresh takes
 * longer than a tile copy. The framebuffers could be read by the DMA directly (this SOC can DMA
 * from PSRAM), but a window's rows are not contiguous in them, so they are gathered into a tile
 * either way.
 *
 * BUSY is noisy, so it is sampled, never edge-triggered (ACR-001): the panel says how long to leave
 * it unsampled, then a sample is taken every busy_debounce_us, with two consecutive ready readings
 * required.
 *
 * The panel sleeps when it has been idle for sleep_idle_ms after a refresh: a timer tells the
 * owner, which runs the panel's sleep steps and reports them through display_service() like a
 * refresh. A refresh that comes first cancels the sleep. A sleeping panel wakes only on a reset,
 * which the panel driver does as the first step of the refresh that follows.
 *
 * @version 0.4
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "display.h"

#include "display_fb.h"
#include "display_panel.h"
#include "display_port.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

#if DISPLAY_TIMING_MONITOR_ENABLED
#include <inttypes.h>
#endif

/* Private definitions ---------------------------------------------------------------------------*/

/* One image is far shorter than this at 10 MHz (a whole frame takes ~40 ms), so the guard fires
 * only if the bus stalls. */
#define DISPLAY_STREAM_TIMEOUT_MS (1000U)
/* Longest wait for one tile to finish when a stream is aborted (a tile takes ~7 ms). */
#define DISPLAY_STREAM_DRAIN_MS   (50U)
/* How often the emergency refresh looks at the panel while it polls. */
#define DISPLAY_EMERGENCY_POLL_MS (10U)
/* The step every operation starts at. */
#define DISPLAY_FIRST_STEP        (0U)

/* Private types ---------------------------------------------------------------------------------*/

/**
 * @brief What the in-flight operation is waiting on.
 */
typedef enum {
    DISPLAY_STATE_IDLE,   /**< Nothing in flight */
    DISPLAY_STATE_TIMER,  /**< A timer wait: a delay or BUSY */
    DISPLAY_STATE_STREAM, /**< An image going out */
} display_state_t;

/**
 * @brief What the timer callback does when it fires.
 */
typedef enum {
    DISPLAY_WATCH_DELAY, /**< End a fixed delay */
    DISPLAY_WATCH_BUSY,  /**< Sample BUSY */
    DISPLAY_WATCH_GUARD, /**< Report a stalled stream */
} display_watch_t;

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "display";

static const display_panel_t *s_p_panel = NULL;
static uint8_t *s_stage[DISPLAY_PORT_TILE_COUNT];
static uint32_t s_tile_bytes = 0U;

/* Owner-task state -- only ever touched from the task that calls begin/service. */
static TaskHandle_t s_notify_task = NULL;
static uint32_t s_notify_bit = 0U;
static display_op_t s_op = DISPLAY_OP_NONE;
static display_state_t s_state = DISPLAY_STATE_IDLE;
static uint8_t s_panel_step = DISPLAY_FIRST_STEP;
/* Last completed refresh's mode; NONE = glass unknown. */
static display_op_t s_mode = DISPLAY_OP_NONE;
static bool s_asleep = false; /* the panel is in deep sleep and wakes only on a reset */
static display_request_t s_request;
/* Area added to the quick refresh in flight by display_quick_extend(), for its next scan. */
static epd_rect_t s_quick_extend;
static bool s_quick_extend_on = false;
/* Quick scans run since display_init(), for display_quick_scan_count(). */
static uint32_t s_quick_scans = 0U;
#if DISPLAY_TIMING_MONITOR_ENABLED
static int64_t s_op_start_us = 0;
static int64_t s_step_start_us = 0;
static int64_t s_stream_start_us = 0;
#endif

/* The stream in flight. Owner-task state. */
static bool s_stream_active = false;
static display_src_t s_stream_src = DISPLAY_SRC_ZERO;
static uint16_t s_stream_x_byte = 0U;
static uint16_t s_stream_y = 0U;
static uint16_t s_stream_rows = 0U;
static uint16_t s_stream_w_bytes = 0U;
static uint16_t s_stream_rows_per_tile = 0U;
static uint16_t s_stream_rows_queued = 0U;
static uint8_t s_stream_inflight = 0U;
static uint8_t s_stream_tile_next = 0U;

/* Shared with the timer callback. Written there before it notifies the owner; the owner reads
 * them only after that notification, so the kernel call orders the accesses. */
static esp_timer_handle_t s_watch_timer = NULL;
static esp_timer_handle_t s_sleep_timer = NULL;
static volatile bool s_sleep_due = false;
static volatile display_watch_t s_watch_kind = DISPLAY_WATCH_DELAY;
static volatile bool s_timer_fired = false;
static volatile bool s_watch_confirming = false;
static volatile int64_t s_watch_deadline_us = 0;
static volatile esp_err_t s_watch_result = ESP_OK;
static volatile uint32_t s_watch_glitches = 0U;

/* Private function declarations -----------------------------------------------------------------*/

static esp_err_t display_alloc_stage(void);
static esp_err_t display_create_timers(void);
static esp_err_t display_begin_check(void);
static esp_err_t display_region_check(const epd_rect_t *p_rect);
static esp_err_t display_quick_next(void);
static epd_rect_t display_rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b);
static esp_err_t display_grey_pass(void);
static esp_err_t display_start(display_op_t op);
static esp_err_t display_run_panel(uint8_t step);
static esp_err_t display_start_wait(const display_wait_t *p_wait);
static esp_err_t display_stream_begin(const display_wait_t *p_wait);
static esp_err_t display_stream_pump(void);
static esp_err_t display_stream_service(void);
static void display_stream_end(void);
static void display_stream_abort(void);
static esp_err_t emergency_wait(void);
static void display_watch_cb(void *arg);
static esp_err_t display_watch_delay(uint32_t delay_ms);
static esp_err_t display_watch_busy(uint32_t timeout_ms, uint32_t pre_delay_ms);
static void display_sleep_cb(void *p_arg);
static esp_err_t display_sleep_begin(void);
static esp_err_t display_settle_after(esp_err_t result);
static esp_err_t display_finish(esp_err_t result);
static void display_clip_region(const epd_rect_t *p_rect);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Bring up the hardware, allocate the framebuffers and the staging tiles, create the timers.
 *
 * @return esp_err_t
 */
esp_err_t display_init(void)
{
    s_p_panel = display_select_panel();
    if ((s_p_panel == NULL) || (s_p_panel->init == NULL) || (s_p_panel->advance == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = display_fb_init(s_p_panel->size);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = display_alloc_stage();
    if (ret != ESP_OK) {
        return ret;
    }
    if (s_p_panel->quick_budget_black != 0U) {
        display_fb_grey_enable();
    }

    ret = s_p_panel->init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = display_create_timers();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "display_init: %s, bring-up done, first refresh must be full", s_p_panel->name);
    return ESP_OK;
}

/**
 * @brief The size of the panel, in landscape pixels.
 *
 * @return epd_size_t
 */
epd_size_t display_get_size(void)
{
    epd_size_t none = {0U, 0U};
    return (s_p_panel == NULL) ? none : s_p_panel->size;
}

/**
 * @brief Set the task notified on every wait-step completion.
 *
 * @return esp_err_t
 */
esp_err_t display_set_notify(TaskHandle_t task, uint32_t bit)
{
    if ((task == NULL) || (bit == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_notify_task = task;
    s_notify_bit = bit;
    display_port_set_notify(task, bit);
    return ESP_OK;
}

/**
 * @brief Whether a refresh is in flight.
 *
 * @return bool
 */
bool display_busy(void)
{
    return s_state != DISPLAY_STATE_IDLE;
}

/**
 * @brief Begin a full-quality refresh of the whole framebuffer.
 *
 * @return esp_err_t
 */
esp_err_t display_begin_full(void)
{
    esp_err_t ret = display_begin_check();
    if (ret != ESP_OK) {
        return ret;
    }
    display_fb_quick_reset();
    return display_start(DISPLAY_OP_FULL);
}

/**
 * @brief Begin a fast refresh of the whole framebuffer.
 *
 * @return esp_err_t
 */
esp_err_t display_begin_fast(void)
{
    esp_err_t ret = display_begin_check();
    if (ret != ESP_OK) {
        return ret;
    }
    display_fb_quick_reset();
    return display_start(DISPLAY_OP_FAST);
}

/**
 * @brief Begin a windowed partial refresh.
 *
 * @return esp_err_t
 */
esp_err_t display_begin_partial(const epd_rect_t *p_rect)
{
    esp_err_t ret = display_region_check(p_rect);
    if (ret != ESP_OK) {
        return ret;
    }

    display_clip_region(p_rect);
    return display_start(DISPLAY_OP_PARTIAL);
}

/**
 * @brief Begin a quick refresh of a region: short scans until no pixel has drive budget left.
 *
 * @return esp_err_t
 */
esp_err_t display_begin_quick(const epd_rect_t *p_rect)
{
    return display_begin_quick_budget(p_rect, 0U, 0U);
}

/**
 * @brief Begin a quick refresh with the caller's drive budgets; 0 takes the panel's own.
 *
 * @return esp_err_t
 */
esp_err_t display_begin_quick_budget(const epd_rect_t *p_rect,
                                     uint8_t budget_black,
                                     uint8_t budget_white)
{
    if ((s_p_panel != NULL) && (s_p_panel->quick_budget_black == 0U)) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t ret = display_region_check(p_rect);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = display_fb_quick_init(
        (budget_black == 0U) ? s_p_panel->quick_budget_black : budget_black,
        (budget_white == 0U) ? s_p_panel->quick_budget_white : budget_white);
    if (ret != ESP_OK) {
        return ret;
    }

    display_clip_region(p_rect);
    display_fb_quick_scan_start();
    display_fb_quick_touch(&s_request.region);
    s_quick_extend_on = false;
    return display_start(DISPLAY_OP_QUICK);
}

/**
 * @brief How many quick scans have run.
 *
 * @return uint32_t
 */
uint32_t display_quick_scan_count(void)
{
    return s_quick_scans;
}

/**
 * @brief Add an area to the quick refresh in flight: its next scan covers it, so pixels drawn there
 * since the last scan get their budget and start moving while the earlier ones finish.
 *
 * @return esp_err_t
 */
esp_err_t display_quick_extend(const epd_rect_t *p_rect)
{
    if (s_op != DISPLAY_OP_QUICK) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Not display_region_check(): that one also wants the display idle, and a run in flight is
     * the whole point here. */
    if ((p_rect->x >= s_p_panel->size.w) ||
        (p_rect->y >= s_p_panel->size.h) ||
        (p_rect->w == 0U) ||
        (p_rect->h == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    epd_rect_t add = *p_rect;
    if (((uint32_t)add.x + add.w) > s_p_panel->size.w) {
        add.w = (uint16_t)(s_p_panel->size.w - add.x);
    }
    if (((uint32_t)add.y + add.h) > s_p_panel->size.h) {
        add.h = (uint16_t)(s_p_panel->size.h - add.y);
    }
    if (s_quick_extend_on == false) {
        s_quick_extend = add;
        s_quick_extend_on = true;
    } else {
        s_quick_extend = display_rect_union(&s_quick_extend, &add);
    }
    return ESP_OK;
}

/**
 * @brief Run the in-flight refresh's next step. Call on every notification bit.
 *
 * @return esp_err_t
 */
esp_err_t display_service(void)
{
    if (s_state == DISPLAY_STATE_IDLE) {
        if (s_sleep_due == true) {
            s_sleep_due = false;
            return display_sleep_begin();
        }
        return ESP_OK;
    }
    if (s_watch_result != ESP_OK) {
        return display_finish(s_watch_result);
    }

    esp_err_t ret = ESP_OK;
    if (s_state == DISPLAY_STATE_STREAM) {
        ret = display_stream_service();
    } else if (s_timer_fired) {
        s_timer_fired = false;
        ret = display_run_panel(s_panel_step);
    } else {
        /* A late bus wake-up while a timer step is pending: nothing to advance. */
        return ESP_ERR_NOT_FINISHED;
    }

    if (ret != ESP_OK) {
        return display_finish(ret);
    }
    return (s_state == DISPLAY_STATE_IDLE) ? ESP_OK : ESP_ERR_NOT_FINISHED;
}

/**
 * @brief Refresh the panel from the framebuffer at once, blocking, when the owner cannot.
 *
 * @return esp_err_t
 */
esp_err_t display_emergency_refresh(void)
{
    if ((display_fb_is_ready() == false) || (s_watch_timer == NULL) || (s_p_panel == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_notify_task == NULL) {
        s_notify_task = xTaskGetCurrentTaskHandle();
        s_notify_bit = 1U;
        display_port_set_notify(s_notify_task, s_notify_bit);
    }

    /* Ignoring: the timer may not be running. */
    (void)esp_timer_stop(s_watch_timer);
    (void)esp_timer_stop(s_sleep_timer);
    s_sleep_due = false;
    display_stream_abort();
    s_state = DISPLAY_STATE_IDLE;
    s_op = DISPLAY_OP_NONE;
    s_mode = DISPLAY_OP_NONE;
    s_timer_fired = false;
    display_fb_quick_reset();

    esp_err_t ret = display_start(DISPLAY_OP_FAST);
    if (ret != ESP_OK) {
        return ret;
    }
    return emergency_wait();
}

/**
 * @brief Switch on the panel's power rails, for a panel that has them.
 *
 * @return esp_err_t
 */
esp_err_t display_power_on(void)
{
    if ((s_p_panel == NULL) || (s_state != DISPLAY_STATE_IDLE)) {
        return ESP_ERR_INVALID_STATE;
    }
    return (s_p_panel->power_on == NULL) ? ESP_OK : s_p_panel->power_on();
}

/**
 * @brief Switch off the panel's power rails, for a panel that has them.
 *
 * @return esp_err_t
 */
esp_err_t display_power_off(void)
{
    if ((s_p_panel == NULL) || (s_state != DISPLAY_STATE_IDLE)) {
        return ESP_ERR_INVALID_STATE;
    }
    return (s_p_panel->power_off == NULL) ? ESP_OK : s_p_panel->power_off();
}

/**
 * @brief Shut the panel down in the fewest steps that leave it safe, for a brownout.
 *
 * @return esp_err_t
 */
esp_err_t IRAM_ATTR display_safe_shutdown(void)
{
    if ((s_p_panel == NULL) || (s_p_panel->safe_shutdown == NULL)) {
        return ESP_OK;
    }
    return s_p_panel->safe_shutdown();
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Drive the refresh begun by display_emergency_refresh() to its end by polling.
 *
 * @return ESP_OK when the refresh finished, otherwise the error that ended it.
 */
static esp_err_t emergency_wait(void)
{
    int64_t deadline_us = esp_timer_get_time() +
                          ((int64_t)s_p_panel->emergency_limit_ms * 1000);
    esp_err_t ret = ESP_ERR_NOT_FINISHED;

    while (ret == ESP_ERR_NOT_FINISHED) {
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_EMERGENCY_POLL_MS));
        ret = display_service();
        if ((ret == ESP_ERR_NOT_FINISHED) && (esp_timer_get_time() > deadline_us)) {
            ret = display_finish(ESP_ERR_TIMEOUT);
        }
    }
    return ret;
}

/**
 * @brief Allocate the DMA staging tiles in memory the port says the DMA can read.
 *
 * @return esp_err_t
 */
static esp_err_t display_alloc_stage(void)
{
    s_tile_bytes = display_port_tile_bytes();
    for (uint8_t i = 0U; i < DISPLAY_PORT_TILE_COUNT; i++) {
        s_stage[i] = display_port_alloc(DISPLAY_ALLOC_DMA, s_tile_bytes);
        if (s_stage[i] == NULL) {
            ESP_LOGE(TAG, "Staging tile %u allocation failed", (unsigned)i);
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

/**
 * @brief Create the wait timer and the idle-sleep timer.
 *
 * @return esp_err_t
 */
static esp_err_t display_create_timers(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = display_watch_cb,
        .name = "epd_watch",
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_watch_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const esp_timer_create_args_t sleep_args = {
        .callback = display_sleep_cb,
        .name = "epd_sleep",
    };
    ret = esp_timer_create(&sleep_args, &s_sleep_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Preconditions shared by every begin: initialized, owner registered, nothing in flight.
 *
 * @return esp_err_t
 */
static esp_err_t display_begin_check(void)
{
    if ((display_fb_is_ready() == false) || (s_watch_timer == NULL) || (s_notify_task == NULL)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state != DISPLAY_STATE_IDLE) {
        return ESP_ERR_INVALID_STATE;
    }

    (void)esp_timer_stop(s_sleep_timer);
    s_sleep_due = false;
    return ESP_OK;
}

/**
 * @brief Preconditions of a refresh of a region: those of every begin, a glass whose state is
 * known, and a region that is not empty and starts on the panel.
 *
 * @param[in] p_rect the requested region.
 * @return esp_err_t
 */
static esp_err_t display_region_check(const epd_rect_t *p_rect)
{
    esp_err_t ret = display_begin_check();
    if (ret != ESP_OK) {
        return ret;
    }
    if (s_mode == DISPLAY_OP_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((p_rect->x >= s_p_panel->size.w) ||
        (p_rect->y >= s_p_panel->size.h) ||
        (p_rect->w == 0U) ||
        (p_rect->h == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

/**
 * @brief The smallest rectangle holding two rectangles.
 *
 * @param[in] p_a one rectangle.
 * @param[in] p_b the other.
 * @return epd_rect_t
 */
static epd_rect_t display_rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b)
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
 * @brief A quick scan has ended: run the next one over the pixels still with budget, or end the
 * quick refresh when none has any. The next scan finds the panel powered and its LUT loaded.
 *
 * @return esp_err_t
 */
static esp_err_t display_quick_next(void)
{
#if DISPLAY_TIMING_MONITOR_ENABLED
    ESP_LOGI(TAG,
             "quick scan %ux%u px: refresh %" PRId64 "ms",
             (unsigned)s_request.region.w,
             (unsigned)s_request.region.h,
             (esp_timer_get_time() - s_step_start_us) / 1000);
#endif
    s_mode = DISPLAY_OP_QUICK;
    s_quick_scans++;
    epd_rect_t pending;
    const bool any_pending = display_fb_quick_pending(&pending);
    if ((any_pending == false) && (s_quick_extend_on == false)) {
        return display_finish(ESP_OK);
    }
    if (any_pending == false) {
        s_request.region = s_quick_extend;
    } else if (s_quick_extend_on == true) {
        s_request.region = display_rect_union(&pending, &s_quick_extend);
    } else {
        s_request.region = pending;
    }
    s_quick_extend_on = false;

    display_fb_quick_scan_start();
    s_request.last_mode = DISPLAY_OP_QUICK;
    s_request.asleep = false;
    s_timer_fired = false;
    return display_run_panel(DISPLAY_FIRST_STEP);
}

/**
 * @brief A whole-screen refresh has ended, which sends grey pixels white: when there are any,
 * bring them to grey with a quick refresh over them before the refresh is reported done.
 *
 * @return esp_err_t
 */
static esp_err_t display_grey_pass(void)
{
    s_mode = s_op;
    if (display_fb_grey_pending(&s_request.region) == false) {
        return display_finish(ESP_OK);
    }
    esp_err_t ret = display_fb_quick_init(s_p_panel->quick_budget_black,
                                          s_p_panel->quick_budget_white);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "grey pass skipped: %s", esp_err_to_name(ret));
        return display_finish(ESP_OK);
    }

    display_fb_quick_scan_start();
    s_request.last_mode = s_op;
    s_request.asleep = false;
    s_op = DISPLAY_OP_QUICK;
    s_timer_fired = false;
    return display_run_panel(DISPLAY_FIRST_STEP);
}

/**
 * @brief Keep the part of a requested region that is on the panel, as the region to refresh.
 *
 * @param[in] p_rect the requested region, which starts on the panel.
 */
static void display_clip_region(const epd_rect_t *p_rect)
{
    s_request.region = *p_rect;
    if (((uint32_t)p_rect->x + p_rect->w) > s_p_panel->size.w) {
        s_request.region.w = (uint16_t)(s_p_panel->size.w - p_rect->x);
    }
    if (((uint32_t)p_rect->y + p_rect->h) > s_p_panel->size.h) {
        s_request.region.h = (uint16_t)(s_p_panel->size.h - p_rect->y);
    }
}

/**
 * @brief Start an operation: tell the panel how it was left, and run its first step.
 *
 * @param[in] op FULL, FAST or PARTIAL.
 * @return esp_err_t ESP_OK if started; the error that aborted it, after the operation was ended.
 */
static esp_err_t display_start(display_op_t op)
{
    s_op = op;
    s_request.last_mode = s_mode;
    s_request.asleep = s_asleep;
    s_asleep = false;
    s_timer_fired = false;
#if DISPLAY_TIMING_MONITOR_ENABLED
    s_op_start_us = esp_timer_get_time();
    s_step_start_us = s_op_start_us;
#endif

    esp_err_t ret = display_run_panel(DISPLAY_FIRST_STEP);
    if (ret != ESP_OK) {
        return display_finish(ret);
    }
    return ESP_OK;
}

/**
 * @brief Run one step of the operation in flight in the panel driver, then start the wait it asks
 * for.
 *
 * @param[in] step the step to run.
 * @return esp_err_t
 */
static esp_err_t display_run_panel(uint8_t step)
{
    display_wait_t wait;
    memset(&wait, 0, sizeof(wait));

    esp_err_t ret = s_p_panel->advance(s_op, step, &s_request, &wait);
    if (ret != ESP_OK) {
        return ret;
    }
    return display_start_wait(&wait);
}

/**
 * @brief Start the wait a panel step asked for: arm a timer, start an image, or end the operation.
 *
 * @param[in] p_wait what to wait for, and the step to run when it is over.
 * @return esp_err_t
 */
static esp_err_t display_start_wait(const display_wait_t *p_wait)
{
    s_panel_step = p_wait->next;

    switch (p_wait->kind) {
    case DISPLAY_WAIT_DELAY:
        s_state = DISPLAY_STATE_TIMER;
        return display_watch_delay(p_wait->delay_ms);

    case DISPLAY_WAIT_BUSY:
        s_state = DISPLAY_STATE_TIMER;
#if DISPLAY_TIMING_MONITOR_ENABLED
        s_step_start_us = esp_timer_get_time();
#endif
        return display_watch_busy(p_wait->timeout_ms, p_wait->delay_ms);

    case DISPLAY_WAIT_STREAM:
        return display_stream_begin(p_wait);

    case DISPLAY_WAIT_DONE:
        if (s_op == DISPLAY_OP_QUICK) {
            return display_quick_next();
        }
        if ((s_op == DISPLAY_OP_FULL) || (s_op == DISPLAY_OP_FAST)) {
            return display_grey_pass();
        }
        if (s_op != DISPLAY_OP_SLEEP) {
            s_mode = s_op;
        }
        return display_finish(ESP_OK);

    default:
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief Start streaming one image to the panel by DMA. The panel has already sent the command
 * that precedes it. Chip select stays low until the last tile has left (display_stream_end()).
 *
 * The image is a window of the 1bpp framebuffer layout: rows rows of w_bytes bytes, starting at
 * byte column x_byte and row y.
 *
 * @param[in] p_wait the window and where its bytes come from. With DISPLAY_SRC_FRAME the bytes are
 * also copied into the glass mirror as they are taken, so the mirror matches what the panel was
 * given even if the owner draws during the stream.
 * @return esp_err_t
 */
static esp_err_t display_stream_begin(const display_wait_t *p_wait)
{
    if ((p_wait->w_bytes == 0U) || (p_wait->w_bytes > s_tile_bytes)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_stream_src = p_wait->src;
    s_stream_x_byte = p_wait->x_byte;
    s_stream_y = p_wait->y;
    s_stream_rows = p_wait->rows;
    s_stream_w_bytes = p_wait->w_bytes;
    s_stream_rows_per_tile = (uint16_t)(s_tile_bytes / p_wait->w_bytes);
    s_stream_rows_queued = 0U;
    s_stream_inflight = 0U;
    s_stream_tile_next = 0U;
#if DISPLAY_TIMING_MONITOR_ENABLED
    s_stream_start_us = esp_timer_get_time();
#endif

    esp_err_t ret = display_port_stream_open();
    if (ret != ESP_OK) {
        return ret;
    }

    s_stream_active = true;
    s_state = DISPLAY_STATE_STREAM;
    s_watch_kind = DISPLAY_WATCH_GUARD;
    s_watch_result = ESP_OK;
    ret = esp_timer_start_once(s_watch_timer, (uint64_t)DISPLAY_STREAM_TIMEOUT_MS * 1000U);
    if (ret != ESP_OK) {
        return ret;
    }
    return display_stream_pump();
}

/**
 * @brief Collect finished tiles, then fill and queue tiles while there is room and data left.
 *
 * @return esp_err_t
 */
static esp_err_t display_stream_pump(void)
{
    while (s_stream_inflight > 0U) {
        esp_err_t ret = display_port_stream_reap(0U);
        if (ret == ESP_ERR_TIMEOUT) {
            break;
        }
        if (ret != ESP_OK) {
            return ret;
        }
        s_stream_inflight--;
    }

    while ((s_stream_inflight < DISPLAY_PORT_TILE_COUNT) &&
           (s_stream_rows_queued < s_stream_rows)) {
        uint16_t left = (uint16_t)(s_stream_rows - s_stream_rows_queued);
        uint16_t rows = (left < s_stream_rows_per_tile) ? left : s_stream_rows_per_tile;
        uint8_t slot = s_stream_tile_next;

        display_fb_stage_rows(s_stream_src,
                              s_stage[slot],
                              s_stream_x_byte,
                              (uint16_t)(s_stream_y + s_stream_rows_queued),
                              rows,
                              s_stream_w_bytes);
        esp_err_t ret = display_port_stream_queue(s_stage[slot],
                                                  (size_t)rows * s_stream_w_bytes,
                                                  slot);
        if (ret != ESP_OK) {
            return ret;
        }

        s_stream_inflight++;
        s_stream_tile_next = (uint8_t)((slot + 1U) % DISPLAY_PORT_TILE_COUNT);
        s_stream_rows_queued = (uint16_t)(s_stream_rows_queued + rows);
    }
    return ESP_OK;
}

/**
 * @brief Advance the stream on a wake-up; when its last tile has left, go on to the panel's next
 * step.
 *
 * @return esp_err_t
 */
static esp_err_t display_stream_service(void)
{
    esp_err_t ret = display_stream_pump();
    if (ret != ESP_OK) {
        return ret;
    }
    if ((s_stream_rows_queued < s_stream_rows) || (s_stream_inflight > 0U)) {
        return ESP_OK;
    }

    display_stream_end();
    if (s_op == DISPLAY_OP_QUICK) {
        s_request.quick_grey = display_fb_quick_scan_has_grey();
    }
    return display_run_panel(s_panel_step);
}

/**
 * @brief Close a finished stream: release chip select and stop its guard.
 */
static void display_stream_end(void)
{
    display_port_stream_close();
    (void)esp_timer_stop(s_watch_timer);
    s_stream_active = false;
#if DISPLAY_TIMING_MONITOR_ENABLED
    ESP_LOGI(TAG,
             "stream: %u B in %" PRId64 "ms",
             (unsigned)((uint32_t)s_stream_rows * s_stream_w_bytes),
             (esp_timer_get_time() - s_stream_start_us) / 1000);
#endif
}

/**
 * @brief Abandon a stream in flight: wait for the tiles already queued to leave, release chip
 * select.
 */
static void display_stream_abort(void)
{
    if (s_stream_active == false) {
        return;
    }

    while (s_stream_inflight > 0U) {
        if (display_port_stream_reap(DISPLAY_STREAM_DRAIN_MS) != ESP_OK) {
            ESP_LOGE(TAG, "stream abort: a queued tile did not finish");
            break;
        }
        s_stream_inflight--;
    }
    display_port_stream_close();
    s_stream_active = false;
}

/**
 * @brief Timer callback: end a fixed delay, sample BUSY once, or report a stalled stream (the
 * guard). Runs in the esp_timer task.
 *
 * Touches only BUSY (read), its own watch state, and the owner's notification -- never the bus or
 * a framebuffer. A BUSY-watch re-arms itself every busy_debounce_us until two consecutive ready
 * samples (ready) or the deadline (timeout). A real ready stays ready, so the second sample never
 * delays or misses one; a glitch reads busy on it and restarts confirmation.
 *
 * @param[in] arg unused.
 */
static void display_watch_cb(void *arg)
{
    (void)arg;

    if (s_watch_kind == DISPLAY_WATCH_BUSY) {
        if (display_port_busy_ready() == true) {
            if (s_watch_confirming == false) {
                s_watch_confirming = true;
                (void)esp_timer_start_once(s_watch_timer, display_port_busy_debounce_us());
                return;
            }
            s_watch_result = ESP_OK;
        } else {
            if (s_watch_confirming) {
                s_watch_glitches++;
            }
            s_watch_confirming = false;
            if (esp_timer_get_time() < s_watch_deadline_us) {
                (void)esp_timer_start_once(s_watch_timer, display_port_busy_debounce_us());
                return;
            }
            s_watch_result = ESP_ERR_TIMEOUT;
        }
    } else if (s_watch_kind == DISPLAY_WATCH_GUARD) {
        s_watch_result = ESP_ERR_TIMEOUT;
    } else {
        s_watch_result = ESP_OK;
    }

    s_timer_fired = true;
    (void)xTaskNotify(s_notify_task, s_notify_bit, eSetBits);
}

/**
 * @brief Arm a plain delay -- the owner is notified after delay_ms.
 *
 * @param[in] delay_ms delay.
 * @return esp_err_t
 */
static esp_err_t display_watch_delay(uint32_t delay_ms)
{
    s_watch_kind = DISPLAY_WATCH_DELAY;
    s_watch_result = ESP_OK;
    return esp_timer_start_once(s_watch_timer, (uint64_t)delay_ms * 1000U);
}

/**
 * @brief Arm a BUSY watch: no sampling for pre_delay_ms, then debounced sampling until ready or
 * timeout_ms after now.
 *
 * @param[in] timeout_ms overall budget from now.
 * @param[in] pre_delay_ms unsampled lead-in; 0 starts sampling after one debounce interval.
 * @return esp_err_t
 */
static esp_err_t display_watch_busy(uint32_t timeout_ms, uint32_t pre_delay_ms)
{
    s_watch_kind = DISPLAY_WATCH_BUSY;
    s_watch_confirming = false;
    s_watch_glitches = 0U;
    s_watch_result = ESP_OK;
    s_watch_deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    uint64_t first_us = (pre_delay_ms == 0U) ? display_port_busy_debounce_us() :
                                               ((uint64_t)pre_delay_ms * 1000U);
    return esp_timer_start_once(s_watch_timer, first_us);
}

/**
 * @brief Timer callback: the panel has been idle long enough to sleep. Runs in the esp_timer task
 * and only tells the owner, which runs the sleep steps.
 *
 * @param[in] p_arg unused.
 */
static void display_sleep_cb(void *p_arg)
{
    (void)p_arg;

    s_sleep_due = true;
    (void)xTaskNotify(s_notify_task, s_notify_bit, eSetBits);
}

/**
 * @brief Start the panel's sleep: its first step, then a BUSY watch.
 *
 * @return ESP_ERR_NOT_FINISHED once started, ESP_OK if the panel is asleep already, or the error
 * that stopped it starting.
 */
static esp_err_t display_sleep_begin(void)
{
    if (s_asleep == true) {
        return ESP_OK;
    }

    s_op = DISPLAY_OP_SLEEP;
    s_timer_fired = false;
    esp_err_t ret = display_run_panel(DISPLAY_FIRST_STEP);
    if (ret != ESP_OK) {
        return display_finish(ret);
    }
    return ESP_ERR_NOT_FINISHED;
}

/**
 * @brief What ending an operation leaves behind. A refresh that failed leaves the glass unknown. A
 * refresh that ended starts the idle wait before the panel sleeps. A sleep that did not finish is
 * only logged: the next refresh resets the controller anyway, so it is not a fault.
 *
 * @param[in] result How the operation ended.
 * @return result, or ESP_OK for a sleep.
 */
static esp_err_t display_settle_after(esp_err_t result)
{
    if (s_op == DISPLAY_OP_SLEEP) {
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "panel sleep incomplete: %s", esp_err_to_name(result));
        }
        s_asleep = true;
        return ESP_OK;
    }

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "refresh op %d failed: %s", (int)s_op, esp_err_to_name(result));
        s_mode = DISPLAY_OP_NONE;
    }
    (void)esp_timer_start_once(s_sleep_timer, (uint64_t)s_p_panel->sleep_idle_ms * 1000U);
    return result;
}

/**
 * @brief End the in-flight refresh. A failure leaves the glass state unknown, so the next refresh
 * must be full (display_begin_partial() refuses until then).
 *
 * @param[in] result ESP_OK, or the error that aborted the refresh.
 * @return result, except that a panel sleep that did not finish is reported as ESP_OK.
 */
static esp_err_t display_finish(esp_err_t result)
{
    (void)esp_timer_stop(s_watch_timer);
    display_stream_abort();
    s_timer_fired = false;

#if DISPLAY_TIMING_MONITOR_ENABLED
    int64_t now_us = esp_timer_get_time();
    ESP_LOGI(TAG,
             "op %d %s: total %" PRId64 "ms, last wait %" PRId64 "ms, %u glitches filtered",
             (int)s_op,
             (result == ESP_OK) ? "done" : "failed",
             (now_us - s_op_start_us) / 1000,
             (now_us - s_step_start_us) / 1000,
             (unsigned)s_watch_glitches);
#endif

    result = display_settle_after(result);
    s_op = DISPLAY_OP_NONE;
    s_state = DISPLAY_STATE_IDLE;
    return result;
}
