/**
 * @file main.c
 * @brief Paper Ink Boy -- a Game Boy emulator on the Atlas board (ESP32-C5 + 7.5" e-paper + GT911
 * touch).
 *
 * Three tasks:
 *  - gb_emu (gb_emu.c): runs the game at full speed and publishes frames. Never waits on the panel.
 *  - display: owns the panel. It streams: a quick refresh runs scan after scan, and each new frame
 *    is drawn where it changed and joins the run, its changed pixels getting a drive budget of a
 *    few scans while pixels from earlier frames finish theirs; pixels that do not change are never
 *    driven. (Without quick mode it falls back to one stock partial refresh per frame.) Every
 *    WHOLE_REFRESH_EVERY frames, and when the side button is pressed, it lets the run end and
 *    runs a whole-screen fast refresh to clear the ghosting.
 *  - touch: reads the GT911 and turns fingers on the on-screen controls into buttons; also reads
 *    the side button. A separate task because the touch ISR and the display both signal through
 *    their owner task's notification value.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "board_config.h"
#include "display.h"
#include "gb_emu.h"
#include "hal_touch.h"
#include "renderer.h"
#include "touch_pad.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <inttypes.h>
#include <stdbool.h>

/* Private definitions ---------------------------------------------------------------------------*/

/* Display task notification bits */
#define NTF_DISPLAY (1UL << 0) /* display_service() has a step to run */
#define NTF_FRAME   (1UL << 1) /* the emulator published a frame */
#define NTF_CLEAN   (1UL << 2) /* whole-screen refresh asked for (side button) */

/* Frames shown between two whole-screen fast refreshes, which clear the ghosting. */
#define WHOLE_REFRESH_EVERY   (150U)
/* Quick-mode drive budgets: how many scans a pixel that changes is driven for, 1 to 15 (0 = the
 * panel driver's own, 1). Several short scans per change, with new frames joining the run in
 * between, is what lets a pixel finish moving without holding up the next frame. */
#define QUICK_BUDGET_BLACK    (3U)
#define QUICK_BUDGET_WHITE    (3U)

#define DISPLAY_TASK_STACK    (6144U)
#define DISPLAY_TASK_PRIORITY (6U)
#define TOUCH_TASK_STACK      (4096U)
#define TOUCH_TASK_PRIORITY   (7U)

/* Touch: how long to wait for INT before polling anyway, and how long with no fresh report before
 * held buttons are let go (the GT911 reports continuously while a finger is down). */
#define TOUCH_POLL_MS         (20U)
#define TOUCH_RELEASE_US      (300 * 1000)
#define BUTTON_DEBOUNCE_US    (50 * 1000)

#define STATS_PERIOD_US       (5 * 1000 * 1000)

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "main";

static TaskHandle_t s_display_task = NULL;
static esp_err_t s_rom_status = ESP_FAIL;

/* Private function declarations -----------------------------------------------------------------*/

static void board_gpio_init(void);
static void display_task(void *p_arg);
static void touch_task(void *p_arg);
static void draw_rom_error(esp_err_t err);
static epd_rect_t rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b);

/* Public function definitions -------------------------------------------------------------------*/

void app_main(void)
{
    ESP_LOGI(TAG, "Paper Ink Boy starting");
    board_gpio_init();

    s_rom_status = gb_emu_init();

    if (xTaskCreate(display_task, "display", DISPLAY_TASK_STACK, NULL, DISPLAY_TASK_PRIORITY,
                    &s_display_task) != pdPASS) {
        ESP_LOGE(TAG, "display task create failed");
        return;
    }
    if (xTaskCreate(touch_task, "touch", TOUCH_TASK_STACK, NULL, TOUCH_TASK_PRIORITY, NULL) !=
        pdPASS) {
        ESP_LOGE(TAG, "touch task create failed");
    }
    if (s_rom_status == ESP_OK) {
        ESP_ERROR_CHECK(gb_emu_start(s_display_task, NTF_FRAME));
    }
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Park the SPI bus's other user and set up the side button.
 *
 * The SD card shares the SPI bus with the panel. Its chip select is held high so the card ignores
 * the panel's traffic, and MISO gets a pull-up, as on the factory firmware.
 */
static void board_gpio_init(void)
{
    const gpio_config_t sd_cs = {
        .pin_bit_mask = 1ULL << SD_PIN_CS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sd_cs));
    ESP_ERROR_CHECK(gpio_set_level(SD_PIN_CS, 1));
    ESP_ERROR_CHECK(gpio_pullup_en(SPI_PIN_MISO));
    ESP_ERROR_CHECK(gpio_pullup_en(EPD_PIN_CS));

    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = BUTTON_INTERNAL_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&button));
}

/**
 * @brief Draw the screen shown instead of the game when the ROM did not load.
 *
 * @param[in] err gb_emu_init()'s result.
 */
static void draw_rom_error(esp_err_t err)
{
    static const char *const no_rom[] = {
        "NO GAME",
        "FLASH A ROM WITH",
        "TOOLS/FLASH_ROM.PS1",
    };
    static const char *const unsupported[] = {
        "UNSUPPORTED",
        "THIS CARTRIDGE TYPE",
        "IS NOT SUPPORTED",
    };
    static const char *const failed[] = {
        "ROM ERROR",
        "SEE THE SERIAL LOG",
    };

    if (err == ESP_ERR_INVALID_STATE) {
        touch_pad_draw_message(no_rom, 3U);
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        touch_pad_draw_message(unsupported, 3U);
    } else {
        touch_pad_draw_message(failed, 2U);
    }
}

/**
 * @brief The smallest rectangle holding two rectangles.
 */
static epd_rect_t rect_union(const epd_rect_t *p_a, const epd_rect_t *p_b)
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
 * @brief Own the panel: show the newest frame whenever the panel is free.
 *
 * @param[in] p_arg Unused.
 */
static void display_task(void *p_arg)
{
    (void)p_arg;

    esp_err_t ret = display_init();
    if (ret == ESP_OK) {
        ret = display_set_notify(xTaskGetCurrentTaskHandle(), NTF_DISPLAY);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "display init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    uint8_t *p_frame = heap_caps_calloc(GB_EMU_WIDTH * GB_EMU_HEIGHT, 1U, MALLOC_CAP_INTERNAL);
    if (p_frame == NULL) {
        ESP_LOGE(TAG, "no memory for the frame");
        vTaskDelete(NULL);
        return;
    }
    renderer_init();

    const bool playing = (s_rom_status == ESP_OK);
    if (playing == true) {
        touch_pad_draw(gb_emu_title());
    } else {
        draw_rom_error(s_rom_status);
    }

    uint32_t frame_seq = 0U;
    bool in_flight = false;
    bool streaming = false; /* the refresh in flight is a quick run new frames can join */
    /* Area drawn into the framebuffer that no refresh has covered yet */
    epd_rect_t carry = {0};
    bool carry_on = false;
    bool need_whole = true;
    bool quick = true;
    uint32_t since_whole = 0U;

    uint32_t stat_refreshes = 0U;
    uint64_t stat_area = 0U;
    int64_t stat_start_us = esp_timer_get_time();

    for (;;) {
        uint32_t bits = 0U;
        (void)xTaskNotifyWait(0U, UINT32_MAX, &bits, pdMS_TO_TICKS(in_flight ? 5000U : 100U));

        if ((bits & NTF_CLEAN) != 0U) {
            need_whole = true;
        }
        if ((bits & NTF_DISPLAY) != 0U) {
            ret = display_service();
            if ((ret != ESP_ERR_NOT_FINISHED) && (in_flight == true)) {
                in_flight = false;
                streaming = false;
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "refresh failed: %s", esp_err_to_name(ret));
                    need_whole = true;
                }
            }
        }

        const int64_t now_us = esp_timer_get_time();
        if ((now_us - stat_start_us) >= STATS_PERIOD_US) {
            const uint32_t secs_x10 = (uint32_t)((now_us - stat_start_us) / 100000);
            ESP_LOGI(TAG,
                     "display %" PRIu32 ".%" PRIu32 " fps (%s), avg area %" PRIu32 " px",
                     (stat_refreshes * 10U) / secs_x10,
                     ((stat_refreshes * 100U) / secs_x10) % 10U,
                     (stat_refreshes == 0U) ? "idle" : (quick ? "quick" : "partial"),
                     (stat_refreshes > 0U) ? (uint32_t)(stat_area / stat_refreshes) : 0U);
            stat_start_us = now_us;
            stat_refreshes = 0U;
            stat_area = 0U;
        }

        const bool want_whole = (need_whole == true) || (since_whole >= WHOLE_REFRESH_EVERY);

        /* Streaming: while a quick run is in flight, each new frame joins it. Its changed pixels
         * get their drive budget on the run's next scan, while pixels from earlier frames finish
         * theirs. When a whole-screen refresh is due, frames stop joining so the run can end. */
        if ((streaming == true) && (want_whole == false)) {
            epd_rect_t dirty;
            if ((gb_emu_take_frame(p_frame, &frame_seq) == true) &&
                (renderer_draw(p_frame, false, &dirty) == true)) {
                ret = display_quick_extend(&dirty);
                if (ret == ESP_OK) {
                    since_whole++;
                    stat_refreshes++;
                    stat_area += (uint64_t)dirty.w * dirty.h;
                } else {
                    /* The run ended just now: the frame is in the framebuffer already, so its
                     * area goes out with the next refresh. */
                    carry = carry_on ? rect_union(&carry, &dirty) : dirty;
                    carry_on = true;
                }
            }
            continue;
        }

        /* Busy with a refresh, or with the panel's idle sleep */
        if ((in_flight == true) || (display_busy() == true)) {
            continue;
        }

        if (playing == false) {
            if (need_whole == true) {
                need_whole = false;
                in_flight = (display_begin_fast() == ESP_OK);
            }
            continue;
        }

        const bool new_frame = gb_emu_take_frame(p_frame, &frame_seq);

        if (want_whole == true) {
            epd_rect_t unused;
            (void)renderer_draw(p_frame, true, &unused);
            ret = display_begin_fast();
            if (ret == ESP_OK) {
                in_flight = true;
                need_whole = false;
                since_whole = 0U;
                carry_on = false;
            } else {
                ESP_LOGE(TAG, "fast refresh: %s", esp_err_to_name(ret));
            }
            continue;
        }

        epd_rect_t dirty;
        const bool drawn = (new_frame == true) && (renderer_draw(p_frame, false, &dirty) == true);
        if (drawn == false) {
            if (carry_on == false) {
                continue;
            }
            dirty = carry;
        } else if (carry_on == true) {
            dirty = rect_union(&dirty, &carry);
        }
        carry_on = false;

        ret = ESP_ERR_NOT_SUPPORTED;
        if (quick == true) {
            ret = display_begin_quick_budget(&dirty, QUICK_BUDGET_BLACK, QUICK_BUDGET_WHITE);
            if (ret == ESP_ERR_NOT_SUPPORTED) {
                ESP_LOGW(TAG, "panel built without quick mode -- using the stock partial refresh");
                quick = false;
            }
        }
        if (quick == false) {
            ret = display_begin_partial(&dirty);
        }

        if (ret == ESP_OK) {
            in_flight = true;
            streaming = quick;
            since_whole++;
            stat_refreshes++;
            stat_area += (uint64_t)dirty.w * dirty.h;
        } else {
            ESP_LOGE(TAG, "refresh begin: %s", esp_err_to_name(ret));
            need_whole = true;
        }
    }
}

/**
 * @brief Turn fingers on the controls into buttons, and the side button into a clean refresh.
 *
 * @param[in] p_arg Unused.
 */
static void touch_task(void *p_arg)
{
    (void)p_arg;

    const bool touch_ok = (hal_touch_init() == ESP_OK);
    if (touch_ok == false) {
        ESP_LOGE(TAG, "touch init failed -- controls disabled, side button still works");
    }

    uint8_t buttons = 0U;
    int64_t last_report_us = 0;
    bool side_down = false;
    int64_t side_change_us = 0;

    for (;;) {
        int64_t now_us = esp_timer_get_time();

        if (touch_ok == true) {
            hal_touch_point_t points[HAL_TOUCH_MAX_POINTS];
            const int count =
                hal_touch_read_points(points, HAL_TOUCH_MAX_POINTS, pdMS_TO_TICKS(TOUCH_POLL_MS));
            now_us = esp_timer_get_time();
            uint8_t next = buttons;
            if (count >= 0) {
                next = touch_pad_buttons(points, count);
                last_report_us = now_us;
            } else if ((now_us - last_report_us) > TOUCH_RELEASE_US) {
                next = 0U;
            }
            if (next != buttons) {
                buttons = next;
                gb_emu_set_buttons(buttons);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
            now_us = esp_timer_get_time();
        }

        /* Side button: press = clear the ghosting and save the game now */
        const bool down = (gpio_get_level(BUTTON_GPIO) == (int)BUTTON_ACTIVE_LEVEL);
        if ((down != side_down) && ((now_us - side_change_us) >= BUTTON_DEBOUNCE_US)) {
            side_down = down;
            side_change_us = now_us;
            if (down == true) {
                ESP_LOGI(TAG, "side button: clean refresh + save");
                (void)xTaskNotify(s_display_task, NTF_CLEAN, eSetBits);
                gb_emu_request_save();
            }
        }
    }
}
