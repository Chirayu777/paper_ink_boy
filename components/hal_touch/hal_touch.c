/**
 * @file hal_touch.c
 * @brief GT911 capacitive touch controller driver -- I2C transport, raw point read.
 *
 * Ported from the hardware-proven legacy PoC driver (gt911_touch.c; fw/legacy/pre_freeze in git
 * history up to 6142a8e), rewritten to the v5.2 coding standard and stripped to the controller
 * driver only -- no queues, no chess mapping, no touch_task. INT-pin handoff is ISR-driven via
 * xTaskNotify, not the legacy xSemaphoreGiveFromISR -- the architecture doc's §iv ISR-handoff rule
 * (PR-3) is task notification, never a semaphore.
 *
 * @version 0.1
 * @date 2026-09-21
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "hal_touch.h"

#include "board_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include <stddef.h>

#if HAL_TOUCH_DEBUG_LOG_ENABLED
#include "esp_timer.h"

#include <inttypes.h>
#endif

/* Private definitions ---------------------------------------------------------------------------*/

/* INT LOW during hardware reset selects this I2C address (see gt911_hardware_reset()). */
#define GT911_I2C_ADDR              (0x5DU)

#define GT911_REG_PRODUCT_ID        (0x8140U)
#define GT911_REG_STATUS            (0x814EU)
#define GT911_REG_POINT1_DATA       (0x814FU)

#define GT911_STATUS_VALID_MASK     (0x80U)
#define GT911_STATUS_POINTS_MASK    (0x0FU)
#define GT911_POINT_DATA_SIZE       (8U)
#define GT911_PRODUCT_ID_LENGTH     (4U) /* "911" + null terminator */

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "hal_touch";

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_gt911_dev_handle = NULL;
static volatile TaskHandle_t s_touch_task_handle = NULL;
static bool s_isr_service_ready = false;
static bool s_isr_handler_added = false;
#if HAL_TOUCH_DEBUG_LOG_ENABLED
/* Incremented in ISR context, read/reset from hal_touch_read() -- shows how many raw INT
 * edges (touch-down/re-report events) landed while nobody was servicing them, since
 * ulTaskNotifyTake(pdTRUE, ...) collapses any number of pending gives into a single wake. */
static volatile uint32_t s_isr_fire_count = 0U;
#endif

/* Private function declarations -----------------------------------------------------------------*/

static void touch_release(void);
static void gt911_hardware_reset(void);
static esp_err_t gt911_i2c_init(void);
static esp_err_t gt911_intr_init(void);
static esp_err_t gt911_read_register(uint16_t reg_addr, uint8_t *data, size_t len);
static esp_err_t gt911_clear_status(void);
static int touch_read_status(void);
#if HAL_TOUCH_DEBUG_LOG_ENABLED
static uint32_t touch_notify_wait_logged(TickType_t wait_ticks);
#endif

/* IRAM_ATTR expands via __COUNTER__, so it goes on the definition only -- on both, GCC assigns two
 * different .iram1.N sections to the same symbol and errors on the conflict. */
static void gt911_isr_handler(void *p_arg);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Hardware reset the GT911, bring up I2C, and arm the INT-pin ISR.
 *
 * @return esp_err_t
 */
esp_err_t hal_touch_init(void)
{
    touch_release();

    /* Must be assigned before gt911_intr_init() arms the INT ISR below -- the controller can
     * assert INT on its own once booted, before this driver ever calls hal_touch_read(),
     * and the ISR needs a valid target the moment it's armed. */
    s_touch_task_handle = xTaskGetCurrentTaskHandle();

    gt911_hardware_reset();

    esp_err_t ret = gt911_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    char product_id[GT911_PRODUCT_ID_LENGTH];
    uint8_t reg_addr[2] = {
        (uint8_t)((GT911_REG_PRODUCT_ID >> 8) & 0xFFU),
        (uint8_t)(GT911_REG_PRODUCT_ID & 0xFFU),
    };
    ret = i2c_master_transmit_receive(s_gt911_dev_handle,
                                      reg_addr,
                                      sizeof(reg_addr),
                                      (uint8_t *)product_id,
                                      sizeof(product_id),
                                      I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Product ID read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "GT911 product ID: %s", product_id);

    ret = gt911_intr_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "hal_touch_init: ready");
    return ESP_OK;
}

/**
 * @brief Block for one touch report, up to wait_ticks, and classify it.
 *
 * @return hal_touch_result_t
 */
hal_touch_result_t hal_touch_read(uint16_t *x, uint16_t *y, TickType_t wait_ticks)
{
#if HAL_TOUCH_DEBUG_LOG_ENABLED
    uint32_t got = touch_notify_wait_logged(wait_ticks);
#else
    uint32_t got = ulTaskNotifyTake(pdTRUE, wait_ticks);
#endif
    if (got == 0U) {
        return HAL_TOUCH_TIMEOUT;
    }

    if (touch_read_status() <= 0) {
        return HAL_TOUCH_NO_POINT;
    }

    uint8_t point_data[GT911_POINT_DATA_SIZE];
    esp_err_t ret = gt911_read_register(GT911_REG_POINT1_DATA, point_data, sizeof(point_data));
    (void)gt911_clear_status();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Point data read failed: %s", esp_err_to_name(ret));
        return HAL_TOUCH_NO_POINT;
    }

    *x = (uint16_t)(point_data[1] | (point_data[2] << 8));
    *y = (uint16_t)(point_data[3] | (point_data[4] << 8));
#if HAL_TOUCH_DEBUG_LOG_ENABLED
    ESP_LOGI(TAG, "point x=%u y=%u", *x, *y);
#endif
    return HAL_TOUCH_POINT;
}

/**
 * @brief Wait up to wait_ticks for the INT ISR, then read every point of the latest report.
 *
 * @return int
 */
int hal_touch_read_points(hal_touch_point_t *p_points, uint8_t max_points, TickType_t wait_ticks)
{
    (void)ulTaskNotifyTake(pdTRUE, wait_ticks);

    uint8_t status = 0U;
    if (gt911_read_register(GT911_REG_STATUS, &status, 1U) != ESP_OK) {
        return -1;
    }
    if ((status & GT911_STATUS_VALID_MASK) == 0U) {
        return -1;
    }

    uint8_t count = status & GT911_STATUS_POINTS_MASK;
    if (count > HAL_TOUCH_MAX_POINTS) {
        count = HAL_TOUCH_MAX_POINTS;
    }
    if (count > max_points) {
        count = max_points;
    }

    /* The points follow each other at GT911_POINT_DATA_SIZE bytes apart from point 1. */
    uint8_t point_data[HAL_TOUCH_MAX_POINTS * GT911_POINT_DATA_SIZE];
    esp_err_t ret = ESP_OK;
    if (count > 0U) {
        ret = gt911_read_register(GT911_REG_POINT1_DATA,
                                  point_data,
                                  (size_t)count * GT911_POINT_DATA_SIZE);
    }
    (void)gt911_clear_status();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Point data read failed: %s", esp_err_to_name(ret));
        return -1;
    }

    for (uint8_t i = 0U; i < count; i++) {
        const uint8_t *p = &point_data[i * GT911_POINT_DATA_SIZE];
        p_points[i].x = (uint16_t)(p[1] | (p[2] << 8));
        p_points[i].y = (uint16_t)(p[3] | (p[4] << 8));
    }
    return (int)count;
}

/**
 * @brief Stop the INT-pin ISR from notifying any task.
 */
void hal_touch_stop(void)
{
    s_touch_task_handle = NULL;

    if (s_isr_handler_added == true) {
        /* Ignoring: the handler is being discarded either way. */
        (void)gpio_isr_handler_remove(GT911_INT_GPIO);
        s_isr_handler_added = false;
    }
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Release the ISR, the device and the bus from a previous hal_touch_init(), if any.
 */
static void touch_release(void)
{
    hal_touch_stop();

    if (s_gt911_dev_handle != NULL) {
        /* Ignoring: the device is being discarded either way. */
        (void)i2c_master_bus_rm_device(s_gt911_dev_handle);
        s_gt911_dev_handle = NULL;
    }
    if (s_i2c_bus_handle != NULL) {
        /* Ignoring: the bus is being discarded either way. */
        (void)i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }
}

/**
 * @brief Reset the GT911 and select I2C address 0x5D (INT held LOW during reset).
 *
 * Sequence: RST+INT LOW >=15ms, RST HIGH, wait >=5ms, release INT as a floating input, wait for
 * the controller to boot.
 */
static void gt911_hardware_reset(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << GT911_RST_GPIO) | (1ULL << GT911_INT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_set_level(GT911_RST_GPIO, 0);
    gpio_set_level(GT911_INT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(GT911_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << GT911_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_conf);

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "GT911 hardware reset done, INT released");
}

/**
 * @brief Bring up the I2C master bus and add the GT911 device.
 *
 * @return esp_err_t
 */
static esp_err_t gt911_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GT911_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_FREQ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_config, &s_gt911_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Arm the falling-edge ISR on GT911_INT_GPIO.
 *
 * @return esp_err_t
 */
static esp_err_t gt911_intr_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GT911_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_isr_service_ready == false) {
        ret = gpio_install_isr_service(0);
        if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE)) {
            ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(ret));
            return ret;
        }
        s_isr_service_ready = true;
    }

    ret = gpio_isr_handler_add(GT911_INT_GPIO, gt911_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT ISR handler add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_isr_handler_added = true;

    return ESP_OK;
}

/**
 * @brief Read len bytes starting at a GT911 register address.
 *
 * @param[in] reg_addr 16-bit register address.
 * @param[out] data destination buffer.
 * @param[in] len number of bytes to read.
 * @return esp_err_t
 */
static esp_err_t gt911_read_register(uint16_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t addr_buf[2] = {
        (uint8_t)((reg_addr >> 8) & 0xFFU),
        (uint8_t)(reg_addr & 0xFFU),
    };
    return i2c_master_transmit_receive(s_gt911_dev_handle,
                                       addr_buf,
                                       sizeof(addr_buf),
                                       data,
                                       len,
                                       I2C_TIMEOUT_MS);
}

/**
 * @brief Clear the GT911 status register so the next touch report can be latched.
 *
 * @return esp_err_t
 */
static esp_err_t gt911_clear_status(void)
{
    uint8_t write_buf[3] = {
        (uint8_t)((GT911_REG_STATUS >> 8) & 0xFFU),
        (uint8_t)(GT911_REG_STATUS & 0xFFU),
        0x00U,
    };
    return i2c_master_transmit(s_gt911_dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
}

/**
 * @brief Read and validate the status register.
 *
 * Status not ready (bit 7 clear -- GT911 shows this transiently, ~2ms, while updating) is left
 * uncleared and reported as nothing to read: per the GT911 protocol only a ready buffer is
 * cleared, and the next report ~10ms later carries the real state. Whether a finger is still
 * down is decided by tsk_input across several reads, not from any single one here.
 *
 * @return number of points (1+) on a valid report, 0 if there is nothing to read, or -1 on an
 * I2C failure.
 */
static int touch_read_status(void)
{
    uint8_t status = 0U;
    esp_err_t ret = gt911_read_register(GT911_REG_STATUS, &status, 1U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Status read failed: %s", esp_err_to_name(ret));
        return -1;
    }
#if HAL_TOUCH_DEBUG_LOG_ENABLED
    ESP_LOGI(TAG, "status=0x%02X", status);
#endif

    if ((status & GT911_STATUS_VALID_MASK) == 0U) {
        return 0;
    }

    uint8_t num_points = status & GT911_STATUS_POINTS_MASK;
    if (num_points == 0U) {
        (void)gt911_clear_status();
#if HAL_TOUCH_DEBUG_LOG_ENABLED
        ESP_LOGW(TAG, "num_points=0 -- no point in this report");
#endif
        return 0;
    }

    return (int)num_points;
}

/**
 * @brief GPIO ISR: INT fell (touch state changed). Notifies the task waiting in
 * hal_touch_read().
 *
 * @param[in] p_arg Unused.
 */
static void IRAM_ATTR gt911_isr_handler(void *p_arg)
{
    (void)p_arg;
#if HAL_TOUCH_DEBUG_LOG_ENABLED
    s_isr_fire_count++;
#endif
    TaskHandle_t task = s_touch_task_handle;
    if (task == NULL) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

#if HAL_TOUCH_DEBUG_LOG_ENABLED
/**
 * @brief ulTaskNotifyTake() with diagnostic logging of wait time and coalesced ISR fire count.
 *
 * @param[in] wait_ticks maximum ticks to block.
 * @return notification count (0 = timeout).
 */
static uint32_t touch_notify_wait_logged(TickType_t wait_ticks)
{
    int64_t start_us = esp_timer_get_time();
    uint32_t got = ulTaskNotifyTake(pdTRUE, wait_ticks);
    uint32_t isr_fires = s_isr_fire_count;
    s_isr_fire_count = 0U;
    int64_t wait_ms = (esp_timer_get_time() - start_us) / 1000;

    if (got == 0U) {
        ESP_LOGD(TAG,
                 "notify timeout after %" PRId64 "ms (isr_fires=%u)",
                 wait_ms,
                 (unsigned)isr_fires);
    } else {
        ESP_LOGI(TAG,
                 "notified after %" PRId64 "ms wait (isr_fires=%u)",
                 wait_ms,
                 (unsigned)isr_fires);
    }
    return got;
}
#endif
