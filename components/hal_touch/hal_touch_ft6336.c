/**
 * @file hal_touch_ft6336.c
 * @brief FT6336U capacitive touch controller driver (GDEQ0426T82-T01C 4.26" module) -- I2C
 * transport, raw point read. Implements hal_touch.h in place of the GT911 driver.
 *
 * Ported from Good Display's GDEQ0426T82-T01C Arduino demo (FT6336.cpp, i2c.cpp): reset, then
 * device mode 0, touch threshold 22 and active report period 14. Interrupt mode is set to trigger,
 * so INT pulses for every report rather than staying low while a finger is down.
 *
 * The controller reports in a portrait frame, x 0-479 and y 0-799, turned half a turn from the
 * app's screen (measured on this board, 2026-10-31: START, A, B, SELECT and the d-pad pressed in
 * turn all landed at screen x = 479 - x, screen y = 799 - y). Points are turned into the app's
 * screen frame, then into landscape framebuffer coordinates with the mapping screen.h uses
 * (landscape x = 799 - screen y, landscape y = screen x). CONFIG_PIB_TOUCH_FT6336_ROTATE_180 turns
 * them a further half turn, for a module whose touch layer is fitted the other way round.
 *
 * The module's touch connector carries the same signals as the GT911's, so it uses the GT911's
 * INT and RST pins.
 *
 * @version 0.1
 * @date 2026-10-31
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "hal_touch.h"

#include "board_config.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_err.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Private definitions ---------------------------------------------------------------------------*/

#define FT6336_I2C_ADDR          (0x38U)
#define FT6336_INT_GPIO          (GT911_INT_GPIO) /* same connector pin */
#define FT6336_RST_GPIO          (GT911_RST_GPIO) /* same connector pin */

#define FT6336_REG_DEVICE_MODE   (0x00U)
#define FT6336_REG_TD_STATUS     (0x02U) /* points touched, then the point records */
#define FT6336_REG_THRESHOLD     (0x80U)
#define FT6336_REG_PERIOD_ACTIVE (0x88U)
#define FT6336_REG_CHIP_ID       (0xA3U)
#define FT6336_REG_INT_MODE      (0xA4U)

#define FT6336_DEVICE_MODE_WORK  (0x00U)
#define FT6336_THRESHOLD         (22U) /* demo value */
#define FT6336_PERIOD_ACTIVE     (14U) /* demo value */
#define FT6336_INT_MODE_TRIGGER  (0x01U)

#define FT6336_MAX_POINTS        (2U)
#define FT6336_POINT_SIZE        (6U)
#define FT6336_POINTS_MASK       (0x0FU)
/* Event flag, bits 7-6 of a point's first byte: 0 press down, 1 lift up, 2 contact, 3 none. */
#define FT6336_EVENT_SHIFT       (6U)
#define FT6336_EVENT_LIFT_UP     (1U)
#define FT6336_EVENT_NONE        (3U)
#define FT6336_COORD_HIGH_MASK   (0x0FU)

/* The controller's portrait frame. */
#define FT6336_RAW_W             (EPD_PANEL_HEIGHT)
#define FT6336_RAW_H             (EPD_PANEL_WIDTH)

#define FT6336_RST_LOW_MS        (50U)  /* demo: 50 ms */
#define FT6336_BOOT_MS           (200U) /* demo waits 120 ms before the first access */

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "hal_touch";

static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_ft6336_dev_handle = NULL;
static volatile TaskHandle_t s_touch_task_handle = NULL;
static bool s_isr_service_ready = false;
static bool s_isr_handler_added = false;

/* Private function declarations -----------------------------------------------------------------*/

static void touch_release(void);
static void ft6336_hardware_reset(void);
static esp_err_t ft6336_i2c_init(void);
static esp_err_t ft6336_intr_init(void);
static esp_err_t ft6336_read_register(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t ft6336_write_register(uint8_t reg_addr, uint8_t value);
static void ft6336_to_land(uint16_t raw_x, uint16_t raw_y, hal_touch_point_t *p_point);

/* IRAM_ATTR expands via __COUNTER__, so it goes on the definition only. */
static void ft6336_isr_handler(void *p_arg);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Hardware reset the FT6336, bring up I2C, set it up, and arm the INT-pin ISR.
 *
 * @return esp_err_t
 */
esp_err_t hal_touch_init(void)
{
    touch_release();

    /* Before the ISR is armed: the controller may pulse INT as soon as it is set up. */
    s_touch_task_handle = xTaskGetCurrentTaskHandle();

    ft6336_hardware_reset();

    esp_err_t ret = ft6336_i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t chip_id = 0U;
    ret = ft6336_read_register(FT6336_REG_CHIP_ID, &chip_id, 1U);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Chip ID read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "FT6336 chip ID: 0x%02X", chip_id);

    const uint8_t setup[][2] = {
        {FT6336_REG_DEVICE_MODE, FT6336_DEVICE_MODE_WORK},
        {FT6336_REG_THRESHOLD, FT6336_THRESHOLD},
        {FT6336_REG_PERIOD_ACTIVE, FT6336_PERIOD_ACTIVE},
        {FT6336_REG_INT_MODE, FT6336_INT_MODE_TRIGGER},
    };
    for (size_t i = 0U; i < (sizeof(setup) / sizeof(setup[0])); i++) {
        ret = ft6336_write_register(setup[i][0], setup[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Setup write 0x%02X failed: %s", setup[i][0], esp_err_to_name(ret));
            return ret;
        }
    }

    ret = ft6336_intr_init();
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
    if (ulTaskNotifyTake(pdTRUE, wait_ticks) == 0U) {
        return HAL_TOUCH_TIMEOUT;
    }

    hal_touch_point_t point;
    if (hal_touch_read_points(&point, 1U, 0U) <= 0) {
        return HAL_TOUCH_NO_POINT;
    }

    *x = point.x;
    *y = point.y;
    return HAL_TOUCH_POINT;
}

/**
 * @brief Wait up to wait_ticks for the INT ISR, then read every point of the latest report.
 *
 * The FT6336 always holds the current state, so every read that gets through I2C is a fresh
 * report; a point that is lifting or has no event is not counted.
 *
 * @return int
 */
int hal_touch_read_points(hal_touch_point_t *p_points, uint8_t max_points, TickType_t wait_ticks)
{
    (void)ulTaskNotifyTake(pdTRUE, wait_ticks);

    uint8_t data[1U + (FT6336_MAX_POINTS * FT6336_POINT_SIZE)];
    esp_err_t ret = ft6336_read_register(FT6336_REG_TD_STATUS, data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Point data read failed: %s", esp_err_to_name(ret));
        return -1;
    }

#if HAL_TOUCH_DEBUG_LOG_ENABLED
    /* Every change of the report, raw, so a controller that reports nothing usable still shows */
    static uint8_t s_last[1U + FT6336_POINT_SIZE];
    if (memcmp(s_last, data, sizeof(s_last)) != 0) {
        memcpy(s_last, data, sizeof(s_last));
        ESP_LOGI(TAG,
                 "report: status=0x%02X p1=%02X %02X %02X %02X",
                 data[0],
                 data[1],
                 data[2],
                 data[3],
                 data[4]);
    }
#endif

    uint8_t touched = data[0] & FT6336_POINTS_MASK;
    if (touched > FT6336_MAX_POINTS) {
        touched = 0U; /* out of range: the controller has no valid report */
    }

    int count = 0;
    for (uint8_t i = 0U; (i < touched) && (count < (int)max_points); i++) {
        const uint8_t *p = &data[1U + (i * FT6336_POINT_SIZE)];
        const uint8_t event = (uint8_t)(p[0] >> FT6336_EVENT_SHIFT);
        if ((event == FT6336_EVENT_LIFT_UP) || (event == FT6336_EVENT_NONE)) {
            continue;
        }
        const uint16_t raw_x = (uint16_t)(((p[0] & FT6336_COORD_HIGH_MASK) << 8) | p[1]);
        const uint16_t raw_y = (uint16_t)(((p[2] & FT6336_COORD_HIGH_MASK) << 8) | p[3]);
        ft6336_to_land(raw_x, raw_y, &p_points[count]);
#if HAL_TOUCH_DEBUG_LOG_ENABLED
        ESP_LOGI(TAG,
                 "raw x=%u y=%u -> land x=%u y=%u",
                 raw_x,
                 raw_y,
                 p_points[count].x,
                 p_points[count].y);
#endif
        count++;
    }
    return count;
}

/**
 * @brief Stop the INT-pin ISR from notifying any task.
 */
void hal_touch_stop(void)
{
    s_touch_task_handle = NULL;

    if (s_isr_handler_added == true) {
        /* Ignoring: the handler is being discarded either way. */
        (void)gpio_isr_handler_remove(FT6336_INT_GPIO);
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

    if (s_ft6336_dev_handle != NULL) {
        /* Ignoring: the device is being discarded either way. */
        (void)i2c_master_bus_rm_device(s_ft6336_dev_handle);
        s_ft6336_dev_handle = NULL;
    }
    if (s_i2c_bus_handle != NULL) {
        /* Ignoring: the bus is being discarded either way. */
        (void)i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }
}

/**
 * @brief Reset the FT6336: RST low, then high, then wait for it to boot. Its I2C address is
 * fixed, so INT is left an input throughout.
 */
static void ft6336_hardware_reset(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << FT6336_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);

    gpio_set_level(FT6336_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(FT6336_RST_LOW_MS));
    gpio_set_level(FT6336_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(FT6336_BOOT_MS));
    ESP_LOGI(TAG, "FT6336 hardware reset done");
}

/**
 * @brief Bring up the I2C master bus and add the FT6336 device.
 *
 * @return esp_err_t
 */
static esp_err_t ft6336_i2c_init(void)
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
        .device_address = FT6336_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_FREQ,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus_handle, &dev_config, &s_ft6336_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Arm the falling-edge ISR on FT6336_INT_GPIO.
 *
 * @return esp_err_t
 */
static esp_err_t ft6336_intr_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FT6336_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
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

    ret = gpio_isr_handler_add(FT6336_INT_GPIO, ft6336_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INT ISR handler add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_isr_handler_added = true;

    return ESP_OK;
}

/**
 * @brief Read len bytes starting at an FT6336 register address.
 *
 * @param[in] reg_addr 8-bit register address.
 * @param[out] data destination buffer.
 * @param[in] len number of bytes to read.
 * @return esp_err_t
 */
static esp_err_t ft6336_read_register(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_ft6336_dev_handle,
                                       &reg_addr,
                                       1U,
                                       data,
                                       len,
                                       I2C_TIMEOUT_MS);
}

/**
 * @brief Write one FT6336 register.
 *
 * @param[in] reg_addr 8-bit register address.
 * @param[in] value the value.
 * @return esp_err_t
 */
static esp_err_t ft6336_write_register(uint8_t reg_addr, uint8_t value)
{
    const uint8_t write_buf[2] = {reg_addr, value};
    return i2c_master_transmit(s_ft6336_dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
}

/**
 * @brief Turn a point in the controller's portrait frame into landscape framebuffer coordinates.
 *
 * @param[in] raw_x portrait x, 0 to FT6336_RAW_W - 1.
 * @param[in] raw_y portrait y, 0 to FT6336_RAW_H - 1.
 * @param[out] p_point the landscape point.
 */
static void ft6336_to_land(uint16_t raw_x, uint16_t raw_y, hal_touch_point_t *p_point)
{
    raw_x = (raw_x < FT6336_RAW_W) ? raw_x : (uint16_t)(FT6336_RAW_W - 1U);
    raw_y = (raw_y < FT6336_RAW_H) ? raw_y : (uint16_t)(FT6336_RAW_H - 1U);
#if CONFIG_PIB_TOUCH_FT6336_ROTATE_180
    const uint16_t screen_x = raw_x;
    const uint16_t screen_y = raw_y;
#else
    const uint16_t screen_x = (uint16_t)(FT6336_RAW_W - 1U - raw_x);
    const uint16_t screen_y = (uint16_t)(FT6336_RAW_H - 1U - raw_y);
#endif
    p_point->x = (uint16_t)(FT6336_RAW_H - 1U - screen_y);
    p_point->y = screen_x;
}

/**
 * @brief GPIO ISR: INT fell (a new report). Notifies the task waiting in a read.
 *
 * @param[in] p_arg Unused.
 */
static void IRAM_ATTR ft6336_isr_handler(void *p_arg)
{
    (void)p_arg;
    TaskHandle_t task = s_touch_task_handle;
    if (task == NULL) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
