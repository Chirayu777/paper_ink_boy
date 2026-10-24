/**
 * @file display_port_idf.c
 * @brief display_port_idf implementation -- the display port on ESP-IDF's SPI master and GPIO.
 *
 * Commands and parameter bytes go out by polling, one transfer each. An image goes out by DMA from
 * tiles in internal RAM: the SPI completion callback runs in the SPI interrupt and only notifies
 * the owner, so the owner can refill a tile while another is on the wire.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "display_port.h"

#include "hal_spi.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"

#include <string.h>

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "display_port";

static display_port_config_t s_config;
static spi_device_handle_t s_spi_dev = NULL;
static spi_transaction_t s_tile_trans[DISPLAY_PORT_TILE_COUNT];
static TaskHandle_t s_notify_task = NULL;
static uint32_t s_notify_bit = 0U;

/* Private function declarations -----------------------------------------------------------------*/

static esp_err_t port_gpio_init(void);
static esp_err_t port_spi_init(void);
static void port_spi_post_cb(spi_transaction_t *p_trans);
static esp_err_t port_write_byte(uint32_t dc_level, uint8_t value);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Bring up the pins and the bus device.
 *
 * @return esp_err_t
 */
esp_err_t display_port_init(const display_port_config_t *p_config)
{
    if (p_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_config = *p_config;

    esp_err_t ret = port_gpio_init();
    if (ret != ESP_OK) {
        return ret;
    }
    return port_spi_init();
}

/**
 * @brief The BUSY sampling interval the panel driver asked for.
 *
 * @return uint32_t
 */
uint32_t display_port_busy_debounce_us(void)
{
    return s_config.busy_debounce_us;
}

/**
 * @brief Allocate memory that suits its use.
 *
 * @return void *
 */
void *display_port_alloc(display_alloc_t kind, size_t bytes)
{
    if (kind == DISPLAY_ALLOC_FRAMEBUFFER) {
        return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    }

    void *p_mem = heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (p_mem == NULL) {
        return NULL;
    }
    if ((esp_ptr_internal(p_mem) == false) || (esp_ptr_dma_capable(p_mem) == false)) {
        ESP_LOGE(TAG, "DMA tile is not internal DMA-capable RAM (M4)");
        heap_caps_free(p_mem);
        return NULL;
    }
    return p_mem;
}

/**
 * @brief The largest tile a stream can queue.
 *
 * @return uint32_t
 */
uint32_t display_port_tile_bytes(void)
{
    return HAL_SPI_MAX_TRANSFER_BYTES;
}

/**
 * @brief Register the task and notification bit set when a queued tile has left.
 */
void display_port_set_notify(TaskHandle_t task, uint32_t bit)
{
    s_notify_task = task;
    s_notify_bit = bit;
}

/**
 * @brief Send a command byte: data/command line low, chip select asserted for the transfer only.
 *
 * @return esp_err_t
 */
esp_err_t display_port_cmd(uint8_t cmd)
{
    return port_write_byte(0U, cmd);
}

/**
 * @brief Send a data byte: data/command line high, chip select asserted for the transfer only.
 *
 * @return esp_err_t
 */
esp_err_t display_port_data(uint8_t data)
{
    return port_write_byte(1U, data);
}

/**
 * @brief Drive the panel's reset line.
 */
void display_port_rst(bool high)
{
    (void)gpio_set_level(s_config.pin_rst, (high == true) ? 1U : 0U);
}

/**
 * @brief Sample the panel's BUSY line once.
 *
 * @return bool
 */
bool display_port_busy_ready(void)
{
    return gpio_get_level(s_config.pin_busy) == (int)s_config.busy_ready;
}

/**
 * @brief Open a stream: collect results left by an aborted one, take the bus and assert chip
 * select with the data line high.
 *
 * @return esp_err_t
 */
esp_err_t display_port_stream_open(void)
{
    /* A tile that never finished in an aborted stream may return late; discard such results. */
    spi_transaction_t *p_stale = NULL;
    while (spi_device_get_trans_result(s_spi_dev, &p_stale, 0) == ESP_OK) {
    }

    esp_err_t ret = spi_device_acquire_bus(s_spi_dev, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "stream open: bus not acquired: %s", esp_err_to_name(ret));
        return ret;
    }

    (void)gpio_set_level(s_config.pin_cs, 0U);
    (void)gpio_set_level(s_config.pin_dc, 1U);
    return ESP_OK;
}

/**
 * @brief Queue one tile of the stream for DMA.
 *
 * @return esp_err_t
 */
esp_err_t display_port_stream_queue(const uint8_t *p_tile, size_t bytes, uint8_t slot)
{
    if (slot >= DISPLAY_PORT_TILE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_tile_trans[slot], 0, sizeof(s_tile_trans[slot]));
    s_tile_trans[slot].length = bytes * 8U;
    s_tile_trans[slot].tx_buffer = p_tile;
    s_tile_trans[slot].user = (void *)p_tile;
    esp_err_t ret = spi_device_queue_trans(s_spi_dev, &s_tile_trans[slot], 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_queue_trans failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Collect one tile that has left.
 *
 * @return esp_err_t
 */
esp_err_t display_port_stream_reap(uint32_t wait_ms)
{
    spi_transaction_t *p_done = NULL;
    esp_err_t ret = spi_device_get_trans_result(s_spi_dev, &p_done, pdMS_TO_TICKS(wait_ms));
    if ((ret != ESP_OK) && (ret != ESP_ERR_TIMEOUT)) {
        ESP_LOGE(TAG, "spi_device_get_trans_result failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Close a stream: release chip select and the bus.
 */
void display_port_stream_close(void)
{
    (void)gpio_set_level(s_config.pin_cs, 1U);
    spi_device_release_bus(s_spi_dev);
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief Configure CS/DC/RES as outputs and BUSY as a plain pulled-up input.
 *
 * BUSY is open-drain on the panels this port has been used with, so it needs the internal pull-up:
 * the pin floats LOW on ESP32-C5 without it, and every BUSY watch would time out. No interrupt:
 * BUSY is sampled by the core, never edge-triggered (ACR-001).
 *
 * @return esp_err_t
 */
static esp_err_t port_gpio_init(void)
{
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << s_config.pin_cs) |
                        (1ULL << s_config.pin_dc) |
                        (1ULL << s_config.pin_rst),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&out_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Output GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_config_t busy_conf = {
        .pin_bit_mask = (1ULL << s_config.pin_busy),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&busy_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BUSY GPIO config failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Bring up the shared bus and add the display's device to it (software CS).
 *
 * @return esp_err_t
 */
static esp_err_t port_spi_init(void)
{
    esp_err_t ret = hal_spi_init();
    if (ret != ESP_OK) {
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)s_config.spi_clock_hz,
        .mode = 0,
        .spics_io_num = -1, /* software CS, held low across a whole image */
        .queue_size = DISPLAY_PORT_TILE_COUNT,
        .post_cb = port_spi_post_cb,
    };
    ret = spi_bus_add_device(HAL_SPI_HOST, &dev_cfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief SPI transaction-done callback, in the SPI interrupt: wake the owner to refill a tile.
 *
 * Only stream tiles carry a user pointer, so the command and parameter bytes sent by polling do
 * not wake anyone.
 *
 * @param[in] p_trans The finished transaction.
 */
static void IRAM_ATTR port_spi_post_cb(spi_transaction_t *p_trans)
{
    if ((p_trans->user == NULL) || (s_notify_task == NULL)) {
        return;
    }

    BaseType_t woken = pdFALSE;
    (void)xTaskNotifyFromISR(s_notify_task, s_notify_bit, eSetBits, &woken);
    portYIELD_FROM_ISR(woken);
}

/**
 * @brief Send one byte with the data/command line at a level, chip select asserted only for the
 * transfer.
 *
 * @param[in] dc_level 0 for a command, 1 for data.
 * @param[in] value the byte.
 * @return esp_err_t
 */
static esp_err_t port_write_byte(uint32_t dc_level, uint8_t value)
{
    esp_err_t ret = spi_device_acquire_bus(s_spi_dev, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X: bus not acquired: %s", value, esp_err_to_name(ret));
        return ret;
    }

    (void)gpio_set_level(s_config.pin_cs, 0U);
    (void)gpio_set_level(s_config.pin_dc, dc_level);
    spi_transaction_t trans = {
        .length = 8U,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    trans.tx_data[0] = value;
    ret = spi_device_polling_transmit(s_spi_dev, &trans);
    (void)gpio_set_level(s_config.pin_cs, 1U);
    spi_device_release_bus(s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X failed: %s", value, esp_err_to_name(ret));
    }
    return ret;
}
