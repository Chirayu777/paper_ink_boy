/**
 * @file hal_spi.c
 * @brief hal_spi implementation -- brings up the board's shared SPI bus.
 *
 * @version 0.1
 * @date 2026-10-12
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "hal_spi.h"

#include "esp_log.h"

/* Private variables -----------------------------------------------------------------------------*/

static const char *TAG = "hal_spi";

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Bring up the shared SPI bus with DMA; a bus that is already up is not an error.
 *
 * @return esp_err_t
 */
esp_err_t hal_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_PIN_MOSI,
        .miso_io_num = SPI_PIN_MISO,
        .sclk_io_num = SPI_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = HAL_SPI_MAX_TRANSFER_BYTES,
    };
    esp_err_t ret = spi_bus_initialize(HAL_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE)) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
