/**
 * @file board_atlas.h
 * @brief Pin map and bus parameters for the Atlas production PCB (ESP32-C5).
 *
 * Copied from the Atlas firmware (fw/components/board_config/board_config.h); battery, charger and
 * button pins from its dev branch. Include board_config.h, not this file.
 *
 * @version 0.4
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

/* Board variant ---------------------------------------------------------------------------------*/

/* Every pin below matches the production PCB. */
#define BOARD_PINS_VERIFIED (1)

/* 1 for development kit hardware, 0 for the custom production PCB. Selects the GT911 wiring. */
#define DEV_KIT_MODE (0)

/* SPI2 bus, shared by the EPD and the SD card ---------------------------------------------------*/

#define SPI_PIN_SCLK            (6u)     /* SD_EPD_CLK */
#define SPI_PIN_MOSI            (7u)     /* SD_CMD_EPD_SDI */
#define SPI_PIN_MISO            (26u)    /* SD_DAT0, read by the SD card only */
#define SPI_MAX_TRANSFER_BYTES  (8192U)  /* largest single transfer the bus is set up for */

/* EPD pins (UC8179C, ESP32-FTS02 board) ---------------------------------------------------------*/

#define EPD_PIN_CS    (8u)   /* chip select */
#define EPD_PIN_RES   (9u)   /* reset */
#define EPD_PIN_DC    (10u)  /* data/command -- A16 schematic */
#define EPD_PIN_BUSY  (23u)  /* 0 = busy, 1 = ready */

/* The panel fitted to this board, in its native (landscape) pixels. */
#define EPD_PANEL_WIDTH   (800U)
#define EPD_PANEL_HEIGHT  (480U)

/* SD card pins (SPI mode) -----------------------------------------------------------------------*/

#define SD_PIN_CS  (5u)  /* chip select */

/* GT911 touch controller pins -------------------------------------------------------------------*/

#if DEV_KIT_MODE
#define GT911_INT_GPIO  (36)  /* interrupt -- dev kit */
#define GT911_RST_GPIO  (39)  /* reset -- dev kit */
#else
#define GT911_INT_GPIO  (0)   /* interrupt -- custom board */
#define GT911_RST_GPIO  (24)  /* reset -- custom board */
#endif

/* I2C bus parameters ----------------------------------------------------------------------------*/

#define I2C_SCL_IO      (2)
#define I2C_SDA_IO      (3)
#define I2C_CLK_FREQ    (100000)  /* 100 kHz standard mode */
#define I2C_TIMEOUT_MS  (2000)

/* Battery and charger (BQ24090) -----------------------------------------------------------------*/

/* BATT_LVL is the cell voltage through a 100k/100k divider: cell = 2 x ADC input. */
#define BATT_LVL_ADC_GPIO          (4U)       /* ADC1 */
#define BATT_CHG_STATUS_GPIO       (25U)      /* CHG, active-low, external 10k pull-up */
#define BATT_PG_STATUS_GPIO        (27U)      /* PG_L, active-low, external 10k pull-up */
#define BATT_DIVIDER_R_TOP_OHM     (100000U)
#define BATT_DIVIDER_R_BOTTOM_OHM  (100000U)
#define BATT_HAS_CHARGER_STATUS    (1)        /* CHG / PG pins are fitted */

/* Button ----------------------------------------------------------------------------------------*/

#define BUTTON_GPIO            (1U)  /* wake-up button */
#define BUTTON_ACTIVE_LEVEL    (0U)  /* active-low */
#define BUTTON_INTERNAL_PULLUP (0)   /* external pull-up fitted */
