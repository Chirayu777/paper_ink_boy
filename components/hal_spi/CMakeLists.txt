idf_component_register(
    SRCS "hal_spi.c"
    INCLUDE_DIRS "."
    REQUIRES board_config esp_driver_spi
)
