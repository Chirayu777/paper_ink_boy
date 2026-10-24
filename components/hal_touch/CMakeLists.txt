idf_component_register(
    SRCS "hal_touch.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES board_config esp_driver_gpio esp_driver_i2c esp_timer
)
