# display_port_idf: the display port (display_port.h) for ESP-IDF. Link it in with the application;
# the display core never names it.
idf_component_register(
    SRCS "display_port_idf.c"
    INCLUDE_DIRS "."
    REQUIRES display
    PRIV_REQUIRES esp_driver_gpio esp_driver_spi hal_spi
)
