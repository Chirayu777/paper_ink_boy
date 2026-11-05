# hal_touch: the touch controller of the panel module chosen in menuconfig (see display's Kconfig):
# the GT911 on the 7.5" module, the FT6336U on the 4.26" one. Both implement hal_touch.h.
if(CONFIG_PIB_PANEL_GDEQ0426T82)
    set(hal_touch_srcs "hal_touch_ft6336.c")
else()
    set(hal_touch_srcs "hal_touch.c")
endif()

idf_component_register(
    SRCS ${hal_touch_srcs}
    INCLUDE_DIRS "."
    PRIV_REQUIRES board_config esp_driver_gpio esp_driver_i2c esp_timer
)

# Diagnostics: idf.py -DPIB_TOUCH_DEBUG=1 reconfigure build logs every touch report and point.
set(PIB_TOUCH_DEBUG "0" CACHE STRING "Log every touch report")
if(PIB_TOUCH_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE HAL_TOUCH_DEBUG_LOG_ENABLED=1)
endif()
