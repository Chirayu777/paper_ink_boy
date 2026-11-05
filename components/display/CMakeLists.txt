# display: the panel-independent half of the display driver -- framebuffer and drawing, the
# refresh engine and the interfaces a panel driver and a port implement. It names no panel and no
# hardware. The panel driver and the port are named here, and only here: this is the one place that
# says which panel this board has. The panel is picked in menuconfig (Paper Ink Boy display ->
# E-paper panel, see Kconfig). Every panel driver is required here, because IDF works out component
# requirements before it reads sdkconfig; each driver compiles its source only when it is the panel
# selected, so only one display_select_panel() exists.
#
# display calls functions that those components define, and they in turn include display's
# headers, so the dependency runs both ways. Listing them here is what makes the linker see
# display's libraries after theirs; without it the port and display_select_panel() are undefined
# at link time.
idf_component_register(
    SRCS "display.c" "display_fb.c" "display_panel.c"
    INCLUDE_DIRS "."
    REQUIRES epd_geom
    PRIV_REQUIRES display_port_idf esp_timer panel_uc8179c panel_ssd1677
)

# Diagnostics: idf.py -DPIB_DISPLAY_TIMING=1 reconfigure build logs every refresh's waveform time,
# data streams and total.
set(PIB_DISPLAY_TIMING "0" CACHE STRING "Log the display's per-refresh timing")
if(PIB_DISPLAY_TIMING)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DISPLAY_TIMING_MONITOR_ENABLED=1)
endif()
