# panel_ssd1677: panel driver for the SSD1677 (GDEQ0426T82-T01C, 4.26"). It reaches the hardware
# only through display_port.h. Built in only with CONFIG_PIB_PANEL_GDEQ0426T82 (see display's
# Kconfig); otherwise the component is empty.
if(NOT CONFIG_PIB_PANEL_GDEQ0426T82)
    idf_component_register(INCLUDE_DIRS "." REQUIRES display PRIV_REQUIRES board_config)
    return()
endif()

idf_component_register(
    SRCS "panel_ssd1677.c"
    INCLUDE_DIRS "."
    REQUIRES display
    PRIV_REQUIRES board_config
)

# Quick-mode tunables, e.g. idf.py -DPIB_SSD1677_QUICK_FRAMES=2 reconfigure build:
#   PIB_SSD1677_QUICK_FRAMES       frames per quick scan (fewer = faster scans, fainter steps); a
#                                  changed pixel gets PIB_QUICK_BUDGET scans (main/CMakeLists.txt)
#   PIB_SSD1677_QUICK_FR           LUT frame rate setting, 0-15 (the demo uses 2)
#   PIB_SSD1677_QUICK_REST_FRAMES  grey only: undriven frames before the grey drive
#   PIB_SSD1677_QUICK_GREY_FRAMES  grey only: frames a grey pixel is driven toward black
set(PIB_SSD1677_QUICK_FRAMES "3" CACHE STRING "SSD1677 quick mode: frames per scan")
set(PIB_SSD1677_QUICK_FR "2" CACHE STRING "SSD1677 quick mode: LUT frame rate setting")
set(PIB_SSD1677_QUICK_REST_FRAMES "2" CACHE STRING "SSD1677 quick mode: rest frames before grey")
set(PIB_SSD1677_QUICK_GREY_FRAMES "1" CACHE STRING "SSD1677 quick mode: grey drive frames")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    PANEL_QUICK_FRAMES=${PIB_SSD1677_QUICK_FRAMES}U
    PANEL_QUICK_FR=${PIB_SSD1677_QUICK_FR}U
    PANEL_QUICK_REST_FRAMES=${PIB_SSD1677_QUICK_REST_FRAMES}U
    PANEL_QUICK_GREY_FRAMES=${PIB_SSD1677_QUICK_GREY_FRAMES}U)

# display refers to display_select_panel(), which this driver defines, but the components depend
# on each other both ways, so this library can come before display's in the link and the reference
# would be left undefined. Asking for the symbol up front pulls it in wherever the library sits.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=display_select_panel")
