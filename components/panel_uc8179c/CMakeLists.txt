# panel_uc8179c: panel driver for the UC8179C (GDEY075T7-T01). It reaches the hardware only through
# display_port.h.
#
# nda/panel_uc8179c_lut_nda.h holds the quick-mode waveform values, which are under NDA with Good
# Display and not in git (see panel_uc8179c_lut_nda.h.example). When the file is there, quick mode
# is built in; without it the panel falls back to the controller's OTP partial refresh (~430 ms).
#
# Quick-mode tunables, overridable on the command line, e.g. idf.py -DPIB_QUICK_FRAMES=10 build:
#   PIB_QUICK_FRAMES        frames per quick scan (fewer = faster, fainter changes)
#   PIB_QUICK_STRONG_DRIVE  1 = full-refresh drive voltages, moves pixels further per frame
#
# Built in only when menuconfig's panel is the 7.5" (see display's Kconfig); otherwise the component
# is empty.
if(CONFIG_PIB_PANEL_GDEQ0426T82)
    idf_component_register(INCLUDE_DIRS "." REQUIRES display PRIV_REQUIRES board_config)
    return()
endif()

set(PIB_QUICK_FRAMES "16" CACHE STRING "UC8179C quick-mode frames per scan")
set(PIB_QUICK_STRONG_DRIVE "0" CACHE STRING "UC8179C quick mode: use full-refresh drive voltages")
#   PIB_QUICK_PLL           PLL/frame-rate byte (command 0x30) for quick mode, e.g. 0x3A; empty =
#                           the NDA header's value. A faster frame rate shortens every frame.
set(PIB_QUICK_PLL "" CACHE STRING "UC8179C quick mode: PLL (0x30) byte override, empty = NDA value")
#   PIB_QUICK_GREY_FRAMES   frames a grey pixel is driven toward black after going white: sets how
#                           dark the grey is (more = darker)
#   PIB_QUICK_REST_FRAMES   undriven frames between the white and the grey drive, for the pigment
#                           to settle; a scan that drives any grey takes REST + GREY frames longer
set(PIB_QUICK_GREY_FRAMES "2" CACHE STRING "UC8179C quick mode: grey drive frames (darkness)")
set(PIB_QUICK_REST_FRAMES "4" CACHE STRING "UC8179C quick mode: rest frames before the grey drive")

set(panel_priv_includes "")
set(panel_quick OFF)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/nda/panel_uc8179c_lut_nda.h")
    set(panel_priv_includes "nda")
    set(panel_quick ON)
else()
    message(WARNING "panel_uc8179c: nda/panel_uc8179c_lut_nda.h not found -- quick mode is off, "
                    "the display uses the stock partial refresh (~2 FPS)")
endif()

idf_component_register(
    SRCS "panel_uc8179c.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS ${panel_priv_includes}
    REQUIRES display
    PRIV_REQUIRES board_config
)

if(panel_quick)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE
        PANEL_QUICK_ENABLED=1
        PANEL_QUICK_FRAMES=${PIB_QUICK_FRAMES}U
        PANEL_QUICK_STRONG_DRIVE=${PIB_QUICK_STRONG_DRIVE}
        PANEL_QUICK_GREY_FRAMES=${PIB_QUICK_GREY_FRAMES}U
        PANEL_QUICK_REST_FRAMES=${PIB_QUICK_REST_FRAMES}U)
    if(NOT PIB_QUICK_PLL STREQUAL "")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE PANEL_QUICK_PLL=${PIB_QUICK_PLL}U)
    endif()
endif()

# display refers to display_select_panel(), which this driver defines, but the components depend
# on each other both ways, so this library can come before display's in the link and the reference
# would be left undefined. Asking for the symbol up front pulls it in wherever the library sits.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--undefined=display_select_panel")
