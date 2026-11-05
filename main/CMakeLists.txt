idf_component_register(
    SRCS "main.c" "gb_emu.c" "renderer.c" "touch_pad.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES board_config display epd_geom hal_touch peanut_gb
                  esp_driver_gpio esp_partition esp_timer
)

# Peanut-GB is compiled inside gb_emu.c; keep its warnings from failing the build.
set_source_files_properties(gb_emu.c PROPERTIES COMPILE_OPTIONS "-Wno-error;-Wno-unused-function")

# Game picture scale: 3 fills the panel width, 2 or 1 is smaller but sends less data per refresh.
# idf.py -DPIB_GAME_SCALE=2 reconfigure build
set(PIB_GAME_SCALE "3" CACHE STRING "Game Boy pixels per screen pixel: 1, 2 or 3")
target_compile_definitions(${COMPONENT_LIB} PRIVATE GAME_SCALE=${PIB_GAME_SCALE}U)

# Real grey: light grey as the panel's own grey, dark grey as a grey/black checker, instead of
# black-dot dithering. Needs quick mode (the NDA header). idf.py -DPIB_GREY=1 reconfigure build
set(PIB_GREY "0" CACHE STRING "1 = draw Game Boy greys with the panel's real grey")
target_compile_definitions(${COMPONENT_LIB} PRIVATE RENDER_GREY=${PIB_GREY})

# Quick-mode drive budget: scans a changed pixel is driven for (1-15). With short scans
# (PIB_QUICK_FRAMES), a bigger budget gives clean final pixels; a smaller one, sharper motion.
# idf.py -DPIB_QUICK_BUDGET=8 reconfigure build
set(PIB_QUICK_BUDGET "6" CACHE STRING "Quick-mode drive budget per changed pixel, in scans")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    QUICK_BUDGET_BLACK=${PIB_QUICK_BUDGET}U
    QUICK_BUDGET_WHITE=${PIB_QUICK_BUDGET}U)
