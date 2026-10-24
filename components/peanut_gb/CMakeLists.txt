# peanut_gb: Peanut-GB, a single-header Game Boy (DMG) emulator core by Mahyar Koshkouei, MIT
# licensed (see the header). Upstream: https://github.com/deltabeard/Peanut-GB
# Header only: exactly one source file (main/gb_emu.c) includes it without PEANUT_GB_HEADER_ONLY.
idf_component_register(
    INCLUDE_DIRS "."
)
