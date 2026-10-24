# Paper Ink Boy

A Game Boy emulator on e-paper, for the **Atlas board**:
- ESP32-C5 N16R8
- GoodDisplay GDEY075T7-T01: 7.5", 800×480, UC8179C controller
- GT911 touch controller

The device is held upright like a Game Boy. The screen sits at the top at 3× scale (480×432), and the touch controls are below it.

## How it works

| Part | Where |
|---|---|
| Emulator core: [Peanut-GB](https://github.com/deltabeard/Peanut-GB) (MIT), DMG only, no sound | `components/peanut_gb/` |
| ROM loading, save files, frame pacing | `main/gb_emu.c` |
| 3× scaling, 4-shade dithering, dirty-area detection | `main/renderer.c` |
| On-screen controls and multi-touch | `main/touch_pad.c` |
| Tasks, refresh scheduling | `main/main.c` |
| Display, touch and board drivers, copied from `factory_fw_atlas_memoir` | `components/` |

The emulator always runs at full speed (59.7 Hz). The display is much slower than that, so the display task waits until the panel is free, takes the newest frame, redraws only the area that changed, and refreshes just that window. Frames the panel can't keep up with are skipped.

Every 100 refreshes, or when you press the side button, it runs a full-screen fast refresh (~1.7 s) to clear ghosting.

### Display speed

| Mode | Refresh time | When |
|---|---|---|
| Quick (custom LUT) | depends on `PIB_QUICK_FRAMES` | `components/panel_uc8179c/nda/panel_uc8179c_lut_nda.h` is present |
| Stock partial | ~430 ms (~2 FPS) | fallback without the NDA header |

The quick-mode waveform values are under NDA with Good Display, so they are **not in git** (`nda/` is ignored). To enable quick mode:
1. Copy your `panel_uc8179c_lut_nda.h` into `components/panel_uc8179c/nda/`.
2. Run `idf.py reconfigure build`. The file is detected at configure time.

The serial log shows `display N fps (quick|partial)` every 5 s.

Quick-mode tuning options:
- `idf.py -DPIB_QUICK_FRAMES=10 reconfigure build`: fewer frames per scan gives a faster refresh but fainter changes. The default is 16.
- `-DPIB_QUICK_STRONG_DRIVE=1`: uses the full-refresh drive voltages.
- `QUICK_BUDGET_BLACK` / `QUICK_BUDGET_WHITE` and `WHOLE_REFRESH_EVERY` in `main/main.c`.

## Build and flash

```powershell
. C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1   # ESP-IDF v6.1
idf.py set-target esp32c5        # first time only
idf.py build
idf.py -p COM5 flash monitor
```

## Load a game

ROMs go in their own 8 MB flash partition, so you can switch games without rebuilding:

```powershell
.\tools\flash_rom.ps1 -Rom .\game.gb -Port COM5
```

If no valid ROM is flashed, the screen shows **NO GAME**.

Saves (battery-backed cartridge RAM) go to the `saves` partition automatically a few seconds after the game writes them. Pressing the side button also saves.

## Controls

- **D-pad** (lower left): diagonals work; touch between two arms.
- **A / B** (lower right).
- **SELECT / START** (bottom).
- **Side button** (GPIO1): clears ghosting with a full refresh and saves now.

Multi-touch is supported, so a direction plus A works.

If the picture is upside down on your unit, build with `SCREEN_FLIP=1`. Add `-DSCREEN_FLIP=1` to `main/CMakeLists.txt` with `target_compile_definitions`.

## Limits

- No sound: the board has no audio output.
- No Game Boy Color mode: Peanut-GB is DMG only. Most GBC-compatible carts run in DMG mode.
- Fast action games are playable but choppy. Turn-based games (Pokémon, Tetris, puzzle and RPG games) suit e-paper best.
