/**
 * @file renderer.c
 * @brief renderer implementation.
 *
 * Each Game Boy pixel becomes a 3x3 cell of panel pixels. The 1-bit panel shows the 4 shades as
 * 0, 3, 6 and 9 black dots in the cell, a fixed ordered pattern, so a still image stays still and
 * only cells whose shade changed flip any panel pixels.
 *
 * The game area is rotated into landscape (screen.h): a landscape row of it is one column of
 * screen pixels, so each landscape row maps to one fixed Game Boy column and each landscape column
 * to one fixed Game Boy row. Both maps are tables built once.
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "renderer.h"

#include "display_fb.h"
#include "gb_emu.h"
#include "screen.h"

#include <string.h>

/* Private definitions ---------------------------------------------------------------------------*/

/* The game area in landscape: GAME_H wide, GAME_W tall. */
#define IMG_W        (GAME_H)
#define IMG_H        (GAME_W)
#define IMG_STRIDE   (IMG_W / 8U)

/* Dither cells: bit (j * 3 + i) set = panel pixel (i, j) of the cell is black, i along screen x. */
#define CELL_LIGHT   (0x0A1U) /* (0,0) (2,1) (1,2) */
#define CELL_DARK    (0x15EU) /* the complement of light */
#define CELL_BLACK   (0x1FFU)

/* Side of the repeating dither pattern: the 3x3 cell at scale 3, a 2x2 ordered dither otherwise. */
#if GAME_SCALE == 3
#define CELL_N       (3U)
#else
#define CELL_N       (2U)
#endif

/* Private variables -----------------------------------------------------------------------------*/

static const uint16_t s_cells[4] = {0x000U, CELL_LIGHT, CELL_DARK, CELL_BLACK};
/* 2x2 ordered dither: a pixel is black when its threshold is below the shade's dot count. */
static const uint8_t s_bayer[2][2] = {{0U, 2U}, {3U, 1U}};
static const uint8_t s_bayer_dots[4] = {0U, 1U, 3U, 4U};

static epd_rect_t s_game_land;
/* Per landscape row of the game area: the Game Boy column and the x inside its cell. */
static uint8_t s_row_gx[IMG_H];
static uint8_t s_row_i[IMG_H];
/* Per landscape column of the game area: the Game Boy row and the y inside its cell. */
static uint8_t s_col_gy[IMG_W];
static uint8_t s_col_j[IMG_W];

static uint8_t s_img[IMG_H * IMG_STRIDE];
static uint8_t s_shown[GB_EMU_WIDTH * GB_EMU_HEIGHT];

/* Private function declarations -----------------------------------------------------------------*/

static bool find_dirty(const uint8_t *p_frame, epd_rect_t *p_cells);
static void render_row(const uint8_t *p_frame, uint16_t r);
static bool cell_black(uint32_t shade, uint32_t i, uint32_t j);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Build the lookup tables and set the shown frame to all white.
 */
void renderer_init(void)
{
    const epd_rect_t game_screen = {.x = GAME_X, .y = GAME_Y, .w = GAME_W, .h = GAME_H};
    s_game_land = screen_to_land_rect(&game_screen);

    for (uint16_t r = 0U; r < IMG_H; r++) {
        uint16_t sx = 0U;
        uint16_t sy = 0U;
        land_to_screen(s_game_land.x, (uint16_t)(s_game_land.y + r), &sx, &sy);
        sx = (uint16_t)(sx - GAME_X);
        s_row_gx[r] = (uint8_t)(sx / GAME_SCALE);
        s_row_i[r] = (uint8_t)(sx % CELL_N);
    }
    for (uint16_t c = 0U; c < IMG_W; c++) {
        uint16_t sx = 0U;
        uint16_t sy = 0U;
        land_to_screen((uint16_t)(s_game_land.x + c), s_game_land.y, &sx, &sy);
        sy = (uint16_t)(sy - GAME_Y);
        s_col_gy[c] = (uint8_t)(sy / GAME_SCALE);
        s_col_j[c] = (uint8_t)(sy % CELL_N);
    }

    memset(s_shown, 0, sizeof(s_shown));
}

/**
 * @brief Draw a frame into the framebuffer where it differs from the last frame drawn.
 *
 * @return bool
 */
bool renderer_draw(const uint8_t *p_frame, bool whole, epd_rect_t *p_dirty)
{
    epd_rect_t cells = {.x = 0U, .y = 0U, .w = GB_EMU_WIDTH, .h = GB_EMU_HEIGHT};
    if ((whole == false) && (find_dirty(p_frame, &cells) == false)) {
        return false;
    }

    const epd_rect_t dirty_screen = {
        .x = (uint16_t)(GAME_X + (cells.x * GAME_SCALE)),
        .y = (uint16_t)(GAME_Y + (cells.y * GAME_SCALE)),
        .w = (uint16_t)(cells.w * GAME_SCALE),
        .h = (uint16_t)(cells.h * GAME_SCALE),
    };
    const epd_rect_t dirty_land = screen_to_land_rect(&dirty_screen);

    /* Whole landscape rows of the game area are redrawn: a row is cheap, and draw_image then
     * writes byte-aligned spans. Columns outside the dirty area come out as they already are. */
    const uint16_t r0 = (uint16_t)(dirty_land.y - s_game_land.y);
    for (uint16_t r = r0; r < (uint16_t)(r0 + dirty_land.h); r++) {
        render_row(p_frame, r);
    }
    const epd_rect_t rows = {
        .x = s_game_land.x,
        .y = dirty_land.y,
        .w = IMG_W,
        .h = dirty_land.h,
    };
    display_draw_image(&rows, &s_img[(size_t)r0 * IMG_STRIDE]);

    memcpy(s_shown, p_frame, sizeof(s_shown));
    *p_dirty = dirty_land;
    return true;
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief The bounding box, in Game Boy pixels, of where a frame differs from the shown one.
 *
 * @param[in] p_frame the new frame.
 * @param[out] p_cells bounding box.
 * @return true if anything differs.
 */
static bool find_dirty(const uint8_t *p_frame, epd_rect_t *p_cells)
{
    uint16_t x0 = GB_EMU_WIDTH;
    uint16_t x1 = 0U;
    uint16_t y0 = GB_EMU_HEIGHT;
    uint16_t y1 = 0U;

    for (uint16_t y = 0U; y < GB_EMU_HEIGHT; y++) {
        const uint8_t *p_new = &p_frame[(size_t)y * GB_EMU_WIDTH];
        const uint8_t *p_old = &s_shown[(size_t)y * GB_EMU_WIDTH];
        if (memcmp(p_new, p_old, GB_EMU_WIDTH) == 0) {
            continue;
        }
        if (y < y0) {
            y0 = y;
        }
        y1 = y;
        uint16_t left = 0U;
        while (p_new[left] == p_old[left]) {
            left++;
        }
        uint16_t right = GB_EMU_WIDTH - 1U;
        while (p_new[right] == p_old[right]) {
            right--;
        }
        if (left < x0) {
            x0 = left;
        }
        if (right > x1) {
            x1 = right;
        }
    }

    if (y0 == GB_EMU_HEIGHT) {
        return false;
    }
    p_cells->x = x0;
    p_cells->y = y0;
    p_cells->w = (uint16_t)(x1 - x0 + 1U);
    p_cells->h = (uint16_t)(y1 - y0 + 1U);
    return true;
}

/**
 * @brief Whether pixel (i, j) of the dither pattern is black for a shade.
 *
 * @param[in] shade 0 = white to 3 = black.
 * @param[in] i x inside the pattern, along screen x.
 * @param[in] j y inside the pattern, along screen y.
 * @return true for black.
 */
static bool cell_black(uint32_t shade, uint32_t i, uint32_t j)
{
#if GAME_SCALE == 3
    (void)s_bayer;
    (void)s_bayer_dots;
    return (s_cells[shade] & (1U << ((j * CELL_N) + i))) != 0U;
#else
    (void)s_cells;
    return s_bayer[j][i] < s_bayer_dots[shade];
#endif
}

/**
 * @brief Render one landscape row of the game area into s_img, 1 = black.
 *
 * @param[in] p_frame the frame.
 * @param[in] r row of the game area.
 */
static void render_row(const uint8_t *p_frame, uint16_t r)
{
    const uint8_t gx = s_row_gx[r];
    const uint8_t i = s_row_i[r];
    /* For this row's x inside the cell: bit j set = pixel (i, j) of a cell of that shade is black. */
    uint8_t black_j[4];
    for (uint32_t shade = 0U; shade < 4U; shade++) {
        black_j[shade] = 0U;
        for (uint32_t j = 0U; j < CELL_N; j++) {
            if (cell_black(shade, i, j) == true) {
                black_j[shade] |= (uint8_t)(1U << j);
            }
        }
    }

    uint8_t *p_out = &s_img[(size_t)r * IMG_STRIDE];
    for (uint16_t b = 0U; b < IMG_STRIDE; b++) {
        uint8_t byte = 0U;
        for (uint16_t bit = 0U; bit < 8U; bit++) {
            const uint16_t c = (uint16_t)((b * 8U) + bit);
            const uint8_t shade = p_frame[((size_t)s_col_gy[c] * GB_EMU_WIDTH) + gx];
            if ((black_j[shade] & (1U << s_col_j[c])) != 0U) {
                byte |= (uint8_t)(0x80U >> bit);
            }
        }
        p_out[b] = byte;
    }
}
