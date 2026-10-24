/**
 * @file touch_pad.c
 * @brief touch_pad implementation.
 *
 * Hit areas are larger than the drawn controls, as a thumb on glass needs, and the D-pad reads
 * diagonals: a touch counts in a direction when it is within about 68 degrees of it, so between
 * two arms both are held. Several fingers at once hold several buttons (direction + A, A + B).
 *
 * @version 0.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 */

/* Includes --------------------------------------------------------------------------------------*/
#include "touch_pad.h"

#include "display_fb.h"
#include "gb_emu.h"
#include "screen.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* Private definitions ---------------------------------------------------------------------------*/

/* Title strip */
#define TITLE_Y          (446U)
#define TITLE_SCALE      (2U)
#define SEPARATOR_Y      (434U)
#define SEPARATOR_H      (3U)

/* D-pad */
#define DPAD_CX          (120)
#define DPAD_CY          (610)
#define DPAD_ARM_HALF    (96)  /* centre to arm tip */
#define DPAD_ARM_THICK   (64)
#define DPAD_DEADZONE    (18)
#define DPAD_HIT_X_MAX   (250)
#define DPAD_HIT_Y_MIN   (470)
#define DPAD_HIT_Y_MAX   (730)

/* A and B */
#define BTN_A_CX         (410)
#define BTN_A_CY         (580)
#define BTN_B_CX         (300)
#define BTN_B_CY         (640)
#define BTN_R            (46)
#define BTN_HIT_R        (85)
#define BTN_LABEL_SCALE  (4U)

/* SELECT and START */
#define PILL_Y           (760)
#define PILL_SELECT_CX   (170)
#define PILL_START_CX    (310)
#define PILL_W           (116)
#define PILL_H           (38)
#define PILL_HIT_Y_MIN   (735)
#define PILL_HIT_X_MIN   (90)
#define PILL_HIT_X_MAX   (390)
#define PILL_LABEL_SCALE (2U)

/* Font: 5x7, one byte per column, bit 0 at the top */
#define FONT_W           (5U)
#define FONT_H           (7U)
#define FONT_ADVANCE     (6U)

/* Private variables -----------------------------------------------------------------------------*/

static const char s_font_chars[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.:/!_()";
static const uint8_t s_font[][FONT_W] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 9 */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* - */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* . */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* : */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* / */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* ! */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* _ */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* ( */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* ) */
};

/* Private function declarations -----------------------------------------------------------------*/

static void fill_rect_s(int x, int y, int w, int h, bool black);
static void fill_rounded_rect_s(int x, int y, int w, int h, int radius, bool black);
static void fill_circle_s(int cx, int cy, int r, bool black);
static void text_s(int x, int y, const char *p_text, uint32_t scale, bool black);
static void text_centered_s(int cx, int y, const char *p_text, uint32_t scale, bool black);
static uint32_t text_width(const char *p_text, uint32_t scale);
static const uint8_t *glyph(char c);
static uint8_t dpad_buttons(int sx, int sy);

/* Public function definitions -------------------------------------------------------------------*/

/**
 * @brief Clear the framebuffer and draw the static screen.
 */
void touch_pad_draw(const char *p_title)
{
    fill_rect_s(0, 0, SCREEN_W, SCREEN_H, false);
    fill_rect_s(0, SEPARATOR_Y, SCREEN_W, SEPARATOR_H, true);
    if (GAME_SCALE < 3U) {
        /* A frame around the smaller picture, like the Game Boy's screen bezel */
        const int fx = (int)GAME_X - 8;
        const int fy = (int)GAME_Y - 8;
        const int fw = (int)GAME_W + 16;
        const int fh = (int)GAME_H + 16;
        fill_rect_s(fx, fy, fw, 3, true);
        fill_rect_s(fx, fy + fh - 3, fw, 3, true);
        fill_rect_s(fx, fy, 3, fh, true);
        fill_rect_s(fx + fw - 3, fy, 3, fh, true);
    }
    text_centered_s(SCREEN_W / 2, TITLE_Y, p_title, TITLE_SCALE, true);

    /* D-pad: a cross with a dimple in the middle */
    fill_rect_s(DPAD_CX - DPAD_ARM_HALF, DPAD_CY - (DPAD_ARM_THICK / 2), 2 * DPAD_ARM_HALF,
                DPAD_ARM_THICK, true);
    fill_rect_s(DPAD_CX - (DPAD_ARM_THICK / 2), DPAD_CY - DPAD_ARM_HALF, DPAD_ARM_THICK,
                2 * DPAD_ARM_HALF, true);
    fill_circle_s(DPAD_CX, DPAD_CY, 12, false);

    fill_circle_s(BTN_B_CX, BTN_B_CY, BTN_R, true);
    fill_circle_s(BTN_A_CX, BTN_A_CY, BTN_R, true);
    const int label_h = (int)(FONT_H * BTN_LABEL_SCALE);
    text_centered_s(BTN_B_CX, BTN_B_CY - (label_h / 2), "B", BTN_LABEL_SCALE, false);
    text_centered_s(BTN_A_CX, BTN_A_CY - (label_h / 2), "A", BTN_LABEL_SCALE, false);

    const int pill_label_y = PILL_Y - (int)((FONT_H * PILL_LABEL_SCALE) / 2U);
    fill_rounded_rect_s(PILL_SELECT_CX - (PILL_W / 2), PILL_Y - (PILL_H / 2), PILL_W, PILL_H,
                        PILL_H / 2, true);
    fill_rounded_rect_s(PILL_START_CX - (PILL_W / 2), PILL_Y - (PILL_H / 2), PILL_W, PILL_H,
                        PILL_H / 2, true);
    text_centered_s(PILL_SELECT_CX, pill_label_y, "SELECT", PILL_LABEL_SCALE, false);
    text_centered_s(PILL_START_CX, pill_label_y, "START", PILL_LABEL_SCALE, false);
}

/**
 * @brief Clear the framebuffer and draw a message screen.
 */
void touch_pad_draw_message(const char *const *pp_lines, uint32_t count)
{
    const uint32_t scale = 2U;
    const int line_h = (int)(FONT_H * scale) + 14;
    int y = (SCREEN_H - ((int)count * line_h)) / 2;

    fill_rect_s(0, 0, SCREEN_W, SCREEN_H, false);
    for (uint32_t i = 0U; i < count; i++) {
        text_centered_s(SCREEN_W / 2, y, pp_lines[i], (i == 0U) ? 3U : scale, true);
        y += line_h + ((i == 0U) ? 16 : 0);
    }
}

/**
 * @brief Which buttons a set of touch points holds.
 *
 * @return uint8_t
 */
uint8_t touch_pad_buttons(const hal_touch_point_t *p_points, int count)
{
    uint8_t buttons = 0U;

    for (int n = 0; n < count; n++) {
        uint16_t usx = 0U;
        uint16_t usy = 0U;
        land_to_screen(p_points[n].x, p_points[n].y, &usx, &usy);
        const int sx = usx;
        const int sy = usy;

        if ((sx < DPAD_HIT_X_MAX) && (sy >= DPAD_HIT_Y_MIN) && (sy < DPAD_HIT_Y_MAX)) {
            buttons |= dpad_buttons(sx, sy);
        } else if ((sx >= DPAD_HIT_X_MAX) && (sy >= DPAD_HIT_Y_MIN) && (sy < PILL_HIT_Y_MIN)) {
            const int da = ((sx - BTN_A_CX) * (sx - BTN_A_CX)) + ((sy - BTN_A_CY) * (sy - BTN_A_CY));
            const int db = ((sx - BTN_B_CX) * (sx - BTN_B_CX)) + ((sy - BTN_B_CY) * (sy - BTN_B_CY));
            const int hit = BTN_HIT_R * BTN_HIT_R;
            if ((da <= db) && (da < hit)) {
                buttons |= GB_BTN_A;
            } else if ((db < da) && (db < hit)) {
                buttons |= GB_BTN_B;
            }
        } else if ((sy >= PILL_HIT_Y_MIN) && (sx >= PILL_HIT_X_MIN) && (sx < PILL_HIT_X_MAX)) {
            buttons |= (sx < ((PILL_SELECT_CX + PILL_START_CX) / 2)) ? GB_BTN_SELECT : GB_BTN_START;
        }
    }
    return buttons;
}

/* Private function definitions ------------------------------------------------------------------*/

/**
 * @brief The directions a touch on the D-pad holds; two at once between two arms.
 */
static uint8_t dpad_buttons(int sx, int sy)
{
    const int dx = sx - DPAD_CX;
    const int dy = sy - DPAD_CY;
    if (((dx * dx) + (dy * dy)) < (DPAD_DEADZONE * DPAD_DEADZONE)) {
        return 0U;
    }

    uint8_t buttons = 0U;
    const int ax = abs(dx);
    const int ay = abs(dy);
    /* Within ~68 degrees of an axis: |other| / |this| < 2.5 */
    if ((ax * 5) > (ay * 2)) {
        buttons |= (dx > 0) ? GB_BTN_RIGHT : GB_BTN_LEFT;
    }
    if ((ay * 5) > (ax * 2)) {
        buttons |= (dy > 0) ? GB_BTN_DOWN : GB_BTN_UP;
    }
    return buttons;
}

/**
 * @brief Fill a screen rectangle, clipped to the screen.
 */
static void fill_rect_s(int x, int y, int w, int h, bool black)
{
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if ((x + w) > SCREEN_W) {
        w = SCREEN_W - x;
    }
    if ((y + h) > SCREEN_H) {
        h = SCREEN_H - y;
    }
    if ((w <= 0) || (h <= 0)) {
        return;
    }
    const epd_rect_t s = {.x = (uint16_t)x, .y = (uint16_t)y, .w = (uint16_t)w, .h = (uint16_t)h};
    const epd_rect_t land = screen_to_land_rect(&s);
    display_fill_rect(&land, black);
}

/**
 * @brief Fill a screen rectangle with rounded corners. Must lie inside the screen.
 */
static void fill_rounded_rect_s(int x, int y, int w, int h, int radius, bool black)
{
    const epd_rect_t s = {.x = (uint16_t)x, .y = (uint16_t)y, .w = (uint16_t)w, .h = (uint16_t)h};
    const epd_rect_t land = screen_to_land_rect(&s);
    display_fill_rounded_rect(&land, (uint16_t)radius, black);
}

/**
 * @brief Fill a circle given in screen coordinates. Must lie inside the screen.
 */
static void fill_circle_s(int cx, int cy, int r, bool black)
{
    uint16_t lx = 0U;
    uint16_t ly = 0U;
    screen_to_land((uint16_t)cx, (uint16_t)cy, &lx, &ly);
    display_fill_circle(lx, ly, (uint16_t)r, black);
}

/**
 * @brief Draw text in screen coordinates, each font pixel a scale x scale block.
 */
static void text_s(int x, int y, const char *p_text, uint32_t scale, bool black)
{
    for (const char *p = p_text; *p != '\0'; p++) {
        const uint8_t *p_glyph = glyph(*p);
        for (uint32_t col = 0U; col < FONT_W; col++) {
            for (uint32_t row = 0U; row < FONT_H; row++) {
                if ((p_glyph[col] & (1U << row)) != 0U) {
                    fill_rect_s(x + (int)(col * scale), y + (int)(row * scale), (int)scale,
                                (int)scale, black);
                }
            }
        }
        x += (int)(FONT_ADVANCE * scale);
    }
}

/**
 * @brief Draw text centred on cx.
 */
static void text_centered_s(int cx, int y, const char *p_text, uint32_t scale, bool black)
{
    text_s(cx - (int)(text_width(p_text, scale) / 2U), y, p_text, scale, black);
}

/**
 * @brief Width of text in pixels, without the trailing gap.
 */
static uint32_t text_width(const char *p_text, uint32_t scale)
{
    const uint32_t len = (uint32_t)strlen(p_text);
    return (len == 0U) ? 0U : (((len * FONT_ADVANCE) - 1U) * scale);
}

/**
 * @brief The glyph of a character; lower case is shown as upper case, unknown as a space.
 */
static const uint8_t *glyph(char c)
{
    if ((c >= 'a') && (c <= 'z')) {
        c = (char)(c - 'a' + 'A');
    }
    const char *p_hit = (c != '\0') ? strchr(s_font_chars, c) : NULL;
    return s_font[(p_hit != NULL) ? (p_hit - s_font_chars) : 0];
}
