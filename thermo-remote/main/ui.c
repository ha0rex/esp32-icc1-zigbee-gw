#include "ui.h"

#include "epd.h"
#include "font_ui.h"
#include "pins.h"

#include <stdio.h>
#include <string.h>

static int face_width(const ui_face_t *face, const char *s)
{
    int w = 0;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        if (*p < face->first || *p >= face->first + face->count) {
            continue;
        }
        w += face->glyphs[*p - face->first].advance;
    }
    return w;
}

static int draw_face(const ui_face_t *face, int x, int y, const char *s)
{
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        if (*p < face->first || *p >= face->first + face->count) {
            continue;
        }
        const ui_glyph_t *g = &face->glyphs[*p - face->first];
        int stride = (g->w + 7) / 8;
        for (int row = 0; row < g->h; row++) {
            for (int col = 0; col < g->w; col++) {
                uint8_t bits = face->bits[g->off + row * stride + col / 8];
                if (bits & (uint8_t)(1 << (7 - (col & 7)))) {
                    epd_pixel(x + g->dx + col, y + g->dy + row, true);
                }
            }
        }
        x += g->advance;
    }
    return x;
}

static void centered(int y, const char *s)
{
    int x = (EPD_W - face_width(&FONT_SM, s)) / 2;
    if (x < 4) {
        x = 4;
    }
    draw_face(&FONT_SM, x, y, s);
}

static void hline(int x, int y, int w, int t)
{
    for (int i = 0; i < w; i++) {
        for (int k = 0; k < t; k++) {
            epd_pixel(x + i, y + k, true);
        }
    }
}

static void vline(int x, int y, int h, int t)
{
    for (int i = 0; i < h; i++) {
        for (int k = 0; k < t; k++) {
            epd_pixel(x + k, y + i, true);
        }
    }
}

static void ring(int cx, int cy, int r)
{
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= r * r && d2 >= (r - 1) * (r - 1)) {
                epd_pixel(cx + dx, cy + dy, true);
            }
        }
    }
}

/* Small ring and a normal C, sitting on the top of the big digits. */
static int draw_celsius(int x, int top)
{
    int r = 2;
    ring(x + r, top + r + 1, r);
    draw_face(&FONT_SM, x + r * 2 + 3, top, "C");
    return r * 2 + 3 + face_width(&FONT_SM, "C");
}

static void draw_drop(int x, int y, int h)
{
    int r = h / 3;
    if (r < 3) {
        r = 3;
    }
    int cx = x + r;
    int cy = y + h - r - 1;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r) {
                epd_pixel(cx + dx, cy + dy, true);
            }
        }
    }
    int base = cy - r / 3;
    int span = base - y;
    if (span < 1) {
        span = 1;
    }
    for (int row = y; row <= base; row++) {
        int half = (row - y) * r / span;
        for (int dx = -half; dx <= half; dx++) {
            epd_pixel(cx + dx, row, true);
        }
    }
}

static void stamp(int x, int y, const uint8_t *rows, int n, bool black)
{
    for (int row = 0; row < n; row++) {
        uint8_t bits = rows[row];
        for (int col = 0; col < 8; col++) {
            if (bits & (1 << (7 - col))) {
                epd_pixel(x + col, y + row, black);
            }
        }
    }
}

static void draw_bolt(int x, int y)
{
    static const uint8_t bolt[] = {
        0x18, 0x38, 0x7c, 0x3e, 0x1c, 0x3c, 0x78, 0x70, 0x60,
    };
    static const uint8_t halo[] = {
        0x3c, 0x7c, 0xfe, 0x7f, 0x3e, 0x7e, 0xfc, 0xf8, 0xf0, 0xe0,
    };
    stamp(x - 1, y - 1, halo, 10, false);
    stamp(x, y, bolt, 9, true);
}

static void draw_battery(int x, int y, int pct, bool charging)
{
    const int w = 28;
    const int h = 14;
    const int nub_w = 3;
    const int nub_h = 6;
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    hline(x, y, w, 1);
    hline(x, y + h - 1, w, 1);
    vline(x, y, h, 1);
    vline(x + w - 1, y, h, 1);
    int ny = y + (h - nub_h) / 2;
    hline(x + w, ny, nub_w, 1);
    hline(x + w, ny + nub_h - 1, nub_w, 1);
    vline(x + w + nub_w - 1, ny, nub_h, 1);
    int inner_w = w - 4;
    int fill = (pct * inner_w + 50) / 100;
    if (pct > 0 && fill < 2) {
        fill = 2;
    }
    for (int i = 0; i < fill; i++) {
        vline(x + 2 + i, y + 2, h - 4, 1);
    }
    if (charging) {
        draw_bolt(x + (w - 8) / 2, y + (h - 9) / 2);
    }
}

static void frame(void)
{
    epd_clear();
    for (int i = 0; i < EPD_W; i++) {
        epd_pixel(i, 0, true);
        epd_pixel(i, EPD_H - 1, true);
    }
    for (int i = 0; i < EPD_H; i++) {
        epd_pixel(0, i, true);
        epd_pixel(EPD_W - 1, i, true);
    }
}

void ui_message(const char *title, const char *line2)
{
    frame();
    centered(72, title ? title : "");
    if (line2 && line2[0]) {
        centered(100, line2);
    }
    epd_show();
}

void ui_setup(void)
{
    frame();
    centered(16, "WIFI SETUP");
    centered(42, "JOIN");
    centered(64, "THERMOREMOTE");
    centered(92, "PASS");
    centered(114, "THERMOSTAT");
    centered(142, "OPEN");
    centered(164, "192.168.4.1");
    epd_show();
}

void ui_thermo(const char *name, bool has_temp, float temp_c, bool has_hum, float humidity_pct,
               float target_c, int index, int count, const char *status, int battery_pct,
               bool charging)
{
    frame();
    char title[48];
    const char *nm = (name && name[0]) ? name : "Thermo";
    if (count > 1) {
        snprintf(title, sizeof(title), "%.18s %d/%d", nm, index + 1, count);
    } else {
        snprintf(title, sizeof(title), "%.24s", nm);
    }
    centered(8, title);

    char temp[8];
    if (!has_temp) {
        snprintf(temp, sizeof(temp), "--.-");
    } else {
        snprintf(temp, sizeof(temp), "%.1f", (double)temp_c);
    }
    int unit_w = 4 + 2 * 2 + 3 + face_width(&FONT_SM, "C");
    int num_w = face_width(&FONT_LG, temp);
    int tx = (EPD_W - num_w - unit_w) / 2;
    if (tx < 2) {
        tx = 2;
    }
    int temp_y = 32;
    int after = draw_face(&FONT_LG, tx, temp_y, temp);
    int digit_top = temp_y + FONT_LG.glyphs['0' - FONT_LG.first].dy;
    draw_celsius(after + 4, digit_top);

    char hum[8];
    if (has_hum) {
        snprintf(hum, sizeof(hum), "%.0f%%", (double)humidity_pct);
    } else {
        snprintf(hum, sizeof(hum), "--%%");
    }
    int drop_h = 16;
    int drop_w = (drop_h / 3) * 2 + 1;
    int gap = 6;
    int row_w = drop_w + gap + face_width(&FONT_MD, hum);
    int hx = (EPD_W - row_w) / 2;
    if (hx < 4) {
        hx = 4;
    }
    int hy = 112;
    draw_drop(hx, hy + 4, drop_h);
    draw_face(&FONT_MD, hx + drop_w + gap, hy, hum);

    char tgt[20];
    snprintf(tgt, sizeof(tgt), "Target %.1f", (double)target_c);
    centered(150, tgt);

    draw_battery(6, EPD_H - 8 - 14, battery_pct, charging);
    if (status && status[0]) {
        int sw = face_width(&FONT_SM, status);
        draw_face(&FONT_SM, EPD_W - 8 - sw, EPD_H - 8 - FONT_SM.height, status);
    }
    epd_show();
}
