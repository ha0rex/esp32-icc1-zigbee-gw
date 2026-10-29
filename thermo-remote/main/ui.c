#include "ui.h"

#include "epd.h"
#include "font5x7.h"
#include "pins.h"

#include <stdio.h>
#include <string.h>

static void text(int x, int y, const char *s, int scale)
{
    for (const char *p = s; *p; p++) {
        char c = *p;
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }
        int idx = (unsigned char)c - 32;
        if (idx < 0 || idx >= 59) {
            idx = 0;
        }
        for (int col = 0; col < 5; col++) {
            uint8_t bits = FONT5X7[idx][col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1 << row)) {
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            epd_pixel(x + col * scale + sx, y + row * scale + sy, true);
                        }
                    }
                }
            }
        }
        x += 6 * scale;
    }
}

static int text_width(const char *s, int scale)
{
    return (int)strlen(s) * 6 * scale;
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

static void seg_digit(int x, int y, char ch, int w, int h, int t)
{
    if (ch == ' ') {
        return;
    }
    int mid = h / 2;
    bool a = false, b = false, c = false, d = false, e = false, f = false, g = false;
    switch (ch) {
    case '0': a = b = c = d = e = f = true; break;
    case '1': b = c = true; break;
    case '2': a = b = d = e = g = true; break;
    case '3': a = b = c = d = g = true; break;
    case '4': b = c = f = g = true; break;
    case '5': a = c = d = f = g = true; break;
    case '6': a = c = d = e = f = g = true; break;
    case '7': a = b = c = true; break;
    case '8': a = b = c = d = e = f = g = true; break;
    case '9': a = b = c = d = f = g = true; break;
    case '-': g = true; break;
    default: break;
    }
    if (a) {
        hline(x + t, y, w - 2 * t, t);
    }
    if (g) {
        hline(x + t, y + mid - t / 2, w - 2 * t, t);
    }
    if (d) {
        hline(x + t, y + h - t, w - 2 * t, t);
    }
    if (f) {
        vline(x, y + t, mid - t, t);
    }
    if (b) {
        vline(x + w - t, y + t, mid - t, t);
    }
    if (e) {
        vline(x, y + mid + t / 2, h - mid - t, t);
    }
    if (c) {
        vline(x + w - t, y + mid + t / 2, h - mid - t, t);
    }
}

static void big_temp(int y, float value, bool known, int digit_h)
{
    char buf[8];
    if (!known) {
        snprintf(buf, sizeof(buf), "--.-");
    } else {
        snprintf(buf, sizeof(buf), "%4.1f", (double)value);
    }
    int w = digit_h / 2;
    int t = digit_h > 40 ? 4 : 3;
    int gap = 4;
    int total = 0;
    for (char *p = buf; *p; p++) {
        total += (*p == '.') ? t + 2 : w + gap;
    }
    int x = (EPD_W - total) / 2;
    for (char *p = buf; *p; p++) {
        if (*p == '.') {
            epd_pixel(x + 1, y + digit_h - 3, true);
            epd_pixel(x + 2, y + digit_h - 3, true);
            epd_pixel(x + 1, y + digit_h - 2, true);
            epd_pixel(x + 2, y + digit_h - 2, true);
            x += t + 2;
        } else {
            seg_digit(x, y, *p, w, digit_h, t);
            x += w + gap;
        }
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

static void centered(int y, const char *s, int scale)
{
    int x = (EPD_W - text_width(s, scale)) / 2;
    if (x < 4) {
        x = 4;
    }
    text(x, y, s, scale);
}

void ui_message(const char *title, const char *line2)
{
    frame();
    centered(40, title ? title : "", 2);
    if (line2 && line2[0]) {
        centered(80, line2, 2);
    }
    epd_show();
}

void ui_setup(void)
{
    frame();
    centered(16, "WIFI SETUP", 2);
    centered(48, "JOIN", 2);
    centered(70, "THERMOREMOTE", 1);
    centered(90, "PASS", 2);
    centered(112, "THERMOSTAT", 1);
    centered(140, "OPEN", 2);
    centered(162, "192.168.4.1", 1);
    epd_show();
}

void ui_thermo(const char *name, bool has_current, float current_c, float target_c, int index,
               int count, const char *status)
{
    frame();
    char title[40];
    const char *nm = (name && name[0]) ? name : "THERMO";
    if (count > 1) {
        snprintf(title, sizeof(title), "%.16s %d/%d", nm, index + 1, count);
    } else {
        snprintf(title, sizeof(title), "%.24s", nm);
    }
    centered(8, title, 1);
    big_temp(28, current_c, has_current, 56);
    centered(96, "NOW", 1);

    char tgt[16];
    snprintf(tgt, sizeof(tgt), "TARGET %.1f", (double)target_c);
    centered(130, tgt, 2);
    centered(160, "LEFT-  RIGHT+", 1);
    if (count > 1) {
        centered(174, "CENTER NEXT", 1);
    }
    centered(EPD_H - 16, status ? status : "", 1);
    epd_show();
}
