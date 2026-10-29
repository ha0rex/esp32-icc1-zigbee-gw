#pragma once

/*
 * 1.54" ESP32-C3 reader (stock firmware: XiaoMoReader / V2026_154V2, 8MB).
 * Pins are the ones that firmware programs: SPI SCK/MOSI/CS, then DC, RST,
 * and BUSY (busy high). Panel size in that init is 200x200.
 *
 * The ebook sets GPIO0 and GPIO1 as inputs. Left is GPIO0, right is GPIO1.
 * Center is unused on this board; it stays pulled up so it never fires.
 */

#define EPD_SCK 10
#define EPD_MOSI 7
#define EPD_CS 12
#define EPD_DC 13
#define EPD_RST 20
#define EPD_BUSY 21

#define BTN_LEFT 0
#define BTN_RIGHT 1
#define BTN_CENTER 5

#define EPD_W 200
#define EPD_H 200

#define THERMO_MIN_C 10.0f
#define THERMO_MAX_C 38.0f
#define THERMO_STEP_C 0.5f
