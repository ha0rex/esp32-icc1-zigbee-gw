#pragma once

/*
 * 1.54" ESP32-C3 reader (stock firmware: XiaoMoReader / V2026_154V2, 8MB).
 * Pins are the ones that firmware programs: SPI SCK/MOSI/CS, then DC, RST,
 * and BUSY (busy high). Panel size in that init is 200x200.
 *
 * The side keys share one resistor ladder on GPIO1 (ADC1 channel 1).
 * GPIO0 is the battery divider (ADC1 channel 0), not a button.
 */

#define EPD_SCK 10
#define EPD_MOSI 7
#define EPD_CS 12
#define EPD_DC 13
#define EPD_RST 20
#define EPD_BUSY 21

/* Kept so the ladder bands stay next to the panel pins. Classification is in buttons.c. */
#define BTN_ADC_GPIO 1

/* AHT30 (0x38) or SHT30 (0x44). Stock Wire.begin(sda, scl, 400 kHz). */
#define I2C_SDA 3
#define I2C_SCL 2

#define EPD_W 200
#define EPD_H 200

#define THERMO_MIN_C 10.0f
#define THERMO_MAX_C 38.0f
#define THERMO_STEP_C 0.5f
