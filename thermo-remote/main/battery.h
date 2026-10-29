#pragma once

#include "esp_adc/adc_oneshot.h"

#include <stdbool.h>

/** GPIO0 shares the button ADC unit. Call once after that unit exists. */
void battery_bind(adc_oneshot_unit_handle_t adc);

/**
 * Percent is the stock ebook curve (about 2200 counts empty, 2850 full).
 * Charging is true while a USB host is attached. The charger LED is not on a GPIO.
 */
void battery_read(int *percent, bool *charging);
