#pragma once

#include <stdbool.h>

void ui_message(const char *title, const char *line2);
void ui_setup(void);
void ui_thermo(const char *name, bool has_temp, float temp_c, bool has_hum, float humidity_pct,
               float target_c, int index, int count, const char *status, int battery_pct,
               bool charging);
