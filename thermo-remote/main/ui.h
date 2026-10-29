#pragma once

#include <stdbool.h>

void ui_message(const char *title, const char *line2);
void ui_setup(void);
void ui_thermo(const char *name, bool has_current, float current_c, float target_c, int index,
               int count, const char *status);
