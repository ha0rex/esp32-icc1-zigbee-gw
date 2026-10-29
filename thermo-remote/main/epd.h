#pragma once

#include <stdbool.h>

void epd_init(void);
void epd_clear(void);
void epd_pixel(int x, int y, bool black);
void epd_show(void);
