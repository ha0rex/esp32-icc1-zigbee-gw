#pragma once

#include <stdbool.h>

/** Latest onboard AHT30 or SHT30 reading. False until a chip answers. */
bool climate_read(float *temp_c, float *humidity);
