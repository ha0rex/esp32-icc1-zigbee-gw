#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CRC-CCITT over unstuffed frame bytes (control + data), excluding CRC and FLAG. */
uint16_t ash_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
