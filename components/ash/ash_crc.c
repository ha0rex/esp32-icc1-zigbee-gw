/**
 * @file ash_crc.c
 * @brief ASH CRC-CCITT (poly 0x1021, init 0xFFFF, big-endian on the wire).
 *
 * Verified against Silicon Labs UART Gateway Protocol Reference examples:
 *   RST     control 0xC0           -> CRC 0x38BC
 *   ACK(1)+ control 0x81           -> CRC 0x6059
 *   RSTACK  C1 02 02               -> CRC 0x9B7B
 */

#include "ash_crc.h"

uint16_t ash_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
