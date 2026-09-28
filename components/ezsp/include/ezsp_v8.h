#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * EZSP protocol version 8 framing (EmberZNet 6.7.x / EZSP 8):
 *
 *   sequence (1) | frameControlLow (1) | frameControlHigh (1) |
 *   frameIdLow (1) | frameIdHigh (1) | parameters...
 *
 * frameControlHigh bit0 = frameFormatVersion (1 for v8 extended format).
 * Frame IDs are 16-bit little-endian.
 */

#define EZSP_V8_HEADER_LEN 5

esp_err_t ezsp_v8_build(uint8_t seq, uint16_t frame_id, const uint8_t *params, size_t params_len,
                        uint8_t *out, size_t out_max, size_t *out_len);

/**
 * Parse a received EZSP v8 frame.
 * @param frame_id_out  command/response id
 * @param params_out    points into `in` at parameter start (not a copy)
 */
esp_err_t ezsp_v8_parse(const uint8_t *in, size_t in_len, uint8_t *seq_out, uint16_t *frame_id_out,
                        const uint8_t **params_out, size_t *params_len_out);

#ifdef __cplusplus
}
#endif
