/**
 * @file ezsp_v8.c
 * @brief EZSP protocol version 8 frame encode/decode.
 */

#include "ezsp_v8.h"

#include <string.h>

esp_err_t ezsp_v8_build(uint8_t seq, uint16_t frame_id, const uint8_t *params, size_t params_len,
                        uint8_t *out, size_t out_max, size_t *out_len)
{
    size_t need = EZSP_V8_HEADER_LEN + params_len;
    if (out == NULL || out_len == NULL || need > out_max) {
        return ESP_ERR_INVALID_SIZE;
    }

    out[0] = seq;
    out[1] = 0x00; /* frameControlLow: command, networkIndex 0 */
    out[2] = 0x01; /* frameControlHigh: frameFormatVersion = 1 */
    out[3] = (uint8_t)(frame_id & 0xFF);
    out[4] = (uint8_t)((frame_id >> 8) & 0xFF);

    if (params_len && params) {
        memcpy(out + EZSP_V8_HEADER_LEN, params, params_len);
    }

    *out_len = need;
    return ESP_OK;
}

esp_err_t ezsp_v8_parse(const uint8_t *in, size_t in_len, uint8_t *seq_out, uint16_t *frame_id_out,
                        const uint8_t **params_out, size_t *params_len_out)
{
    if (in == NULL || in_len < EZSP_V8_HEADER_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (seq_out) {
        *seq_out = in[0];
    }
    if (frame_id_out) {
        *frame_id_out = (uint16_t)in[3] | ((uint16_t)in[4] << 8);
    }
    if (params_out) {
        *params_out = in + EZSP_V8_HEADER_LEN;
    }
    if (params_len_out) {
        *params_len_out = in_len - EZSP_V8_HEADER_LEN;
    }
    return ESP_OK;
}
