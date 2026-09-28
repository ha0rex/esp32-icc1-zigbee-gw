#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum EZSP payload that fits in one ASH DATA frame (excl. control/CRC). */
#define ASH_MAX_DATA_LEN 128

/** ASH protocol constants (Silicon Labs UG101 / UART Gateway Protocol Reference). */
#define ASH_FLAG        0x7E
#define ASH_ESCAPE      0x7D
#define ASH_XON         0x11
#define ASH_XOFF        0x13
#define ASH_SUBSTITUTE  0x18
#define ASH_CANCEL      0x1A
#define ASH_WAKE        0xFF

#define ASH_CTRL_DATA   0x00
#define ASH_CTRL_ACK    0x80
#define ASH_CTRL_NAK    0xA0
#define ASH_CTRL_RST    0xC0
#define ASH_CTRL_RSTACK 0xC1
#define ASH_CTRL_ERROR  0xC2

typedef enum {
    ASH_STATE_DISCONNECTED = 0,
    ASH_STATE_CONNECTING,
    ASH_STATE_CONNECTED,
    ASH_STATE_FAILED,
} ash_state_t;

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t crc_errors;
    uint32_t timeouts;
    uint32_t retransmissions;
    uint32_t ack_sent;
    uint32_t nak_sent;
    uint32_t data_tx;
    uint32_t data_rx;
} ash_stats_t;

/**
 * Callback invoked when a complete, de-randomized EZSP payload arrives
 * inside an ASH DATA frame. Runs in the ASH RX task context — keep short
 * or queue to another task.
 */
typedef void (*ash_rx_cb_t)(const uint8_t *ezsp, size_t len, void *user);

/** Initialize ASH over icc_uart. Does not yet send RST. */
esp_err_t ash_init(ash_rx_cb_t rx_cb, void *user);

/**
 * Start the ASH RX task and attempt link reset (RST -> wait RSTACK).
 * Blocks up to timeout_ms waiting for RSTACK. Safe to call repeatedly.
 */
esp_err_t ash_connect(uint32_t timeout_ms);

/** True when RSTACK has been received and the link is usable. */
bool ash_is_connected(void);

ash_state_t ash_get_state(void);

/**
 * Send an EZSP frame as an ASH DATA frame (randomize, CRC, stuff, ACK wait).
 * Blocks until acknowledged or retries exhausted.
 */
esp_err_t ash_send_data(const uint8_t *ezsp, size_t len, uint32_t timeout_ms);

void ash_get_stats(ash_stats_t *out);
void ash_reset_stats(void);

/** Human-readable reset/error code from last RSTACK/ERROR. */
uint8_t ash_last_reset_code(void);

#ifdef __cplusplus
}
#endif
