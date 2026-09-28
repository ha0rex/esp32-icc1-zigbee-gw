#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Diagnostic counters for the ICC UART link. */
typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t rx_overflows;
} icc_uart_stats_t;

/**
 * Initialize UART1 (by default) for the ICC-1 NCP.
 * TX=GPIO6, RX=GPIO7, 115200 8N1, no flow control.
 */
esp_err_t icc_uart_init(void);

/** Write raw bytes to the ICC UART. */
esp_err_t icc_uart_write(const uint8_t *data, size_t len);

/**
 * Read up to max_len bytes from the UART RX ring.
 * @param timeout_ticks FreeRTOS ticks to wait for at least one byte.
 * @return number of bytes read, or negative on error.
 */
int icc_uart_read(uint8_t *buf, size_t max_len, TickType_t timeout_ticks);

/** Snapshot diagnostic counters. */
void icc_uart_get_stats(icc_uart_stats_t *out);

/** Total RX bytes since init (also used by ASH timeout diagnostics). */
uint32_t icc_uart_rx_byte_count(void);

#ifdef __cplusplus
}
#endif
