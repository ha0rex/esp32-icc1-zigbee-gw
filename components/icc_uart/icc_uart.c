/**
 * @file icc_uart.c
 * @brief Async UART transport for the IKEA ICC-1 EmberZNet NCP.
 *
 * Wiring (ESP32-C3 Super Mini):
 *   GPIO6 TX -> ICC pad 2 / PB15 RX
 *   GPIO7 RX <- ICC pad 3 / PB14 TX
 *   115200 8N1, no RTS/CTS, no SW flow control.
 */

#include "icc_uart.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "icc_uart";

#define ICC_UART_NUM ((uart_port_t)CONFIG_ICC_UART_PORT_NUM)

static icc_uart_stats_t s_stats;
static bool s_ready;

esp_err_t icc_uart_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    memset(&s_stats, 0, sizeof(s_stats));

    const uart_config_t cfg = {
        .baud_rate = CONFIG_ICC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Init UART%d TX=GPIO%d RX=GPIO%d @ %d 8N1 (no flow control)",
             (int)ICC_UART_NUM, CONFIG_ICC_UART_TX_GPIO, CONFIG_ICC_UART_RX_GPIO,
             CONFIG_ICC_UART_BAUD);

    esp_err_t err = uart_driver_install(ICC_UART_NUM, CONFIG_ICC_UART_RX_BUF_SIZE,
                                        CONFIG_ICC_UART_TX_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(ICC_UART_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(ICC_UART_NUM, CONFIG_ICC_UART_TX_GPIO, CONFIG_ICC_UART_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Disable software XON/XOFF — ASH handles reserved bytes itself. */
    uart_set_sw_flow_ctrl(ICC_UART_NUM, false, 0, 0);

    s_ready = true;
    ESP_LOGI(TAG, "UART ready");
    return ESP_OK;
}

esp_err_t icc_uart_write(const uint8_t *data, size_t len)
{
    if (!s_ready || data == NULL || len == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int written = uart_write_bytes(ICC_UART_NUM, data, len);
    if (written < 0) {
        return ESP_FAIL;
    }
    s_stats.tx_bytes += (uint32_t)written;
    return (size_t)written == len ? ESP_OK : ESP_ERR_TIMEOUT;
}

int icc_uart_read(uint8_t *buf, size_t max_len, TickType_t timeout_ticks)
{
    if (!s_ready || buf == NULL || max_len == 0) {
        return -1;
    }

    int n = uart_read_bytes(ICC_UART_NUM, buf, max_len, timeout_ticks);
    if (n > 0) {
        s_stats.rx_bytes += (uint32_t)n;
    }
    return n;
}

void icc_uart_get_stats(icc_uart_stats_t *out)
{
    if (out) {
        *out = s_stats;
    }
}

uint32_t icc_uart_rx_byte_count(void)
{
    return s_stats.rx_bytes;
}
