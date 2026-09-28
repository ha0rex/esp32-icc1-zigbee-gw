/**
 * @file ash.c
 * @brief Ember ASH (Asynchronous Serial Host) protocol for EZSP NCPs.
 *
 * Implements the Silicon Labs UART Gateway Protocol (ASH) used by EmberZNet
 * NCP firmware such as NCP_USW_115k2_F256_*. Behaviour follows the public
 * UART Gateway Protocol Reference (frames, stuffing, randomization, ACK).
 *
 * Link bring-up:
 *   Host sends CANCEL* + RST (C0 38 BC 7E)
 *   NCP replies with RSTACK (C1 02 <code> <crc> 7E)
 *   Host enters CONNECTED; frmNum/ackNum start at 0
 *
 * DATA frames carry EZSP payloads XOR'd with an 8-bit LFSR starting at 0x42.
 */

#include "ash.h"
#include "ash_crc.h"
#include "icc_uart.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "ash";

#define ASH_MAX_RETRIES         3
#define ASH_RX_FRAME_MAX        160
#define ASH_STUFFED_MAX         320
#define ASH_EVT_RSTACK          BIT0
#define ASH_EVT_ACK             BIT1
#define ASH_EVT_ERROR           BIT2

typedef struct {
    uint8_t ctrl;
    uint8_t data[ASH_MAX_DATA_LEN];
    size_t data_len;
    bool retx;
} ash_pending_t;

static ash_state_t s_state = ASH_STATE_DISCONNECTED;
static ash_stats_t s_stats;
static ash_rx_cb_t s_rx_cb;
static void *s_rx_user;
static uint8_t s_last_reset_code;

static uint8_t s_frm_num;     /* next DATA frmNum to send */
static uint8_t s_ack_num;     /* next DATA frmNum we expect to receive */
static uint8_t s_ackd_num;    /* highest frmNum+1 acknowledged by peer */

static ash_pending_t s_pending;
static bool s_pending_valid;
static uint8_t s_pending_frm;

static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_uart_mutex; /* serializes all UART writes (DATA/ACK/NAK/RST) */
static EventGroupHandle_t s_events;
static TaskHandle_t s_rx_task;

static uint8_t s_rx_accum[ASH_RX_FRAME_MAX];
static size_t s_rx_accum_len;
static bool s_rx_escape;
static bool s_rx_discard; /* after SUBSTITUTE until next FLAG */

#if CONFIG_ASH_LOG_HEX_FRAMES
static void log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    /* Cap log line length for readability. */
    char line[3 * 64 + 8];
    size_t n = len > 64 ? 64 : len;
    size_t pos = 0;
    for (size_t i = 0; i < n && pos + 4 < sizeof(line); i++) {
        pos += (size_t)snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
    }
    if (len > 64 && pos + 4 < sizeof(line)) {
        snprintf(line + pos, sizeof(line) - pos, "...");
    }
    /* Hex dumps are gated by Kconfig; use INFO so bring-up is visible at default log level. */
    ESP_LOGI(TAG, "%s%s", prefix, line);
}
#else
static void log_hex(const char *prefix, const uint8_t *data, size_t len)
{
    (void)prefix;
    (void)data;
    (void)len;
}
#endif

/* ---- randomization (DATA payload only) ---------------------------------- */

static void ash_randomize(uint8_t *buf, size_t len)
{
    uint8_t r = 0x42;
    for (size_t i = 0; i < len; i++) {
        buf[i] ^= r;
        if (r & 0x01U) {
            r = (uint8_t)((r >> 1) ^ 0xB8U);
        } else {
            r = (uint8_t)(r >> 1);
        }
    }
}

/* ---- byte stuffing ------------------------------------------------------ */

static size_t ash_stuff(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_max)
{
    size_t o = 0;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        bool reserved = (b == ASH_FLAG || b == ASH_ESCAPE || b == ASH_XON || b == ASH_XOFF ||
                         b == ASH_SUBSTITUTE || b == ASH_CANCEL);
        if (reserved) {
            if (o + 2 > out_max) {
                return 0;
            }
            out[o++] = ASH_ESCAPE;
            out[o++] = (uint8_t)(b ^ 0x20U);
        } else {
            if (o + 1 > out_max) {
                return 0;
            }
            out[o++] = b;
        }
    }
    return o;
}

static esp_err_t ash_tx_raw(const uint8_t *frame, size_t len)
{
    log_hex("TX ASH: ", frame, len);
    if (s_uart_mutex) {
        xSemaphoreTake(s_uart_mutex, portMAX_DELAY);
    }
    esp_err_t err = icc_uart_write(frame, len);
    if (s_uart_mutex) {
        xSemaphoreGive(s_uart_mutex);
    }
    if (err == ESP_OK) {
        s_stats.tx_bytes += (uint32_t)len;
    }
    return err;
}

/**
 * Build and transmit an ASH frame from unstuffed control[+data].
 * prepend_cancel: RST frames are preceded by CANCEL bytes per the spec.
 */
static esp_err_t ash_send_frame(const uint8_t *unstuffed, size_t len, bool prepend_cancel)
{
    uint8_t with_crc[ASH_RX_FRAME_MAX];
    if (len + 2 > sizeof(with_crc)) {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(with_crc, unstuffed, len);
    uint16_t crc = ash_crc16(unstuffed, len);
    with_crc[len] = (uint8_t)(crc >> 8);
    with_crc[len + 1] = (uint8_t)(crc & 0xFF);

    uint8_t stuffed[ASH_STUFFED_MAX];
    size_t o = 0;
    if (prepend_cancel) {
        /* Cancel any partial RX at the NCP; common practice is several CANCELs. */
        for (int i = 0; i < 4 && o < sizeof(stuffed); i++) {
            stuffed[o++] = ASH_CANCEL;
        }
    }
    size_t stuffed_len = ash_stuff(with_crc, len + 2, stuffed + o, sizeof(stuffed) - o - 1);
    if (stuffed_len == 0) {
        return ESP_ERR_NO_MEM;
    }
    o += stuffed_len;
    stuffed[o++] = ASH_FLAG;

    return ash_tx_raw(stuffed, o);
}

static esp_err_t ash_send_rst(void)
{
    const uint8_t rst[] = {ASH_CTRL_RST};
    ESP_LOGI(TAG, "ASH reset sent");
    return ash_send_frame(rst, sizeof(rst), true);
}

static esp_err_t ash_send_ack(uint8_t ack_num, bool not_ready)
{
    uint8_t ctrl = (uint8_t)(ASH_CTRL_ACK | (ack_num & 0x07U));
    if (not_ready) {
        ctrl |= 0x08U;
    }
    s_stats.ack_sent++;
    return ash_send_frame(&ctrl, 1, false);
}

static esp_err_t ash_send_nak(uint8_t ack_num)
{
    uint8_t ctrl = (uint8_t)(ASH_CTRL_NAK | (ack_num & 0x07U));
    s_stats.nak_sent++;
    ESP_LOGW(TAG, "TX NAK ackNum=%u", (unsigned)(ack_num & 7));
    return ash_send_frame(&ctrl, 1, false);
}

static esp_err_t ash_send_data_frame(uint8_t frm, uint8_t ack, bool retx, const uint8_t *payload,
                                     size_t payload_len)
{
    if (payload_len > ASH_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[1 + ASH_MAX_DATA_LEN];
    frame[0] = (uint8_t)(((frm & 0x07U) << 4) | ((retx ? 1U : 0U) << 3) | (ack & 0x07U));
    memcpy(frame + 1, payload, payload_len);
    ash_randomize(frame + 1, payload_len);

    s_stats.data_tx++;
    ESP_LOGD(TAG, "TX DATA frmNum=%u ackNum=%u reTx=%d len=%u", (unsigned)frm, (unsigned)ack,
             (int)retx, (unsigned)payload_len);
    return ash_send_frame(frame, 1 + payload_len, false);
}

/* ---- receive / parse ---------------------------------------------------- */

static void ash_handle_ack_num(uint8_t ack_num)
{
    if (!s_pending_valid) {
        return;
    }
    /* ackNum acknowledges frames up to but not including ackNum */
    uint8_t next = (uint8_t)((s_pending_frm + 1) & 7);
    if (ack_num == next) {
        s_pending_valid = false;
        s_ackd_num = ack_num;
        xEventGroupSetBits(s_events, ASH_EVT_ACK);
    }
}

static void ash_process_frame(const uint8_t *frame, size_t len)
{
    if (len < 3) {
        s_stats.invalid_frames++;
        return;
    }

    uint16_t rx_crc = ((uint16_t)frame[len - 2] << 8) | frame[len - 1];
    uint16_t calc = ash_crc16(frame, len - 2);
    if (rx_crc != calc) {
        s_stats.crc_errors++;
        s_stats.invalid_frames++;
        ESP_LOGW(TAG, "CRC error: got %04X calc %04X (len=%u)", rx_crc, calc, (unsigned)len);
        if (s_state == ASH_STATE_CONNECTED) {
            ash_send_nak(s_ack_num);
        }
        return;
    }

    uint8_t ctrl = frame[0];
    const uint8_t *data = frame + 1;
    size_t data_len = len - 3;

    s_stats.valid_frames++;
    log_hex("RX ASH: ", frame, len);

    if (ctrl == ASH_CTRL_RSTACK) {
        if (data_len >= 2) {
            s_last_reset_code = data[1];
            ESP_LOGI(TAG, "ASH: RSTACK received (version=0x%02X reset_code=0x%02X)", data[0],
                     data[1]);
        } else {
            ESP_LOGI(TAG, "ASH: RSTACK received");
        }
        s_frm_num = 0;
        s_ack_num = 0;
        s_ackd_num = 0;
        s_pending_valid = false;
        s_state = ASH_STATE_CONNECTED;
        xEventGroupSetBits(s_events, ASH_EVT_RSTACK);
        return;
    }

    if (ctrl == ASH_CTRL_ERROR) {
        if (data_len >= 2) {
            ESP_LOGE(TAG, "ASH ERROR frame code=0x%02X", data[1]);
            s_last_reset_code = data[1];
        }
        s_state = ASH_STATE_FAILED;
        xEventGroupSetBits(s_events, ASH_EVT_ERROR);
        return;
    }

    if ((ctrl & 0x80U) == 0) {
        /* DATA: bits 6:4 frmNum, bit3 reTx, bits 2:0 ackNum */
        uint8_t frm = (ctrl >> 4) & 0x07U;
        uint8_t ack = ctrl & 0x07U;
        bool retx = (ctrl & 0x08U) != 0;
        (void)retx;

        ash_handle_ack_num(ack);

        if (s_state != ASH_STATE_CONNECTED) {
            return;
        }

        if (frm != s_ack_num) {
            ESP_LOGW(TAG, "Unexpected DATA frmNum=%u expect=%u — NAK", (unsigned)frm,
                     (unsigned)s_ack_num);
            ash_send_nak(s_ack_num);
            return;
        }

        uint8_t payload[ASH_MAX_DATA_LEN];
        if (data_len > ASH_MAX_DATA_LEN) {
            ash_send_nak(s_ack_num);
            return;
        }
        memcpy(payload, data, data_len);
        ash_randomize(payload, data_len); /* XOR again restores */

        s_ack_num = (uint8_t)((s_ack_num + 1) & 7);
        ash_send_ack(s_ack_num, false);
        s_stats.data_rx++;

        ESP_LOGD(TAG, "RX DATA frmNum=%u len=%u", (unsigned)frm, (unsigned)data_len);
        if (s_rx_cb && data_len > 0) {
            s_rx_cb(payload, data_len, s_rx_user);
        }
        return;
    }

    if ((ctrl & 0xE0U) == ASH_CTRL_ACK) {
        uint8_t ack = ctrl & 0x07U;
        ESP_LOGD(TAG, "RX ACK ackNum=%u", (unsigned)ack);
        ash_handle_ack_num(ack);
        return;
    }

    if ((ctrl & 0xE0U) == ASH_CTRL_NAK) {
        uint8_t ack = ctrl & 0x07U;
        ESP_LOGW(TAG, "RX NAK ackNum=%u — will retransmit", (unsigned)ack);
        ash_handle_ack_num(ack);
        /* Force retransmission of pending frame if any */
        if (s_pending_valid) {
            ash_send_data_frame(s_pending_frm, s_ack_num, true, s_pending.data, s_pending.data_len);
            s_stats.retransmissions++;
        }
        return;
    }

    s_stats.invalid_frames++;
    ESP_LOGW(TAG, "Unknown ASH control 0x%02X", ctrl);
}

static void ash_rx_byte(uint8_t b)
{
    s_stats.rx_bytes++;

    if (b == ASH_XON || b == ASH_XOFF) {
        return; /* ignore software flow-control bytes */
    }

    if (b == ASH_CANCEL) {
        s_rx_accum_len = 0;
        s_rx_escape = false;
        s_rx_discard = false;
        return;
    }

    if (b == ASH_SUBSTITUTE) {
        s_rx_accum_len = 0;
        s_rx_escape = false;
        s_rx_discard = true;
        return;
    }

    if (b == ASH_FLAG) {
        if (!s_rx_discard && s_rx_accum_len > 0) {
            ash_process_frame(s_rx_accum, s_rx_accum_len);
        }
        s_rx_accum_len = 0;
        s_rx_escape = false;
        s_rx_discard = false;
        return;
    }

    if (s_rx_discard) {
        return;
    }

    if (s_rx_escape) {
        b ^= 0x20U;
        s_rx_escape = false;
    } else if (b == ASH_ESCAPE) {
        s_rx_escape = true;
        return;
    }

    if (s_rx_accum_len < sizeof(s_rx_accum)) {
        s_rx_accum[s_rx_accum_len++] = b;
    } else {
        s_stats.invalid_frames++;
        s_rx_accum_len = 0;
        s_rx_discard = true;
    }
}

static void ash_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    ESP_LOGI(TAG, "ASH RX task started");

    for (;;) {
        int n = icc_uart_read(buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                ash_rx_byte(buf[i]);
            }
        }
    }
}

/* ---- public API --------------------------------------------------------- */

esp_err_t ash_init(ash_rx_cb_t rx_cb, void *user)
{
    s_rx_cb = rx_cb;
    s_rx_user = user;
    s_state = ASH_STATE_DISCONNECTED;
    memset(&s_stats, 0, sizeof(s_stats));
    s_frm_num = 0;
    s_ack_num = 0;
    s_pending_valid = false;

    if (!s_tx_mutex) {
        s_tx_mutex = xSemaphoreCreateMutex();
    }
    if (!s_uart_mutex) {
        s_uart_mutex = xSemaphoreCreateMutex();
    }
    if (!s_events) {
        s_events = xEventGroupCreate();
    }
    if (!s_tx_mutex || !s_uart_mutex || !s_events) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = icc_uart_init();
    if (err != ESP_OK) {
        return err;
    }

    if (!s_rx_task) {
        BaseType_t ok = xTaskCreate(ash_rx_task, "ash_rx", 4096, NULL, 6, &s_rx_task);
        if (ok != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

esp_err_t ash_connect(uint32_t timeout_ms)
{
    if (!s_tx_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    s_state = ASH_STATE_CONNECTING;
    xEventGroupClearBits(s_events, ASH_EVT_RSTACK | ASH_EVT_ACK | ASH_EVT_ERROR);

    uint32_t rx_before = icc_uart_rx_byte_count();
    ESP_LOGI(TAG, "ASH: connecting...");
    ESP_LOGI(TAG, "waiting for RSTACK...");
    esp_err_t err = ash_send_rst();
    if (err != ESP_OK) {
        s_state = ASH_STATE_DISCONNECTED;
        xSemaphoreGive(s_tx_mutex);
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_events, ASH_EVT_RSTACK | ASH_EVT_ERROR, pdTRUE,
                                           pdFALSE, pdMS_TO_TICKS(timeout_ms));
    xSemaphoreGive(s_tx_mutex);

    if (bits & ASH_EVT_RSTACK) {
        ESP_LOGI(TAG, "ASH: connected");
        return ESP_OK;
    }

    s_stats.timeouts++;
    uint32_t rx_after = icc_uart_rx_byte_count();
    ESP_LOGW(TAG, "timeout");
    ESP_LOGW(TAG, "RX bytes received: %lu (delta this attempt: %lu)",
             (unsigned long)rx_after, (unsigned long)(rx_after - rx_before));
    s_state = ASH_STATE_DISCONNECTED;
    return ESP_ERR_TIMEOUT;
}

bool ash_is_connected(void)
{
    return s_state == ASH_STATE_CONNECTED;
}

ash_state_t ash_get_state(void)
{
    return s_state;
}

esp_err_t ash_send_data(const uint8_t *ezsp, size_t len, uint32_t timeout_ms)
{
    if (!ash_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ezsp == NULL || len == 0 || len > ASH_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);

    s_pending_frm = s_frm_num;
    memcpy(s_pending.data, ezsp, len);
    s_pending.data_len = len;
    s_pending_valid = true;
    xEventGroupClearBits(s_events, ASH_EVT_ACK);

    esp_err_t result = ESP_FAIL;
    for (int attempt = 0; attempt <= ASH_MAX_RETRIES; attempt++) {
        bool retx = attempt > 0;
        if (retx) {
            s_stats.retransmissions++;
            ESP_LOGW(TAG, "Retransmit DATA frmNum=%u attempt=%d", (unsigned)s_pending_frm,
                     attempt);
        }

        esp_err_t err =
            ash_send_data_frame(s_pending_frm, s_ack_num, retx, s_pending.data, s_pending.data_len);
        if (err != ESP_OK) {
            result = err;
            break;
        }

        EventBits_t bits =
            xEventGroupWaitBits(s_events, ASH_EVT_ACK | ASH_EVT_ERROR, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(timeout_ms ? timeout_ms : CONFIG_ASH_ACK_TIMEOUT_MS));
        if (bits & ASH_EVT_ACK) {
            s_frm_num = (uint8_t)((s_frm_num + 1) & 7);
            result = ESP_OK;
            break;
        }
        if (bits & ASH_EVT_ERROR) {
            result = ESP_FAIL;
            break;
        }
        s_stats.timeouts++;
        result = ESP_ERR_TIMEOUT;
    }

    if (result != ESP_OK) {
        s_pending_valid = false;
        ESP_LOGE(TAG, "DATA send failed frmNum=%u", (unsigned)s_pending_frm);
    }

    xSemaphoreGive(s_tx_mutex);
    return result;
}

void ash_get_stats(ash_stats_t *out)
{
    if (out) {
        *out = s_stats;
        /* Merge UART counters for a coherent web/status view */
        icc_uart_stats_t uart;
        icc_uart_get_stats(&uart);
        out->tx_bytes = uart.tx_bytes;
        out->rx_bytes = uart.rx_bytes;
    }
}

void ash_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}

uint8_t ash_last_reset_code(void)
{
    return s_last_reset_code;
}
