#include "battery.h"

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "batt";

/* Stock firmware copies this table and maps raw ADC counts, not millivolts. */
static const int s_adc[] = {2200, 2300, 2377, 2440, 2505, 2585, 2652, 2719, 2792, 2850};
static const int s_pct[] = {0, 5, 10, 20, 30, 45, 60, 75, 90, 100};

static adc_oneshot_unit_handle_t s_adc_unit;
static int64_t s_next_us;
static int s_percent;
static bool s_charging;
static bool s_raw_chg;
static int s_chg_run;
static int s_logged_pct = -1;
static bool s_logged_chg;
static bool s_logged_usb;

static int full_scale(void)
{
    /* Stock keeps a calibrated full-scale count here when it is above empty. */
    uint16_t cal = *(volatile uint16_t *)0x500000f8;
    if (cal > 2200 && cal < 3600) {
        return cal;
    }
    return 2850;
}

static int map_percent(int raw)
{
    int full = full_scale();
    if (raw <= 2200) {
        return 0;
    }
    if (raw >= full) {
        return 100;
    }
    int span = full - 2200;
    int scaled[10];
    for (int i = 0; i < 10; i++) {
        if (s_adc[i] <= 2200) {
            scaled[i] = 2200;
        } else {
            scaled[i] = 2200 + (s_adc[i] - 2200) * span / 650;
        }
    }
    for (int i = 0; i < 9; i++) {
        if (raw < scaled[i] || raw > scaled[i + 1]) {
            continue;
        }
        int da = scaled[i + 1] - scaled[i];
        if (da <= 0) {
            return s_pct[i + 1];
        }
        return s_pct[i] + (raw - scaled[i]) * (s_pct[i + 1] - s_pct[i]) / da;
    }
    return 100;
}

void battery_bind(adc_oneshot_unit_handle_t adc)
{
    s_adc_unit = adc;
    adc_oneshot_chan_cfg_t chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, ADC_CHANNEL_0, &chan));
}

void battery_read(int *percent, bool *charging)
{
    int64_t now = esp_timer_get_time();
    if (s_next_us != 0 && now < s_next_us) {
        *percent = s_percent;
        *charging = s_charging;
        return;
    }
    s_next_us = now + 1000000;

    int sum = 0;
    int n = 0;
    for (int i = 0; i < 8; i++) {
        int raw = 0;
        if (s_adc_unit != NULL && adc_oneshot_read(s_adc_unit, ADC_CHANNEL_0, &raw) == ESP_OK) {
            sum += raw;
            n++;
        }
    }
    int raw = n > 0 ? sum / n : 0;
    int pct = n > 0 ? map_percent(raw) : 0;
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }

    /*
     * The stock image never reads a charge pin; the red LED is on the charger.
     * Spare inputs stay high while USB is plugged in, so the host link is the
     * signal the chip can see. A power-only adapter does not send those packets.
     */
    bool usb = usb_serial_jtag_is_connected();
    if (usb == s_raw_chg) {
        if (s_chg_run < 3) {
            s_chg_run++;
        }
    } else {
        s_raw_chg = usb;
        s_chg_run = 1;
    }
    if (s_chg_run >= 2) {
        s_charging = usb;
    }
    s_percent = pct;

    if (pct != s_logged_pct || s_charging != s_logged_chg || usb != s_logged_usb) {
        ESP_LOGI(TAG, "adc %d  %d%%  full %d  usb %d  charging %d", raw, pct, full_scale(),
                 usb ? 1 : 0, s_charging ? 1 : 0);
        s_logged_pct = pct;
        s_logged_chg = s_charging;
        s_logged_usb = usb;
    }

    *percent = s_percent;
    *charging = s_charging;
}
