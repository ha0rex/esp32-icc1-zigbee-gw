#include "buttons.h"

#include "battery.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "btn";

static adc_oneshot_unit_handle_t s_adc;
static bool s_armed;
static int s_stable_key;
static int s_stable_count;
static int s_key;
static int64_t s_center_since;
static bool s_center_fired;

enum {
    KEY_NONE = 0,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_CENTER,
};

static int read_raw(void)
{
    int raw = 4095;
    if (s_adc == NULL || adc_oneshot_read(s_adc, ADC_CHANNEL_1, &raw) != ESP_OK) {
        return 4095;
    }
    return raw;
}

/*
 * One ladder on GPIO1, bands from the ebook firmware.
 * Tried on this reader: high band is left (down), mid band is right (up),
 * the near-ground band is the middle key.
 */
static int classify(int raw)
{
    if (raw <= 500) {
        return KEY_CENTER;
    }
    if (raw >= 1400 && raw < 2250) {
        return KEY_RIGHT;
    }
    if (raw >= 2250 && raw <= 3400) {
        return KEY_LEFT;
    }
    return KEY_NONE;
}

void buttons_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
    adc_oneshot_chan_cfg_t chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_1, &chan));
    battery_bind(s_adc);
    int batt_pct = 0;
    bool charging = false;
    battery_read(&batt_pct, &charging);
    int raw = read_raw();
    s_stable_key = classify(raw);
    s_key = KEY_NONE;
    ESP_LOGI(TAG, "button adc raw %d", raw);
}

btn_ev_t buttons_poll(void)
{
    int raw = read_raw();
    int key = classify(raw);
    if (key == s_stable_key) {
        if (s_stable_count < 3) {
            s_stable_count++;
        }
    } else {
        s_stable_key = key;
        s_stable_count = 1;
    }
    if (s_stable_count < 2) {
        return BTN_NONE;
    }

    if (!s_armed) {
        s_key = key;
        if (key == KEY_NONE) {
            s_armed = true;
        }
        return BTN_NONE;
    }

    if (key == s_key) {
        if (key == KEY_CENTER && !s_center_fired &&
            esp_timer_get_time() - s_center_since > 2000000) {
            s_center_fired = true;
            return BTN_FORGET;
        }
        return BTN_NONE;
    }

    int prev = s_key;
    ESP_LOGI(TAG, "key %d raw %d", key, raw);
    s_key = key;

    if (prev == KEY_CENTER && !s_center_fired && key == KEY_NONE) {
        return BTN_CENTER;
    }
    if (key == KEY_CENTER) {
        s_center_since = esp_timer_get_time();
        s_center_fired = false;
        return BTN_NONE;
    }
    if (key == KEY_LEFT) {
        return BTN_LEFT;
    }
    if (key == KEY_RIGHT) {
        return BTN_RIGHT;
    }
    return BTN_NONE;
}
