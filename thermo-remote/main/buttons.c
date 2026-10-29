#include "buttons.h"

#include "pins.h"

#include "driver/gpio.h"
#include "esp_timer.h"

static int64_t s_both_since;
static int64_t s_left_rpt;
static int64_t s_right_rpt;
static bool s_center_down;
static bool s_left_seen_up;
static bool s_right_seen_up;
static bool s_center_seen_up;

static bool down(int pin)
{
    return gpio_get_level(pin) == 0;
}

void buttons_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_LEFT) | (1ULL << BTN_RIGHT) | (1ULL << BTN_CENTER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&cfg);
}

btn_ev_t buttons_poll(void)
{
    int64_t now = esp_timer_get_time();
    bool left = down(BTN_LEFT);
    bool right = down(BTN_RIGHT);
    bool center = down(BTN_CENTER);

    if (left && right) {
        if (s_both_since == 0) {
            s_both_since = now;
        } else if (now - s_both_since > 2000000) {
            s_both_since = 0;
            s_left_rpt = 0;
            s_right_rpt = 0;
            return BTN_FORGET;
        }
        s_left_rpt = 0;
        s_right_rpt = 0;
        return BTN_NONE;
    }
    s_both_since = 0;

    if (!left) {
        s_left_seen_up = true;
    }
    if (!right) {
        s_right_seen_up = true;
    }
    if (!center) {
        s_center_seen_up = true;
    }

    if (left && !right && s_left_seen_up) {
        if (s_left_rpt == 0 || now - s_left_rpt > 350000) {
            s_left_rpt = now;
            return BTN_LEFT;
        }
    } else {
        s_left_rpt = 0;
    }

    if (right && !left && s_right_seen_up) {
        if (s_right_rpt == 0 || now - s_right_rpt > 350000) {
            s_right_rpt = now;
            return BTN_RIGHT;
        }
    } else {
        s_right_rpt = 0;
    }

    if (center && s_center_seen_up) {
        s_center_down = true;
    } else if (s_center_down && !center) {
        s_center_down = false;
        return BTN_CENTER;
    }
    return BTN_NONE;
}
