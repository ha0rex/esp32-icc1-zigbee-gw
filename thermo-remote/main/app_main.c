#include "buttons.h"
#include "epd.h"
#include "link.h"
#include "pins.h"
#include "ui.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "app";

static float clamp_step(float target, float delta)
{
    float next = target + delta;
    if (next < THERMO_MIN_C) {
        next = THERMO_MIN_C;
    }
    if (next > THERMO_MAX_C) {
        next = THERMO_MAX_C;
    }
    return next;
}

static void forget_and_restart(void)
{
    ui_message("WIFI CLEARED", "RESTARTING");
    link_forget();
    esp_restart();
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    buttons_init();
    epd_init();
    link_init();
    ESP_LOGI(TAG, "thermostat remote");
    ui_message("THERMOSTAT", "REMOTE");

    link_list_t list = {0};
    int index = 0;
    int64_t hold_target_until = 0;
    float held_target = 0;
    bool have_list = false;

    for (;;) {
        btn_ev_t ev = buttons_poll();
        if (ev == BTN_FORGET) {
            forget_and_restart();
        }

        if (!link_have_creds()) {
            ui_setup();
            link_portal_until_saved();
            continue;
        }
        if (!link_sta_up()) {
            static bool told_wifi;
            if (!told_wifi) {
                ui_message("JOINING WIFI", link_ssid());
                told_wifi = true;
            }
            if (!link_connect(20000)) {
                static bool told;
                if (!told) {
                    ui_message("WIFI FAILED", "HOLD L+R");
                    told = true;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            told_wifi = false;
        }
        if (!link_have_gateway()) {
            static bool told_find;
            if (!told_find) {
                ui_message("FINDING", "GATEWAY");
                told_find = true;
            }
            if (!link_discover()) {
                static bool told_miss;
                if (!told_miss) {
                    ui_message("NO GATEWAY", "SET IP IN SETUP");
                    told_miss = true;
                }
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
        }

        bool changed = false;
        if (have_list && list.count > 0) {
            if (index >= list.count) {
                index = 0;
            }
            link_thermo_t *cur = &list.items[index];
            if (ev == BTN_LEFT || ev == BTN_RIGHT) {
                float next = clamp_step(cur->target_c, ev == BTN_RIGHT ? THERMO_STEP_C : -THERMO_STEP_C);
                if (next != cur->target_c) {
                    cur->target_c = next;
                    held_target = next;
                    hold_target_until = esp_timer_get_time() + 3000000;
                    if (!link_set_target(cur->id, next)) {
                        ui_thermo(cur->name, cur->has_current, cur->current_c, cur->target_c, index,
                                  list.count, "NOT SAVED");
                    } else {
                        changed = true;
                    }
                }
            } else if (ev == BTN_CENTER && list.count > 1) {
                index = (index + 1) % list.count;
                changed = true;
            }
        }

        static int64_t next_poll;
        int64_t now = esp_timer_get_time();
        if (!have_list || now >= next_poll) {
            link_list_t fresh;
            if (link_fetch(&fresh)) {
                if (have_list && list.count > 0 && now < hold_target_until) {
                    uint8_t id = list.items[index].id;
                    for (int i = 0; i < fresh.count; i++) {
                        if (fresh.items[i].id == id) {
                            fresh.items[i].target_c = held_target;
                            index = i;
                            break;
                        }
                    }
                } else if (have_list && list.count > 0 && fresh.count > 0) {
                    uint8_t id = list.items[index].id;
                    index = 0;
                    for (int i = 0; i < fresh.count; i++) {
                        if (fresh.items[i].id == id) {
                            index = i;
                            break;
                        }
                    }
                }
                bool same = have_list && fresh.count == list.count;
                if (same) {
                    for (int i = 0; i < fresh.count; i++) {
                        if (fresh.items[i].id != list.items[i].id ||
                            fresh.items[i].has_current != list.items[i].has_current ||
                            fresh.items[i].target_c != list.items[i].target_c ||
                            (fresh.items[i].has_current &&
                             fresh.items[i].current_c != list.items[i].current_c)) {
                            same = false;
                            break;
                        }
                    }
                }
                list = fresh;
                have_list = true;
                if (!same) {
                    changed = true;
                }
            } else if (!have_list) {
                ui_message("NO REPLY", "FROM GATEWAY");
            }
            next_poll = now + 15000000;
        }

        if (changed && list.count > 0) {
            link_thermo_t *cur = &list.items[index];
            const char *st = cur->has_current ? "LINKED" : "NO TEMP YET";
            ui_thermo(cur->name, cur->has_current, cur->current_c, cur->target_c, index, list.count, st);
        } else if (changed && list.count == 0) {
            ui_message("NO THERMOSTAT", "CREATE ONE");
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
