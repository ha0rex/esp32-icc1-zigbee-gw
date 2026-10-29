#include "climate.h"

#include "pins.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "climate";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static int s_kind; /* 1 = AHT30, 2 = SHT30 */
static int64_t s_next_probe;
static int64_t s_next_read;
static float s_temp;
static float s_hum;
static bool s_have;
static int s_fail;

static void bus_drop(void)
{
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_kind = 0;
}

static bool open_bus(uint16_t addr)
{
    bus_drop();
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        s_bus = NULL;
        return false;
    }
    if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) {
        bus_drop();
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        bus_drop();
        return false;
    }
    return true;
}

static bool in_range(float t, float h)
{
    return t > -20.0f && t < 60.0f && h >= 0.0f && h <= 100.0f;
}

static bool read_aht(float *t, float *h)
{
    uint8_t status = 0;
    uint8_t cmd_status = 0x71;
    if (i2c_master_transmit_receive(s_dev, &cmd_status, 1, &status, 1, 50) == ESP_OK &&
        (status & 0x08) == 0) {
        uint8_t init[3] = {0xBE, 0x08, 0x00};
        i2c_master_transmit(s_dev, init, 3, 50);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    if (i2c_master_transmit(s_dev, cmd, 3, 50) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
    uint8_t b[7] = {0};
    if (i2c_master_receive(s_dev, b, 7, 50) != ESP_OK) {
        return false;
    }
    if ((b[0] & 0x80) != 0 || (b[0] & 0x08) == 0) {
        ESP_LOGW(TAG, "AHT status %02x", b[0]);
        return false;
    }
    uint32_t raw_h = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
    uint32_t raw_t = (((uint32_t)b[3] & 0x0f) << 16) | ((uint32_t)b[4] << 8) | b[5];
    *h = (raw_h * 100.0f) / 1048576.0f;
    *t = (raw_t * 200.0f) / 1048576.0f - 50.0f;
    if (!in_range(*t, *h)) {
        ESP_LOGW(TAG, "AHT raw %02x %02x %02x %02x %02x %02x -> %.1f C %.1f %%", b[0], b[1], b[2],
                 b[3], b[4], b[5], (double)*t, (double)*h);
        return false;
    }
    return true;
}

static bool read_sht(float *t, float *h)
{
    uint8_t cmd[2] = {0x2C, 0x06};
    if (i2c_master_transmit(s_dev, cmd, 2, 50) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t b[6] = {0};
    if (i2c_master_receive(s_dev, b, 6, 50) != ESP_OK) {
        return false;
    }
    uint16_t raw_t = ((uint16_t)b[0] << 8) | b[1];
    uint16_t raw_h = ((uint16_t)b[3] << 8) | b[4];
    *t = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    *h = 100.0f * ((float)raw_h / 65535.0f);
    return in_range(*t, *h);
}

static bool measure(float *t, float *h)
{
    if (s_kind == 1) {
        return read_aht(t, h);
    }
    if (s_kind == 2) {
        return read_sht(t, h);
    }
    return false;
}

static bool probe_once(void)
{
    float t = 0;
    float h = 0;
    if (open_bus(0x38)) {
        s_kind = 1;
        if (read_aht(&t, &h) || read_aht(&t, &h)) {
            s_temp = t;
            s_hum = h;
            s_have = true;
            ESP_LOGI(TAG, "AHT30 SDA %d SCL %d  %.1f C  %.0f %%", I2C_SDA, I2C_SCL, (double)t,
                     (double)h);
            return true;
        }
        bus_drop();
    }
    if (open_bus(0x44)) {
        s_kind = 2;
        if (read_sht(&t, &h)) {
            s_temp = t;
            s_hum = h;
            s_have = true;
            ESP_LOGI(TAG, "SHT30 SDA %d SCL %d  %.1f C  %.0f %%", I2C_SDA, I2C_SCL, (double)t,
                     (double)h);
            return true;
        }
        bus_drop();
    }
    ESP_LOGW(TAG, "no AHT30 or SHT30 on SDA %d SCL %d", I2C_SDA, I2C_SCL);
    return false;
}

bool climate_read(float *temp_c, float *humidity)
{
    int64_t now = esp_timer_get_time();
    if (s_kind && s_have && now < s_next_read) {
        if (temp_c) {
            *temp_c = s_temp;
        }
        if (humidity) {
            *humidity = s_hum;
        }
        return true;
    }
    if (!s_kind) {
        if (now < s_next_probe) {
            return false;
        }
        bool ok = probe_once();
        s_next_probe = esp_timer_get_time() + (ok ? 0 : 30000000);
        if (!ok) {
            return false;
        }
        s_next_read = esp_timer_get_time() + 5000000;
    } else {
        float t = 0;
        float h = 0;
        if (!measure(&t, &h)) {
            s_fail++;
            if (s_fail < 3 && s_have) {
                if (temp_c) {
                    *temp_c = s_temp;
                }
                if (humidity) {
                    *humidity = s_hum;
                }
                return true;
            }
            bus_drop();
            s_have = false;
            s_fail = 0;
            s_next_probe = now + 5000000;
            return false;
        }
        s_fail = 0;
        s_temp = t;
        s_hum = h;
        s_have = true;
        s_next_read = now + 5000000;
    }
    if (temp_c) {
        *temp_c = s_temp;
    }
    if (humidity) {
        *humidity = s_hum;
    }
    return s_have;
}
