#include "epd.h"

#include "pins.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "epd";
static spi_device_handle_t s_spi;
static uint8_t s_fb[(EPD_W * EPD_H) / 8];

static void cmd(uint8_t c)
{
    gpio_set_level(EPD_DC, 0);
    gpio_set_level(EPD_CS, 0);
    spi_transaction_t t = {.length = 8, .tx_buffer = &c};
    spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(EPD_CS, 1);
}

static void data_byte(uint8_t d)
{
    gpio_set_level(EPD_DC, 1);
    gpio_set_level(EPD_CS, 0);
    spi_transaction_t t = {.length = 8, .tx_buffer = &d};
    spi_device_polling_transmit(s_spi, &t);
    gpio_set_level(EPD_CS, 1);
}

static void wait_idle(void)
{
    for (int i = 0; i < 400; i++) {
        if (gpio_get_level(EPD_BUSY) == 0) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "busy timeout (level %d)", gpio_get_level(EPD_BUSY));
}

static void reset_panel(void)
{
    gpio_set_level(EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void epd_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << EPD_CS) | (1ULL << EPD_DC) | (1ULL << EPD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);
    gpio_config_t busy = {
        .pin_bit_mask = (1ULL << EPD_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&busy);
    gpio_set_level(EPD_CS, 1);
    gpio_set_level(EPD_DC, 0);
    gpio_set_level(EPD_RST, 1);

    spi_bus_config_t bus = {
        .mosi_io_num = EPD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = EPD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(s_fb),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    reset_panel();
    wait_idle();
    cmd(0x12);
    wait_idle();
    cmd(0x01);
    data_byte((EPD_H - 1) & 0xFF);
    data_byte((EPD_H - 1) >> 8);
    data_byte(0x01);
    cmd(0x11);
    data_byte(0x01);
    cmd(0x44);
    data_byte(0x00);
    data_byte((EPD_W / 8) - 1);
    cmd(0x45);
    data_byte((EPD_H - 1) & 0xFF);
    data_byte((EPD_H - 1) >> 8);
    data_byte(0x00);
    data_byte(0x00);
    cmd(0x3C);
    data_byte(0x01);
    cmd(0x18);
    data_byte(0x80);
    cmd(0x22);
    data_byte(0xB1);
    cmd(0x20);
    wait_idle();
    ESP_LOGI(TAG, "panel %dx%d ready", EPD_W, EPD_H);
}

void epd_clear(void)
{
    memset(s_fb, 0xFF, sizeof(s_fb));
}

void epd_pixel(int x, int y, bool black)
{
    if (x < 0 || y < 0 || x >= EPD_W || y >= EPD_H) {
        return;
    }
    int i = (y * EPD_W + x) / 8;
    uint8_t bit = (uint8_t)(0x80 >> (x & 7));
    if (black) {
        s_fb[i] &= (uint8_t)~bit;
    } else {
        s_fb[i] |= bit;
    }
}

void epd_show(void)
{
    cmd(0x4E);
    data_byte(0x00);
    cmd(0x4F);
    data_byte((EPD_H - 1) & 0xFF);
    data_byte((EPD_H - 1) >> 8);
    cmd(0x24);
    gpio_set_level(EPD_DC, 1);
    gpio_set_level(EPD_CS, 0);
    for (size_t off = 0; off < sizeof(s_fb); off += 256) {
        size_t n = sizeof(s_fb) - off;
        if (n > 256) {
            n = 256;
        }
        spi_transaction_t t = {.length = n * 8, .tx_buffer = s_fb + off};
        spi_device_polling_transmit(s_spi, &t);
    }
    gpio_set_level(EPD_CS, 1);
    cmd(0x22);
    data_byte(0xF7);
    cmd(0x20);
    wait_idle();
}
