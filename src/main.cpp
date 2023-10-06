
#include <Arduino.h>
#include <Wire.h>
#include <esp_lcd_panel_ili9342.h>
#include <lvgl.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "ui.h"
#define LCD_SPI_HOST SPI3_HOST
#define LCD_DMA
#define LCD_BCKL_ON_LEVEL 1
#define LCD_BCKL_OFF_LEVEL !LCD_BCKL_ON_LEVEL
#define LCD_PIN_NUM_MOSI 19
#define LCD_PIN_NUM_CLK 18
#define LCD_PIN_NUM_CS 5
#define LCD_PIN_NUM_DC 16
#define LCD_PIN_NUM_RST 23
#define LCD_PIN_NUM_BCKL 4
#define LCD_PANEL esp_lcd_new_panel_st7789
#define LCD_HRES 135
#define LCD_VRES 240
#define LCD_COLOR_SPACE ESP_LCD_COLOR_SPACE_RGB
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_GAP_X 40
#define LCD_GAP_Y 52
#define LCD_MIRROR_X false
#define LCD_MIRROR_Y true
#define LCD_INVERT_COLOR true
#define LCD_SWAP_XY true

static esp_lcd_panel_handle_t lcd_handle = NULL;
static uint8_t transfer_buffer1[32 * 1024];
static uint8_t transfer_buffer2[32 * 1024];
static lv_display_t *disp;

static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_flush_ready(disp);
    return true;
}
static void lvgl_flush(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2 + 1;
    int offsety1 = area->y1;
    int offsety2 = area->y2 + 1;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(lcd_handle, offsetx1, offsety1, offsetx2, offsety2, (void *)color_map);
}
// initialize the screen using the esp panel API
static void lcd_panel_init() {
#ifdef LCD_PIN_NUM_BCKL
    gpio_set_direction((gpio_num_t)LCD_PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
#endif
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = LCD_PIN_NUM_CLK;
    buscfg.mosi_io_num = LCD_PIN_NUM_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(transfer_buffer1) + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = LCD_PIN_NUM_DC,
    io_config.cs_gpio_num = LCD_PIN_NUM_CS,
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ,
    io_config.lcd_cmd_bits = 8,
    io_config.lcd_param_bits = 8,
    io_config.spi_mode = 0,
    io_config.trans_queue_depth = 10,
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
#ifdef LCD_PIN_NUM_RST
    panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
#else
    panel_config.reset_gpio_num = -1;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (((int)LCD_COLOR_SPACE) == 0) {
        panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
    } else {
        panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
    }
#else
    panel_config.color_space = LCD_COLOR_SPACE;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    LCD_PANEL(io_handle, &panel_config, &lcd_handle);

    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
#ifdef LCD_PIN_NUM_BCKL
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_OFF_LEVEL);
#endif
    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    // esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_SLPOUT, NULL, 0);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, LCD_SWAP_XY);
    esp_lcd_panel_set_gap(lcd_handle, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_mirror(lcd_handle, LCD_MIRROR_X, LCD_MIRROR_Y);
    esp_lcd_panel_invert_color(lcd_handle, LCD_INVERT_COLOR);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
    // Turn on backlight (Different LCD screens may need different levels)
#ifdef LCD_PIN_NUM_BCKL
    gpio_set_level((gpio_num_t)LCD_PIN_NUM_BCKL, LCD_BCKL_ON_LEVEL);
#endif
}

void setup() {
    Serial.begin(115200);
    lcd_panel_init();
    disp = lv_display_create(LCD_VRES, LCD_HRES);
    lv_display_set_flush_cb(disp, lvgl_flush);
    lv_display_set_draw_buffers(disp, transfer_buffer1, transfer_buffer2, sizeof(transfer_buffer1), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ui_init();
}
void loop() {
    lv_timer_handler();
    delay(3);
}

