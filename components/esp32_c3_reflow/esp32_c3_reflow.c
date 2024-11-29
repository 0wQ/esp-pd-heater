/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/display.h"
#include "bsp/esp-bsp.h"
#include "bsp_err_check.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7735.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "iot_knob.h"
#include "lvgl_port_knob.h"

static const char *TAG = "ESP32-C3-Reflow";

static bool buzzer_initialized = false;
static lv_indev_t *disp_indev = NULL;
static uint8_t current_brightness_percent = 0;
static uint32_t current_buzzer_volume = 7168;
static piano_note_t current_buzzer_note = NOTE_C8;

static const button_config_t bsp_encoder_btn_config = {
    .type = BUTTON_TYPE_GPIO,
    .gpio_button_config.active_level = false,
    .gpio_button_config.gpio_num = BSP_BTN_PRESS,
};

static const knob_config_t bsp_encoder_a_b_config = {
    .default_direction = 0,
    .gpio_encoder_a = BSP_ENCODER_A,
    .gpio_encoder_b = BSP_ENCODER_B,
};

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg) {
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }};

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp) {
    const _lvgl_port_encoder_cfg_t encoder = {
        .disp = disp,
        .encoder_a_b = &bsp_encoder_a_b_config,
        .encoder_enter = &bsp_encoder_btn_config,
    };

    return _lvgl_port_add_encoder(&encoder);
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define LCD_LEDC_DUTY_RES (LEDC_TIMER_10_BIT)
#define LCD_LEDC_CH CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.50 steps = 115 range = 0-1023
const uint16_t gamma_lut[101] = {
    0,
    6,
    8,
    9,
    10,
    12,
    13,
    15,
    17,
    19,
    21,
    23,
    25,
    28,
    31,
    33,
    36,
    39,
    43,
    46,
    50,
    53,
    57,
    61,
    66,
    70,
    75,
    79,
    84,
    89,
    95,
    100,
    106,
    112,
    118,
    124,
    130,
    137,
    144,
    151,
    158,
    165,
    173,
    181,
    189,
    197,
    206,
    214,
    223,
    232,
    242,
    251,
    261,
    271,
    281,
    292,
    302,
    313,
    324,
    336,
    347,
    359,
    371,
    384,
    396,
    409,
    422,
    435,
    449,
    463,
    477,
    491,
    506,
    520,
    536,
    551,
    567,
    582,
    599,
    615,
    632,
    649,
    666,
    683,
    701,
    719,
    737,
    756,
    775,
    794,
    813,
    833,
    853,
    873,
    894,
    914,
    936,
    957,
    979,
    1001,
    1023,
};

esp_err_t bsp_display_brightness_init(void) {
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LCD_LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));
    ledc_fade_func_uninstall();
    BSP_ERROR_CHECK_RETURN_ERR(ledc_fade_func_install(0));

    return ESP_OK;
}

uint8_t bsp_display_brightness_get() {
    return current_brightness_percent;
}

esp_err_t bsp_display_brightness_set(int brightness_percent, uint16_t fade_time) {
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }
    current_brightness_percent = brightness_percent;

    ESP_LOGD(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    // uint32_t duty_cycle = (BIT(LCD_LEDC_DUTY_RES) * (brightness_percent)) / 100;
    uint16_t duty_cycle = gamma_lut[brightness_percent];  // 伽马校正亮度
    if (fade_time == 0) {
        BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
        BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
    } else {
        // 改用渐变
        BSP_ERROR_CHECK_RETURN_ERR(ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle, fade_time));
        BSP_ERROR_CHECK_RETURN_ERR(ledc_fade_stop(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
        BSP_ERROR_CHECK_RETURN_ERR(ledc_fade_start(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, LEDC_FADE_WAIT_DONE));
    }

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void) {
    return bsp_display_brightness_set(0, 1000);
}

esp_err_t bsp_display_backlight_on(void) {
    return bsp_display_brightness_set(100, 1000);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io) {
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_PCLK,
        .mosi_io_num = BSP_LCD_DATA0,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7735(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_reset(*ret_panel));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_init(*ret_panel));

#if LCD_TYPE == LCD_TYPE_HSD
    // st7735s 瀚彩 HSD
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_invert_color(*ret_panel, true));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_set_gap(*ret_panel, 1, 26));
#elif LCD_TYPE == LCD_TYPE_BOE
    // st7735s / gc9107 京东方 BOE
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_invert_color(*ret_panel, false));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_set_gap(*ret_panel, 0, 24));
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_disp_on_off(*ret_panel, true));
#else
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_panel_disp_off(*ret_panel, false));
#endif

    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

lv_display_t *bsp_display_start(void) {
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = 1,
#else
        .double_buffer = 0,
#endif
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }};
    return bsp_display_start_with_config(&cfg);
}

lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg) {
    lv_disp_t *disp;
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void) {
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation) {
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms) {
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void) {
    lvgl_port_unlock();
}

esp_err_t bsp_buzzer_init() {
    esp_err_t ret = beep_init(BSP_BUZZER_IO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Buzzer init failed");
        buzzer_initialized = false;
    } else {
        buzzer_initialized = true;
    }
    return ret;
}

esp_err_t bsp_buzzer_beep(piano_note_t current_note, uint32_t current_loudness, float loud_time, float no_loud_time, uint8_t loud_cycle_time) {
    if (!buzzer_initialized) {
        ESP_LOGW(TAG, "Buzzer not initialized");
        return ESP_FAIL;
    }
    beep(current_note, current_loudness, loud_time, no_loud_time, loud_cycle_time);
    return ESP_OK;
}

esp_err_t bsp_buzzer_beep_encoder(void) {
    if (current_buzzer_volume == 0) return ESP_OK;
    return bsp_buzzer_beep(current_buzzer_note, current_buzzer_volume, 0.02, 0, 1);
}

void bsp_buzzer_set_default(piano_note_t note, uint32_t volume) {
    current_buzzer_note = note;
    current_buzzer_volume = volume;
}