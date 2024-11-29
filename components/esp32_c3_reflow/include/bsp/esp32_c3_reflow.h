/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/**
 * @file
 * @brief ESP BSP: ESP32-C3-Reflow
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "iot_button.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "bsp/display.h"
#include "beep.h"
/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          0
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  0
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_LED            0
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 *  ESP32-C3-Reflow pinout
 **************************************************************************************************/

/* I2C */
#define BSP_I2C_SCL_IO        (GPIO_NUM_0)
#define BSP_I2C_SDA_IO        (GPIO_NUM_1)

/* Buzzer */
#define BSP_BUZZER_IO         (GPIO_NUM_10)

/* Display */
#define BSP_LCD_DATA0         (GPIO_NUM_7)
#define BSP_LCD_PCLK          (GPIO_NUM_6)
#define BSP_LCD_CS            (GPIO_NUM_NC)
#define BSP_LCD_DC            (GPIO_NUM_2)
#define BSP_LCD_RST           (GPIO_NUM_5)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_18)

typedef enum {
    LCD_TYPE_HSD = 1,
    LCD_TYPE_BOE = 2,
} lcd_type_t;

#define LCD_TYPE              LCD_TYPE_HSD

/* Buttons */
typedef enum {
    BSP_BTN_PRESS = GPIO_NUM_9,
} bsp_button_t;

#define BSP_ENCODER_A         (GPIO_NUM_19)
#define BSP_ENCODER_B         (GPIO_NUM_8)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
    } flags;
} bsp_display_cfg_t;

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-C3-Reflow is shipped with 0.9inch ST7735S display controller.
 * It features 16-bit colors, 160x80 resolution.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 ****************************************************8**********************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (60 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);


/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);

esp_err_t bsp_buzzer_init();
esp_err_t bsp_buzzer_beep(piano_note_t current_note, uint32_t current_loudness, float loud_time, float no_loud_time, uint8_t loud_cycle_time);
esp_err_t bsp_buzzer_beep_encoder(void);
void bsp_buzzer_set_default(piano_note_t note, uint32_t volume);

#ifdef __cplusplus
}
#endif
