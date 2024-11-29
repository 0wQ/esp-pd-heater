/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LVGL port knob
 */

#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include "iot_knob.h"
#include "iot_button.h"
#if LVGL_VERSION_MAJOR == 8
#include "esp_lvgl_port_compatibility.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration of the encoder structure
 */
typedef struct {
    lv_display_t *disp;    /*!< LVGL display handle (returned from lvgl_port_add_disp) */
    const knob_config_t *encoder_a_b;
    const button_config_t *encoder_enter;  /*!< Navigation button for enter */
} _lvgl_port_encoder_cfg_t;

/**
 * @brief Add encoder as an input device
 *
 * @note Allocated memory in this function is not free in deinit. You must call lvgl_port_remove_encoder for free all memory!
 *
 * @param encoder_cfg Encoder configuration structure
 * @return Pointer to LVGL encoder input device or NULL when error occurred
 */
lv_indev_t *_lvgl_port_add_encoder(const _lvgl_port_encoder_cfg_t *encoder_cfg);

#ifdef __cplusplus
}
#endif
