#pragma once

#include <esp_err.h>

#include "i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 设备地址 */
#define CH32X035_PD_ADDR 0x33

/* 寄存器地址 */
#define REG_DATA 0x00
#define REG_PD_REQUEST 0x01
#define REG_WHO_AM_I 0x0F
#define DEVICE_ID 0x35

typedef void* ch32x035_pd_handle_t;

/**
 * @brief 创建CH32X035 PD控制器对象
 * @param bus I2C总线句柄
 * @param addr I2C设备地址(默认0x66)
 * @return 设备句柄
 */
ch32x035_pd_handle_t ch32x035_pd_create(i2c_bus_handle_t bus, uint8_t addr);

/**
 * @brief 删除CH32X035 PD控制器对象
 * @param handle 设备句柄指针
 * @return ESP_OK 成功
 */
esp_err_t ch32x035_pd_delete(ch32x035_pd_handle_t* handle);

/**
 * @brief 请求指定电压
 * @param handle 设备句柄
 * @param voltage 电压值(5/9/12/15/20)
 * @return ESP_OK 成功
 */
esp_err_t ch32x035_pd_request_voltage(ch32x035_pd_handle_t handle, uint8_t voltage);

/**
 * @brief 读取当前设置的电压值
 * @param handle 设备句柄
 * @param voltage 电压值指针
 * @return ESP_OK 成功
 */
esp_err_t ch32x035_pd_get_voltage(ch32x035_pd_handle_t handle, uint8_t* voltage);

/**
 * @brief 读取设备ID
 * @param handle 设备句柄
 * @param id ID值指针
 * @return ESP_OK 成功
 */
esp_err_t ch32x035_pd_get_device_id(ch32x035_pd_handle_t handle, uint8_t* id);

#ifdef __cplusplus
}
#endif