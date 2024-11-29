#include "ch32x035_pd.h"

#include <stdlib.h>

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
} ch32x035_pd_dev_t;

ch32x035_pd_handle_t ch32x035_pd_create(i2c_bus_handle_t bus, uint8_t addr) {
    ch32x035_pd_dev_t* dev = (ch32x035_pd_dev_t*)calloc(1, sizeof(ch32x035_pd_dev_t));
    dev->i2c_dev = i2c_bus_device_create(bus, addr, i2c_bus_get_current_clk_speed(bus));
    if (dev->i2c_dev == NULL) {
        free(dev);
        return NULL;
    }
    dev->dev_addr = addr;
    return (ch32x035_pd_handle_t)dev;
}

esp_err_t ch32x035_pd_delete(ch32x035_pd_handle_t* handle) {
    if (*handle == NULL) {
        return ESP_OK;
    }
    ch32x035_pd_dev_t* dev = (ch32x035_pd_dev_t*)(*handle);
    i2c_bus_device_delete(&dev->i2c_dev);
    free(dev);
    *handle = NULL;
    return ESP_OK;
}

esp_err_t ch32x035_pd_request_voltage(ch32x035_pd_handle_t handle, uint8_t voltage) {
    ch32x035_pd_dev_t* dev = (ch32x035_pd_dev_t*)handle;
    return i2c_bus_write_bytes(dev->i2c_dev, REG_PD_REQUEST, 1, &voltage);
}

esp_err_t ch32x035_pd_get_voltage(ch32x035_pd_handle_t handle, uint8_t* voltage) {
    ch32x035_pd_dev_t* dev = (ch32x035_pd_dev_t*)handle;
    return i2c_bus_read_byte(dev->i2c_dev, REG_DATA, voltage);
}

esp_err_t ch32x035_pd_get_device_id(ch32x035_pd_handle_t handle, uint8_t* id) {
    ch32x035_pd_dev_t* dev = (ch32x035_pd_dev_t*)handle;
    return i2c_bus_read_byte(dev->i2c_dev, REG_WHO_AM_I, id);
}