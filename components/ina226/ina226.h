#ifndef INA226_H_
#define INA226_H_

#include <stdint.h>

#include "driver/i2c_master.h"
#include "i2c_bus.h"

enum {
    INA226_REG_CONFIGURATION = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_POWER = 0x03,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MASK_ENABLE = 0x06,
    INA226_REG_ALERT_LIMIT = 0x07,
    INA226_REG_DIE_ID = 0xFF,  // 2260
};

enum {
    INA226_CONFIGURATION__RST = 0x8000,

    INA226_CONFIGURATION__AVG_MASK = 0x0E00,
    INA226_CONFIGURATION__AVG_SHIFT = 9,

    INA226_CONFIGURATION__BUS_CONV_TIME_MASK = 0x01C0,
    INA226_CONFIGURATION__BUS_CONV_TIME_SHIFT = 6,

    INA226_CONFIGURATION__SHUNT_CONV_TIME_MASK = 0x0038,
    INA226_CONFIGURATION__SHUNT_CONV_TIME_SHIFT = 3,

    INA226_CONFIGURATION__MODE_MASK = 0x0007,
    INA226_CONFIGURATION__MODE_SHIFT = 0,
};

enum {
    INA226_MASK_ENABLE__CVRF = 0x0008,
};

typedef enum {
    INA226_PERIOD_140us = 0,
    INA226_PERIOD_204us = 1,
    INA226_PERIOD_332us = 2,
    INA226_PERIOD_588us = 3,
    INA226_PERIOD_1100us = 4,
    INA226_PERIOD_2116us = 5,
    INA226_PERIOD_4156us = 6,
    INA226_PERIOD_8244us = 7,
} ina226_sampling_period_t;

typedef enum {
    INA226_AVERAGE_1 = 0,
    INA226_AVERAGE_4 = 1,
    INA226_AVERAGE_16 = 2,
    INA226_AVERAGE_64 = 3,
    INA226_AVERAGE_128 = 4,
    INA226_AVERAGE_256 = 5,
    INA226_AVERAGE_512 = 6,
    INA226_AVERAGE_1024 = 7,
} ina226_averaging_factor_t;

void ina226_init(i2c_bus_handle_t bus, uint8_t dev_addr);

void ina226_calibrate(float r_shunt, float max_current);
void ina226_configure(ina226_sampling_period_t period, ina226_averaging_factor_t average);

void ina226_read(float *voltage, float *current, float *power, float *shunt_voltage);
int ina226_conversion_ready();

#endif