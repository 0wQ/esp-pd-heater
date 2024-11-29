#include "ina226.h"

#include <math.h>

static uint16_t read_reg(uint8_t addr);
static void write_reg(uint8_t addr, uint16_t value);

static struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    float current_lsb;
} ina;

void ina226_init(i2c_bus_handle_t bus, uint8_t dev_addr) {
    // Store the parameters
    ina.i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    ina.dev_addr = dev_addr;

    if (ina.i2c_dev == NULL) {
        printf("Failed ina226_init\n");
    }

    // Reset the chip
    write_reg(INA226_REG_CONFIGURATION, INA226_CONFIGURATION__RST);
}

void ina226_calibrate(float r_shunt, float max_current) {
    // Compute the current LSB as max_expected_current/2**15
    float current_lsb = max_current / (1 << 15);

    // Compute calibration register as 0.00512 / (current_lsb * r_shunt)
    float calib = 0.00512 / (current_lsb * r_shunt);

    // Register is a 16bit unsigned integer, thus convert and round above
    uint16_t calib_reg = (uint16_t)floorf(calib);

    // Re-compute and store real current LSB
    ina.current_lsb = 0.00512 / (r_shunt * calib_reg);

    // Write calibration
    write_reg(INA226_REG_CALIBRATION, calib_reg);
}

void ina226_configure(ina226_sampling_period_t period, ina226_averaging_factor_t average) {
    // Prepare register value;
    uint16_t reg;
    reg = (period << INA226_CONFIGURATION__BUS_CONV_TIME_SHIFT) & INA226_CONFIGURATION__BUS_CONV_TIME_MASK;
    reg |= (period << INA226_CONFIGURATION__SHUNT_CONV_TIME_SHIFT) & INA226_CONFIGURATION__SHUNT_CONV_TIME_MASK;
    reg |= (average << INA226_CONFIGURATION__AVG_SHIFT) & INA226_CONFIGURATION__AVG_MASK;
    reg |= INA226_CONFIGURATION__MODE_MASK;

    // Write the configuration value
    write_reg(INA226_REG_CONFIGURATION, reg);
}

int ina226_conversion_ready() {
    // Read the MASK ENABLE register and check for the ConversionReady flag
    return (read_reg(INA226_REG_MASK_ENABLE) & INA226_MASK_ENABLE__CVRF) != 0;
}

void ina226_read(float *voltage, float *current, float *power, float *shunt_voltage) {
    uint16_t voltage_reg;
    // Read BUS voltage register
    voltage_reg = read_reg(INA226_REG_BUS_VOLTAGE);
    int16_t current_reg;
    // Read current register
    current_reg = (int16_t)read_reg(INA226_REG_CURRENT);
    int16_t power_reg;
    // Read POWER register
    power_reg = (int16_t)read_reg(INA226_REG_POWER);
    int16_t shunt_voltage_reg;
    // Read POWER register
    shunt_voltage_reg = (int16_t)read_reg(INA226_REG_SHUNT_VOLTAGE);

    // Read the mask/enable register to clear it
    (void)read_reg(INA226_REG_MASK_ENABLE);

    // Check for the requested measures and compute their values
    if (voltage) {
        // Convert to Volts
        *voltage = (float)voltage_reg * 1.25e-3;
    }
    if (current) {
        // Convert to Amperes
        *current = (float)current_reg * ina.current_lsb;
    }
    if (power) {
        // Convert to Watts
        *power = (float)power_reg * 25 * ina.current_lsb;
    }
    if (shunt_voltage) {
        // Convert to Volts
        *shunt_voltage = (float)shunt_voltage_reg * 2.5e-6;
    }
}

// static uint16_t read_reg(uint8_t addr) {
//     uint8_t buffer;
//     i2c_bus_read_byte(ina.i2c_dev, addr, &buffer);

//     printf("read_reg: %d, buffer: %d\n", addr, buffer);
//     return buffer;
// }

// static void write_reg(uint8_t addr, uint16_t value) {
//     printf("write_reg: %d %d\n", addr, value);

//     i2c_bus_write_byte(ina.i2c_dev, addr, &value);
//     // // Prepare the 3 byte buffer
//     // uint8_t buf[3];
//     // buf[0] = addr;
//     // buf[1] = value >> 8;
//     // buf[2] = value & 0xFF;
//     // // Write the value
//     // i2c_tx(ina.i2c, ina.i2c_address, buf, 3);
// }

static uint16_t read_reg(uint8_t addr) {
    uint8_t buf[2] = {0x00, 0x00};
    esp_err_t ret;

    // 读取寄存器值（2 字节）
    ret = i2c_bus_read_bytes(ina.i2c_dev, addr, 2, buf);
    if (ret != ESP_OK) {
        printf("I2C Read Error at addr 0x%02X: %s\n", addr, esp_err_to_name(ret));
        return 0xFFFF;  // 这里返回一个异常值以标识读取错误
    }

    // printf("Read addr: 0x%02X, data: 0x%04X\n", addr, buf[0] << 8 | buf[1]);  // 调试输出
    return buf[0] << 8 | buf[1];
}

static void write_reg(uint8_t addr, uint16_t value) {
    uint8_t buf[2];
    buf[0] = value >> 8;    // 高8位
    buf[1] = value & 0xFF;  // 低8位

    esp_err_t ret = i2c_bus_write_bytes(ina.i2c_dev, addr, 2, buf);
    if (ret != ESP_OK) {
        printf("I2C Write Error at addr 0x%02X: %s\n", addr, esp_err_to_name(ret));
        return;
    }

    // printf("Write Reg 0x%02X: 0x%04X\n", addr, value);  // 调试输出
}
