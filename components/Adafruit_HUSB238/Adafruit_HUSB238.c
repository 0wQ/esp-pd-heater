#include "Adafruit_HUSB238.h"

// static const char *TAG = "husb238";

typedef struct
{
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
} husb238_dev_t;

/**
 * @brief   Create and initialization device object and return a device handle
 *
 * @param   bus I2C bus object handle
 * @param   dev_addr I2C device address of device
 *
 * @return
 *     - device object handle of husb238
 */
husb238_handle_t husb238_create(i2c_bus_handle_t bus, uint8_t dev_addr) {
    husb238_dev_t *sensor = (husb238_dev_t *)calloc(1, sizeof(husb238_dev_t));
    sensor->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sensor->i2c_dev == NULL) {
        free(sensor);
        return NULL;
    }
    sensor->dev_addr = dev_addr;
    return (husb238_handle_t)sensor;
}

/**
 * @brief   Delete and release a device object
 *
 * @param   dev object handle of husb238
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t husb238_delete(husb238_handle_t *handle) {
    if (*handle == NULL) {
        return ESP_OK;
    }
    husb238_dev_t *sens = (husb238_dev_t *)(*handle);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens);
    *handle = NULL;
    return ESP_OK;
}

/**
 * @brief   Reads the attachment status from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @return
 *     - 0, HUSB238 is in unattached mode
 *     - 1, HUSB238 is in modes other than unattached mode
 *
 * @details This function reads the 6th bit of the HUSB238_PD_STATUS1 register to get the attachment status.
 * It returns true if the attachment status bit is set.
 */
bool husb238_is_attached(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;

    uint8_t buffer;
    i2c_bus_read_bit(husb238->i2c_dev, HUSB238_PD_STATUS1, 6, &buffer);
    // printf("husb238_is_attached buffer: %d\n", buffer);
    return buffer;
}

/**
 * @brief   Reads the CC direction from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @return  The CC status as a boolean value
 *     - 0, CC1 is connected to CC line or unattached mode
 *     - 1, CC2 is connected to CC line
 *
 * @details This function reads the 7th bit of the HUSB238_PD_STATUS1 register
 */
bool husb238_get_cc_direction(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    i2c_bus_read_bit(husb238->i2c_dev, HUSB238_PD_STATUS1, 7, &buffer);
    // printf("husb238_get_cc_direction buffer: %d\n", buffer);
    return buffer;
}

/**
 * @brief   Reads the PD response from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads bits 3-5 of the HUSB238_PD_STATUS1 register to
 * get the PD response. It returns the response as an HUSB238_ResponseCodes enum value.
 */
esp_err_t husb238_get_pd_response(husb238_handle_t handle, HUSB238_ResponseCodes *response) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_PD_STATUS1, 5, 3, &buffer);
    // printf("husb238_get_pd_response buffer: %d\n", buffer);
    *response = (HUSB238_ResponseCodes)buffer;
    return ret;
}

/**
 * @brief   Reads the 5V contract voltage status from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @return
 *     - 0, Others
 *     - 1, 5V
 *
 * @details This function reads the 2nd bit of the HUSB238_PD_STATUS1 register
 * to get the 5V contract voltage status. It returns true if the 5V contract voltage bit is set.
 */
bool husb238_get_5v_contract_voltage(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    i2c_bus_read_bit(husb238->i2c_dev, HUSB238_PD_STATUS1, 2, &buffer);
    // printf("husb238_get_5v_contract_v buffer: %d\n", buffer);
    return buffer;
}

/**
 * @brief   Reads the 5V contract current from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads the bottom two bits (0-1) of the HUSB238_PD_STATUS1 register to get the 5V contract current.
 * It returns the current as an HUSB238_5VCurrentContract enum value.
 */
esp_err_t husb238_get_5v_contract_current(husb238_handle_t handle, HUSB238_5VCurrentContract *current) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_PD_STATUS1, 1, 2, &buffer);
    // printf("husb238_get_5v_contract_a buffer: %d\n", buffer);
    *current = (HUSB238_5VCurrentContract)buffer;
    return ret;
}

/**
 * @brief   Checks if a specific voltage is detected.
 *
 * @param   dev object handle of husb238
 *
 * @details This function checks if a specific voltage is detected based on the PD selection.
 * It reads the 7th bit of the corresponding register to determine the status.
 */
bool husb238_is_voltage_detected(husb238_handle_t handle, HUSB238_PDSelection pd) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;

    uint8_t registerAddress;
    // Determine the register address based on the PD selection
    switch (pd) {
        case PD_SRC_5V:
            registerAddress = HUSB238_SRC_PDO_5V;
            break;
        case PD_SRC_9V:
            registerAddress = HUSB238_SRC_PDO_9V;
            break;
        case PD_SRC_12V:
            registerAddress = HUSB238_SRC_PDO_12V;
            break;
        case PD_SRC_15V:
            registerAddress = HUSB238_SRC_PDO_15V;
            break;
        case PD_SRC_18V:
            registerAddress = HUSB238_SRC_PDO_18V;
            break;
        case PD_SRC_20V:
            registerAddress = HUSB238_SRC_PDO_20V;
            break;
        default:
            return false;  // Unknown PD
    }

    uint8_t buffer;
    i2c_bus_read_bit(husb238->i2c_dev, registerAddress, 7, &buffer);
    // printf("husb238_is_voltage_detected buffer: %d\n", buffer);
    return buffer;
}

/**
 * @brief   Reads the detected current based on the PD output selection.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads the bottom four bits (0-3) of the corresponding
 * register based on the PD selection to get the detected current.
 * It returns the current as an HUSB238_CurrentSetting enum value.
 */
esp_err_t husb238_current_detected(husb238_handle_t handle, HUSB238_PDSelection pd, HUSB238_CurrentSetting *current) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;

    uint8_t registerAddress;

    // Determine the register address based on the PD selection
    switch (pd) {
        case PD_SRC_5V:
            registerAddress = HUSB238_SRC_PDO_5V;
            break;
        case PD_SRC_9V:
            registerAddress = HUSB238_SRC_PDO_9V;
            break;
        case PD_SRC_12V:
            registerAddress = HUSB238_SRC_PDO_12V;
            break;
        case PD_SRC_15V:
            registerAddress = HUSB238_SRC_PDO_15V;
            break;
        case PD_SRC_18V:
            registerAddress = HUSB238_SRC_PDO_18V;
            break;
        case PD_SRC_20V:
            registerAddress = HUSB238_SRC_PDO_20V;
            break;
        default:
            return CURRENT_0_5_A;  // we'll have to just return 0
    }

    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, registerAddress, 3, 4, &buffer);
    // printf("husb238_current_detected buffer: %d\n", buffer);
    *current = (HUSB238_CurrentSetting)buffer;
    return ret;
}

/**
 * @brief   Reads the source voltage from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads bits 4-7 of the HUSB238_PD_STATUS0 register to
 * get the source voltage. It returns the voltage as an HUSB238_VoltageSetting enum value.
 */
esp_err_t husb238_get_pd_src_voltage(husb238_handle_t handle, HUSB238_VoltageSetting *voltage) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_PD_STATUS0, 7, 4, &buffer);
    // printf("husb238_get_pd_src_voltage buffer: %d\n", buffer);
    *voltage = (HUSB238_VoltageSetting)buffer;
    return ret;
}

/**
 * @brief   Reads the source current from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads the bottom four bits (0-3) of the HUSB238_PD_STATUS0 register to get the source current.
 * It returns the current as an HUSB238_CurrentSetting enum value.
 */
esp_err_t husb238_get_pd_src_current(husb238_handle_t handle, HUSB238_CurrentSetting *current) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_PD_STATUS0, 3, 4, &buffer);
    // printf("husb238_get_pd_src_current buffer: %d\n", buffer);
    *current = (HUSB238_CurrentSetting)buffer;
    return ret;
}

/**
 * @brief   Gets the selected PDO.
 *
 * @param   dev object handle of husb238
 *
 * @details This function reads bits 4-7 of the SRC_PDO register to get the
 * selected PDO. It returns the PDO as an HUSB238_PDSelection enum value.
 */
esp_err_t husb238_get_selected_pd(husb238_handle_t handle, HUSB238_PDSelection *pd) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_SRC_PDO, 7, 4, &buffer);
    // printf("husb238_get_selected_pd buffer: %d\n", buffer);
    *pd = (HUSB238_PDSelection)buffer;
    return ret;
}

/**
 * @brief   Selects a PD output.
 *
 * @param   dev object handle of husb238
 *
 * @details This function writes to bits 4-7 of the SRC_PDO register to select a PD.
 */
esp_err_t husb238_select_pd(husb238_handle_t handle, HUSB238_PDSelection pd) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    esp_err_t ret = i2c_bus_write_bits(husb238->i2c_dev, HUSB238_SRC_PDO, 7, 4, pd);
    return ret;
}

/**
 * @brief   Resets the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function writes to the GO_COMMAND register to initiate a hard reset.
 * Specifically, it writes 0b10000 to the bottom 5 bits of the GO_COMMAND register.
 *
 * @see GO_COMMAND register in HUSB238 Register Information (Page 7)
 */
esp_err_t husb238_reset(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    esp_err_t ret = i2c_bus_write_bits(husb238->i2c_dev, HUSB238_GO_COMMAND, 4, 5, 0b10000);
    return ret;
}

/**
 * @brief   Requests Power Delivery (PD) from the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function writes to the GO_COMMAND register to request a PD contract.
 * Specifically, it writes 0b00001 to bits 0-1 of the GO_COMMAND register.
 *
 * @see GO_COMMAND register in HUSB238 Register Information (Page 7)
 */
esp_err_t husb238_request_pd(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    esp_err_t ret = i2c_bus_write_bits(husb238->i2c_dev, HUSB238_GO_COMMAND, 4, 5, 0b00001);
    return ret;
}

/**
 * @brief   Retrieves the source capabilities of the HUSB238 device.
 *
 * @param   dev object handle of husb238
 *
 * @details This function writes to the GO_COMMAND register to send out a Get_SRC_Cap
 * command. Specifically, it writes 0b00100 to the bottom 5 bits of the GO_COMMAND register.
 *
 * @see GO_COMMAND register in HUSB238 Register Information (Page 7)
 */
esp_err_t husb238_get_source_capabilities(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    esp_err_t ret = i2c_bus_write_bits(husb238->i2c_dev, HUSB238_GO_COMMAND, 4, 5, 0b00100);
    return ret;
}

uint8_t husb238_test(husb238_handle_t handle) {
    husb238_dev_t *husb238 = (husb238_dev_t *)handle;
    uint8_t buffer;
    esp_err_t ret = i2c_bus_read_bits(husb238->i2c_dev, HUSB238_PD_STATUS1, 7, 4, &buffer);
    printf("husb238_test buffer: %d %d\n", buffer, ret);
    return buffer;
}

const char *get_voltage_setting_string(HUSB238_VoltageSetting voltage) {
    switch (voltage) {
        case UNATTACHED:
            return "unattached";
        case PD_5V:
            return "5V";
        case PD_9V:
            return "9V";
        case PD_12V:
            return "12V";
        case PD_15V:
            return "15V";
        case PD_18V:
            return "18V";
        case PD_20V:
            return "20V";
        default:
            return "unknown";
    }
}

const char *get_current_setting_string(HUSB238_CurrentSetting current) {
    switch (current) {
        case CURRENT_0_5_A:
            return "0.5A";
        case CURRENT_0_7_A:
            return "0.7A";
        case CURRENT_1_0_A:
            return "1.0A";
        case CURRENT_1_25_A:
            return "1.25A";
        case CURRENT_1_5_A:
            return "1.5A";
        case CURRENT_1_75_A:
            return "1.75A";
        case CURRENT_2_0_A:
            return "2.0A";
        case CURRENT_2_25_A:
            return "2.25A";
        case CURRENT_2_50_A:
            return "2.50A";
        case CURRENT_2_75_A:
            return "2.75A";
        case CURRENT_3_0_A:
            return "3.0A";
        case CURRENT_3_25_A:
            return "3.25A";
        case CURRENT_3_5_A:
            return "3.5A";
        case CURRENT_4_0_A:
            return "4.0A";
        case CURRENT_4_5_A:
            return "4.5A";
        case CURRENT_5_0_A:
            return "5.0A";
        default:
            return "unknown";
    }
}

const char *get_5v_contract_current_string(HUSB238_5VCurrentContract current) {
    switch (current) {
        case CURRENT5V_DEFAULT:
            return "default current";
        case CURRENT5V_1_5_A:
            return "1.5A";
        case CURRENT5V_2_4_A:
            return "2.4A";
        case CURRENT5V_3_A:
            return "3A";
        default:
            return "unknown";
    }
}

const char *get_pd_selection_string(HUSB238_PDSelection pd) {
    switch (pd) {
        case PD_NOT_SELECTED:
            return "not selected";
        case PD_SRC_5V:
            return "5V";
        case PD_SRC_9V:
            return "9V";
        case PD_SRC_12V:
            return "12V";
        case PD_SRC_15V:
            return "15V";
        case PD_SRC_18V:
            return "18V";
        case PD_SRC_20V:
            return "20V";
        default:
            return "unknown";
    }
}

const char *get_pd_response_string(HUSB238_ResponseCodes response) {
    switch (response) {
        case NO_RESPONSE:
            return "no_response";
        case SUCCESS:
            return "success";
        case INVALID_CMD_OR_ARG:
            return "invalid_cmd_or_arg";
        case CMD_NOT_SUPPORTED:
            return "cmd_not_supported";
        case TRANSACTION_FAIL_NO_GOOD_CRC:
            return "transaction_fail_no_good_crc";
        default:
            return "unknown";
    }
}