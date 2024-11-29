#include "driver/i2c.h"
#include "i2c_bus.h"

typedef void *husb238_handle_t;

/** Default HUSB238 I2C address. */
#define HUSB238_I2CADDR_DEFAULT (0x08)  ///< I2C address

#define HUSB238_PD_STATUS0 0x00   ///< Register value for PD_STATUS0 in HUSB238
#define HUSB238_PD_STATUS1 0x01   ///< Register value for PD_STATUS1 in HUSB238
#define HUSB238_SRC_PDO_5V 0x02   ///< Register value for SRC_PDO_5V in HUSB238
#define HUSB238_SRC_PDO_9V 0x03   ///< Register value for SRC_PDO_9V in HUSB238
#define HUSB238_SRC_PDO_12V 0x04  ///< Register value for SRC_PDO_12V in HUSB238
#define HUSB238_SRC_PDO_15V 0x05  ///< Register value for SRC_PDO_15V in HUSB238
#define HUSB238_SRC_PDO_18V 0x06  ///< Register value for SRC_PDO_18V in HUSB238
#define HUSB238_SRC_PDO_20V 0x07  ///< Register value for SRC_PDO_20V in HUSB238
#define HUSB238_SRC_PDO 0x08      ///< Register value for SRC_PDO in HUSB238
#define HUSB238_GO_COMMAND 0x09   ///< Register value for GO_COMMAND in HUSB238

#define HUSB238_CC_STATUS_0 0X14
#define HUSB238_CC_STATUS_1 0X15
#define HUSB238_DPDM_STATUS 0X16

/**  Available currents per PD output */
typedef enum _husb_currents {
    CURRENT_0_5_A = 0b0000,   ///< 0.5A
    CURRENT_0_7_A = 0b0001,   ///< 0.7A
    CURRENT_1_0_A = 0b0010,   ///< 1.0A
    CURRENT_1_25_A = 0b0011,  ///< 1.25A
    CURRENT_1_5_A = 0b0100,   ///< 1.5A
    CURRENT_1_75_A = 0b0101,  ///< 1.75A
    CURRENT_2_0_A = 0b0110,   ///< 2.0A
    CURRENT_2_25_A = 0b0111,  ///< 2.25A
    CURRENT_2_50_A = 0b1000,  ///< 2.50A
    CURRENT_2_75_A = 0b1001,  ///< 2.75A
    CURRENT_3_0_A = 0b1010,   ///< 3.0A
    CURRENT_3_25_A = 0b1011,  ///< 3.25A
    CURRENT_3_5_A = 0b1100,   ///< 3.5A
    CURRENT_4_0_A = 0b1101,   ///< 4.0A
    CURRENT_4_5_A = 0b1110,   ///< 4.5A
    CURRENT_5_0_A = 0b1111    ///< 5.0A
} HUSB238_CurrentSetting;

/** Available voltages for PD output */
typedef enum _husb_voltages {
    UNATTACHED = 0b0000,  ///< Unattached
    PD_5V = 0b0001,       ///< 5V
    PD_9V = 0b0010,       ///< 9V
    PD_12V = 0b0011,      ///< 12V
    PD_15V = 0b0100,      ///< 15V
    PD_18V = 0b0101,      ///< 18V
    PD_20V = 0b0110       ///< 20V
} HUSB238_VoltageSetting;

/** Responses to PD query */
typedef enum _husb_response_codes {
    NO_RESPONSE = 0b000,                  ///< No response
    SUCCESS = 0b001,                      ///< Success
    INVALID_CMD_OR_ARG = 0b011,           ///< Invalid command or argument
    CMD_NOT_SUPPORTED = 0b100,            ///< Command not supported
    TRANSACTION_FAIL_NO_GOOD_CRC = 0b101  ///< Transaction fail. No GoodCRC is received after sending
} HUSB238_ResponseCodes;

/** Default 5V output current */
typedef enum _husb_5v_current_contract {
    CURRENT5V_DEFAULT = 0b00,  ///< Default current
    CURRENT5V_1_5_A = 0b01,    ///< 1.5A
    CURRENT5V_2_4_A = 0b10,    ///< 2.4A
    CURRENT5V_3_A = 0b11       ///< 3A
} HUSB238_5VCurrentContract;

/** What voltage is selected for PD sink */
typedef enum _husb_pd_selection {
    PD_NOT_SELECTED = 0b0000,  ///< Not selected
    PD_SRC_5V = 0b0001,        ///< SRC_PDO_5V
    PD_SRC_9V = 0b0010,        ///< SRC_PDO_9V
    PD_SRC_12V = 0b0011,       ///< SRC_PDO_12V
    PD_SRC_15V = 0b1000,       ///< SRC_PDO_15V
    PD_SRC_18V = 0b1001,       ///< SRC_PDO_18V
    PD_SRC_20V = 0b1010        ///< SRC_PDO_20V
} HUSB238_PDSelection;

/**************************************************************************/
/*!
    @brief  HUSB238 driver.
*/
/**************************************************************************/

husb238_handle_t husb238_create(i2c_bus_handle_t bus, uint8_t dev_addr);
esp_err_t husb238_delete(husb238_handle_t *husb238);

bool husb238_is_attached(husb238_handle_t husb238);
bool husb238_get_cc_direction(husb238_handle_t husb238);
esp_err_t husb238_get_pd_response(husb238_handle_t husb238, HUSB238_ResponseCodes *response);
bool husb238_get_5v_contract_voltage(husb238_handle_t husb238);
esp_err_t husb238_get_5v_contract_current(husb238_handle_t husb238, HUSB238_5VCurrentContract *current);
bool husb238_is_voltage_detected(husb238_handle_t husb238, HUSB238_PDSelection pd);
esp_err_t husb238_current_detected(husb238_handle_t husb238, HUSB238_PDSelection pd, HUSB238_CurrentSetting *current);
esp_err_t husb238_get_pd_src_voltage(husb238_handle_t husb238, HUSB238_VoltageSetting *voltage);
esp_err_t husb238_get_pd_src_current(husb238_handle_t husb238, HUSB238_CurrentSetting *current);
esp_err_t husb238_get_selected_pd(husb238_handle_t husb238, HUSB238_PDSelection *pd);

esp_err_t husb238_select_pd(husb238_handle_t husb238, HUSB238_PDSelection pd);
esp_err_t husb238_reset(husb238_handle_t husb238);
esp_err_t husb238_request_pd(husb238_handle_t husb238);
esp_err_t husb238_get_source_capabilities(husb238_handle_t husb238);

uint8_t husb238_test(husb238_handle_t husb238);

const char *get_voltage_setting_string(HUSB238_VoltageSetting voltage);
const char *get_current_setting_string(HUSB238_CurrentSetting current);
const char *get_5v_contract_current_string(HUSB238_5VCurrentContract current);
const char *get_pd_selection_string(HUSB238_PDSelection pd);
const char *get_pd_response_string(HUSB238_ResponseCodes response);
