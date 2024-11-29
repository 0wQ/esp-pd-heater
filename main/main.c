// LVGL SCROLL_ANIM_TIME -> managed_components/lvgl__lvgl/src/core/lv_obj_scroll.c

#include <math.h>

#include "Adafruit_HUSB238.h"
#include "bsp/esp-bsp.h"
#include "ch32x035_pd.h"
#include "driver/i2c_master.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c_bus.h"
#include "ina226.h"
#include "lis2dh12.h"
#include "lvgl.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pid_ctrl.h"
#include "ui/ui.h"

#define TAG "ESP-PD-HEATER"

// PIN
#define HEATER_GPIO GPIO_NUM_3

// I2C 配置
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ (400 * 1000)
static i2c_bus_handle_t i2c_bus = NULL;

// PWM 配置参数
#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_3
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT  // 占空比分辨率 10:1024 11:2048 12:4096 13:8192
#define LEDC_FREQUENCY (20 * 1000)       // PWM 频率, 单位 Hz

// 加热
static bool heating_on = false;               // 加热标志位
static uint32_t heating_duty = 0;             // 当前占空比
const uint32_t heating_duty_max = 1023;       // 最大占空比
static uint16_t pcb_resistance_at_20C = 195;  // 加热板 20℃ 时阻值 需要NVS存储
static uint8_t supply_max_power = 65;         // 电源最大功率 需要NVS存储
static bool is_temperature_reached = false;   // 开启加热后已到达温度预设点, 停止加热需设false
static uint32_t heating_start_time = 0;       // 加热开始时间 ms

// PT1000 ADC
#define ADC_CHAN ADC_CHANNEL_4
#define ADC_ATTEN ADC_ATTEN_DB_6
static float pt1000_temperature = -99.0f;
static uint16_t upper_divider_resistance = 3000;  // 分压电阻值 需要NVS存储
static int16_t adc_offset = 0;                    // esp adc校准 需要NVS存储
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// PID
static uint16_t pid_expect_temperature = 45;  // PID 期望温度 需要NVS存储
static pid_ctrl_block_handle_t pid_ctrl;
static pid_ctrl_parameter_t pid_params = {
    .kp = 150,  // 需要NVS存储
    .ki = 10,   // 需要NVS存储
    .kd = 5,    // 需要NVS存储
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
    .max_output = 0,
    .min_output = 0,
    .max_integral = 1000,
    .min_integral = -1000,
};

// HUSB238 & CH32X035 PD
// #define USE_CH32X035_PD 1
#ifdef USE_CH32X035_PD
static ch32x035_pd_handle_t ch32x035_pd = NULL;
#else
static char husb238_label_text[300];
static husb238_handle_t husb238 = NULL;
#endif

// INA226
static float ina226_bus_voltage, ina226_current, ina226_power, ina226_shunt_voltage;

// LIS2DH12
static lis2dh12_handle_t lis2dh12 = NULL;
static lis2dh12_acce_value_t lis2dh12_acce_value;
static float lis2dh12_tilt_angle;

// 芯片温度
static float chip_temperature = -99.0f;

// 蜂鸣器
static uint8_t buzzer_volume_percent = 100;  // 蜂鸣器音量百分比 需要NVS存储
static uint8_t buzzer_note = NOTE_A7;        // 蜂鸣器音符 需要NVS存储
// 屏幕背光
static uint8_t brightness = 70;  // 屏幕背光 需要NVS存储

// 数据记录
lv_chart_series_t *ui_Chart1_series_1 = NULL;
lv_chart_series_t *ui_Chart1_series_2 = NULL;
static bool data_record_need_restart = false;  // 重新记录标志位

// NVS
static bool nvs_need_save = false;  // 是否需要立即保存NVS变量
// Define NVS keys, max length: 14
#define NVS_NAMESPACE "storage"
#define NVS_KEY_PCB_RESISTANCE "pcbR"
#define NVS_KEY_SUPPLY_MAX_POWER "sMaxPwr"
#define NVS_KEY_PID_EXPECT_TEMP "pExpTmp"
#define NVS_KEY_PID_KP "pKp"
#define NVS_KEY_PID_KI "pKi"
#define NVS_KEY_PID_KD "pKd"
#define NVS_KEY_BUZZER_VOLUME "bVol"
#define NVS_KEY_BUZZER_NOTE "bNote"
#define NVS_KEY_BRIGHTNESS "br"
#define NVS_KEY_UPPER_DIVIDER_RESISTANCE "uDivR"
#define NVS_KEY_ADC_OFFSET "adcOffset"

// 倾倒提示
static lv_obj_t *tilt_msgbox = NULL;
static bool is_msgbox_shown = false;

void _pid_init(void);
void _adc_init(void);
void _i2c_init(void);
void update_pid(void);
double calculate_temperature_cvd(double resistance);
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
float power_limit(float supply_voltage, float max_power, float resistance_at_20C, float current_temperature);
void append_text(const char *additional_text);
void format_float_to_width(char *result_str, uint8_t width, float value, const char *unit);
uint32_t calculate_buzzer_volume(uint8_t percent);
void lvgl_widgets_values_init(void);
void app_lvgl_display(void);
static void close_msgbox_cb(void *arg);

void heating_task(void *pvParameter) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = HEATER_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);

    uint32_t _duty = 0;

    while (1) {
        if (heating_duty != _duty) {
            _duty = heating_duty;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, heating_on ? heating_duty : 0));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        }
        if (!heating_on) {
            heating_start_time = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void pd_task(void *pvParameter) {
#ifdef USE_CH32X035_PD
    ch32x035_pd = ch32x035_pd_create(i2c_bus, CH32X035_PD_ADDR);
    // 查询 id
    uint8_t device_id = 0;
    ch32x035_pd_get_device_id(ch32x035_pd, &device_id);
    ESP_LOGI(TAG, "CH32X035 DEVICE ID: %d", device_id);

    while (1) {
        // 申请 20v
        ch32x035_pd_request_voltage(ch32x035_pd, 20);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#else
    husb238 = husb238_create(i2c_bus, HUSB238_I2CADDR_DEFAULT);

    HUSB238_PDSelection supply_pd_max = PD_NOT_SELECTED;
    uint8_t supply_pd_max_applied_count = 0;

    while (1) {
        strcpy(husb238_label_text, "");

        append_text("----- STATUS -----\n");
        bool is_attached = husb238_is_attached(husb238);
        append_text(is_attached ? "HUSB238 attached.\n" : "HUSB238 unattached!\n");

        if (is_attached) {
            append_text(husb238_get_cc_direction(husb238) ? "CC2 connected.\n" : "CC1 connected.\n");
        }

        append_text("----- PD SRC -----\n");
        // 当前电压
        HUSB238_VoltageSetting src_voltage;
        husb238_get_pd_src_voltage(husb238, &src_voltage);
        append_text(get_voltage_setting_string(src_voltage));
        append_text(" - ");
        // 当前电流
        HUSB238_CurrentSetting src_current;
        husb238_get_pd_src_current(husb238, &src_current);
        append_text(get_current_setting_string(src_current));
        append_text("\n");

        if (is_attached) {
            append_text("------ PDO -------\n");
            HUSB238_PDSelection pd_selection_list[] = {PD_SRC_5V, PD_SRC_9V, PD_SRC_12V, PD_SRC_15V, PD_SRC_18V, PD_SRC_20V};
            for (size_t i = 0; i < sizeof(pd_selection_list) / sizeof(pd_selection_list[0]); i++) {
                bool is_voltage_detected = husb238_is_voltage_detected(husb238, pd_selection_list[i]);
                append_text(get_pd_selection_string(pd_selection_list[i]));
                append_text(is_voltage_detected ? " Y " : " N ");

                if (is_voltage_detected) {
                    supply_pd_max = pd_selection_list[i];
                    HUSB238_CurrentSetting currentDetected;
                    husb238_current_detected(husb238, pd_selection_list[i], &currentDetected);
                    append_text(get_current_setting_string(currentDetected));
                }
                append_text("\n");
            }
        }

        append_text("------- 5V -------\n");
        bool is_5v_voltage_detected = husb238_get_5v_contract_voltage(husb238);
        append_text(is_5v_voltage_detected ? "5V - " : "other voltage - ");
        HUSB238_5VCurrentContract _5v_contract_current;
        husb238_get_5v_contract_current(husb238, &_5v_contract_current);
        append_text(get_5v_contract_current_string(_5v_contract_current));
        append_text("\n");

        append_text("----- PD MAX -----\n");
        append_text(get_pd_selection_string(supply_pd_max));
        append_text("\n");

        // 申请最大挡位
        if (supply_pd_max != PD_NOT_SELECTED && supply_pd_max_applied_count <= 3) {
            husb238_select_pd(husb238, supply_pd_max);
            husb238_request_pd(husb238);
            supply_pd_max_applied_count++;
        }

        append_text("----- PD RES -----\n");
        HUSB238_ResponseCodes pd_response;
        husb238_get_pd_response(husb238, &pd_response);
        append_text(get_pd_response_string(pd_response));
        // append_text("\n");

        bsp_display_lock(0);
        lv_label_set_text(ui_LabelPDInfo, husb238_label_text);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(supply_pd_max_applied_count <= 3 ? 1000 : 5000));
    }
#endif
}

void ina226_task(void *pvParameter) {
    ina226_init(i2c_bus, 0x40);
    ina226_calibrate(0.01f, 8);  // 分流电阻 0.01Ω, 最大电流 10A
    ina226_configure(INA226_PERIOD_1100us, INA226_AVERAGE_64);

    char bus_voltage_str[10], current_str[10], power_str[10];

    while (1) {
        ina226_read(&ina226_bus_voltage, &ina226_current, &ina226_power, &ina226_shunt_voltage);
        ina226_current = fabs(ina226_current);
        // ESP_LOGI(TAG, "Bus: %fV, Current: %fA, Power: %fW, Shunt: %fV\n", ina226_bus_voltage, ina226_current, ina226_power, ina226_shunt_voltage);

        // 忽略较小电流
        if (!heating_on || heating_duty == 0) {
            if (ina226_current < 0.02f) {
                ina226_current = 0;
                ina226_power = 0;
            }
        }

        format_float_to_width(bus_voltage_str, 5, ina226_bus_voltage, "V");
        format_float_to_width(current_str, 5, ina226_current, "A");
        format_float_to_width(power_str, 5, ina226_power, "W");

        // 当前电压电流功率控件
        bsp_display_lock(0);
        lv_label_set_text(ui_LabelRealVoltage, bus_voltage_str);
        lv_label_set_text(ui_LabelRealCurrent, current_str);
        lv_label_set_text(ui_LabelRealPower, power_str);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(heating_on ? 80 : 200));
    }
}

void lis2dh12_task(void *pvParameter) {
    lis2dh12 = lis2dh12_create(i2c_bus, LIS2DH12_I2C_ADDRESS);

    uint8_t deviceid;
    lis2dh12_get_deviceid(lis2dh12, &deviceid);
    ESP_LOGI(TAG, "LIS2DH12 device id is: %02x", deviceid);

    if (deviceid != 0x33) {
        ESP_LOGE(TAG, "LIS2DH12 device id is not 0x33, please check the connection");
        // 停止任务
        vTaskDelete(NULL);
    }

    // 配置传感器参数
    lis2dh12_config_t lis2dh12_config;
    lis2dh12_get_config(lis2dh12, &lis2dh12_config);

    // 禁用温度传感器以节省功耗
    lis2dh12_config.temp_enable = LIS2DH12_TEMP_DISABLE;
    // 降低采样率到25Hz，足够检测人工倾倒动作
    lis2dh12_config.odr = LIS2DH12_ODR_25HZ;
    // 使用正常功耗模式以获得更好的精度
    lis2dh12_config.opt_mode = LIS2DH12_OPT_NORMAL;
    // 启用所有轴以检测任意方向的倾倒
    lis2dh12_config.z_enable = LIS2DH12_ENABLE;
    lis2dh12_config.y_enable = LIS2DH12_ENABLE;
    lis2dh12_config.x_enable = LIS2DH12_ENABLE;
    // 禁用BDU以获得连续的数据流
    lis2dh12_config.bdu_status = LIS2DH12_DISABLE;
    // 设置±2g量程 - 足够检测倾倒
    lis2dh12_config.fs = LIS2DH12_FS_2G;

    lis2dh12_set_config(lis2dh12, &lis2dh12_config);

    const float TILT_THRESHOLD = 180.0f - 45.0f;  // 倾倒阈值角度
    bool is_tilted = false;

    while (1) {
        // 读取数据
        if (lis2dh12_get_acce(lis2dh12, &lis2dh12_acce_value) == ESP_OK) {
            // 计算与垂直方向的倾角
            lis2dh12_tilt_angle = acos(lis2dh12_acce_value.acce_z / sqrt(lis2dh12_acce_value.acce_x * lis2dh12_acce_value.acce_x + lis2dh12_acce_value.acce_y * lis2dh12_acce_value.acce_y + lis2dh12_acce_value.acce_z * lis2dh12_acce_value.acce_z)) * 180.0f / M_PI;

            // 更新倾倒状态
            bool new_tilt_state = (lis2dh12_tilt_angle < TILT_THRESHOLD);
            if (new_tilt_state != is_tilted) {
                is_tilted = new_tilt_state;  // 更新倾倒状态

                // 倾倒时停止加热并报警
                if (is_tilted && heating_on) {
                    // 停止加热
                    heating_on = false;

                    // 更新 lvgl 控件
                    bsp_display_lock(0);
                    lv_obj_clear_state(ui_ButtonHeatingToggle, LV_STATE_CHECKED);
                    lv_label_set_text(ui_LabelHeatingToggle, "START");
                    lv_obj_set_style_text_decor(ui_LabelHeatingToggle, LV_TEXT_DECOR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);

                    // 如果没有显示消息框，则创建并显示
                    if (!is_msgbox_shown) {
                        tilt_msgbox = lv_msgbox_create(NULL, "WARNING", "TILT DETECTED, HEATING STOPPED!", NULL, false);
                        lv_obj_center(tilt_msgbox);
                        is_msgbox_shown = true;

                        // 自动关闭消息框
                        static esp_timer_handle_t msgbox_timer;
                        esp_timer_create_args_t timer_args = {
                            .callback = &close_msgbox_cb,
                            .name = "msgbox_timer",
                        };
                        esp_timer_create(&timer_args, &msgbox_timer);
                        esp_timer_start_once(msgbox_timer, 5000000);  // 5s
                    }
                    bsp_display_unlock();

                    // 发出警报
                    beep(buzzer_note, calculate_buzzer_volume(buzzer_volume_percent < 1 ? 1 : buzzer_volume_percent), 0.1, 0.02, 20);
                }
            }
            // ESP_LOGI("LIS2DH12", "Accel: X=%.2fg Y=%.2fg Z=%.2fg Tilt=%.1f°", acce_value.acce_x, acce_value.acce_y, acce_value.acce_z, tilt_angle);
        } else {
            ESP_LOGE("LIS2DH12", "Failed to get acceleration data");
        }

        vTaskDelay(pdMS_TO_TICKS(200));  // 降低到5Hz检测频率
    }
}

void pt1000_task(void *pvParameter) {
    _adc_init();

    int adc_raw_value;
    int voltage;

    const size_t sample_count = 128;            // 采样次数
    double voltage_average = 0.0f;              // 电压平均值
    double filtered_voltage = 0.0f;             // 低通滤波后的电压
    const float low_pass_filter_alpha = 0.01f;  // 低通滤波系数

    double pt1000_resistance;     // PT1000 阻值
    float pid_temperature_error;  // PID 温差
    float pid_output_value;
    float power_limit_value;

    char duty_cycle_buffer[10];
    char temperature_buffer[10];

    bool is_first_sample = true;

    while (1) {
        voltage_average = 0;
        for (size_t i = 0; i < sample_count; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHAN, &adc_raw_value));
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw_value, &voltage));

            // 低通滤波，第一次直接使用采样电压值
            if (is_first_sample) {
                filtered_voltage = voltage;
                is_first_sample = false;
            } else {
                filtered_voltage = low_pass_filter_alpha * voltage + (1 - low_pass_filter_alpha) * filtered_voltage;
            }
            // 累加低通滤波后的电压
            voltage_average += filtered_voltage;
        }
        // 均值滤波
        voltage_average /= sample_count;

        // adc 偏移量校准
        voltage_average += adc_offset;

        // 转换温度, 上分压电阻2-10k, 基准电压源3.0V
        pt1000_resistance = (voltage_average * upper_divider_resistance) / (3000.0f - voltage_average);
        // ESP_LOGI(TAG, "PT1000: %.2fR %.2fmV", pt1000_resistance, voltage_average);
        pt1000_temperature = calculate_temperature_cvd(pt1000_resistance);

        // 计算 PID 误差
        pid_temperature_error = (float)pid_expect_temperature - pt1000_temperature;

        if (heating_on) {
            // 计算限制最大功率
            power_limit_value = power_limit(ina226_bus_voltage, supply_max_power, pcb_resistance_at_20C / 100.0f, pt1000_temperature);
            pid_params.max_output = (float)heating_duty_max * power_limit_value;
            update_pid();
            // PID 计算占空比, 目标温度可以抬高一点
            pid_compute(pid_ctrl, pid_temperature_error, &pid_output_value);
            heating_duty = round(pid_output_value);
        } else {
            heating_duty = 0;
        }

        // 占空比控件
        float duty_cycle_percent = heating_duty == 0 ? 0 : (float)heating_duty * 100 / (float)heating_duty_max;
        format_float_to_width(duty_cycle_buffer, 5, duty_cycle_percent, "%");
        // 铝基板温度控件
        sprintf(temperature_buffer, "%05.1f℃", pt1000_temperature);
        bsp_display_lock(0);
        lv_label_set_text(ui_LabelRealTemp, temperature_buffer);
        lv_label_set_text(ui_LabelRealDuty, duty_cycle_buffer);
        bsp_display_unlock();

        // 开启后第一次到达温度预设点, 蜂鸣器响一下
        if (heating_on && !is_temperature_reached && (pid_temperature_error >= -1 && pid_temperature_error <= 0)) {
            is_temperature_reached = true;
            beep(buzzer_note, calculate_buzzer_volume(buzzer_volume_percent), 1, 0, 1);
        }
        is_temperature_reached = heating_on && is_temperature_reached;

        vTaskDelay(pdMS_TO_TICKS(heating_on ? 100 : 200));
    }
}

void data_record_task(void *pvParameter) {
    bool data_record_is_running = false;

    while (1) {
        // 检测是否需要重置并重新开始记录
        if (data_record_need_restart) {
            data_record_need_restart = false;
            data_record_is_running = true;

            // 重置表格数据
            bsp_display_lock(0);
            lv_chart_set_all_value(ui_Chart1, ui_Chart1_series_1, LV_CHART_POINT_NONE);
            lv_chart_set_all_value(ui_Chart1, ui_Chart1_series_2, LV_CHART_POINT_NONE);
            bsp_display_unlock();
            ESP_LOGI(TAG, "Chart data reset and restarted");
        }

        if (heating_on && data_record_is_running) {
            for (size_t i = 0; i < 121; i++) {
                // 更新图表值
                bsp_display_lock(0);
                lv_chart_set_value_by_id(ui_Chart1, ui_Chart1_series_1, i, round(pt1000_temperature));
                lv_chart_set_value_by_id(ui_Chart1, ui_Chart1_series_2, i, round(ina226_current * 100));
                bsp_display_unlock();
                vTaskDelay(pdMS_TO_TICKS(1000));
                // 当停止加热 或者 需要重新记录时跳出
                if (!heating_on || data_record_need_restart) break;
            }
            data_record_is_running = false;
            ESP_LOGI(TAG, "Save temp data done");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void buzzer_start_task(void *pvParameter) {
    uint32_t vol = calculate_buzzer_volume(buzzer_volume_percent);
    bsp_buzzer_set_default(buzzer_note, vol);
    bsp_buzzer_init();
    vTaskDelay(pdMS_TO_TICKS(250));

    uint8_t note_1 = (buzzer_note + 6) > 88 ? 82 : buzzer_note;
    uint8_t note_2 = note_1 + 3;
    uint8_t note_3 = note_1 + 6;

    bsp_buzzer_beep(note_1, vol, 0.2, 0.02, 1);
    bsp_buzzer_beep(note_2, vol, 0.2, 0.02, 1);
    bsp_buzzer_beep(note_3, vol, 0.2, 0.02, 1);
    vTaskDelete(NULL);
}

void backlight_start_task(void *pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(200));
    bsp_display_brightness_set(brightness, 2000);
    vTaskDelete(NULL);
}

void chip_temp_task(void *pvParameter) {
    ESP_LOGI(TAG, "Install temperature sensor");
    temperature_sensor_handle_t temp_sensor = NULL;
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    ESP_LOGI(TAG, "Enable temperature sensor");
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

    while (1) {
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &chip_temperature));

        // 芯片过热主动停止加热 蜂鸣器持续强制发声报警
        if (chip_temperature > 60) {
            heating_on = false;
            beep(buzzer_note, calculate_buzzer_volume(buzzer_volume_percent < 1 ? 1 : buzzer_volume_percent), 0.1, 0.05, 3);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void display_debug_task(void *pvParameter) {
    char debug_buffer[50];

    while (1) {
        // 屏幕上显示调试信息
        sprintf(debug_buffer, "CHIP: %.1f℃ HEAT: %.1f℃ TILT: %.2f°", chip_temperature, pt1000_temperature, lis2dh12_tilt_angle);
        bsp_display_lock(0);
        lv_label_set_text(ui_LabelDebug, debug_buffer);
        bsp_display_unlock();

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void save_nvs_variables(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_u16(nvs_handle, NVS_KEY_PCB_RESISTANCE, pcb_resistance_at_20C);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set pcb_resistance_at_20C, %d", err);

        err = nvs_set_u8(nvs_handle, NVS_KEY_SUPPLY_MAX_POWER, supply_max_power);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set supply_max_power, %d", err);

        err = nvs_set_u16(nvs_handle, NVS_KEY_PID_EXPECT_TEMP, pid_expect_temperature);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set pid_expect_temperature, %d", err);

        err = nvs_set_u16(nvs_handle, NVS_KEY_PID_KP, pid_params.kp);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set pid_kp, %d", err);

        err = nvs_set_u16(nvs_handle, NVS_KEY_PID_KI, pid_params.ki);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set pid_ki, %d", err);

        err = nvs_set_u16(nvs_handle, NVS_KEY_PID_KD, pid_params.kd);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set pid_kd, %d", err);

        err = nvs_set_u8(nvs_handle, NVS_KEY_BUZZER_VOLUME, buzzer_volume_percent);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set buzzer_volume_percent, %d", err);

        err = nvs_set_u8(nvs_handle, NVS_KEY_BUZZER_NOTE, buzzer_note);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set buzzer_note, %d", err);

        err = nvs_set_u8(nvs_handle, NVS_KEY_BRIGHTNESS, brightness);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set brightness, %d", err);

        err = nvs_set_u16(nvs_handle, NVS_KEY_UPPER_DIVIDER_RESISTANCE, upper_divider_resistance);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set upper_divider_resistance, %d", err);

        err = nvs_set_i16(nvs_handle, NVS_KEY_ADC_OFFSET, adc_offset);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to set adc_offset, %d", err);

        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to commit, %d", err);

        nvs_close(nvs_handle);

        ESP_LOGI("NVS", "NVS variables saved");
    } else {
        ESP_LOGE("NVS", "Failed to open NVS for saving variables");
    }
}

void load_nvs_variables(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_get_u16(nvs_handle, NVS_KEY_PCB_RESISTANCE, &pcb_resistance_at_20C);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get pcb_resistance_at_20C, %d", err);

        err = nvs_get_u8(nvs_handle, NVS_KEY_SUPPLY_MAX_POWER, &supply_max_power);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get supply_max_power, %d", err);

        err = nvs_get_u16(nvs_handle, NVS_KEY_PID_EXPECT_TEMP, &pid_expect_temperature);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get pid_expect_temperature, %d", err);

        uint16_t kp, ki, kd;

        err = nvs_get_u16(nvs_handle, NVS_KEY_PID_KP, &kp);
        if (err == ESP_OK) {
            pid_params.kp = kp;
        } else {
            ESP_LOGE("NVS", "Failed to get pid_kp, %d", err);
        }

        err = nvs_get_u16(nvs_handle, NVS_KEY_PID_KI, &ki);
        if (err == ESP_OK) {
            pid_params.ki = ki;
        } else {
            ESP_LOGE("NVS", "Failed to get pid_ki, %d", err);
        }

        err = nvs_get_u16(nvs_handle, NVS_KEY_PID_KD, &kd);
        if (err == ESP_OK) {
            pid_params.kd = kd;
        } else {
            ESP_LOGE("NVS", "Failed to get pid_kd, %d", err);
        }

        err = nvs_get_u8(nvs_handle, NVS_KEY_BUZZER_VOLUME, &buzzer_volume_percent);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get buzzer_volume_percent, %d", err);

        err = nvs_get_u8(nvs_handle, NVS_KEY_BUZZER_NOTE, &buzzer_note);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get buzzer_note, %d", err);

        err = nvs_get_u8(nvs_handle, NVS_KEY_BRIGHTNESS, &brightness);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get brightness, %d", err);

        err = nvs_get_u16(nvs_handle, NVS_KEY_UPPER_DIVIDER_RESISTANCE, &upper_divider_resistance);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get upper_divider_resistance, %d", err);

        err = nvs_get_i16(nvs_handle, NVS_KEY_ADC_OFFSET, &adc_offset);
        if (err != ESP_OK) ESP_LOGE("NVS", "Failed to get adc_offset, %d", err);

        nvs_close(nvs_handle);
    } else {
        ESP_LOGE("NVS", "Failed to open NVS for loading variables");
    }
}

void app_main(void) {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 加载 NVS 参数
    load_nvs_variables();

    _i2c_init();
    _pid_init();

    xTaskCreate(buzzer_start_task, "buzzer_start_task", 2048, NULL, 10, NULL);

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Add and show objects on display */
    app_lvgl_display();

    /* Turn on display backlight */
    xTaskCreate(backlight_start_task, "backlight_start_task", 2048, NULL, 3, NULL);

    // Tasks
    xTaskCreate(pt1000_task, "pt1000_task", 4096, NULL, 1, NULL);
    xTaskCreate(ina226_task, "ina226_task", 2048, NULL, 3, NULL);
    xTaskCreate(heating_task, "heating_task", 4096, NULL, 2, NULL);
    xTaskCreate(pd_task, "pd_task", 2048, NULL, 10, NULL);
    xTaskCreate(data_record_task, "data_record_task", 2048, NULL, 8, NULL);
    xTaskCreate(chip_temp_task, "chip_temp_task", 2048, NULL, 10, NULL);
    xTaskCreate(lis2dh12_task, "lis2dh12_task", 4096, NULL, 5, NULL);
    xTaskCreate(display_debug_task, "display_debug_task", 2048, NULL, 20, NULL);

    // 初始化控件值
    lvgl_widgets_values_init();

    // 定时保存 NVS 参数
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (nvs_need_save) {
            save_nvs_variables();
            nvs_need_save = false;
        }
    }
}

void update_pid(void) {
    pid_update_parameters(pid_ctrl, &pid_params);
    // ESP_LOGI("PID", "update -> kp: %.1f, ki: %.1f, kd: %.1f", pid_params.kp, pid_params.ki, pid_params.kd);
}

static void button_heating_toggle_event_cb(lv_event_t *e) {
    lv_obj_t *btn = lv_event_get_target(e);

    // // 电压小于 6V 禁止开启加热
    // if (ina226_bus_voltage < 6.0f) {
    //     heating_on = false;
    //     bsp_display_lock(0);
    //     lv_obj_clear_state(btn, LV_STATE_CHECKED);
    //     lv_obj_set_style_text_decor(ui_LabelHeatingToggle, LV_TEXT_DECOR_STRIKETHROUGH, LV_PART_MAIN | LV_STATE_DEFAULT);
    //     bsp_display_unlock();
    //     return;
    // }

    heating_on = lv_obj_has_state(btn, LV_STATE_CHECKED);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelHeatingToggle, heating_on ? "STOP" : "START");
    lv_obj_set_style_text_decor(ui_LabelHeatingToggle, LV_TEXT_DECOR_NONE, LV_PART_MAIN | LV_STATE_DEFAULT);
    bsp_display_unlock();
    data_record_need_restart = heating_on;

    if (heating_on) {
        heating_start_time = esp_timer_get_time() / 1000;
    }
}

static void slider_set_temp_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    pid_expect_temperature = lv_slider_get_value(slider);
    nvs_need_save = true;
    update_pid();

    char buf_set_temp[5];
    sprintf(buf_set_temp, "%d℃", pid_expect_temperature);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetTemp, buf_set_temp);
    bsp_display_unlock();
}

static void slider_set_max_power_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    supply_max_power = lv_slider_get_value(slider);
    nvs_need_save = true;

    char buf_set_max_powerower[5];
    sprintf(buf_set_max_powerower, "%dW", supply_max_power);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetMaxPower, buf_set_max_powerower);
    bsp_display_unlock();
}

static void slider_set_rpcb_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    pcb_resistance_at_20C = lv_slider_get_value(slider);
    nvs_need_save = true;

    char buf_set_pcb_r[5];
    sprintf(buf_set_pcb_r, "%.2fR", pcb_resistance_at_20C / 100.0f);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetRPcb, buf_set_pcb_r);
    bsp_display_unlock();
}

static void slider_set_rdiv_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    upper_divider_resistance = lv_slider_get_value(slider);
    nvs_need_save = true;

    char buf_set_rdiv[5];
    sprintf(buf_set_rdiv, "%dR", upper_divider_resistance);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetRDiv, buf_set_rdiv);
    bsp_display_unlock();
}

static void slider_set_adc_offset_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    adc_offset = lv_slider_get_value(slider);
    nvs_need_save = true;

    char buf_set_adcoffset[10];
    sprintf(buf_set_adcoffset, "%+dmV", adc_offset);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetADCOffset, buf_set_adcoffset);
    bsp_display_unlock();
}

static void slider_set_bl_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    brightness = lv_slider_get_value(slider);
    nvs_need_save = true;
    bsp_display_brightness_set(brightness, 0);

    char buf_set_bl[5];
    sprintf(buf_set_bl, "%d%%", brightness);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetBL, buf_set_bl);
    bsp_display_unlock();
}

static void slider_set_vol_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    buzzer_volume_percent = lv_slider_get_value(slider);
    nvs_need_save = true;
    bsp_buzzer_set_default(buzzer_note, calculate_buzzer_volume(buzzer_volume_percent));

    char buf_set_vol[5];
    sprintf(buf_set_vol, "%d%%", buzzer_volume_percent);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetVol, buf_set_vol);
    bsp_display_unlock();
}

static void slider_set_note_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    buzzer_note = lv_slider_get_value(slider);
    nvs_need_save = true;
    bsp_buzzer_set_default(buzzer_note, calculate_buzzer_volume(buzzer_volume_percent));

    char buf_set_note[5];
    sprintf(buf_set_note, "%d", buzzer_note);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetNote, buf_set_note);
    bsp_display_unlock();
}

static void slider_set_kp_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    uint16_t value = lv_slider_get_value(slider);
    pid_params.kp = value;
    nvs_need_save = true;
    update_pid();

    char buf_set_kp[5];
    sprintf(buf_set_kp, "%d", value);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetKp, buf_set_kp);
    bsp_display_unlock();
}

static void slider_set_ki_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    uint16_t value = lv_slider_get_value(slider);
    pid_params.ki = value;
    nvs_need_save = true;
    update_pid();

    char buf_set_ki[5];
    sprintf(buf_set_ki, "%d", value);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetKi, buf_set_ki);
    bsp_display_unlock();
}

static void slider_set_kd_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    uint16_t value = lv_slider_get_value(slider);
    pid_params.kd = value;
    nvs_need_save = true;
    update_pid();

    char buf_set_kd[5];
    sprintf(buf_set_kd, "%d", value);
    bsp_display_lock(0);
    lv_label_set_text(ui_LabelSetKd, buf_set_kd);
    bsp_display_unlock();
}

static void chart_draw_event_cb(lv_event_t *e) {
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (!lv_obj_draw_part_check_type(dsc, &lv_chart_class, LV_CHART_DRAW_PART_TICK_LABEL))
        return;
    if (dsc->text == NULL)
        return;

    if (dsc->id == LV_CHART_AXIS_PRIMARY_X) {
        const char *px[] = {"0", "", "20", "", "40", "", "60", "", "80", "", "100", "", "120"};
        lv_snprintf(dsc->text, dsc->text_length, "%s", px[dsc->value]);
    }
    if (dsc->id == LV_CHART_AXIS_SECONDARY_Y) {
        lv_snprintf(dsc->text, dsc->text_length, "%d", dsc->value / 100);
    }
}

static void chart_clicked_event_cb(lv_event_t *e) {
    data_record_need_restart = true;
}

static void roller_pd_select_event_cb(lv_event_t *e) {
    lv_obj_t *roller = lv_event_get_target(e);
    uint16_t index = lv_roller_get_selected(roller);
    ESP_LOGI(TAG, "pd select: %d", index);

#ifdef USE_CH32X035_PD
    // CH32X035 诱骗电压
    switch (index) {
        case 0:
            ch32x035_pd_request_voltage(ch32x035_pd, 5);
            break;
        case 1:
            ch32x035_pd_request_voltage(ch32x035_pd, 9);
            break;
        case 2:
            ch32x035_pd_request_voltage(ch32x035_pd, 12);
            break;
        case 3:
            ch32x035_pd_request_voltage(ch32x035_pd, 15);
            break;
        case 4:
            ch32x035_pd_request_voltage(ch32x035_pd, 18);
            break;
        case 5:
            ch32x035_pd_request_voltage(ch32x035_pd, 20);
            break;
        default:
            ch32x035_pd_request_voltage(ch32x035_pd, 5);
            break;
    }
#else
    // HUSB238 诱骗电压
    switch (index) {
        case 0:
            husb238_select_pd(husb238, PD_SRC_5V);
            break;
        case 1:
            husb238_select_pd(husb238, PD_SRC_9V);
            break;
        case 2:
            husb238_select_pd(husb238, PD_SRC_12V);
            break;
        case 3:
            husb238_select_pd(husb238, PD_SRC_15V);
            break;
        case 4:
            husb238_select_pd(husb238, PD_SRC_18V);
            break;
        case 5:
            husb238_select_pd(husb238, PD_SRC_20V);
            break;
        default:
            husb238_select_pd(husb238, PD_NOT_SELECTED);
            break;
    }
    husb238_request_pd(husb238);
#endif
}

void lvgl_widgets_values_init(void) {
    char buf_set_temp[5], buf_set_max_power[5], buf_set_pcb_r[8], buf_set_rdiv[8], buf_set_adcoffset[8];
    char buf_set_bl[5], buf_set_vol[5], buf_set_note[5];
    char buf_set_kp[5], buf_set_ki[5], buf_set_kd[5];

    sprintf(buf_set_temp, "%d℃", pid_expect_temperature);
    sprintf(buf_set_max_power, "%dW", supply_max_power);
    sprintf(buf_set_pcb_r, "%.2fR", pcb_resistance_at_20C / 100.0f);
    sprintf(buf_set_rdiv, "%dR", upper_divider_resistance);
    sprintf(buf_set_adcoffset, "%+dmV", adc_offset);

    sprintf(buf_set_bl, "%d%%", brightness);
    sprintf(buf_set_vol, "%d%%", buzzer_volume_percent);
    sprintf(buf_set_note, "%d", buzzer_note);

    sprintf(buf_set_kp, "%d", (int)pid_params.kp);
    sprintf(buf_set_ki, "%d", (int)pid_params.ki);
    sprintf(buf_set_kd, "%d", (int)pid_params.kd);

    bsp_display_lock(0);

    lv_label_set_text(ui_LabelSetTemp, buf_set_temp);
    lv_slider_set_value(ui_SliderSetTemp, pid_expect_temperature, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetMaxPower, buf_set_max_power);
    lv_slider_set_value(ui_SliderSetMaxPower, supply_max_power, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetRPcb, buf_set_pcb_r);
    lv_slider_set_value(ui_SliderSetRPcb, pcb_resistance_at_20C, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetRDiv, buf_set_rdiv);
    lv_slider_set_value(ui_SliderSetRDiv, upper_divider_resistance, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetADCOffset, buf_set_adcoffset);
    lv_slider_set_value(ui_SliderSetADCOffset, adc_offset, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetBL, buf_set_bl);
    lv_slider_set_value(ui_SliderSetBL, brightness, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetVol, buf_set_vol);
    lv_slider_set_value(ui_SliderSetVol, buzzer_volume_percent, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetNote, buf_set_note);
    lv_slider_set_value(ui_SliderSetNote, buzzer_note, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetKp, buf_set_kp);
    lv_slider_set_value(ui_SliderSetKp, pid_params.kp, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetKi, buf_set_ki);
    lv_slider_set_value(ui_SliderSetKi, pid_params.ki, LV_ANIM_OFF);

    lv_label_set_text(ui_LabelSetKd, buf_set_kd);
    lv_slider_set_value(ui_SliderSetKd, pid_params.kd, LV_ANIM_OFF);

    bsp_display_unlock();
}

void app_lvgl_display(void) {
    bsp_display_lock(0);

    lv_group_t *g = lv_group_create();
    lv_indev_set_group(bsp_display_get_input_dev(), g);
    lv_group_set_wrap(g, false);  // 禁用循环滚动

    ui_init();

    lv_disp_t *dispp = lv_disp_get_default();
    // f784b6
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_color_hex(0x76DAFF), lv_color_hex(0xF784B6), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);

    lv_obj_set_scroll_snap_x(ui_Screen1, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_snap_y(ui_Screen1, LV_SCROLL_SNAP_CENTER);

    // Page -1
    lv_chart_set_update_mode(ui_Chart1, LV_CHART_UPDATE_MODE_SHIFT);
    ui_Chart1_series_1 = lv_chart_add_series(ui_Chart1, lv_color_hex(0x85BFD5), LV_CHART_AXIS_PRIMARY_Y);
    ui_Chart1_series_2 = lv_chart_add_series(ui_Chart1, lv_color_hex(0xF784B6), LV_CHART_AXIS_SECONDARY_Y);
    lv_obj_add_event_cb(ui_Chart1, chart_draw_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
    lv_group_add_obj(g, ui_ButtonChartRestart);
    lv_obj_add_event_cb(ui_ButtonChartRestart, chart_clicked_event_cb, LV_EVENT_CLICKED, NULL);
    // Page 0
    lv_group_add_obj(g, ui_ButtonHeatingToggle);
    lv_obj_add_event_cb(ui_ButtonHeatingToggle, button_heating_toggle_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_scroll_to_view_recursive(ui_ButtonHeatingToggle, LV_ANIM_OFF);
    lv_group_focus_obj(ui_ButtonHeatingToggle);
    // Page 1
    lv_group_add_obj(g, ui_SliderSetKp);
    lv_obj_add_event_cb(ui_SliderSetKp, slider_set_kp_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetKp, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetKi);
    lv_obj_add_event_cb(ui_SliderSetKi, slider_set_ki_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetKi, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetKd);
    lv_obj_add_event_cb(ui_SliderSetKd, slider_set_kd_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetKd, NULL, LV_STATE_EDITED);
    // Page 2
    lv_group_add_obj(g, ui_SliderSetTemp);
    lv_obj_add_event_cb(ui_SliderSetTemp, slider_set_temp_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetTemp, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetMaxPower);
    lv_obj_add_event_cb(ui_SliderSetMaxPower, slider_set_max_power_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetMaxPower, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetRPcb);
    lv_obj_add_event_cb(ui_SliderSetRPcb, slider_set_rpcb_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetRPcb, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetRDiv);
    lv_obj_add_event_cb(ui_SliderSetRDiv, slider_set_rdiv_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetRDiv, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetADCOffset);
    lv_obj_add_event_cb(ui_SliderSetADCOffset, slider_set_adc_offset_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetADCOffset, NULL, LV_STATE_EDITED);
    // Page 3
    lv_group_add_obj(g, ui_SliderSetBL);
    lv_obj_add_event_cb(ui_SliderSetBL, slider_set_bl_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetBL, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetVol);
    lv_obj_add_event_cb(ui_SliderSetVol, slider_set_vol_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetVol, NULL, LV_STATE_EDITED);
    lv_group_add_obj(g, ui_SliderSetNote);
    lv_obj_add_event_cb(ui_SliderSetNote, slider_set_note_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_remove_style(ui_SliderSetNote, NULL, LV_STATE_EDITED);
    // Page 4
    lv_group_add_obj(g, ui_Roller1);
    lv_obj_add_flag(ui_Roller1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_add_event_cb(ui_Roller1, roller_pd_select_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    // Page 5
    lv_group_add_obj(g, ui_Panel1);

    bsp_display_unlock();
}

void _pid_init(void) {
    ESP_LOGI("PID", "Create PID control block");
    pid_ctrl_config_t pid_config = {
        .init_param = pid_params,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
}

void _adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHAN, &config));

    adc_calibration_init(ADC_UNIT_1, ADC_CHAN, ADC_ATTEN, &adc_cali_handle);
}

void _i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA_IO,
        .scl_io_num = BSP_I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

    uint8_t addrs[10] = {0};
    i2c_bus_scan(i2c_bus, addrs, 10);
    for (int i = 0; i < 10; i++) {
        printf("0x%02X ", addrs[i]);
    }
    printf("\n");
}

void append_text(const char *text) {
    // 找到当前字符串的结束位置
    char *end = husb238_label_text + strlen(husb238_label_text);

    // 确保不会溢出
    while (*text && (end - husb238_label_text < sizeof(husb238_label_text) - 1)) {
        *end++ = *text++;  // 逐个字符追加
    }
    *end = '\0';  // 确保字符串以 NULL 结尾
}

double calculate_temperature_cvd(double resistance) {
    // Callendar-Van Dusen 方程, 根据电阻值计算温度

    double R0 = 1000.0;     // PT1000 在 0°C 时的电阻值
    double A = 3.9083e-3;   // CVD 方程 A 系数
    double B = -5.775e-7;   // CVD 方程 B 系数
    double C = -4.183e-12;  // CVD 方程 C 系数（仅用于 T < 0）

    double temperature = -200.0;
    double temp_step = 0.1;  // 温度步进值
    double calculated_resistance;

    // 对于温度 >= 0 的情况, 使用简化的 CVD 方程
    if (resistance >= R0) {
        while (temperature <= 850) {
            calculated_resistance = R0 * (1 + A * temperature + B * temperature * temperature);
            if (calculated_resistance >= resistance) {
                break;
            }
            temperature += temp_step;
        }
    }
    // 对于温度 < 0 的情况, 使用完整的 CVD 方程
    else {
        while (temperature <= 0) {
            calculated_resistance = R0 * (1 + A * temperature + B * temperature * temperature + C * (temperature - 100) * temperature * temperature * temperature);
            if (calculated_resistance >= resistance) {
                break;
            }
            temperature += temp_step;
        }
    }
    return temperature;
}

bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void format_float_to_width(char *result_str, uint8_t width, float value, const char *unit) {
    // 获取整数部分
    uint16_t integer_part = value;

    // 计算整数部分的长度
    uint8_t integer_digits = (integer_part == 0) ? 1 : log10(fabs((double)integer_part)) + 1;

    // 如果数值是负数, 需要考虑负号
    if (value < 0) {
        integer_digits++;  // 增加1位用于负号
    }

    // 小数位数
    uint8_t decimal_places = width - 2;

    // 如果整数部分的位数超过最大宽度, 直接格式化输出
    if (integer_digits >= width) {
        snprintf(result_str, width + 1, "%.*f", decimal_places, value);  // 允许溢出
    } else {
        // 构造格式化字符串
        char format[10];
        snprintf(format, sizeof(format), "%%%d.%df", width, decimal_places);
        snprintf(result_str, width + 1, format, value);
    }

    // 添加单位
    strcat(result_str, unit);
}

float power_limit(float supply_voltage, float max_power, float resistance_at_20C, float current_temperature) {
    const float alpha = 0.00393f;      // 铜的温度系数
    static float fixed_max_power = 0;  // 修正后的最大功率
    static float last_max_power = 0;   // 上一次的最大功率

    // 当设定功率改变时重置修正值
    if (last_max_power != max_power) {
        last_max_power = max_power;
        fixed_max_power = max_power;
    }

    // 计算当前温度下的阻值
    float current_resistance = resistance_at_20C * (1 + alpha * (current_temperature - 20));

    // 计算已加热时间(ms)
    uint32_t heating_time = heating_start_time > 0 ? (esp_timer_get_time() / 1000 - heating_start_time) : 0;

    // 加热开始 5 秒后进行功率修正
    if (heating_time > 5000) {
        // 计算实际功率和设定最大功率的偏差
        float power_deviation = max_power - ina226_power;

        // 根据偏差动态调整修正功率， 只在偏差 0-20
        if (power_deviation > 0 && power_deviation <= 20) {
            // 使用比例系数来避免过度修正
            float adjustment = power_deviation * 0.1f;
            fixed_max_power = fminf(fixed_max_power + adjustment, max_power * 1.2f);  // 限制最大修正幅度
        } else if (power_deviation < 0) {
            // 当实际功率超过设定值时，缓慢降低修正功率
            float adjustment = power_deviation * 0.05f;                               // 使用更小的调整系数
            fixed_max_power = fmaxf(fixed_max_power + adjustment, max_power * 0.9f);  // 提高最小修正幅度下限
        }
    } else {
        // 在开始2秒内使用原始设定功率
        fixed_max_power = max_power;
    }

    // 根据修正后的功率计算限制比率
    float limit = (fixed_max_power * current_resistance) / (supply_voltage * supply_voltage);

    // 限制在 0-1 范围内
    return fminf(fmaxf(limit, 0.0f), 1.0f);
}

uint32_t calculate_buzzer_volume(uint8_t percent) {
    return percent * 7168 / 100;
}

static void close_msgbox_cb(void *arg) {
    if (is_msgbox_shown && tilt_msgbox != NULL) {
        bsp_display_lock(0);
        lv_msgbox_close(tilt_msgbox);
        bsp_display_unlock();
        is_msgbox_shown = false;
        tilt_msgbox = NULL;
    }
}
