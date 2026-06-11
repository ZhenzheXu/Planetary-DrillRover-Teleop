#include "battery_monitor.h"

extern ADC_HandleTypeDef hadc3;

static battery_info_t battery_info = {0};

void battery_monitor_init(void)
{
    HAL_ADC_Start(&hadc3);
    battery_info.state = BAT_NORMAL;
    battery_info.dm4340_ratio = 100;
}

void battery_monitor_update(void)
{
    uint32_t adc_val = 0;

    HAL_ADC_Start(&hadc3);
    if(HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
    {
        adc_val = HAL_ADC_GetValue(&hadc3);
    }

    // Convert ADC raw value to actual battery voltage
    float adc_voltage = (float)adc_val / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    battery_info.voltage = adc_voltage * (R_UP + R_DOWN) / R_DOWN;

    // Update state based on voltage level
    if(battery_info.voltage >= BAT_WARN_VOLTAGE)
    {
        battery_info.state = BAT_NORMAL;
        battery_info.dm4340_ratio = 100;
    }
    else if(battery_info.voltage >= BAT_LIMIT_VOLTAGE)
    {
        battery_info.state = BAT_WARNING;
        battery_info.dm4340_ratio = 50;
    }
    else if(battery_info.voltage >= BAT_CUTOFF_VOLTAGE)
    {
        battery_info.state = BAT_LIMIT;
        battery_info.dm4340_ratio = 0;
    }
    else
    {
        battery_info.state = BAT_CUTOFF;
        battery_info.dm4340_ratio = 0;
    }
}

const battery_info_t* get_battery_info(void)
{
    return &battery_info;
}