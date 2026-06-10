#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "main.h"

// Voltage thresholds (unit: V)
#define BAT_WARN_VOLTAGE    23.0f   // Warning threshold
#define BAT_LIMIT_VOLTAGE   22.0f   // Limit DM4340 power
#define BAT_CUTOFF_VOLTAGE  21.0f   // Stop all motion

// Voltage divider resistors from schematic
// R72 = 200K, R83 = 22K
#define R_UP    200000.0f
#define R_DOWN   22000.0f
#define ADC_REF_VOLTAGE  3.3f
#define ADC_MAX_VALUE    4096.0f

typedef enum {
    BAT_NORMAL  = 0,   // Normal operation
    BAT_WARNING = 1,   // DM4340 limited to 50% power
    BAT_LIMIT   = 2,   // DM4340 stopped
    BAT_CUTOFF  = 3,   // All motors stopped
} battery_state_e;

typedef struct {
    float voltage;          // Current battery voltage (V)
    battery_state_e state;  // Current battery state
    uint8_t dm4340_ratio;   // DM4340 power ratio 0-100
} battery_info_t;

void battery_monitor_init(void);
void battery_monitor_update(void);
const battery_info_t* get_battery_info(void);

#endif