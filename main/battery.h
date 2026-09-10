#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 // GPIO1
#define BATTERY_VDIV_RTOP_OHMS 47000.0f   // Resistor from battery to ADC node
#define BATTERY_VDIV_RBOT_OHMS 100000.0f  // Resistor from ADC node to GND
#define BATTERY_VOLTAGE_DIVIDER                                                \
  (1.0f + (BATTERY_VDIV_RTOP_OHMS / BATTERY_VDIV_RBOT_OHMS)) // About 1.47x
#define BATTERY_MIN_VOLTAGE 3.0f // 1S Li-ion minimum voltage (3.0V per cell)
#define BATTERY_MAX_VOLTAGE 4.2f // 1S Li-ion maximum voltage (4.2V per cell)
#define BATTERY_REPORTING_INTERVAL                                             \
  (60 * 60 * 1000) // 60 minutes in milliseconds for better power savings

esp_err_t battery_init(void);

float battery_get_voltage(void);

uint8_t battery_get_percentage(void);

bool battery_should_report(void);

float battery_get_cached_voltage(void);

uint8_t battery_get_cached_percentage(void);

void battery_start_reporting_task(void);

void battery_mark_adc_invalid(void);

#ifdef __cplusplus
}
#endif
