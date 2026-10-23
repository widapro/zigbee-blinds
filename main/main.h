#pragma once

#include <stdint.h>
#include "esp_err.h"

// Default values for stored parameters (defined in storage.h)

// Global variables for stored parameters
extern uint8_t g_current_position;      // Current window position (0-100%)
extern uint16_t g_motor_steps;          // Motor steps (raw units, scaled internally)
extern uint8_t g_motor_mode;            // Motor direction (1=reversed, 2=normal)
extern uint8_t g_motor_irun_active;     // Motor active current (IRUN, 0-31)
extern uint8_t g_motor_power_mode;      // Motor power mode (0=StealthChop, 1=SpreadCycle)
extern uint8_t g_motor_speed;           // Motor speed (1-100, 50 is the default midpoint)
extern uint8_t g_tmc_mres_requested;    // Requested TMC2208 MRES value
extern uint8_t g_tmc_mres_actual;       // Last verified TMC2208 MRES value
extern uint8_t g_tmc_last_error;        // Last TMC2208 driver error code
extern uint8_t g_tmc_uart_register;     // UART register at failed driver setup (255=none)
extern uint32_t g_tmc_uart_error;       // esp_err_t from failed driver setup
extern uint8_t g_motor_startup_attempts; // Attempts used by the latest move
extern uint32_t g_motor_startup_recoveries; // Pre-pulse driver recoveries this boot
extern uint8_t g_rmt_status;            // STEP RMT status code
extern uint8_t g_rmt_fail_stage;        // Last STEP RMT failure stage
extern uint32_t g_rmt_last_error;       // Last STEP RMT esp_err_t value
extern uint8_t g_last_reset_reason;     // ESP reset reason from previous boot
extern uint8_t g_last_restart_cause;    // Firmware reason saved before esp_restart()
extern uint32_t g_reboot_count;         // Total boot counter stored in NVS
extern uint8_t g_last_crash_stage;      // Runtime stage before last panic/WDT/brownout


// Common defines
#define FACTORY_RESET_GPIO             GPIO_NUM_9
#define FACTORY_RESET_HOLD_TIME_MS     5000
#define WS2812_GPIO                    GPIO_NUM_8
#define WS2812_LED_COUNT               1

// Zigbee endpoint configuration
#define ENDPOINT_ID                    1
#define DEVICE_ID                      ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID
#define DEVICE_VERSION                 0
#define BASIC_MANUFACTURER_NAME        "\x07""widapro"
#define BASIC_MODEL_IDENTIFIER         "\x07""esp32h2"

// Factory reset functionality
void factory_reset(void);
