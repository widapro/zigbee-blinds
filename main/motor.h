#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmc2208_uart.h"
#include <stdbool.h>
#include <stdint.h>

#define DIR_PIN GPIO_NUM_10
#define STEP_PIN GPIO_NUM_11
#define ENABLE_PIN GPIO_NUM_12
#define MOTOR_POWER_DC_EN GPIO_NUM_3

#define MOTOR_DRIVER_UART_PORT UART_NUM_1
#define MOTOR_DRIVER_UART_TX_PIN GPIO_NUM_13
#define MOTOR_DRIVER_UART_RX_PIN GPIO_NUM_14
#define MOTOR_DRIVER_UART_RTS_PIN UART_PIN_NO_CHANGE
#define MOTOR_DRIVER_UART_BAUD 115200

#define MOTOR_DRIVER_UART_DEBUG 0
#define MOTOR_VERIFY_UART_ON_WAKE 0

// Speed is controlled dynamically via g_motor_speed (1-100).
// These defines set the mapping boundaries:
//   speed=100 -> MOTOR_SPEED_MIN_DELAY_US (fastest, least torque)
//   speed=1   -> MOTOR_SPEED_MAX_DELAY_US (slowest, most torque)
#define MOTOR_SPEED_MIN_DELAY_US 500   // Cruise delay at speed=100 (fast half-step)
#define MOTOR_SPEED_MAX_DELAY_US 5000  // Cruise delay at speed=1 (max torque)
#define MOTOR_START_SPEED_FACTOR 3     // Start delay = cruise delay x this factor
#define MOTOR_START_DELAY_CAP_US 15000 // Maximum start delay cap

// Extra-soft StealthChop startup. Short 1% nudges use a quieter profile than
// long moves so the motor does not snap from standstill into a fast pulse train.
#define MOTOR_STEALTHCHOP_START_SPEED_FACTOR 8
#define MOTOR_STEALTHCHOP_START_DELAY_CAP_US 30000
#define MOTOR_STEALTHCHOP_SHORT_MOVE_MAX_STEPS 64
#define MOTOR_STEALTHCHOP_SHORT_MOVE_MIN_DELAY_US 9000
#define MOTOR_STEALTHCHOP_PRESTEP_SETTLE_MS 120
#define MOTOR_IHOLD_DELAY 10

#define MOTOR_RMT_RESOLUTION_HZ 1000000
#define MOTOR_RMT_MEM_BLOCK_SYMBOLS 48
#define MOTOR_RMT_MAX_CHUNK_STEPS 48
#define MOTOR_RMT_MAX_CHUNK_US 50000

#define MOTOR_ACCEL_STEPS 200 // Steps to accelerate from start to target speed
#define MOTOR_DECEL_STEPS 200 // Steps to decelerate before stopping
#define MOTOR_WAKE_DELAY_MS 30
#define MOTOR_SHUTDOWN_DELAY_MS 5
#define MOTOR_DISABLE_DELAY_MS 80
#define MOTOR_DRIVER_RECOVERY_FIRST_DELAY_MS 250
#define MOTOR_DRIVER_RECOVERY_SECOND_DELAY_MS 1000
#define MOTOR_DRIVER_RECOVERY_SETTLE_MS 100
#define MOTOR_DIR_SETTLE_US 50
#define MOTOR_KEEP_DRIVER_READY 0
#define MOTOR_KEEP_UART_READY 0
#define MOTOR_IRUN_ACTIVE 31
#define MOTOR_IHOLD_IDLE 0
#define MOTOR_MICROSTEP_MODE TMC2208_MRES_2
#define MOTOR_STEPS_MULTIPLIER 8
#define MOTOR_STEPS_TO_ACTUAL(raw) ((uint32_t)(raw) * MOTOR_STEPS_MULTIPLIER)

// Power mode: 0=StealthChop (quiet), 1=SpreadCycle (high torque)
#define MOTOR_POWER_MODE_STEALTHCHOP 0
#define MOTOR_POWER_MODE_SPREADCYCLE 1

void motor_init(void);
typedef enum {
  MOTOR_MOVE_COMPLETED = 0,
  MOTOR_MOVE_STOPPED,
  MOTOR_MOVE_DRIVER_ERROR,
  MOTOR_MOVE_PULSE_ERROR,
  MOTOR_MOVE_TIMEOUT,
  MOTOR_MOVE_INVALID,
} motor_move_result_t;
// Blocking; the worker clears cancellation before publishing active work.
motor_move_result_t motor_move_to_position(uint8_t target_position);
void motor_stop(void);
void motor_request_stop(void);
void motor_clear_stop_request(void);
bool motor_stop_requested(void);
bool motor_position_valid(void);
void motor_set_position_valid(bool valid);
bool motor_is_busy(void);
void motor_prepare_for_sleep(void);
void motor_resume_from_sleep(void);
void motor_set_irun_active(uint8_t motor_irun_active);
void motor_set_power_mode(uint8_t mode);
void motor_set_speed(uint8_t speed);

#endif // MOTOR_H
