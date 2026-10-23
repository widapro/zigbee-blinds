#pragma once

#include "esp_err.h"
#include "motor.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Storage keys for NVS
#define STORAGE_NAMESPACE "blinds_cfg"
#define KEY_POSITION "position"
#define KEY_MOTION_STATE "motion_state"
#define KEY_MOTOR_STEPS "motor_steps"
#define KEY_MOTOR_MODE "motor_mode"
#define KEY_MOTOR_IRUN_ACTIVE "motor_irun"
#define KEY_MOTOR_POWER_MODE "motor_pmode"
#define KEY_MOTOR_SPEED "motor_speed_v2"
#define KEY_PENDING_RESTART_CAUSE "pending_rst"
#define KEY_LAST_RESET_REASON "last_reset"
#define KEY_LAST_RESTART_CAUSE "last_rst_c"
#define KEY_REBOOT_COUNT "boot_count"

// Validation ranges
#define POSITION_MIN 0
#define POSITION_MAX 100
#define MOTOR_STEPS_MIN 10
#define MOTOR_STEPS_MAX 1000

// Default values
#define DEFAULT_POSITION 50 // 50%
#define DEFAULT_MOTOR_STEPS 100 // 100 raw units = 800 STEP pulses
#define DEFAULT_MOTOR_MODE 2 // Normal direction (1=reversed, 2=normal)
#define DEFAULT_MOTOR_IRUN_ACTIVE MOTOR_IRUN_ACTIVE // Defined in motor.h
#define DEFAULT_MOTOR_POWER_MODE MOTOR_POWER_MODE_STEALTHCHOP // Quiet mode
#define DEFAULT_MOTOR_SPEED 50 // Default midpoint in the Z2M range

typedef enum {
  RESTART_CAUSE_NONE = 0,
  RESTART_CAUSE_FACTORY_RESET = 1,
  RESTART_CAUSE_OTA_MODE_TRIGGER = 2,
  RESTART_CAUSE_OTA_TIMEOUT = 3,
  RESTART_CAUSE_OTA_SUCCESS = 4,
  RESTART_CAUSE_ZIGBEE_OFFLINE_STALL = 5,
  RESTART_CAUSE_ZIGBEE_INACTIVE = 6,
} restart_cause_t;

/**
 * @brief Initialize NVS storage
 * @return ESP_OK on success
 */
esp_err_t storage_init(void);

/**
 * @brief Load window covering position from storage
 * @param position Pointer to store the loaded position (0-100%)
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_position(uint8_t *position);

/**
 * @brief Save window covering position to storage
 * @param position Position to save (0-100%)
 * @return ESP_OK on success
 */
esp_err_t storage_save_position(uint8_t position);
// Position and validity share one NVS value so a reset cannot split them.
esp_err_t storage_load_position_valid(bool *valid);
esp_err_t storage_save_motion_state(uint8_t position, bool valid);

/**
 * @brief Load motor steps from storage
 * @param motor_steps Pointer to store the loaded steps
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_motor_steps(uint16_t *motor_steps);

/**
 * @brief Save motor steps to storage
 * @param motor_steps Steps to save (0-65535)
 * @return ESP_OK on success
 */
esp_err_t storage_save_motor_steps(uint16_t motor_steps);

/**
 * @brief Load motor mode from storage
 * @param motor_mode Pointer to store the loaded mode (1=reversed, 2=normal)
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_motor_mode(uint8_t *motor_mode);

/**
 * @brief Save motor mode to storage
 * @param motor_mode Mode to save (1=reversed, 2=normal)
 * @return ESP_OK on success
 */
esp_err_t storage_save_motor_mode(uint8_t motor_mode);

/**
 * @brief Load active motor current (IRUN) from storage
 * @param motor_irun_active Pointer to store the loaded IRUN (0-31)
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_motor_irun_active(uint8_t *motor_irun_active);

/**
 * @brief Save active motor current (IRUN) to storage
 * @param motor_irun_active IRUN to save (0-31)
 * @return ESP_OK on success
 */
esp_err_t storage_save_motor_irun_active(uint8_t motor_irun_active);

/**
 * @brief Load OTA mode flag from storage
 * @param enabled Pointer to store the flag (true/false)
 * @return ESP_OK on success
 */
esp_err_t storage_load_ota_mode(bool *enabled);

/**
 * @brief Save OTA mode flag to storage
 * @param enabled Flag to save (true/false)
 * @return ESP_OK on success
 */
esp_err_t storage_save_ota_mode(bool enabled);

/**
 * @brief Save the reason before calling esp_restart()
 * @param cause Restart cause code
 * @return ESP_OK on success
 */
esp_err_t storage_save_pending_restart_cause(restart_cause_t cause);

/**
 * @brief Persist boot diagnostics and clear pending restart cause
 * @param reset_reason ESP reset reason code
 * @param last_restart_cause Pointer to store the previous planned restart cause
 * @param reboot_count Pointer to store incremented boot count
 * @return ESP_OK on success
 */
esp_err_t storage_record_boot_diagnostics(uint8_t reset_reason,
                                          uint8_t *last_restart_cause,
                                          uint32_t *reboot_count);

/**
 * @brief Reset all stored parameters to defaults
 * @return ESP_OK on success
 */
esp_err_t storage_reset_all(void);

/**
 * @brief Load motor power mode from storage
 * @param motor_power_mode Pointer to store the loaded mode (0=StealthChop, 1=SpreadCycle)
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_motor_power_mode(uint8_t *motor_power_mode);

/**
 * @brief Save motor power mode to storage
 * @param motor_power_mode Mode to save (0=StealthChop, 1=SpreadCycle)
 * @return ESP_OK on success
 */
esp_err_t storage_save_motor_power_mode(uint8_t motor_power_mode);

/**
 * @brief Load motor speed from storage
 * @param motor_speed Pointer to store the loaded speed (1-100)
 * @return ESP_OK on success, uses default if not found
 */
esp_err_t storage_load_motor_speed(uint8_t *motor_speed);

/**
 * @brief Save motor speed to storage
 * @param motor_speed Speed to save (1-100)
 * @return ESP_OK on success
 */
esp_err_t storage_save_motor_speed(uint8_t motor_speed);

#ifdef __cplusplus
}
#endif
