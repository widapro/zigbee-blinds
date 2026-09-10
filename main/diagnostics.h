#pragma once

#include "esp_system.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  DIAG_STAGE_UNKNOWN = 0,
  DIAG_STAGE_APP_START = 1,
  DIAG_STAGE_LED_INIT = 2,
  DIAG_STAGE_STORAGE_INIT = 3,
  DIAG_STAGE_BOOT_DIAGNOSTICS = 4,
  DIAG_STAGE_OTA_MODE_CHECK = 5,
  DIAG_STAGE_BLE_OTA_START = 6,
  DIAG_STAGE_LOAD_CONFIG = 7,
  DIAG_STAGE_BATTERY_INIT = 8,
  DIAG_STAGE_MOTOR_INIT = 9,
  DIAG_STAGE_ZIGBEE_NETWORK_INIT = 10,
  DIAG_STAGE_START_TASKS = 11,
  DIAG_STAGE_ZIGBEE_TASK_START = 20,
  DIAG_STAGE_ZIGBEE_CLUSTER_SETUP = 21,
  DIAG_STAGE_ZIGBEE_DEVICE_REGISTER = 22,
  DIAG_STAGE_ZIGBEE_START = 23,
  DIAG_STAGE_ZIGBEE_RUNNING = 24,
  DIAG_STAGE_MOTOR_WORKER = 30,
  DIAG_STAGE_MOTOR_MOVE = 31,
  DIAG_STAGE_CALIBRATION = 32,
  DIAG_STAGE_BLE_OTA_WRITE = 40,
} diag_stage_t;

extern uint8_t g_last_crash_stage;

void diagnostics_init(esp_reset_reason_t reset_reason);
void diagnostics_mark_stage(diag_stage_t stage);

#ifdef __cplusplus
}
#endif
