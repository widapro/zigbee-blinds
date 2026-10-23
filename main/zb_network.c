#include "zb_network.h"
#include "zb_attributes.h"
#include "command_control.h"
#include <inttypes.h>
#include "battery.h"
#include "diagnostics.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led.h"
#include "main.h"
#include "motor.h"
#include "nwk/esp_zigbee_nwk.h"
#include "storage.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include <string.h>

static const char *TAG = "ZB_NETWORK";

#define POST_MOTION_AWAKE_WINDOW_MS 20000
#define ZB_PARENT_LINK_FAILURES_BEFORE_REJOIN 3
#define ZB_REJOIN_INITIAL_DELAY_MS 60000
#define ZB_REJOIN_MAX_DELAY_MS 900000
#define ZB_HEALTH_CHECK_PERIOD_MS 15000
#define ZB_IDLE_LOG_PERIOD_MS 600000
#define ZB_REJOIN_STALL_TIMEOUT_MS 1800000

// Network join status to control sleep behavior
static volatile bool network_joined = false;

// Calibration status to prevent sleep during motor calibration
static volatile bool calibration_in_progress = false;
static volatile bool s_rejoin_alarm_scheduled = false;
static volatile uint32_t s_rejoin_delay_ms = ZB_REJOIN_INITIAL_DELAY_MS;
static volatile uint32_t s_parent_link_failure_count = 0;
static volatile int64_t s_last_zb_activity_us = 0;
static volatile int64_t s_last_idle_log_us = 0;
static volatile bool s_ever_joined = false;
static volatile int64_t s_last_join_state_change_us = 0;
static bool s_health_task_started = false;

static TaskHandle_t s_motor_worker = NULL;
static SemaphoreHandle_t s_motor_sleep_gate = NULL;
#ifdef CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_motor_pm_lock = NULL;
#endif
static command_control_t s_commands;
static bool s_worker_busy = false;
static bool s_report_dirty = false;
static int64_t s_awake_until_us = 0;
static portMUX_TYPE s_move_cmd_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
  uint16_t steps;
  uint8_t mode;
  uint8_t irun;
  uint8_t power_mode;
  uint8_t speed;
} motor_config_t;
enum {
  CONFIG_STEPS = 1, CONFIG_MODE = 2, CONFIG_IRUN = 4,
  CONFIG_POWER_MODE = 8, CONFIG_SPEED = 16,
};
static motor_config_t s_desired_config;
static motor_config_t s_attr_config;
static uint8_t s_config_pending = 0;
static bool s_ota_pending = false;
static uint8_t s_command_protocol = COMMAND_PROTOCOL_VERSION;
static uint32_t s_command_request = 0;
static uint32_t s_command_result = 0;
static uint8_t s_command_target = 0;
static uint32_t s_command_received = 0;
static uint32_t s_command_completed = 0;
static uint32_t s_command_failed = 0;
static uint32_t s_command_duplicates = 0;
static uint32_t s_command_cursor = 0;
static uint8_t s_position_valid = 1;
static uint8_t s_tilt_position = 0;

typedef struct {
  uint16_t id;
  const void *source;
  void *zcl_value;
  uint8_t size;
} zb_backing_t;

static zb_backing_t s_live_attrs[] = {
    {ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
     &s_tilt_position, NULL, sizeof(s_tilt_position)},
    {ZB_ATTR_COMMAND_RESULT, &s_command_result, NULL, sizeof(s_command_result)},
    {ZB_ATTR_COMMAND_TARGET, &s_command_target, NULL, sizeof(s_command_target)},
    {ZB_ATTR_COMMAND_RECEIVED, &s_command_received, NULL, sizeof(s_command_received)},
    {ZB_ATTR_COMMAND_COMPLETED, &s_command_completed, NULL, sizeof(s_command_completed)},
    {ZB_ATTR_COMMAND_FAILED, &s_command_failed, NULL, sizeof(s_command_failed)},
    {ZB_ATTR_COMMAND_DUPLICATES, &s_command_duplicates, NULL, sizeof(s_command_duplicates)},
    {ZB_ATTR_COMMAND_CURSOR, &s_command_cursor, NULL, sizeof(s_command_cursor)},
    {ZB_ATTR_POSITION_VALID, &s_position_valid, NULL, sizeof(s_position_valid)},
    {ZB_ATTR_TMC_MRES_REQUESTED, &g_tmc_mres_requested, NULL, sizeof(g_tmc_mres_requested)},
    {ZB_ATTR_TMC_MRES_ACTUAL, &g_tmc_mres_actual, NULL, sizeof(g_tmc_mres_actual)},
    {ZB_ATTR_TMC_ERROR, &g_tmc_last_error, NULL, sizeof(g_tmc_last_error)},
    {ZB_ATTR_TMC_UART_REGISTER, &g_tmc_uart_register, NULL, sizeof(g_tmc_uart_register)},
    {ZB_ATTR_TMC_UART_ERROR, &g_tmc_uart_error, NULL, sizeof(g_tmc_uart_error)},
    {ZB_ATTR_MOTOR_STARTUP_ATTEMPTS, &g_motor_startup_attempts, NULL, sizeof(g_motor_startup_attempts)},
    {ZB_ATTR_MOTOR_STARTUP_RECOVERIES, &g_motor_startup_recoveries, NULL, sizeof(g_motor_startup_recoveries)},
    {ZB_ATTR_RMT_STATUS, &g_rmt_status, NULL, sizeof(g_rmt_status)},
    {ZB_ATTR_RMT_FAIL_STAGE, &g_rmt_fail_stage, NULL, sizeof(g_rmt_fail_stage)},
    {ZB_ATTR_RMT_ERROR, &g_rmt_last_error, NULL, sizeof(g_rmt_last_error)},
};

// Forward declarations
static void zb_check_battery_report(uint8_t param);
static void zb_bdb_start_commissioning_wrapper(uint8_t mode_mask);
static void zb_send_battery_report_attr(uint16_t attribute_id);
static void zb_report_motor_config(void);
static void zb_report_motor_config_delayed(uint8_t param);
static void zb_send_tilt_report(uint8_t endpoint);
static void motor_worker_task(void *pvParameters);
static esp_err_t enqueue_motor_command(motor_cmd_t cmd);
static void zb_sync_and_optionally_report(uint8_t endpoint, bool report_tilt);
static void zb_sync_command_attributes(void);
static void zb_send_wc_report_attr(uint16_t attribute_id, const char *attr_name);
static bool motor_work_pending(void);
static void queue_motor_config(uint8_t mask);
static uint8_t clamp_position(uint8_t position);
static uint16_t clamp_motor_steps(uint16_t steps);
static uint8_t sanitize_motor_mode(uint8_t mode);
static void extend_awake_window_after_motion(void);
static bool awake_window_active(uint32_t *remaining_ms);
static void zb_mark_activity(void);
static bool zb_attr_can_report(uint8_t endpoint, uint16_t cluster_id,
                               uint16_t attribute_id, const char *attr_name);
static void zb_schedule_rejoin(const char *reason);
static void zb_schedule_network_steering(const char *reason, uint32_t delay_ms);
static void zb_rejoin_alarm_cb(uint8_t param);
static void zb_health_monitor_task(void *pvParameters);

// Power save initialization following official example pattern
static esp_err_t zb_power_save_init(void) {
  esp_err_t ret = ESP_OK;

#ifdef CONFIG_PM_ENABLE
  esp_pm_config_t pm_config = {
      .max_freq_mhz = ZB_PM_MAX_FREQ_MHZ,
      .min_freq_mhz = ZB_PM_MIN_FREQ_MHZ,
      .light_sleep_enable = ZB_SLEEP_ENABLE};

  ret = esp_pm_configure(&pm_config);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to configure power management: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG,
           "PM configured: max=%dMHz min=%dMHz light_sleep=%s keep_alive=%ums",
           ZB_PM_MAX_FREQ_MHZ, ZB_PM_MIN_FREQ_MHZ,
           ZB_SLEEP_ENABLE ? "enabled" : "disabled", (unsigned)ED_KEEP_ALIVE);
#else
  ESP_LOGW(TAG, "Power management not enabled in config");
#endif

  return ret;
}

esp_err_t zb_network_init_power_management(void) {
  return zb_power_save_init();
}

// Wrapper function to match esp_zb_callback_t signature
static void zb_bdb_start_commissioning_wrapper(uint8_t mode_mask) {
  esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

static uint8_t clamp_position(uint8_t position) {
  if (position > POSITION_MAX) {
    return POSITION_MAX;
  }
  return position;
}

static uint16_t clamp_motor_steps(uint16_t steps) {
  if (steps < MOTOR_STEPS_MIN) {
    return MOTOR_STEPS_MIN;
  }
  if (steps > MOTOR_STEPS_MAX) {
    return MOTOR_STEPS_MAX;
  }
  return steps;
}

static uint8_t sanitize_motor_mode(uint8_t mode) {
  if (mode == 1 || mode == 2) {
    return mode;
  }
  return DEFAULT_MOTOR_MODE;
}

static void extend_awake_window_after_motion(void) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  s_awake_until_us = esp_timer_get_time() + POST_MOTION_AWAKE_WINDOW_MS * 1000LL;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  ESP_LOGI(TAG, "Post-motion awake window enabled for %u ms",
           (unsigned)POST_MOTION_AWAKE_WINDOW_MS);
}

static bool awake_window_active(uint32_t *remaining_ms) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  int64_t us_left = s_awake_until_us - esp_timer_get_time();
  portEXIT_CRITICAL(&s_move_cmd_lock);
  if (us_left > 0) {
    if (remaining_ms) {
      *remaining_ms = (uint32_t)(us_left / 1000);
    }
    return true;
  }

  if (remaining_ms) {
    *remaining_ms = 0;
  }
  return false;
}

static void zb_mark_activity(void) { s_last_zb_activity_us = esp_timer_get_time(); }

static bool zb_attr_can_report(uint8_t endpoint, uint16_t cluster_id,
                               uint16_t attribute_id, const char *attr_name) {
  if (!network_joined) {
    ESP_LOGD(TAG, "Skipping %s report - network not joined", attr_name);
    return false;
  }

  esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
      endpoint, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attribute_id);
  if (!attr) {
    ESP_LOGW(TAG, "Skipping %s report - attribute 0x%04X not found",
             attr_name, attribute_id);
    return false;
  }

  if ((attr->access & ESP_ZB_ZCL_ATTR_ACCESS_REPORTING) == 0) {
    ESP_LOGW(TAG, "Skipping %s report - attribute 0x%04X is not reportable",
             attr_name, attribute_id);
    return false;
  }

  return true;
}

static void zb_rejoin_alarm_cb(uint8_t param) {
  (void)param;
  s_rejoin_alarm_scheduled = false;
  ESP_LOGW(TAG, "Attempting Zigbee network steering recovery");
  esp_err_t err = esp_zb_bdb_start_top_level_commissioning(
      ESP_ZB_BDB_MODE_NETWORK_STEERING);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Recovery start failed: %s", esp_err_to_name(err));
    zb_schedule_rejoin("recovery start failed");
  }
}

static void zb_schedule_network_steering(const char *reason, uint32_t delay_ms) {
  if (s_rejoin_alarm_scheduled) {
    ESP_LOGW(TAG, "Rejoin already scheduled (%s)", reason);
    return;
  }

  if (network_joined || !s_last_join_state_change_us) {
    s_last_join_state_change_us = esp_timer_get_time();
  }
  network_joined = false;
  s_rejoin_alarm_scheduled = true;
  ESP_LOGW(TAG, "Scheduling Zigbee rejoin in %lu ms: %s",
           (unsigned long)delay_ms, reason);
  esp_zb_scheduler_alarm(zb_rejoin_alarm_cb, 0, delay_ms);
}

static void zb_schedule_rejoin(const char *reason) {
  if (s_rejoin_alarm_scheduled) {
    ESP_LOGW(TAG, "Rejoin already scheduled (%s)", reason);
    return;
  }

  uint32_t delay_ms = s_rejoin_delay_ms;
  if (s_rejoin_delay_ms < ZB_REJOIN_MAX_DELAY_MS) {
    uint32_t next_delay = s_rejoin_delay_ms * 2;
    s_rejoin_delay_ms =
        next_delay > ZB_REJOIN_MAX_DELAY_MS ? ZB_REJOIN_MAX_DELAY_MS
                                            : next_delay;
  }
  zb_schedule_network_steering(reason, delay_ms);
}

static void zb_health_monitor_task(void *pvParameters) {
  (void)pvParameters;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(ZB_HEALTH_CHECK_PERIOD_MS));

    int64_t now_us = esp_timer_get_time();
    bool motor_busy = motor_is_busy();

    if (!network_joined) {
      if (s_ever_joined) {
        int64_t offline_ms = (now_us - s_last_join_state_change_us) / 1000;
        if (offline_ms > ZB_REJOIN_STALL_TIMEOUT_MS) {
          ESP_LOGE(TAG,
                   "Device offline for %" PRId64
                   " ms after previous join, restarting",
                   offline_ms);
          storage_save_pending_restart_cause(
              RESTART_CAUSE_ZIGBEE_OFFLINE_STALL);
          vTaskDelay(pdMS_TO_TICKS(200));
          esp_restart();
        }
      }
      continue;
    }

    if (calibration_in_progress || motor_busy) {
      continue;
    }

    int64_t last_us = s_last_zb_activity_us;
    if (last_us == 0) {
      s_last_zb_activity_us = now_us;
      continue;
    }

    int64_t idle_ms = (now_us - last_us) / 1000;
    int64_t idle_log_elapsed_ms = (now_us - s_last_idle_log_us) / 1000;
    if (idle_ms > ZB_IDLE_LOG_PERIOD_MS &&
        idle_log_elapsed_ms > ZB_IDLE_LOG_PERIOD_MS) {
      ESP_LOGW(TAG,
               "No Zigbee callbacks for %" PRId64
               " ms in joined state; keeping device online",
               idle_ms);
      s_last_idle_log_us = now_us;
    }
  }
}

static bool motor_work_pending(void) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  bool pending = command_control_busy(&s_commands) || s_worker_busy ||
                 s_config_pending || s_ota_pending;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  return pending;
}

static esp_err_t enqueue_motor_command(motor_cmd_t cmd) {
  if (!s_motor_worker) {
    return ESP_ERR_INVALID_STATE;
  }
  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_COMMAND_ENQUEUE);
  if (!cmd.endpoint) {
    cmd.endpoint = ENDPOINT_ID;
  }
  bool interrupt_active;
  portENTER_CRITICAL(&s_move_cmd_lock);
  command_submit_t submitted = command_control_submit(
      &s_commands, cmd, &interrupt_active);
  // Clear/request cancellation under the same lock used when taking work.
  // A request received just before motor startup must not be erased there.
  if (interrupt_active) {
    motor_request_stop();
  }
  s_report_dirty = true;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  ESP_LOGI(TAG, "Command received: id=%lu type=%u target=%u disposition=%u",
           (unsigned long)cmd.id, cmd.type, cmd.target_position, submitted);
  if (submitted == COMMAND_SUBMIT_INVALID) {
    return ESP_ERR_INVALID_ARG;
  }
  extend_awake_window_after_motion();
  // This function runs in a ZCL command/write callback. Updating many ZCL
  // attributes and sending a report here re-enters the stack during its own
  // receive path; defer all publishing until the worker owns the Zigbee lock.
  xTaskNotifyGive(s_motor_worker);
  return ESP_OK;
}

// Each attribute was registered with a pointer to a static value. Mirror it
// into the stack's descriptor if registration produced a separate backing.
// Avoid esp_zb_zcl_set_attribute_val() here: setting nine read-only/reportable
// attributes before every move entered the SDK attribute-change path and
// caused a panic on the device before the motor could start.
// Called only while the Zigbee lock is held.
static void zb_sync_command_attributes(void) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  s_command_result = command_control_result(&s_commands);
  s_command_target = s_commands.latest.command.type == MOTOR_CMD_STOP ?
                     COMMAND_STOP_TARGET : s_commands.latest.command.target_position;
  s_command_received = s_commands.received;
  s_command_completed = s_commands.completed;
  s_command_failed = s_commands.failed;
  s_command_duplicates = s_commands.duplicates;
  s_command_cursor = s_commands.reliable_cursor;
  s_report_dirty = false;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  s_position_valid = motor_position_valid();
  s_tilt_position = s_position_valid ? clamp_position(g_current_position) : 0xFF;
  for (unsigned i = 0; i < sizeof(s_live_attrs) / sizeof(s_live_attrs[0]); ++i) {
    zb_backing_t *attr = &s_live_attrs[i];
    if (attr->zcl_value && attr->zcl_value != attr->source) {
      // The stack may keep its own backing rather than the pointer supplied
      // at registration. Copy to either representation without invoking the
      // ZCL attribute-change hook or generating automatic reports.
      memcpy(attr->zcl_value, attr->source, attr->size);
    }
  }
}

static void zb_sync_and_optionally_report(uint8_t endpoint, bool report_tilt) {
  uint8_t ep = endpoint ? endpoint : ENDPOINT_ID;
  if (!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
    portENTER_CRITICAL(&s_move_cmd_lock);
    s_report_dirty = true;
    portEXIT_CRITICAL(&s_move_cmd_lock);
    ESP_LOGW(TAG, "Zigbee publish deferred: lock unavailable");
    return;
  }
  diagnostics_mark_stage(DIAG_STAGE_MOTOR_STATE_SYNC);
  zb_sync_command_attributes();
  // The converter reads the correlated result. Only the final tilt needs an
  // unsolicited report for coordinators that do not use that converter.
  if (report_tilt) {
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_TILT_REPORT);
    zb_send_tilt_report(ep);
  }
  esp_zb_lock_release();
}

static void queue_motor_config(uint8_t mask) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  s_config_pending |= mask;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  if (s_motor_worker) {
    xTaskNotifyGive(s_motor_worker);
  }
}

static esp_err_t apply_pending_motor_config(void) {
  portENTER_CRITICAL(&s_move_cmd_lock);
  uint8_t mask = s_config_pending;
  motor_config_t cfg = s_desired_config;
  s_config_pending = 0;
  portEXIT_CRITICAL(&s_move_cmd_lock);
  esp_err_t first_error = ESP_OK;
#define SAVE_CONFIG(bit, global, value, save) \
  if (mask & (bit)) { \
    global = value; \
    esp_err_t err = save(value); \
    if (first_error == ESP_OK) { first_error = err; } \
  }
  SAVE_CONFIG(CONFIG_STEPS, g_motor_steps, cfg.steps, storage_save_motor_steps);
  SAVE_CONFIG(CONFIG_MODE, g_motor_mode, cfg.mode, storage_save_motor_mode);
  SAVE_CONFIG(CONFIG_IRUN, g_motor_irun_active, cfg.irun, storage_save_motor_irun_active);
  SAVE_CONFIG(CONFIG_POWER_MODE, g_motor_power_mode, cfg.power_mode, storage_save_motor_power_mode);
  SAVE_CONFIG(CONFIG_SPEED, g_motor_speed, cfg.speed, storage_save_motor_speed);
#undef SAVE_CONFIG
  if (mask & CONFIG_IRUN) {
    motor_set_irun_active(cfg.irun);
  }
  if (mask & CONFIG_POWER_MODE) {
    motor_set_power_mode(cfg.power_mode);
  }
  if (mask & CONFIG_SPEED) {
    motor_set_speed(cfg.speed);
  }
  if (first_error != ESP_OK) {
    ESP_LOGE(TAG, "Motor configuration could not be saved: %s",
             esp_err_to_name(first_error));
  }
  return first_error;
}

static command_status_t motor_result_status(motor_move_result_t result) {
  switch (result) {
  case MOTOR_MOVE_COMPLETED: return COMMAND_COMPLETED;
  case MOTOR_MOVE_STOPPED: return COMMAND_STOPPED;
  case MOTOR_MOVE_DRIVER_ERROR: return COMMAND_DRIVER_ERROR;
  case MOTOR_MOVE_PULSE_ERROR: return COMMAND_PULSE_ERROR;
  case MOTOR_MOVE_TIMEOUT: return COMMAND_TIMEOUT;
  default: return COMMAND_INVALID;
  }
}

static command_status_t execute_motor_command(motor_cmd_t cmd) {
  if (cmd.type == MOTOR_CMD_STOP) {
    motor_stop();
    return COMMAND_STOPPED;
  }
  if (cmd.type == MOTOR_CMD_MOVE && !motor_position_valid()) {
    return COMMAND_POSITION_UNKNOWN;
  }
  bool needs_motion = cmd.type == MOTOR_CMD_CALIBRATE ||
                      cmd.target_position != g_current_position;
  // Mark the persisted estimate unknown before the first pulse, so an
  // interrupted power supply cannot restore a stale position as confirmed.
  if (needs_motion && storage_save_motion_state(g_current_position, false) != ESP_OK) {
    return COMMAND_STORAGE_ERROR;
  }
  if (cmd.type == MOTOR_CMD_MOVE) {
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_MOVE);
    return motor_result_status(motor_move_to_position(cmd.target_position));
  }
  diagnostics_mark_stage(DIAG_STAGE_CALIBRATION);
  motor_set_position_valid(true);
  g_current_position = 100; // Existing calibration assumes this starting end.
  const uint8_t targets[] = {0, 100, 50};
  for (unsigned i = 0; i < sizeof(targets); ++i) {
    if (motor_stop_requested()) {
      motor_set_position_valid(false);
      return COMMAND_STOPPED;
    }
    motor_move_result_t result = motor_move_to_position(targets[i]);
    if (result != MOTOR_MOVE_COMPLETED) {
      motor_set_position_valid(false);
      return motor_result_status(result);
    }
    if (i < 2) {
      zb_sync_and_optionally_report(cmd.endpoint, true);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  return COMMAND_COMPLETED;
}

static void motor_worker_task(void *pvParameters) {
  (void)pvParameters;
  diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);
  while (1) {
    portENTER_CRITICAL(&s_move_cmd_lock);
    bool retry_report = s_report_dirty;
    portEXIT_CRITICAL(&s_move_cmd_lock);
    // Idle workers do not periodically wake the battery-powered device.
    // A deferred attribute update gets a bounded retry instead.
    ulTaskNotifyTake(pdTRUE, retry_report ? pdMS_TO_TICKS(1000) : portMAX_DELAY);
    if (!motor_work_pending()) {
      portENTER_CRITICAL(&s_move_cmd_lock);
      bool dirty = s_report_dirty;
      portEXIT_CRITICAL(&s_move_cmd_lock);
      if (dirty) {
        diagnostics_mark_stage(DIAG_STAGE_MOTOR_REPORT);
        zb_sync_and_optionally_report(ENDPOINT_ID, false);
        diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);
      }
      continue;
    }
    // The same mutex covers GPIO hold + explicit Zigbee sleep. The worker
    // cannot start peripherals until the sleep callback has restored them.
    xSemaphoreTake(s_motor_sleep_gate, portMAX_DELAY);
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_COMMAND_TAKE);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_acquire(s_motor_pm_lock);
#endif
    portENTER_CRITICAL(&s_move_cmd_lock);
    s_worker_busy = true;
    portEXIT_CRITICAL(&s_move_cmd_lock);
    apply_pending_motor_config();

    portENTER_CRITICAL(&s_move_cmd_lock);
    bool ota = s_ota_pending;
    motor_cmd_t cmd;
    motor_clear_stop_request();
    bool have_command = !ota && command_control_take(&s_commands, &cmd);
    calibration_in_progress = have_command && cmd.type == MOTOR_CMD_CALIBRATE;
    portEXIT_CRITICAL(&s_move_cmd_lock);
    if (ota) {
      motor_stop();
      if (storage_save_ota_mode(true) == ESP_OK) {
        storage_save_pending_restart_cause(RESTART_CAUSE_OTA_MODE_TRIGGER);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }
      portENTER_CRITICAL(&s_move_cmd_lock);
      s_ota_pending = false;
      portEXIT_CRITICAL(&s_move_cmd_lock);
      ESP_LOGE(TAG, "Cannot save OTA request");
    } else if (have_command) {
      ESP_LOGI(TAG, "Command started: id=%lu generation=%lu target=%u",
               (unsigned long)cmd.id, (unsigned long)cmd.generation, cmd.target_position);
      // Make MOVING readable without triggering an outbound report burst.
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_REPORT);
      zb_sync_and_optionally_report(cmd.endpoint, false);
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_EXECUTE);
      command_status_t result = execute_motor_command(cmd);
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_STORE);
      if (storage_save_motion_state(g_current_position, motor_position_valid()) != ESP_OK) {
        motor_set_position_valid(false);
        result = COMMAND_STORAGE_ERROR;
      }
      portENTER_CRITICAL(&s_move_cmd_lock);
      if (result == COMMAND_STOPPED && s_commands.pending) {
        result = COMMAND_SUPERSEDED;
      }
      command_control_finish(&s_commands, result);
      calibration_in_progress = false;
      s_report_dirty = true;
      portEXIT_CRITICAL(&s_move_cmd_lock);
      ESP_LOGI(TAG, "Command finished: id=%lu result=%u position=%u valid=%u",
               (unsigned long)cmd.id, result, g_current_position, motor_position_valid());
    }
    extend_awake_window_after_motion();
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_REPORT);
    zb_sync_and_optionally_report(ENDPOINT_ID, true);
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);
    portENTER_CRITICAL(&s_move_cmd_lock);
    s_worker_busy = false;
    portEXIT_CRITICAL(&s_move_cmd_lock);
#ifdef CONFIG_PM_ENABLE
    esp_pm_lock_release(s_motor_pm_lock);
#endif
    xSemaphoreGive(s_motor_sleep_gate);
    if (motor_work_pending()) {
      xTaskNotifyGive(s_motor_worker);
    }
  }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  switch (sig_type) {
  case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
    ESP_LOGI(TAG, "Initialize Zigbee stack");
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
    break;
  case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
  case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
    if (err_status == ESP_OK) {
      ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
               esp_zb_bdb_is_factory_new() ? "" : "non");
      if (esp_zb_bdb_is_factory_new()) {
        ESP_LOGI(TAG, "Start network steering");
        esp_zb_bdb_start_top_level_commissioning(
            ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        ESP_LOGI(TAG, "Device rebooted - already joined network");
        // Device was already joined, enable sleep immediately
        network_joined = true;
        s_ever_joined = true;
        s_last_join_state_change_us = esp_timer_get_time();
        ESP_LOGI(TAG, "Network already joined");
        s_rejoin_delay_ms = ZB_REJOIN_INITIAL_DELAY_MS;
        s_parent_link_failure_count = 0;
        s_rejoin_alarm_scheduled = false;
        esp_zb_scheduler_alarm_cancel(zb_rejoin_alarm_cb, 0);
        zb_mark_activity();

        // Report motor configuration values so Z2M displays them (delayed)
        esp_zb_scheduler_alarm(zb_report_motor_config_delayed, 0, 10000);

        // Start battery reporting immediately on reboot as well
        ESP_LOGI(TAG, "Starting battery reporting for rebooted device");
        zb_check_battery_report(0); // Call immediately
      }
    } else {
      ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)",
               esp_err_to_name(err_status));
      esp_zb_scheduler_alarm(zb_bdb_start_commissioning_wrapper,
                             ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
    }
    break;
  case ESP_ZB_BDB_SIGNAL_STEERING:
  case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
    if (err_status == ESP_OK) {
      esp_zb_ieee_addr_t extended_pan_id;
      esp_zb_get_extended_pan_id(extended_pan_id);
      ESP_LOGI(TAG,
               "Joined network successfully (Extended PAN ID: "
               "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
               "Channel:%d, Short Address: 0x%04hx)",
               extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
               extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
               extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(),
               esp_zb_get_current_channel(), esp_zb_get_short_address());

      // Mark network as joined to allow sleep
      network_joined = true;
      s_ever_joined = true;
      s_last_join_state_change_us = esp_timer_get_time();
      s_rejoin_alarm_scheduled = false;
      s_rejoin_delay_ms = ZB_REJOIN_INITIAL_DELAY_MS;
      s_parent_link_failure_count = 0;
      esp_zb_scheduler_alarm_cancel(zb_rejoin_alarm_cb, 0);
      zb_mark_activity();
      ESP_LOGI(TAG, "Network joined successfully");

      // Report motor configuration values so Z2M displays them
      // Delayed by 10s to ensure Z2M interview is complete on first pair
      esp_zb_scheduler_alarm(zb_report_motor_config_delayed, 0, 10000);

      // Start battery reporting immediately, then schedule regular reporting
      ESP_LOGI(TAG, "Performing initial battery check immediately");
      zb_check_battery_report(0); // Call immediately

      // Log the active sleep profile after a successful join.
      ESP_LOGI(TAG, "END DEVICE: Keep-alive interval: %u ms",
               (unsigned)ED_KEEP_ALIVE);
      ESP_LOGI(TAG, "END DEVICE: Zigbee sleep is %s",
               ZB_SLEEP_ENABLE ? "enabled" : "disabled (responsive mode)");
    } else {
      ESP_LOGI(TAG, "Network steering was not successful (status: %s)",
               esp_err_to_name(err_status));
      if (s_ever_joined) {
        zb_schedule_rejoin("network steering failed");
      } else {
        zb_schedule_network_steering("initial network steering failed", 1000);
      }
    }
    break;
  case ESP_ZB_ZDO_SIGNAL_LEAVE: {
    esp_zb_zdo_signal_leave_params_t *leave_params =
        (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(
            p_sg_p);
    uint8_t leave_type = leave_params ? leave_params->leave_type : 0xFF;
    ESP_LOGW(TAG, "Zigbee leave signal received (type=0x%02x)", leave_type);
    zb_schedule_rejoin("leave signal");
    break;
  }
  case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
    ESP_LOGW(TAG, "No active Zigbee links left");
    zb_schedule_rejoin("no active links");
    break;
  case ESP_ZB_NLME_STATUS_INDICATION: {
    esp_zb_zdo_signal_nwk_status_indication_params_t *status_params =
        (esp_zb_zdo_signal_nwk_status_indication_params_t *)
            esp_zb_app_signal_get_params(p_sg_p);
    if (status_params) {
      ESP_LOGW(TAG, "NLME status indication: status=0x%02x addr=0x%04x",
               status_params->status, status_params->network_addr);
      if (status_params->status ==
          ESP_ZB_NWK_COMMAND_STATUS_PARENT_LINK_FAILURE) {
        s_parent_link_failure_count++;
        if (s_parent_link_failure_count >=
            ZB_PARENT_LINK_FAILURES_BEFORE_REJOIN) {
          zb_schedule_rejoin("repeated parent link failure");
        } else {
          ESP_LOGW(TAG, "Parent link failure #%lu - waiting before rejoin",
                   (unsigned long)s_parent_link_failure_count);
        }
      }
    }
    break;
  }
  case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
#if !ZB_SLEEP_ENABLE
    break;
#else

    if (motor_work_pending() || motor_is_busy() || awake_window_active(NULL)) {
      break;
    }
    if ((network_joined || (s_ever_joined && s_rejoin_alarm_scheduled)) &&
        xSemaphoreTake(s_motor_sleep_gate, 0) == pdTRUE) {
      if (!motor_work_pending()) {
        motor_prepare_for_sleep();
        esp_zb_sleep_now();
        motor_resume_from_sleep();
        battery_mark_adc_invalid();
      }
      xSemaphoreGive(s_motor_sleep_gate);
    }

#endif
    break;
  default:
    // Keep uncommon Zigbee signals visible during pairing and recovery.
    ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
             esp_zb_zdo_signal_to_string(sig_type), sig_type,
             esp_err_to_name(err_status));
    break;
  }
}

esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                            const void *message) {
  esp_err_t ret = ESP_OK;
  zb_mark_activity();
  s_parent_link_failure_count = 0;
  switch (callback_id) {
  case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
    esp_zb_zcl_set_attr_value_message_t *set_attr_msg =
        (esp_zb_zcl_set_attr_value_message_t *)message;

    ESP_RETURN_ON_FALSE(set_attr_msg, ESP_FAIL, TAG,
                        "Empty set attribute message");

    ESP_LOGD(TAG, "Attribute set: Endpoint: %d, Cluster: 0x%x, Attr: 0x%x",
             set_attr_msg->info.dst_endpoint, set_attr_msg->info.cluster,
             set_attr_msg->attribute.id);

    ESP_RETURN_ON_FALSE(
        set_attr_msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS &&
            set_attr_msg->attribute.data.value != NULL,
        ESP_ERR_INVALID_ARG, TAG, "Invalid attribute write");

    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING) {
      uint16_t attr = set_attr_msg->attribute.id;
      const void *value = set_attr_msg->attribute.data.value;
      uint8_t type = set_attr_msg->attribute.data.type;
      uint8_t mask = 0;
      uint16_t steps = 0;
      uint8_t setting = 0;
      switch (attr) {
      case ZB_ATTR_MOTOR_STEPS:
        ESP_RETURN_ON_FALSE(type == ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ERR_INVALID_ARG,
                            TAG, "Invalid steps type");
        // ZCL value buffers are byte-addressed; do not assume U16 alignment.
        memcpy(&steps, value, sizeof(steps));
        steps = clamp_motor_steps(steps);
        mask = CONFIG_STEPS;
        break;
      case ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID:
        ESP_RETURN_ON_FALSE(type == ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ERR_INVALID_ARG,
                            TAG, "Invalid direction type");
        setting = sanitize_motor_mode(*(const uint8_t *)value);
        mask = CONFIG_MODE;
        break;
      case ZB_ATTR_MOTOR_IRUN:
      case ZB_ATTR_MOTOR_POWER_MODE:
      case ZB_ATTR_MOTOR_SPEED:
        ESP_RETURN_ON_FALSE(type == ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ERR_INVALID_ARG,
                            TAG, "Invalid motor setting type");
        setting = *(const uint8_t *)value;
        if (attr == ZB_ATTR_MOTOR_IRUN) {
          if (setting > 31) setting = 31;
          mask = CONFIG_IRUN;
        } else if (attr == ZB_ATTR_MOTOR_POWER_MODE) {
          if (setting > 1) setting = MOTOR_POWER_MODE_STEALTHCHOP;
          mask = CONFIG_POWER_MODE;
        } else {
          if (setting < 1) setting = 1;
          if (setting > 100) setting = 100;
          mask = CONFIG_SPEED;
        }
        break;
      case ZB_ATTR_COMMAND_REQUEST: {
        ESP_RETURN_ON_FALSE(type == ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ERR_INVALID_ARG,
                            TAG, "Invalid reliable command type");
        diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_COMMAND_DECODE);
        uint32_t packed = command_control_unpack_request(value);
        uint32_t id = packed >> 8;
        uint8_t target = packed & 0xFF;
        ESP_RETURN_ON_FALSE(id && target <= COMMAND_STOP_TARGET,
                            ESP_ERR_INVALID_ARG, TAG, "Invalid reliable command");
        motor_cmd_t cmd = {
            .type = target == COMMAND_STOP_TARGET ? MOTOR_CMD_STOP : MOTOR_CMD_MOVE,
            .endpoint = set_attr_msg->info.dst_endpoint,
            .target_position = target,
            .id = id,
        };
        ret = enqueue_motor_command(cmd);
        break;
      }
      default: break;
      }
      if (mask) {
        portENTER_CRITICAL(&s_move_cmd_lock);
        switch (mask) {
        case CONFIG_STEPS: s_desired_config.steps = steps; break;
        case CONFIG_MODE: s_desired_config.mode = setting; break;
        case CONFIG_IRUN: s_desired_config.irun = setting; break;
        case CONFIG_POWER_MODE: s_desired_config.power_mode = setting; break;
        case CONFIG_SPEED: s_desired_config.speed = setting; break;
        }
        portEXIT_CRITICAL(&s_move_cmd_lock);
        esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                    ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr,
                                    mask == CONFIG_STEPS ? (void *)&steps : (void *)&setting,
                                    false);
        queue_motor_config(mask);
      }
    }

    // Handle calibration trigger (custom attribute 0xF010)
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_CALIBRATE) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        uint8_t trigger_value = *(uint8_t *)set_attr_msg->attribute.data.value;
        if (trigger_value == 1) {
          ESP_LOGI(TAG, "*** CALIBRATION TRIGGERED VIA ZIGBEE ***");

          // Reset the trigger value back to 0 immediately to turn off button
          uint8_t reset_value = 0;
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_CALIBRATE,
                                       &reset_value, false);

          // Prevent sleep during calibration
          if (motor_work_pending() || motor_is_busy()) {
            ESP_LOGW(TAG, "Calibration request ignored (already running/pending)");
            break;
          }

          motor_cmd_t cmd = {.type = MOTOR_CMD_CALIBRATE,
                             .endpoint = set_attr_msg->info.dst_endpoint,
                             .target_position = 0};
          esp_err_t queue_ret = enqueue_motor_command(cmd);
          if (queue_ret != ESP_OK) {
            ret = queue_ret;
            ESP_LOGW(TAG, "Failed to enqueue calibration command: %s",
                     esp_err_to_name(queue_ret));
          }
        }
      }
    }

    // Handle OTA Update Trigger (custom attribute 0xF013)
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_OTA_MODE) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        uint8_t trigger_value = *(uint8_t *)set_attr_msg->attribute.data.value;
        if (trigger_value == 1) {
          ESP_LOGI(TAG, "*** OTA UPDATE TRIGGERED VIA ZIGBEE ***");

          // Reset the trigger value back to 0
          uint8_t reset_value = 0;
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_OTA_MODE,
                                       &reset_value, false);

          portENTER_CRITICAL(&s_move_cmd_lock);
          s_ota_pending = true;
          motor_request_stop();
          portEXIT_CRITICAL(&s_move_cmd_lock);
          xTaskNotifyGive(s_motor_worker);
        }
      }
    }
    break;
  }
  case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
    ESP_LOGD(TAG, "Received default response callback");
    break;
  case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID: {
    esp_zb_zcl_window_covering_movement_message_t *movement_msg =
        (esp_zb_zcl_window_covering_movement_message_t *)message;

    ESP_RETURN_ON_FALSE(movement_msg, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(movement_msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Received message: error status(%d)",
                        movement_msg->info.status);

    ESP_LOGI(TAG, "Window Covering movement command: 0x%04x",
             movement_msg->command);
    motor_cmd_t cmd = {
        .type = MOTOR_CMD_MOVE,
        .endpoint = movement_msg->info.dst_endpoint,
        .target_position = g_current_position,
    };
    bool should_enqueue = false;

    switch (movement_msg->command) {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
      ESP_LOGI(TAG, "  Command: UP/OPEN (tilt to 0%%)");
      cmd.type = MOTOR_CMD_MOVE;
      cmd.target_position = 0;
      should_enqueue = true;
      break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
      ESP_LOGI(TAG, "  Command: DOWN/CLOSE (tilt to 100%%)");
      cmd.type = MOTOR_CMD_MOVE;
      cmd.target_position = 100;
      should_enqueue = true;
      break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
      ESP_LOGI(TAG, "  Command: STOP");
      cmd.type = MOTOR_CMD_STOP;
      cmd.target_position = COMMAND_STOP_TARGET;
      should_enqueue = true;
      break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE:
      ESP_LOGI(TAG, "  Command: GO_TO_TILT_PERCENTAGE = %d%%",
               movement_msg->payload.percentage_tilt_value);
      cmd.type = MOTOR_CMD_MOVE;
      ESP_RETURN_ON_FALSE(movement_msg->payload.percentage_tilt_value <= 100,
                          ESP_ERR_INVALID_ARG, TAG, "Invalid tilt percentage");
      cmd.target_position = movement_msg->payload.percentage_tilt_value;
      should_enqueue = true;
      break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE:
      ESP_LOGI(TAG,
               "  Command: GO_TO_TILT_VALUE = %d (ignored, using percentage)",
               movement_msg->payload.tilt_value);
      break;
    default:
      ESP_LOGI(TAG, "  Command: Unknown (0x%04x)", movement_msg->command);
      break;
    }

    if (should_enqueue) {
      esp_err_t queue_ret = enqueue_motor_command(cmd);
      ret = queue_ret;
      if (queue_ret != ESP_OK) {
        ESP_LOGW(TAG, "  Failed to enqueue motor command: %s",
                 esp_err_to_name(queue_ret));
      }
    }
    break;
  }
  default:
    ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
    break;
  }
  return ret;
}

void esp_zb_task(void *pvParameters) {
  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_TASK_START);
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();

  g_current_position = clamp_position(g_current_position);
  g_motor_steps = clamp_motor_steps(g_motor_steps);
  g_motor_mode = sanitize_motor_mode(g_motor_mode);
  if (g_motor_irun_active > 31) {
    g_motor_irun_active = 31;
  }
  if (g_motor_power_mode > 1) {
    g_motor_power_mode = MOTOR_POWER_MODE_STEALTHCHOP;
  }
  if (g_motor_speed < 1) {
    g_motor_speed = 1;
  }
  if (g_motor_speed > 100) {
    g_motor_speed = 100;
  }

#ifdef CONFIG_PM_ENABLE
  esp_zb_sleep_enable(ZB_SLEEP_ENABLE);
  ESP_LOGI(TAG, "Zigbee sleep %s before stack init",
           ZB_SLEEP_ENABLE ? "enabled" : "disabled");
#endif

  esp_zb_init(&zb_nwk_cfg);

  esp_zb_set_tx_power(ZB_TX_POWER_DBM);
  int8_t applied_tx_power = 0;
  esp_zb_get_tx_power(&applied_tx_power);
  ESP_LOGI(TAG, "Zigbee TX power configured: requested=%d dBm, applied=%d dBm",
           ZB_TX_POWER_DBM, applied_tx_power);

  esp_zb_set_rx_on_when_idle(ZB_RX_ON_WHEN_IDLE);
  ESP_LOGI(TAG, "Zigbee RX_ON_WHEN_IDLE set to %s",
           ZB_RX_ON_WHEN_IDLE ? "true" : "false");

  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_CLUSTER_SETUP);
  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
  esp_zb_endpoint_config_t endpoint_config = {
      .endpoint = ENDPOINT_ID,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = DEVICE_ID,
      .app_device_version = DEVICE_VERSION};

  esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

  esp_zb_attribute_list_t *esp_zb_basic_cluster =
      esp_zb_basic_cluster_create(NULL);
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
      BASIC_MANUFACTURER_NAME));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
      BASIC_MODEL_IDENTIFIER));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
      esp_zb_cluster_list, esp_zb_basic_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  esp_zb_attribute_list_t *esp_zb_identify_cluster =
      esp_zb_identify_cluster_create(NULL);
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
      esp_zb_cluster_list, esp_zb_identify_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  esp_zb_window_covering_cluster_cfg_t window_covering_cfg = {
      .covering_type =
          ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_TILT_BLIND_TILT_ONLY,
      .covering_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL |
                         ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_ONLINE,
      .covering_mode = g_motor_mode, // Use loaded motor mode
  };
  esp_zb_attribute_list_t *esp_zb_window_covering_cluster =
      esp_zb_window_covering_cluster_create(&window_covering_cfg);

  // Tilt position attribute tracks actual blind tilt (0-100%).
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
      ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_tilt_position));

  // Set proper installed tilt limits for correct ZCL tilt percentage math
  uint16_t installed_open_limit_tilt = 0;
  uint16_t installed_closed_limit_tilt = 100;
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_OPEN_LIMIT_TILT_ID,
      ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &installed_open_limit_tilt));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_INSTALLED_CLOSED_LIMIT_TILT_ID,
      ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &installed_closed_limit_tilt));

  // Motor steps stored in custom attribute 0xF004 (not in ZCL limit attributes)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_STEPS, ESP_ZB_ZCL_ATTR_TYPE_U16,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_attr_config.steps));

  // Motor reversal control is handled by the existing Mode attribute
  // Bit 0 (0x01) = Reversed motor direction, already created by cluster config

  // Add calibration trigger attribute (custom attribute 0xF010)
  uint8_t calibration_trigger = 0;
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_CALIBRATE, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
      &calibration_trigger));

  // Reserved legacy attribute 0xF011; no sensorless calibration is implemented.
  uint8_t sensorless_calibration_trigger = 0;
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_SENSORLESS_RESERVED, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
      &sensorless_calibration_trigger));

  // Add motor current control (custom attribute 0xF012, IRUN 0-31)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_IRUN, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_attr_config.irun));

  // Add motor power mode control (custom attribute 0xF014, 0=StealthChop, 1=SpreadCycle)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_POWER_MODE, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_attr_config.power_mode));

  // Add motor speed control (custom attribute 0xF015, 1-100)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_SPEED, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &s_attr_config.speed));

  // TMC2208 diagnostics (custom attributes 0xF030-0xF032)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_TMC_MRES_REQUESTED, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_tmc_mres_requested));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_TMC_MRES_ACTUAL, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_tmc_mres_actual));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_TMC_ERROR, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_tmc_last_error));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_TMC_UART_REGISTER, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_tmc_uart_register));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_TMC_UART_ERROR, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_tmc_uart_error));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_STARTUP_ATTEMPTS, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_motor_startup_attempts));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_STARTUP_RECOVERIES, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_motor_startup_recoveries));

  // STEP RMT diagnostics (custom attributes 0xF033-0xF035)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_RMT_STATUS, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_rmt_status));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_RMT_FAIL_STAGE, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_rmt_fail_stage));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_RMT_ERROR, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &g_rmt_last_error));

  // Add OTA Trigger attribute (custom attribute 0xF013)
  uint8_t ota_trigger = 0;
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_OTA_MODE, ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
      &ota_trigger));

  // Diagnostic attributes for remote reset/restart analysis
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_RESET_REASON, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_last_reset_reason));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_RESTART_CAUSE, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_last_restart_cause));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_REBOOT_COUNT, ESP_ZB_ZCL_ATTR_TYPE_U32,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_reboot_count));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_CRASH_STAGE, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_last_crash_stage));

  struct { uint16_t id; uint8_t type; void *value; } command_attrs[] = {
      {ZB_ATTR_COMMAND_PROTOCOL, ESP_ZB_ZCL_ATTR_TYPE_U8, &s_command_protocol},
      {ZB_ATTR_COMMAND_RESULT, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_result},
      {ZB_ATTR_COMMAND_TARGET, ESP_ZB_ZCL_ATTR_TYPE_U8, &s_command_target},
      {ZB_ATTR_COMMAND_RECEIVED, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_received},
      {ZB_ATTR_COMMAND_COMPLETED, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_completed},
      {ZB_ATTR_COMMAND_FAILED, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_failed},
      {ZB_ATTR_COMMAND_DUPLICATES, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_duplicates},
      {ZB_ATTR_COMMAND_CURSOR, ESP_ZB_ZCL_ATTR_TYPE_U32, &s_command_cursor},
      {ZB_ATTR_POSITION_VALID, ESP_ZB_ZCL_ATTR_TYPE_U8, &s_position_valid},
  };
  for (unsigned i = 0; i < sizeof(command_attrs) / sizeof(command_attrs[0]); ++i) {
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
        esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        command_attrs[i].id, command_attrs[i].type,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        command_attrs[i].value));
  }
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_COMMAND_REQUEST, ESP_ZB_ZCL_ATTR_TYPE_U32,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s_command_request));

  ESP_ERROR_CHECK(esp_zb_cluster_list_add_window_covering_cluster(
      esp_zb_cluster_list, esp_zb_window_covering_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  // Add Power Configuration cluster for battery reporting
  ESP_LOGI(
      TAG,
      "END DEVICE: Power Configuration cluster enabled for battery monitoring");
  esp_zb_power_config_cluster_cfg_t power_cfg = {
      .main_voltage = 0x00,       // Not mains powered (0 = invalid/unknown)
      .main_freq = 0x00,          // Not mains powered (0 = DC supply)
      .main_alarm_mask = 0x00,    // No main power alarms
      .main_voltage_min = 0x00,   // Not applicable for battery device
      .main_voltage_max = 0x00,   // Not applicable for battery device
      .main_voltage_dwell = 0x00, // Not applicable for battery device
  };
  esp_zb_attribute_list_t *esp_zb_power_config_cluster =
      esp_zb_power_config_cluster_create(&power_cfg);

  // Add standard ZCL Power Configuration battery attributes that Z2M expects
  uint8_t battery_voltage =
      0x00; // Attribute 0x0020 - BatteryVoltage (in units of 100mV)
  uint8_t battery_percentage =
      0x00; // Attribute 0x0021 - BatteryPercentageRemaining (in units of 0.5%)
  uint8_t battery_size =
      0xFF; // Attribute 0x0031 - BatterySize (0xFF = unknown)
  uint8_t battery_quantity = 1; // Attribute 0x0033 - BatteryQuantity
  uint8_t battery_rated_voltage =
      42; // Attribute 0x0034 - BatteryRatedVoltage (4.2V in units of 100mV)
  uint8_t battery_alarm_mask = 0x00; // Attribute 0x0035 - BatteryAlarmMask
  uint8_t battery_voltage_min_threshold =
      30; // Attribute 0x0036 - BatteryVoltageMinThreshold (3.0V in units of
          // 100mV)
  uint16_t battery_voltage_precise =
      0x0000; // Custom attribute 0xF100 - Battery voltage in centivolts (0.01V
              // units)

  // Add standard ZCL battery attributes
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0020,
      ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &battery_voltage));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0021,
      ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &battery_percentage));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0031,
      ESP_ZB_ZCL_ATTR_TYPE_8BIT_ENUM, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &battery_size));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0033,
      ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &battery_quantity));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0034,
      ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
      &battery_rated_voltage));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0035,
      ESP_ZB_ZCL_ATTR_TYPE_8BITMAP, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
      &battery_alarm_mask));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 0x0036,
      ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
      &battery_voltage_min_threshold));
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_power_config_cluster, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ZB_ATTR_BATTERY_CENTIVOLTS,
      ESP_ZB_ZCL_ATTR_TYPE_U16,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &battery_voltage_precise));

  ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(
      esp_zb_cluster_list, esp_zb_power_config_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  ESP_ERROR_CHECK(
      esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list, endpoint_config));

  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_DEVICE_REGISTER);
  ESP_ERROR_CHECK(esp_zb_device_register(ep_list));
  esp_zb_core_action_handler_register(zb_action_handler);
  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_START);
  ESP_ERROR_CHECK(esp_zb_start(false));
  for (unsigned i = 0; i < sizeof(s_live_attrs) / sizeof(s_live_attrs[0]); ++i) {
    zb_backing_t *backing = &s_live_attrs[i];
    esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(
        ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, backing->id);
    if (attr && attr->data_p) {
      backing->zcl_value = attr->data_p;
    } else {
      ESP_LOGW(TAG, "Cannot locate Zigbee backing for attribute 0x%04x",
               backing->id);
    }
  }
  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_RUNNING);
  esp_zb_stack_main_loop();
}

void zb_network_init(void) {
  esp_log_level_set(TAG, ESP_LOG_INFO);
  // Initialize power management for light sleep
  ESP_ERROR_CHECK(zb_network_init_power_management());

  esp_zb_platform_config_t config = {
      .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
      .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  s_desired_config = (motor_config_t){g_motor_steps, g_motor_mode,
      g_motor_irun_active, g_motor_power_mode, g_motor_speed};
  s_attr_config = s_desired_config;
  s_position_valid = motor_position_valid();
  s_tilt_position = s_position_valid ? clamp_position(g_current_position) : 0xFF;
  s_motor_sleep_gate = xSemaphoreCreateMutex();
  ESP_ERROR_CHECK(s_motor_sleep_gate ? ESP_OK : ESP_ERR_NO_MEM);
#ifdef CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "motor_work",
                                     &s_motor_pm_lock));
#endif
  BaseType_t task_created = xTaskCreate(motor_worker_task, "motor_worker",
                                         4096, NULL, 4, &s_motor_worker);
  ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

  if (!s_health_task_started) {
    BaseType_t health_task_created = xTaskCreate(
        zb_health_monitor_task, "zb_health", 3072, NULL, 2, NULL);
    if (health_task_created != pdPASS) {
      ESP_LOGE(TAG, "Failed to create Zigbee health monitor task");
    } else {
      s_health_task_started = true;
      zb_mark_activity();
    }
  }

  ESP_LOGI(TAG, "Zigbee platform configured");
}

static void zb_check_battery_report(uint8_t param) {
  ESP_LOGI(TAG, "Battery check task triggered (param: %d)", param);

  // Schedule the next check first so a failed measurement does not stop
  // reporting.
  ESP_LOGI(TAG, "Scheduling next battery check in %d minutes",
           BATTERY_REPORTING_INTERVAL / (60 * 1000));
  esp_zb_scheduler_alarm_cancel(zb_check_battery_report, 0);
  esp_zb_scheduler_alarm(zb_check_battery_report, 0,
                         BATTERY_REPORTING_INTERVAL);

  // Only proceed with battery measurement if network is joined
  if (!network_joined) {
    ESP_LOGW(TAG, "Battery check skipped - network not joined yet");
    return;
  }

  // Each scheduled check measures and reports battery state.
  ESP_LOGI(TAG, "Performing battery measurement and reporting");

  // battery_should_report() measures once and caches the values below.
  if (battery_should_report()) {
    // Use cached values to avoid duplicate ADC readings in this cycle.
    float voltage = battery_get_cached_voltage();
    uint8_t percentage = battery_get_cached_percentage();

    ESP_LOGD(TAG, "Battery check: updating Zigbee attributes");

    // Update battery voltage attribute (0x0020) - in units of 100mV
    uint8_t battery_voltage_100mv = (uint8_t)(voltage * 10.0f);
    esp_err_t ret = esp_zb_zcl_set_attribute_val(
        ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0x0020, &battery_voltage_100mv, false);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set battery voltage attribute 0x0020: %s",
               esp_err_to_name(ret));
    }

    // Update precise battery voltage attribute (0xF100) - in units of 0.01V
    // (centivolts)
    uint16_t battery_voltage_centivolt = (uint16_t)(voltage * 100.0f);
    ret = esp_zb_zcl_set_attribute_val(ENDPOINT_ID,
                                       ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_BATTERY_CENTIVOLTS,
                                       &battery_voltage_centivolt, false);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "Failed to set precise battery voltage attribute ZB_ATTR_BATTERY_CENTIVOLTS: %s",
               esp_err_to_name(ret));
    }

    // Update battery percentage attribute (0x0021) - in units of 0.5%
    // Use cached percentage (0-100%) and convert to ZCL format (0-200 in 0.5%
    // units)
    uint8_t battery_percentage_half = (uint8_t)(percentage * 2);
    if (battery_percentage_half > 200) {
      battery_percentage_half = 200;
    }
    ret = esp_zb_zcl_set_attribute_val(ENDPOINT_ID,
                                       ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0x0021,
                                       &battery_percentage_half, false);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set battery percentage attribute 0x0021: %s",
               esp_err_to_name(ret));
    }

    // Send reports immediately after updating local ZCL attributes.
    zb_send_battery_report_attr(0x0020); // Battery voltage (100mV)
    zb_send_battery_report_attr(0x0021); // Battery percentage (0.5%)
    zb_send_battery_report_attr(ZB_ATTR_BATTERY_CENTIVOLTS); // Precise battery voltage (0.01V)

    ESP_LOGI(TAG,
             "Battery reported: %.2fV (%d%%) - ZCL values: voltage=%d (100mV "
             "units), precise=%d (centivolts), percentage=%d (0.5%% units)",
             voltage, percentage, battery_voltage_100mv,
             battery_voltage_centivolt, battery_percentage_half);
  } else {
    ESP_LOGE(TAG, "Battery measurement failed - will retry in next cycle");
  }
}

static void zb_send_battery_report_attr(uint16_t attribute_id) {
  if (!zb_attr_can_report(ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
                          attribute_id, "battery")) {
    return;
  }

  esp_zb_zcl_report_attr_cmd_t report_cmd = {
      .zcl_basic_cmd =
          {
              .src_endpoint = ENDPOINT_ID,
          },
      .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
      .clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      .manuf_specific = 0,
      .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
      .dis_default_resp = 1,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attributeID = attribute_id,
  };

  esp_err_t report_ret = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
  if (report_ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send battery report for attribute 0x%04X: %s",
             attribute_id, esp_err_to_name(report_ret));
  } else {
    ESP_LOGD(TAG, "Battery attribute 0x%04X report sent to coordinator",
             attribute_id);
  }
}

// Send a single window covering cluster attribute report to the coordinator.
static void zb_send_wc_report_attr(uint16_t attribute_id,
                                   const char *attr_name) {
  if (!zb_attr_can_report(ENDPOINT_ID, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                          attribute_id, attr_name)) {
    return;
  }

  esp_zb_zcl_report_attr_cmd_t cmd = {
      .zcl_basic_cmd =
          {
              .src_endpoint = ENDPOINT_ID,
          },
      .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
      .clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      .manuf_specific = 0,
      .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
      .dis_default_resp = 1,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attributeID = attribute_id,
  };
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&cmd);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to report %s (0x%04X): %s", attr_name, attribute_id,
             esp_err_to_name(ret));
  }
}

// Report motor configuration values to the coordinator so Z2M/HA shows them.
// Called on network join and device reboot.
static void zb_report_motor_config(void) {
  ESP_LOGI(TAG, "Reporting motor config: irun=%u, pmode=%u, speed=%u, steps=%u",
           g_motor_irun_active, g_motor_power_mode, g_motor_speed, g_motor_steps);

  zb_send_wc_report_attr(ZB_ATTR_MOTOR_IRUN, "motor_irun_active");
  zb_send_wc_report_attr(ZB_ATTR_MOTOR_POWER_MODE, "motor_power_mode");
  zb_send_wc_report_attr(ZB_ATTR_MOTOR_SPEED, "motor_speed");
  zb_send_wc_report_attr(ZB_ATTR_MOTOR_STEPS, "motor_steps");
  // Non-reportable state is read by the coordinator during configure/get.
  zb_send_wc_report_attr(ZB_ATTR_RESET_REASON, "last_reset_reason");
  zb_send_wc_report_attr(ZB_ATTR_RESTART_CAUSE, "restart_cause");
  zb_send_wc_report_attr(ZB_ATTR_REBOOT_COUNT, "reboot_count");
  zb_send_wc_report_attr(ZB_ATTR_CRASH_STAGE, "last_crash_stage");
  zb_send_wc_report_attr(
      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
      "tilt_position");
}

// Wrapper matching esp_zb_callback_t for scheduler alarm
static void zb_report_motor_config_delayed(uint8_t param) {
  (void)param;
  zb_report_motor_config();
}

// Send a Zigbee attribute report for the tilt position to the coordinator.
// This is needed so Z2M and Home Assistant update the cover state after
// movement.
static void zb_send_tilt_report(uint8_t endpoint) {
  if (!zb_attr_can_report(
          endpoint, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
          ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
          "tilt_position")) {
    return;
  }

  esp_zb_zcl_report_attr_cmd_t report_cmd = {
      .zcl_basic_cmd =
          {
              .src_endpoint = endpoint,
          },
      .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
      .clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      .manuf_specific = 0,
      .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
      .dis_default_resp = 1,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attributeID =
          ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
  };

  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_cmd);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send tilt position report: %s",
             esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "Tilt position report sent to coordinator (pos=%d%%)",
             g_current_position);
  }
}
