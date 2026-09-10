#include "zb_network.h"
#include "zb_attributes.h"
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
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led.h"
#include "main.h"
#include "motor.h"
#include "nwk/esp_zigbee_nwk.h"
#include "storage.h"
#include "zcl/esp_zigbee_zcl_command.h"

static const char *TAG = "ZB_NETWORK";

#define MOTOR_CMD_QUEUE_LEN 8
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

typedef enum {
  MOTOR_CMD_MOVE = 0,
  MOTOR_CMD_STOP,
  MOTOR_CMD_CALIBRATE,
} motor_cmd_type_t;

typedef struct {
  motor_cmd_type_t type;
  uint8_t endpoint;
  uint8_t target_position; // only for MOTOR_CMD_MOVE
} motor_cmd_t;

static QueueHandle_t s_motor_cmd_queue = NULL;
static bool s_calibration_pending = false;
static volatile TickType_t s_awake_until_tick = 0;
static portMUX_TYPE s_move_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_move_token_pending = false;
static uint8_t s_latest_move_endpoint = ENDPOINT_ID;
static uint8_t s_latest_move_target = 0;

// Forward declarations
static void zb_check_battery_report(uint8_t param);
static void zb_bdb_start_commissioning_wrapper(uint8_t mode_mask);
static void zb_send_battery_report_attr(uint16_t attribute_id);
static void zb_report_motor_config(void);
static void zb_report_motor_config_delayed(uint8_t param);
static void zb_send_tilt_report(uint8_t endpoint);
static void motor_worker_task(void *pvParameters);
static esp_err_t enqueue_motor_command(motor_cmd_t cmd);
static void zb_publish_tilt_position(uint8_t endpoint);
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
  TickType_t now = xTaskGetTickCount();
  s_awake_until_tick = now + pdMS_TO_TICKS(POST_MOTION_AWAKE_WINDOW_MS);
  ESP_LOGI(TAG, "Post-motion awake window enabled for %u ms",
           (unsigned)POST_MOTION_AWAKE_WINDOW_MS);
}

static bool awake_window_active(uint32_t *remaining_ms) {
  TickType_t now = xTaskGetTickCount();
  TickType_t until = s_awake_until_tick;
  int32_t ticks_left = (int32_t)(until - now);
  if (ticks_left > 0) {
    if (remaining_ms) {
      *remaining_ms = (uint32_t)ticks_left * (uint32_t)portTICK_PERIOD_MS;
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
  esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

static void zb_schedule_network_steering(const char *reason, uint32_t delay_ms) {
  if (s_rejoin_alarm_scheduled) {
    ESP_LOGW(TAG, "Rejoin already scheduled (%s)", reason);
    return;
  }

  network_joined = false;
  s_last_join_state_change_us = esp_timer_get_time();
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

static esp_err_t enqueue_motor_command(motor_cmd_t cmd) {
  if (!s_motor_cmd_queue) {
    ESP_LOGE(TAG, "Motor queue not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (cmd.type == MOTOR_CMD_MOVE) {
    bool should_enqueue_move = false;
    uint8_t endpoint = cmd.endpoint ? cmd.endpoint : ENDPOINT_ID;
    cmd.target_position = clamp_position(cmd.target_position);

    // Preempt current motion so the new target is handled quickly.
    if (motor_is_busy()) {
      motor_request_stop();
    }

    portENTER_CRITICAL(&s_move_cmd_lock);
    s_latest_move_endpoint = endpoint;
    s_latest_move_target = cmd.target_position;
    if (!s_move_token_pending) {
      s_move_token_pending = true;
      should_enqueue_move = true;
    }
    portEXIT_CRITICAL(&s_move_cmd_lock);

    if (!should_enqueue_move) {
      return ESP_OK;
    }

    cmd.endpoint = endpoint;
  }

  if (xQueueSend(s_motor_cmd_queue, &cmd, 0) != pdTRUE) {
    if (cmd.type == MOTOR_CMD_MOVE) {
      portENTER_CRITICAL(&s_move_cmd_lock);
      s_move_token_pending = false;
      portEXIT_CRITICAL(&s_move_cmd_lock);
    }
    ESP_LOGW(TAG, "Motor queue full, dropping command type=%d", (int)cmd.type);
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

static void zb_publish_tilt_position(uint8_t endpoint) {
  uint8_t ep = endpoint ? endpoint : ENDPOINT_ID;
  uint8_t position = clamp_position(g_current_position);
  g_current_position = position;

  if (!esp_zb_lock_acquire(pdMS_TO_TICKS(2000))) {
    ESP_LOGW(TAG, "Failed to acquire Zigbee lock for tilt publish");
    return;
  }

  esp_err_t ret = esp_zb_zcl_set_attribute_val(
      ep, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID,
      &position, false);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set tilt position attribute: %s",
             esp_err_to_name(ret));
  }

  zb_send_tilt_report(ep);
  esp_zb_lock_release();
}

static void motor_worker_task(void *pvParameters) {
  (void)pvParameters;
  motor_cmd_t cmd;
  diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);

  while (1) {
    if (xQueueReceive(s_motor_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    uint8_t endpoint = cmd.endpoint ? cmd.endpoint : ENDPOINT_ID;
    diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);

    switch (cmd.type) {
    case MOTOR_CMD_STOP:
      motor_stop();
      portENTER_CRITICAL(&s_move_cmd_lock);
      s_move_token_pending = false;
      s_latest_move_endpoint = endpoint;
      s_latest_move_target = g_current_position;
      portEXIT_CRITICAL(&s_move_cmd_lock);
      calibration_in_progress = false;
      extend_awake_window_after_motion();
      break;
    case MOTOR_CMD_MOVE: {
      uint8_t move_endpoint = ENDPOINT_ID;
      uint8_t move_target = g_current_position;

      portENTER_CRITICAL(&s_move_cmd_lock);
      move_endpoint = s_latest_move_endpoint;
      move_target = s_latest_move_target;
      s_move_token_pending = false;
      portEXIT_CRITICAL(&s_move_cmd_lock);

      calibration_in_progress = true;
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_MOVE);
      (void)motor_move_to_position(move_target);
      calibration_in_progress = false;
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);
      extend_awake_window_after_motion();

      bool next_move_pending = false;
      portENTER_CRITICAL(&s_move_cmd_lock);
      next_move_pending = s_move_token_pending;
      portEXIT_CRITICAL(&s_move_cmd_lock);

      // Prioritize command responsiveness: if there is already a newer move
      // queued, execute it first and publish only final position afterwards.
      if (!next_move_pending) {
        storage_save_position(g_current_position);
        zb_publish_tilt_position(move_endpoint);
      } else {
        ESP_LOGD(TAG, "Skipping intermediate save/report, newer move pending");
      }
      break;
    }
    case MOTOR_CMD_CALIBRATE: {
      ESP_LOGI(TAG, "*** CALIBRATION STARTED ***");
      calibration_in_progress = true;
      diagnostics_mark_stage(DIAG_STAGE_CALIBRATION);

      g_current_position = 100;
      bool completed = motor_move_to_position(0);
      if (completed) {
        zb_publish_tilt_position(endpoint);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }

      if (completed) {
        completed = motor_move_to_position(100);
        if (completed) {
          zb_publish_tilt_position(endpoint);
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }

      if (completed) {
        completed = motor_move_to_position(50);
      }

      calibration_in_progress = false;
      s_calibration_pending = false;
      diagnostics_mark_stage(DIAG_STAGE_MOTOR_WORKER);
      extend_awake_window_after_motion();

      storage_save_position(g_current_position);
      zb_publish_tilt_position(endpoint);

      if (completed) {
        ESP_LOGI(TAG, "*** CALIBRATION COMPLETED - pos=%u%% ***",
                 g_current_position);
      } else {
        ESP_LOGW(TAG, "*** CALIBRATION ABORTED - pos=%u%% ***",
                 g_current_position);
      }
      break;
    }
    default:
      ESP_LOGW(TAG, "Unknown motor command type: %d", (int)cmd.type);
      break;
    }
  }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  zb_mark_activity();

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

    if (calibration_in_progress) {
      break;
    } else if (network_joined) {
      if (awake_window_active(NULL)) {
        break;
      }

      s_parent_link_failure_count = 0;
      motor_prepare_for_sleep();
      esp_zb_sleep_now();
      motor_resume_from_sleep();
      zb_mark_activity();
      // ADC handles need to be recreated after light sleep.
      battery_mark_adc_invalid();
    } else if (s_ever_joined && s_rejoin_alarm_scheduled) {
      motor_prepare_for_sleep();
      esp_zb_sleep_now();
      motor_resume_from_sleep();
      battery_mark_adc_invalid();
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

    // Handle motor steps configuration via custom attribute 0xF004
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_MOTOR_STEPS) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
        uint16_t requested_steps = *(uint16_t *)set_attr_msg->attribute.data.value;
        uint16_t new_motor_steps = clamp_motor_steps(requested_steps);
        if (new_motor_steps != g_motor_steps) {
          g_motor_steps = new_motor_steps;
          ESP_LOGI(TAG, "*** MOTOR STEPS SET TO: %d (actual=%u) ***",
                   g_motor_steps,
                   (unsigned int)MOTOR_STEPS_TO_ACTUAL(g_motor_steps));
          storage_save_motor_steps(g_motor_steps);
        }

        if (new_motor_steps != requested_steps) {
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_MOTOR_STEPS,
                                       &new_motor_steps, false);
        }
      }
    }

    // Handle motor reversal via window covering mode attribute
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BITMAP) {
        uint8_t requested_mode = *(uint8_t *)set_attr_msg->attribute.data.value;
        uint8_t new_mode = sanitize_motor_mode(requested_mode);
        if (new_mode != g_motor_mode) {
          g_motor_mode = new_mode;
          ESP_LOGI(TAG, "*** MOTOR DIRECTION SET TO: %d (%s) ***", g_motor_mode,
                   (g_motor_mode == 1) ? "REVERSED" : "NORMAL");
          storage_save_motor_mode(g_motor_mode);
        }

        if (new_mode != requested_mode) {
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                       ESP_ZB_ZCL_ATTR_WINDOW_COVERING_MODE_ID,
                                       &new_mode, false);
        }
      }
    }

    // Handle motor active current (IRUN) via custom attribute 0xF012
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_MOTOR_IRUN) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        uint8_t requested_irun = *(uint8_t *)set_attr_msg->attribute.data.value;
        uint8_t new_irun = requested_irun;
        if (new_irun > 31) {
          new_irun = 31;
        }

        if (new_irun != g_motor_irun_active) {
          g_motor_irun_active = new_irun;
          ESP_LOGI(TAG, "*** MOTOR IRUN ACTIVE SET TO: %u ***",
                   g_motor_irun_active);
          storage_save_motor_irun_active(g_motor_irun_active);
          motor_set_irun_active(g_motor_irun_active);
        }

        // If caller sent an out-of-range value, correct the attribute
        // on-device.
        if (new_irun != requested_irun) {
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_MOTOR_IRUN,
                                       &new_irun, false);
        }
      }
    }

    // Handle motor power mode (custom attribute 0xF014)
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_MOTOR_POWER_MODE) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        uint8_t requested_mode = *(uint8_t *)set_attr_msg->attribute.data.value;
        uint8_t new_mode = requested_mode > 1 ? 0 : requested_mode;

        if (new_mode != g_motor_power_mode) {
          g_motor_power_mode = new_mode;
          ESP_LOGI(TAG, "*** MOTOR POWER MODE SET TO: %u (%s) ***",
                   g_motor_power_mode,
                   g_motor_power_mode ? "SpreadCycle" : "StealthChop");
          storage_save_motor_power_mode(g_motor_power_mode);
          motor_set_power_mode(g_motor_power_mode);
        }

        if (new_mode != requested_mode) {
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_MOTOR_POWER_MODE,
                                       &new_mode, false);
        }
      }
    }

    // Handle motor speed (custom attribute 0xF015)
    if (set_attr_msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING &&
        set_attr_msg->attribute.id == ZB_ATTR_MOTOR_SPEED) {
      if (set_attr_msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
        uint8_t requested_speed = *(uint8_t *)set_attr_msg->attribute.data.value;
        uint8_t new_speed = requested_speed;
        if (new_speed < 1) new_speed = 1;
        if (new_speed > 100) new_speed = 100;

        if (new_speed != g_motor_speed) {
          g_motor_speed = new_speed;
          ESP_LOGI(TAG, "*** MOTOR SPEED SET TO: %u ***", g_motor_speed);
          storage_save_motor_speed(g_motor_speed);
          motor_set_speed(g_motor_speed);
        }

        if (new_speed != requested_speed) {
          esp_zb_zcl_set_attribute_val(set_attr_msg->info.dst_endpoint,
                                       ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                       ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ATTR_MOTOR_SPEED,
                                       &new_speed, false);
        }
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
          if (calibration_in_progress || s_calibration_pending || motor_is_busy()) {
            ESP_LOGW(TAG, "Calibration request ignored (already running/pending)");
            break;
          }

          s_calibration_pending = true;
          motor_cmd_t cmd = {.type = MOTOR_CMD_CALIBRATE,
                             .endpoint = set_attr_msg->info.dst_endpoint,
                             .target_position = 0};
          esp_err_t queue_ret = enqueue_motor_command(cmd);
          if (queue_ret != ESP_OK) {
            s_calibration_pending = false;
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

          ESP_LOGI(TAG, "Setting OTA mode flag and rebooting...");
          storage_save_ota_mode(true);
          storage_save_pending_restart_cause(RESTART_CAUSE_OTA_MODE_TRIGGER);

          vTaskDelay(pdMS_TO_TICKS(1000));
          esp_restart();
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
      motor_request_stop();
      cmd.type = MOTOR_CMD_STOP;
      should_enqueue = true;
      break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE:
      ESP_LOGI(TAG, "  Command: GO_TO_TILT_PERCENTAGE = %d%%",
               movement_msg->payload.percentage_tilt_value);
      cmd.type = MOTOR_CMD_MOVE;
      cmd.target_position = clamp_position(movement_msg->payload.percentage_tilt_value);
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
      &g_current_position));

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
      &g_motor_steps));

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
      &g_motor_irun_active));

  // Add motor power mode control (custom attribute 0xF014, 0=StealthChop, 1=SpreadCycle)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_POWER_MODE, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_motor_power_mode));

  // Add motor speed control (custom attribute 0xF015, 1-100)
  ESP_ERROR_CHECK(esp_zb_cluster_add_attr(
      esp_zb_window_covering_cluster, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
      ZB_ATTR_MOTOR_SPEED, ESP_ZB_ZCL_ATTR_TYPE_U8,
      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
      &g_motor_speed));

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
  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_RUNNING);
  esp_zb_stack_main_loop();
}

void zb_network_init(void) {
  // Initialize power management for light sleep
  ESP_ERROR_CHECK(zb_network_init_power_management());

  esp_zb_platform_config_t config = {
      .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
      .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  if (!s_motor_cmd_queue) {
    s_motor_cmd_queue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(motor_cmd_t));
    if (!s_motor_cmd_queue) {
      ESP_LOGE(TAG, "Failed to create motor command queue");
      return;
    }

    BaseType_t task_created =
        xTaskCreate(motor_worker_task, "motor_worker", 4096, NULL, 4, NULL);
    if (task_created != pdPASS) {
      ESP_LOGE(TAG, "Failed to create motor worker task");
      vQueueDelete(s_motor_cmd_queue);
      s_motor_cmd_queue = NULL;
      return;
    }
  }

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
