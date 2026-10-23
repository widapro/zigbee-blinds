#include "main.h"
#include "battery.h"
#include "ble_ota.h"
#include "diagnostics.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "motor.h"
#include "nvs_flash.h"
#include "storage.h"
#include "zb_network.h"

static const char *TAG = "MAIN";

// Runtime settings are loaded from NVS before the Zigbee worker starts.
uint8_t g_current_position = DEFAULT_POSITION;
uint16_t g_motor_steps = DEFAULT_MOTOR_STEPS;
uint8_t g_motor_mode = DEFAULT_MOTOR_MODE;
uint8_t g_motor_irun_active = DEFAULT_MOTOR_IRUN_ACTIVE;
uint8_t g_motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
uint8_t g_motor_speed = DEFAULT_MOTOR_SPEED;
uint8_t g_tmc_mres_requested = MOTOR_MICROSTEP_MODE;
uint8_t g_tmc_mres_actual = 0xFF;
uint8_t g_tmc_last_error = 0;
uint8_t g_tmc_uart_register = 0xFF;
uint32_t g_tmc_uart_error = ESP_OK;
uint8_t g_motor_startup_attempts = 0;
uint32_t g_motor_startup_recoveries = 0;
uint8_t g_rmt_status = 0;
uint8_t g_rmt_fail_stage = 0;
uint32_t g_rmt_last_error = 0;
uint8_t g_last_reset_reason = ESP_RST_UNKNOWN;
uint8_t g_last_restart_cause = RESTART_CAUSE_NONE;
uint32_t g_reboot_count = 0;

// Function declarations
static void factory_reset_monitor_task(void *pvParameters);
static const char *reset_reason_to_str(esp_reset_reason_t reason);
static const char *restart_cause_to_str(uint8_t cause);

static const char *reset_reason_to_str(esp_reset_reason_t reason) {
  switch (reason) {
  case ESP_RST_POWERON:
    return "POWERON";
  case ESP_RST_SW:
    return "SW";
  case ESP_RST_PANIC:
    return "PANIC";
  case ESP_RST_INT_WDT:
    return "INT_WDT";
  case ESP_RST_TASK_WDT:
    return "TASK_WDT";
  case ESP_RST_WDT:
    return "WDT";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT";
  case ESP_RST_SDIO:
    return "SDIO";
  default:
    return "UNKNOWN";
  }
}

static const char *restart_cause_to_str(uint8_t cause) {
  switch (cause) {
  case RESTART_CAUSE_NONE:
    return "NONE";
  case RESTART_CAUSE_FACTORY_RESET:
    return "FACTORY_RESET";
  case RESTART_CAUSE_OTA_MODE_TRIGGER:
    return "OTA_MODE_TRIGGER";
  case RESTART_CAUSE_OTA_TIMEOUT:
    return "OTA_TIMEOUT";
  case RESTART_CAUSE_OTA_SUCCESS:
    return "OTA_SUCCESS";
  case RESTART_CAUSE_ZIGBEE_OFFLINE_STALL:
    return "ZIGBEE_OFFLINE_STALL";
  case RESTART_CAUSE_ZIGBEE_INACTIVE:
    return "ZIGBEE_INACTIVE";
  default:
    return "UNKNOWN";
  }
}

void factory_reset(void) {
  ESP_LOGI(TAG, "Factory reset triggered! Blinking LED blue first...");
  led_blink_blue(2); // Blink blue twice before factory reset.

  ESP_LOGI(TAG, "Performing factory reset...");

  // Clear app NVS before Zigbee credentials so both settings and pairing reset.
  ESP_LOGI(TAG, "Clearing custom NVS storage...");
  esp_err_t storage_ret = storage_reset_all();
  if (storage_ret == ESP_OK) {
    ESP_LOGI(TAG, "Custom NVS storage cleared successfully");
  } else {
    ESP_LOGE(TAG, "Failed to clear custom NVS storage: %s",
             esp_err_to_name(storage_ret));
  }

  // Reset runtime values to match the cleared NVS defaults.
  g_current_position = DEFAULT_POSITION;
  g_motor_steps = DEFAULT_MOTOR_STEPS;
  g_motor_mode = DEFAULT_MOTOR_MODE;
  g_motor_irun_active = DEFAULT_MOTOR_IRUN_ACTIVE;
  g_motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
  g_motor_speed = DEFAULT_MOTOR_SPEED;
  ESP_LOGI(TAG,
           "Reset global variables to defaults: pos=%d%%, steps=%d "
           "(actual=%u), mode=%d, irun=%u",
           g_current_position, g_motor_steps,
           (unsigned int)MOTOR_STEPS_TO_ACTUAL(g_motor_steps), g_motor_mode,
           g_motor_irun_active);

  // Verify the reset by reading defaults back from storage.
  uint8_t verify_pos;
  uint16_t verify_steps;
  uint8_t verify_mode;
  uint8_t verify_irun;
  storage_load_position(&verify_pos);
  storage_load_motor_steps(&verify_steps);
  storage_load_motor_mode(&verify_mode);
  storage_load_motor_irun_active(&verify_irun);
  ESP_LOGI(TAG,
           "Verification - loaded from storage after reset: pos=%d%%, steps=%d "
           "(actual=%u), mode=%d, irun=%u",
           verify_pos, verify_steps,
           (unsigned int)MOTOR_STEPS_TO_ACTUAL(verify_steps), verify_mode,
           verify_irun);

  // The SDK factory-reset call erases Zigbee storage and restarts internally.
  // Persist diagnostics before calling it, and hold the lock from this task.
  storage_save_pending_restart_cause(RESTART_CAUSE_FACTORY_RESET);
  if (!esp_zb_lock_acquire(pdMS_TO_TICKS(2000))) {
    ESP_LOGE(TAG, "Cannot acquire Zigbee lock for factory reset; retry BOOT");
    return;
  }
  ESP_LOGI(TAG, "Clearing Zigbee network credentials and restarting...");
  esp_zb_factory_reset();
  esp_zb_lock_release();
}

// FreeRTOS queue for button events
static QueueHandle_t gpio_evt_queue = NULL;

// ISR handler for factory reset button
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void factory_reset_monitor_task(void *pvParameters) {
  // Create queue for GPIO events
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  if (gpio_evt_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create GPIO event queue");
    vTaskDelete(NULL);
    return;
  }

  // Configure GPIO with interrupt on falling edge (button press)
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE // Interrupt on button press (falling edge)
  };
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure factory reset GPIO%d: %s",
             FACTORY_RESET_GPIO, esp_err_to_name(ret));
    vTaskDelete(NULL);
    return;
  }

  // Install GPIO ISR service
  gpio_install_isr_service(0);
  gpio_isr_handler_add(FACTORY_RESET_GPIO, gpio_isr_handler,
                       (void *)FACTORY_RESET_GPIO);

  ESP_LOGI(TAG,
           "Factory reset monitor started (interrupt-based) - hold boot button "
           "for %d seconds to factory reset",
           FACTORY_RESET_HOLD_TIME_MS / 1000);

  uint32_t gpio_num;
  while (1) {
    // Wait indefinitely for button press interrupts. The task remains blocked
    // while the CPU is allowed to sleep.
    if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
      ESP_LOGI(TAG,
               "Boot button pressed (GPIO%d), starting factory reset timer...",
               FACTORY_RESET_GPIO);

      uint32_t start_time = xTaskGetTickCount();
      while (gpio_get_level(FACTORY_RESET_GPIO) == 0) {
        uint32_t elapsed =
            (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;

        // Show progress every second
        if (elapsed % 1000 < 100) {
          ESP_LOGI(TAG, "Factory reset timer: %lu/%d ms", elapsed,
                   FACTORY_RESET_HOLD_TIME_MS);
        }

        if (elapsed >= FACTORY_RESET_HOLD_TIME_MS) {
          ESP_LOGI(TAG, "Factory reset triggered after %lu ms!", elapsed);
          factory_reset();
          break; // If reset could not start, wait for another button press.
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      ESP_LOGI(TAG, "Boot button released after %lu ms",
               (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS);
    }
  }
}

void app_main(void) {
  esp_reset_reason_t reset_reason = esp_reset_reason();
  diagnostics_init(reset_reason);
  g_last_reset_reason = (uint8_t)reset_reason;
  ESP_LOGI(TAG, "Boot reset reason: %s (%d)", reset_reason_to_str(reset_reason),
           (int)reset_reason);

  // Initialize LED and show startup indication
  diagnostics_mark_stage(DIAG_STAGE_LED_INIT);
  ESP_ERROR_CHECK(led_init());
  led_blink_green(1);
  ESP_LOGI(TAG, "LED system initialized");

  // Initialize NVS storage first
  diagnostics_mark_stage(DIAG_STAGE_STORAGE_INIT);
  ESP_LOGI(TAG, "Initializing NVS storage...");
  ESP_ERROR_CHECK(storage_init());
  diagnostics_mark_stage(DIAG_STAGE_BOOT_DIAGNOSTICS);
  esp_err_t diag_err = storage_record_boot_diagnostics(
      g_last_reset_reason, &g_last_restart_cause, &g_reboot_count);
  if (diag_err == ESP_OK) {
    ESP_LOGW(TAG,
             "Boot diagnostics: reset=%s (%u), restart_cause=%s (%u), "
             "reboot_count=%lu",
             reset_reason_to_str(reset_reason), g_last_reset_reason,
             restart_cause_to_str(g_last_restart_cause), g_last_restart_cause,
             (unsigned long)g_reboot_count);
  }

  // Check for OTA mode immediately after NVS is available.
  diagnostics_mark_stage(DIAG_STAGE_OTA_MODE_CHECK);
  bool ota_mode = false;
  esp_err_t ota_err = storage_load_ota_mode(&ota_mode);
  ESP_LOGI(TAG, "OTA mode check: err=%d, ota_mode=%d", ota_err, ota_mode);

  if (ota_mode) {
    ESP_LOGI(TAG, "*** BOOTING INTO BLE OTA MODE ***");

    // Clear the flag immediately (if OTA fails/timeout, device returns to
    // Zigbee)
    storage_save_ota_mode(false);

    // Show solid blue LED for OTA mode
    led_solid_blue();

    diagnostics_mark_stage(DIAG_STAGE_BLE_OTA_START);
    start_ble_ota_mode();
    // start_ble_ota_mode() normally owns the task after this point.
    return;
  }

  // Normal boot - continue with full initialization
  ESP_LOGI(TAG, "Normal boot mode - continuing with Zigbee...");

  // Load saved parameters or use defaults
  diagnostics_mark_stage(DIAG_STAGE_LOAD_CONFIG);
  ESP_LOGI(TAG, "Loading saved parameters...");
  ESP_ERROR_CHECK(storage_load_position(&g_current_position));
  bool position_valid;
  ESP_ERROR_CHECK(storage_load_position_valid(&position_valid));
  motor_set_position_valid(position_valid);
  if (!position_valid) {
    ESP_LOGW(TAG, "Position unknown after interrupted motion; calibration required");
  }
  ESP_ERROR_CHECK(storage_load_motor_steps(&g_motor_steps));
  ESP_ERROR_CHECK(storage_load_motor_mode(&g_motor_mode));
  ESP_ERROR_CHECK(storage_load_motor_irun_active(&g_motor_irun_active));
  ESP_ERROR_CHECK(storage_load_motor_power_mode(&g_motor_power_mode));
  ESP_ERROR_CHECK(storage_load_motor_speed(&g_motor_speed));

  ESP_LOGI(TAG,
           "Configuration loaded - Position: %d%%, Steps: %d (actual=%u), "
           "Mode: %d, IRUN: %u, PowerMode: %s, Speed: %u",
           g_current_position, g_motor_steps,
           (unsigned int)MOTOR_STEPS_TO_ACTUAL(g_motor_steps), g_motor_mode,
           g_motor_irun_active,
           g_motor_power_mode ? "SpreadCycle" : "StealthChop",
           g_motor_speed);

  // Initialize battery monitoring
  diagnostics_mark_stage(DIAG_STAGE_BATTERY_INIT);
  ESP_LOGI(TAG, "Initializing battery monitoring...");
  ESP_ERROR_CHECK(battery_init());

  // Initialize motor control
  diagnostics_mark_stage(DIAG_STAGE_MOTOR_INIT);
  ESP_LOGI(TAG, "Initializing motor control...");
  motor_init();

  // Extra GPIO sleep optimization is left off because it caused ESP32-H2 boot
  // loops during hardware testing.
  ESP_LOGI(TAG, "Skipping GPIO optimization (ESP32-H2 hardware limitation)");

  // OTA mode was already handled above, after NVS init.

  // Initialize Zigbee platform
  diagnostics_mark_stage(DIAG_STAGE_ZIGBEE_NETWORK_INIT);
  zb_network_init();

  ESP_LOGI(TAG, "Starting basic Zigbee end device");

  // Start factory reset monitor task
  diagnostics_mark_stage(DIAG_STAGE_START_TASKS);
  xTaskCreate(factory_reset_monitor_task, "factory_reset_monitor", 2048, NULL,
              1, NULL);

  // Start battery reporting task
  battery_start_reporting_task();

  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
