/*
 * TMC2208 single-wire UART wiring:
 *   ESP32 TX ----------> TMC2208 PDN_UART
 *   ESP32 RX -- 1 kOhm -> TMC2208 PDN_UART
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "main.h"
#include "motor.h"
#include "storage.h"
#include "tmc2208_uart.h"

static const char *TAG = "MOTOR";

enum {
  TMC_DIAG_OK = 0,
  TMC_DIAG_UART_INIT_FAILED = 1,
  TMC_DIAG_GSTAT_CLEAR_FAILED = 2,
  TMC_DIAG_CONFIG_FAILED = 3,
  TMC_DIAG_MRES_WRITE_FAILED = 4,
  TMC_DIAG_MRES_READ_FAILED = 5,
  TMC_DIAG_MRES_MISMATCH = 6,
};

enum {
  RMT_DIAG_UNKNOWN = 0,
  RMT_DIAG_READY = 1,
  RMT_DIAG_FALLBACK_INIT = 2,
  RMT_DIAG_FALLBACK_TX = 3,
};

enum {
  RMT_STAGE_NONE = 0,
  RMT_STAGE_NEW_TX_CHANNEL = 1,
  RMT_STAGE_NEW_COPY_ENCODER = 2,
  RMT_STAGE_ENABLE = 3,
  RMT_STAGE_TRANSMIT = 4,
  RMT_STAGE_WAIT_DONE = 5,
};

static tmc2208_uart_t s_tmc_driver;
static rmt_channel_handle_t s_step_rmt_channel = NULL;
static rmt_encoder_handle_t s_step_rmt_encoder = NULL;
static bool s_step_rmt_ready = false;
static bool s_step_rmt_enabled = false;

static void motor_driver_set_active(bool active);
static esp_err_t motor_driver_ensure_ready(void);
static esp_err_t motor_driver_apply_settings(void);
#if MOTOR_VERIFY_UART_ON_WAKE
static esp_err_t motor_driver_verify_uart_link(void);
#endif
#if MOTOR_DRIVER_UART_DEBUG
static void motor_driver_log_settings(void);
#endif
static inline int motor_total_steps_actual(void);
static void motor_driver_set_coils(bool enabled);
static void motor_driver_hw_enable(bool enable);
static void motor_driver_set_power(bool on);
static void motor_step_gpio_configure(void);
static void motor_step_gpio_pulse(uint32_t delay_us);
static esp_err_t motor_step_rmt_init(void);
static esp_err_t motor_step_rmt_enable(void);
static void motor_step_rmt_disable(void);
static esp_err_t motor_step_rmt_transmit(const rmt_symbol_word_t *symbols,
                                         size_t symbol_count);
static void motor_gpio_hold_enable(void);
static void motor_gpio_hold_disable(void);
static inline bool motor_uses_stealthchop(void);
static inline uint8_t motor_calc_ihold_idle(void);
static inline uint32_t motor_profile_delay_us(int step_index, int accel_len,
                                              int decel_start, int decel_len,
                                              uint32_t target_delay_us,
                                              uint32_t start_delay_us);
static int motor_execute_gpio_motion_range(int start_step, int steps_total,
                                           int accel_len, int decel_start,
                                           int decel_len,
                                           uint32_t target_delay_us,
                                           uint32_t start_delay_us);
static int motor_execute_motion(int steps_total);

typedef enum {
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_MOVING,
} motor_state_t;

static bool s_driver_configured = false;
static bool s_coils_enabled = false;
static bool s_power_enabled = false;
static bool s_uart_initialized = false;
static uint8_t s_current_ihold = MOTOR_IHOLD_IDLE;
static uint8_t s_current_irun = MOTOR_IRUN_ACTIVE;
static volatile bool s_stop_requested = false;
static volatile motor_state_t s_motor_state = MOTOR_STATE_IDLE;
static SemaphoreHandle_t s_motor_mutex = NULL;

static inline int motor_total_steps_actual(void) {
  return (int)MOTOR_STEPS_TO_ACTUAL(g_motor_steps);
}

static inline bool motor_uses_stealthchop(void) {
  return g_motor_power_mode != MOTOR_POWER_MODE_SPREADCYCLE;
}

static inline uint8_t motor_calc_ihold_idle(void) {
  if (g_motor_irun_active == 0) {
    return 0;
  }

  uint8_t ihold = (uint8_t)((g_motor_irun_active + 3U) / 4U);
  if (ihold < 4) {
    ihold = 4;
  }
  if (ihold > g_motor_irun_active) {
    ihold = g_motor_irun_active;
  }
  return ihold;
}

static void motor_step_gpio_configure(void) {
  gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(STEP_PIN, 0);
}

static void motor_step_gpio_pulse(uint32_t delay_us) {
  gpio_set_level(STEP_PIN, 1);
  esp_rom_delay_us(delay_us);
  gpio_set_level(STEP_PIN, 0);
  esp_rom_delay_us(delay_us);
}

static void motor_driver_set_coils(bool enabled) {
  if (enabled) {
    if (!s_coils_enabled) {
      motor_driver_hw_enable(true);
    }
  } else if (s_coils_enabled) {
    motor_driver_hw_enable(false);
  }
}

static void motor_driver_hw_enable(bool enable) {
  if (enable) {
    gpio_set_level(ENABLE_PIN, 1);
    motor_driver_set_power(true);
    gpio_set_level(ENABLE_PIN, 0);
    s_coils_enabled = true;
  } else {
#if MOTOR_KEEP_DRIVER_READY
    gpio_set_level(ENABLE_PIN, 1);
#else
    motor_driver_set_power(false);
    gpio_set_level(ENABLE_PIN, 1);
#endif
    s_coils_enabled = false;
  }
}

static void motor_driver_set_power(bool on) {
  if (on) {
    if (!s_power_enabled) {
      gpio_set_level(MOTOR_POWER_DC_EN, 0);
      vTaskDelay(pdMS_TO_TICKS(MOTOR_WAKE_DELAY_MS));
      s_power_enabled = true;
    }
  } else {
    if (s_power_enabled) {
      gpio_set_level(MOTOR_POWER_DC_EN, 1);
      vTaskDelay(pdMS_TO_TICKS(MOTOR_SHUTDOWN_DELAY_MS));
      s_power_enabled = false;
    }
  }
}

static void motor_gpio_hold_enable(void) {
  gpio_hold_en(DIR_PIN);
  gpio_hold_en(STEP_PIN);
  gpio_hold_en(ENABLE_PIN);
  gpio_hold_en(MOTOR_POWER_DC_EN);
}

static void motor_gpio_hold_disable(void) {
  gpio_hold_dis(DIR_PIN);
  gpio_hold_dis(STEP_PIN);
  gpio_hold_dis(ENABLE_PIN);
  gpio_hold_dis(MOTOR_POWER_DC_EN);
  gpio_set_level(MOTOR_POWER_DC_EN, s_power_enabled ? 0 : 1);
  gpio_set_level(ENABLE_PIN, s_coils_enabled ? 0 : 1);
}

static esp_err_t motor_step_rmt_init(void) {
  if (s_step_rmt_channel && s_step_rmt_encoder) {
    s_step_rmt_ready = true;
    g_rmt_status = RMT_DIAG_READY;
    g_rmt_fail_stage = RMT_STAGE_NONE;
    g_rmt_last_error = ESP_OK;
    return ESP_OK;
  }

  rmt_tx_channel_config_t tx_chan_config = {
      .gpio_num = STEP_PIN,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = MOTOR_RMT_RESOLUTION_HZ,
      .mem_block_symbols = MOTOR_RMT_MEM_BLOCK_SYMBOLS,
      .trans_queue_depth = 1,
  };

  esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &s_step_rmt_channel);
  if (ret != ESP_OK) {
    s_step_rmt_ready = false;
    g_rmt_status = RMT_DIAG_FALLBACK_INIT;
    g_rmt_fail_stage = RMT_STAGE_NEW_TX_CHANNEL;
    g_rmt_last_error = (uint32_t)ret;
    ESP_LOGE(TAG, "Failed to create STEP RMT channel: %s",
             esp_err_to_name(ret));
    return ret;
  }

  rmt_copy_encoder_config_t encoder_config = {};
  ret = rmt_new_copy_encoder(&encoder_config, &s_step_rmt_encoder);
  if (ret != ESP_OK) {
    s_step_rmt_ready = false;
    g_rmt_status = RMT_DIAG_FALLBACK_INIT;
    g_rmt_fail_stage = RMT_STAGE_NEW_COPY_ENCODER;
    g_rmt_last_error = (uint32_t)ret;
    ESP_LOGE(TAG, "Failed to create STEP RMT encoder: %s",
             esp_err_to_name(ret));
    rmt_del_channel(s_step_rmt_channel);
    s_step_rmt_channel = NULL;
    return ret;
  }

  ESP_LOGI(TAG, "STEP RMT initialized (%u Hz, %u symbols)",
           (unsigned)MOTOR_RMT_RESOLUTION_HZ,
           (unsigned)MOTOR_RMT_MEM_BLOCK_SYMBOLS);
  s_step_rmt_ready = true;
  g_rmt_status = RMT_DIAG_READY;
  g_rmt_fail_stage = RMT_STAGE_NONE;
  g_rmt_last_error = ESP_OK;
  return ESP_OK;
}

static esp_err_t motor_step_rmt_enable(void) {
  if (!s_step_rmt_channel || !s_step_rmt_encoder) {
    return ESP_ERR_INVALID_STATE;
  }
  if (s_step_rmt_enabled) {
    return ESP_OK;
  }

  esp_err_t ret = rmt_enable(s_step_rmt_channel);
  if (ret != ESP_OK) {
    g_rmt_status = RMT_DIAG_FALLBACK_INIT;
    g_rmt_fail_stage = RMT_STAGE_ENABLE;
    g_rmt_last_error = (uint32_t)ret;
    ESP_LOGE(TAG, "Failed to enable STEP RMT channel: %s",
             esp_err_to_name(ret));
    return ret;
  }

  s_step_rmt_enabled = true;
  return ESP_OK;
}

static void motor_step_rmt_disable(void) {
  if (!s_step_rmt_enabled || !s_step_rmt_channel) {
    return;
  }

  esp_err_t ret = rmt_disable(s_step_rmt_channel);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to disable STEP RMT channel: %s",
             esp_err_to_name(ret));
    return;
  }
  s_step_rmt_enabled = false;
}

static esp_err_t motor_step_rmt_transmit(const rmt_symbol_word_t *symbols,
                                         size_t symbol_count) {
  if (!s_step_rmt_channel || !s_step_rmt_encoder || !symbols ||
      symbol_count == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  rmt_transmit_config_t tx_config = {
      .loop_count = 0,
  };
  esp_err_t ret = rmt_transmit(s_step_rmt_channel, s_step_rmt_encoder, symbols,
                               symbol_count * sizeof(symbols[0]), &tx_config);
  if (ret != ESP_OK) {
    g_rmt_status = RMT_DIAG_FALLBACK_TX;
    g_rmt_fail_stage = RMT_STAGE_TRANSMIT;
    g_rmt_last_error = (uint32_t)ret;
    return ret;
  }

  ret = rmt_tx_wait_all_done(s_step_rmt_channel, -1);
  if (ret != ESP_OK) {
    g_rmt_status = RMT_DIAG_FALLBACK_TX;
    g_rmt_fail_stage = RMT_STAGE_WAIT_DONE;
    g_rmt_last_error = (uint32_t)ret;
    return ret;
  }

  g_rmt_status = RMT_DIAG_READY;
  g_rmt_fail_stage = RMT_STAGE_NONE;
  g_rmt_last_error = ESP_OK;
  return ESP_OK;
}

static int motor_execute_gpio_motion_range(int start_step, int steps_total,
                                           int accel_len, int decel_start,
                                           int decel_len,
                                           uint32_t target_delay_us,
                                           uint32_t start_delay_us) {
  int executed_steps = 0;

  motor_step_gpio_configure();
  ESP_LOGW(TAG, "Using GPIO STEP fallback");

  for (int step_index = start_step; step_index < steps_total; step_index++) {
    if (s_stop_requested) {
      ESP_LOGW(TAG, "Stop requested, halting motion at step %d/%d", step_index,
               steps_total);
      break;
    }

    uint32_t delay_us =
        motor_profile_delay_us(step_index, accel_len, decel_start, decel_len,
                               target_delay_us, start_delay_us);
    motor_step_gpio_pulse(delay_us);
    executed_steps++;

    if ((executed_steps % 32) == 0) {
      taskYIELD();
    }
  }

  return executed_steps;
}

void motor_init(void) {
  ESP_LOGI(TAG, "Initializing motor driver (TMC2208)...");

  if (!s_motor_mutex) {
    s_motor_mutex = xSemaphoreCreateMutex();
    if (!s_motor_mutex) {
      ESP_LOGE(TAG, "Failed to create motor mutex");
      return;
    }
  }

  esp_err_t ret = ESP_OK;

  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) |
                      (1ULL << ENABLE_PIN) | (1ULL << MOTOR_POWER_DC_EN),
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
  };

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "GPIO configuration failed: %s", esp_err_to_name(ret));
    return;
  }

  gpio_set_level(DIR_PIN, 0);
  gpio_set_level(STEP_PIN, 0);
  gpio_set_level(ENABLE_PIN, 1);
  gpio_set_level(MOTOR_POWER_DC_EN, 1);
  motor_gpio_hold_disable();

  ret = motor_step_rmt_init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "STEP RMT unavailable (%s), falling back to GPIO pulses",
             esp_err_to_name(ret));
    motor_step_gpio_configure();
  }

  const tmc2208_uart_config_t driver_cfg = {
      .uart_port = MOTOR_DRIVER_UART_PORT,
      .rx_pin = MOTOR_DRIVER_UART_RX_PIN,
      .tx_pin = MOTOR_DRIVER_UART_TX_PIN,
      .baud_rate = MOTOR_DRIVER_UART_BAUD,
  };

  ret = tmc2208_uart_init(&s_tmc_driver, &driver_cfg);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_UART_INIT_FAILED;
    ESP_LOGE(TAG, "TMC2208 init failed: %s", esp_err_to_name(ret));
    return;
  }
  s_uart_initialized = true;

  ret = tmc2208_clear_gstat(&s_tmc_driver);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_GSTAT_CLEAR_FAILED;
    ESP_LOGW(TAG, "Unable to clear GSTAT: %s", esp_err_to_name(ret));
  } else {
    g_tmc_last_error = TMC_DIAG_OK;
  }

  motor_driver_set_active(true);

  esp_err_t driver_ready = motor_driver_ensure_ready();
  if (driver_ready != ESP_OK) {
    ESP_LOGW(TAG, "TMC2208 UART setup failed: %s",
             esp_err_to_name(driver_ready));
  }

  motor_driver_set_active(false);

  ESP_LOGI(TAG, "Motor driver initialized successfully");
}

// Calculate the cruise step delay in us from g_motor_speed (1-100).
// speed=100 -> MOTOR_SPEED_MIN_DELAY_US (fastest)
// speed=1   -> MOTOR_SPEED_MAX_DELAY_US (slowest, most torque)
static inline uint32_t motor_calc_target_delay(void) {
  uint8_t speed = g_motor_speed;
  if (speed < 1) speed = 1;
  if (speed > 100) speed = 100;
  return MOTOR_SPEED_MAX_DELAY_US -
         (uint32_t)((uint32_t)(speed - 1) *
                    (MOTOR_SPEED_MAX_DELAY_US - MOTOR_SPEED_MIN_DELAY_US)) / 99;
}

static inline uint32_t
motor_apply_short_stealthchop_delay(uint32_t target_delay, int steps_total,
                                    bool use_stealthchop) {
  if (use_stealthchop &&
      steps_total <= MOTOR_STEALTHCHOP_SHORT_MOVE_MAX_STEPS &&
      target_delay < MOTOR_STEALTHCHOP_SHORT_MOVE_MIN_DELAY_US) {
    return MOTOR_STEALTHCHOP_SHORT_MOVE_MIN_DELAY_US;
  }
  return target_delay;
}

// Calculate the start (slow) step delay from the cruise delay.
static inline uint32_t motor_calc_start_delay(uint32_t target_delay,
                                              bool use_stealthchop) {
  uint32_t factor = use_stealthchop ? MOTOR_STEALTHCHOP_START_SPEED_FACTOR
                                    : MOTOR_START_SPEED_FACTOR;
  uint32_t cap = use_stealthchop ? MOTOR_STEALTHCHOP_START_DELAY_CAP_US
                                 : MOTOR_START_DELAY_CAP_US;
  uint32_t start = target_delay * factor;
  if (start > cap) {
    start = cap;
  }
  // Start delay must be >= target delay
  if (start < target_delay) {
    start = target_delay;
  }
  return start;
}

static void motor_prestep_settle(bool use_stealthchop) {
#if MOTOR_STEALTHCHOP_PRESTEP_SETTLE_MS > 0
  if (use_stealthchop) {
    vTaskDelay(pdMS_TO_TICKS(MOTOR_STEALTHCHOP_PRESTEP_SETTLE_MS));
  }
#else
  (void)use_stealthchop;
#endif
}

static inline uint32_t motor_s_curve_progress_q16(int index, int length) {
  if (length <= 1 || index >= (length - 1)) {
    return 65535;
  }
  if (index <= 0) {
    return 0;
  }

  uint32_t p = (uint32_t)(((uint64_t)index * 65535ULL) /
                          (uint32_t)(length - 1));
  uint64_t p2 = (uint64_t)p * p;
  uint64_t p3 = p2 * p;
  uint64_t smooth = ((3ULL * p2) / 65535ULL) -
                    ((2ULL * p3) / (65535ULL * 65535ULL));

  if (smooth > 65535ULL) {
    smooth = 65535ULL;
  }
  return (uint32_t)smooth;
}

static inline uint32_t motor_profile_delay_us(int step_index, int accel_len,
                                              int decel_start, int decel_len,
                                              uint32_t target_delay_us,
                                              uint32_t start_delay_us) {
  if (step_index < accel_len) {
    uint32_t progress_q16 =
        motor_s_curve_progress_q16(step_index, accel_len);
    return start_delay_us -
           (uint32_t)(((uint64_t)(start_delay_us - target_delay_us) *
                       progress_q16) /
                      65535ULL);
  }

  if (step_index >= decel_start) {
    int steps_into_decel = step_index - decel_start;
    uint32_t progress_q16 =
        motor_s_curve_progress_q16(steps_into_decel, decel_len);
    return target_delay_us +
           (uint32_t)(((uint64_t)(start_delay_us - target_delay_us) *
                       progress_q16) /
                      65535ULL);
  }

  return target_delay_us;
}

static int motor_execute_motion(int steps_total) {
  if (steps_total <= 0) {
    return 0;
  }

  int executed_steps = 0;

  // Calculate speed from g_motor_speed at motion start
  const bool use_stealthchop = motor_uses_stealthchop();
  const uint32_t requested_target_delay_us = motor_calc_target_delay();
  const uint32_t target_delay_us = motor_apply_short_stealthchop_delay(
      requested_target_delay_us, steps_total, use_stealthchop);
  const uint32_t start_delay_us =
      motor_calc_start_delay(target_delay_us, use_stealthchop);

  // Determine effective ramp lengths (clamp if fewer steps than ramp needs)
  int accel_len = MOTOR_ACCEL_STEPS;
  int decel_len = MOTOR_DECEL_STEPS;
  if (accel_len + decel_len > steps_total) {
    // Scale both ramps proportionally so they fit
    accel_len = steps_total / 2;
    decel_len = steps_total - accel_len;
  }
  int decel_start = steps_total - decel_len;

  if (s_stop_requested) {
    return 0;
  }
  motor_prestep_settle(use_stealthchop);

  if (!s_step_rmt_ready || !s_step_rmt_channel || !s_step_rmt_encoder) {
    if (g_rmt_status == RMT_DIAG_UNKNOWN) {
      g_rmt_status = RMT_DIAG_FALLBACK_INIT;
    }
    return motor_execute_gpio_motion_range(
        0, steps_total, accel_len, decel_start, decel_len, target_delay_us,
        start_delay_us);
  }

  esp_err_t enable_ret = motor_step_rmt_enable();
  if (enable_ret != ESP_OK) {
    s_step_rmt_ready = false;
    return motor_execute_gpio_motion_range(
        0, steps_total, accel_len, decel_start, decel_len, target_delay_us,
        start_delay_us);
  }

  rmt_symbol_word_t symbols[MOTOR_RMT_MAX_CHUNK_STEPS];

  for (int step_index = 0; step_index < steps_total;) {
    if (s_stop_requested) {
      ESP_LOGW(TAG, "Stop requested, halting motion at step %d/%d", step_index,
               steps_total);
      break;
    }

    size_t chunk_steps = 0;
    uint32_t chunk_duration_us = 0;

    while ((step_index + (int)chunk_steps) < steps_total &&
           chunk_steps < MOTOR_RMT_MAX_CHUNK_STEPS) {
      int i = step_index + (int)chunk_steps;
      uint32_t delay_us =
          motor_profile_delay_us(i, accel_len, decel_start, decel_len,
                                 target_delay_us, start_delay_us);

      symbols[chunk_steps].level0 = 1;
      symbols[chunk_steps].duration0 = (uint16_t)delay_us;
      symbols[chunk_steps].level1 = 0;
      symbols[chunk_steps].duration1 = (uint16_t)delay_us;

      chunk_duration_us += delay_us * 2;
      chunk_steps++;

      if (chunk_duration_us >= MOTOR_RMT_MAX_CHUNK_US) {
        break;
      }
    }

    if (chunk_steps == 0 || s_stop_requested) {
      break;
    }

    esp_err_t ret = motor_step_rmt_transmit(symbols, chunk_steps);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "STEP RMT transmit failed at step %d/%d: %s", step_index,
               steps_total, esp_err_to_name(ret));
      s_step_rmt_ready = false;
      motor_step_rmt_disable();
      // A timeout can occur after some pulses have reached the driver.
      // Replaying this chunk would move beyond the requested position.
      if (g_rmt_fail_stage == RMT_STAGE_WAIT_DONE) {
        return executed_steps;
      }
      executed_steps += motor_execute_gpio_motion_range(
          step_index, steps_total, accel_len, decel_start, decel_len,
          target_delay_us, start_delay_us);
      return executed_steps;
    }

    step_index += (int)chunk_steps;
    executed_steps += (int)chunk_steps;
    taskYIELD();
  }
  motor_step_rmt_disable();
  return executed_steps;
}

#if !MOTOR_KEEP_DRIVER_READY && !MOTOR_KEEP_UART_READY
static void motor_uart_deinit(void) {
  if (s_uart_initialized) {
    tmc2208_uart_deinit(&s_tmc_driver);
    s_uart_initialized = false;
    ESP_LOGD(TAG, "UART deinitialized for power saving");
  }
}
#endif

static void motor_uart_reinit(void) {
  if (!s_uart_initialized) {
    const tmc2208_uart_config_t driver_cfg = {
        .uart_port = MOTOR_DRIVER_UART_PORT,
        .rx_pin = MOTOR_DRIVER_UART_RX_PIN,
        .tx_pin = MOTOR_DRIVER_UART_TX_PIN,
        .baud_rate = MOTOR_DRIVER_UART_BAUD,
    };

    esp_err_t ret = tmc2208_uart_init(&s_tmc_driver, &driver_cfg);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "UART reinit failed: %s", esp_err_to_name(ret));
      return;
    }
    s_uart_initialized = true;
    ESP_LOGD(TAG, "UART reinitialized for motor operation");
  }
}

static void motor_driver_set_active(bool active) {
  if (active) {
    int polarity = (g_motor_mode == 1) ? 1 : 0;
    gpio_set_level(DIR_PIN, polarity);
    motor_driver_set_power(true);
    motor_uart_reinit();
    gpio_set_level(ENABLE_PIN, 1);
    s_coils_enabled = false;
  } else {
    motor_driver_set_coils(false);
#if MOTOR_KEEP_DRIVER_READY
    // Keep power rail and UART configured for near-zero startup latency on
    // the next command.
#else
    motor_driver_set_power(false);
    vTaskDelay(pdMS_TO_TICKS(MOTOR_DISABLE_DELAY_MS));
#if !MOTOR_KEEP_UART_READY
    motor_uart_deinit();
#endif
    s_driver_configured = false;
    s_current_ihold = 0;
#endif
  }
}

bool motor_move_to_position(uint8_t target_position) {
  if (!s_motor_mutex) {
    ESP_LOGE(TAG, "Motor mutex not initialized");
    return false;
  }

  if (xSemaphoreTake(s_motor_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGW(TAG, "Motor busy, move request dropped");
    return false;
  }

  bool movement_completed = false;
  bool driver_active = false;

  do {
    if (s_motor_state != MOTOR_STATE_IDLE) {
      ESP_LOGW(TAG, "Motor already active (state=%d), move rejected",
               (int)s_motor_state);
      break;
    }

    s_motor_state = MOTOR_STATE_MOVING;
    s_stop_requested = false;

    if (target_position > 100) {
      ESP_LOGE(TAG, "Invalid position: %d (must be 0-100)", target_position);
      break;
    }

    if (g_current_position == target_position) {
      ESP_LOGI(TAG, "Already at target position %d%%", target_position);
      movement_completed = true;
      break;
    }

    ESP_LOGI(TAG, "Moving from %d%% to %d%%", g_current_position,
             target_position);

    motor_driver_set_active(true);
    driver_active = true;

    esp_err_t driver_ready = motor_driver_ensure_ready();
    if (driver_ready != ESP_OK) {
      ESP_LOGE(TAG, "Motor driver not ready: %s", esp_err_to_name(driver_ready));
      break;
    }

    motor_driver_set_coils(true);

    int total_steps = motor_total_steps_actual();
    int current_steps = (g_current_position * total_steps) / 100;
    int target_steps = (target_position * total_steps) / 100;
    int steps_to_move = abs(target_steps - current_steps);

    if (steps_to_move == 0) {
      movement_completed = true;
      break;
    }

    int direction;
    if (target_position > g_current_position) {
      direction = (g_motor_mode == 1) ? 0 : 1;
    } else {
      direction = (g_motor_mode == 1) ? 1 : 0;
    }

    gpio_set_level(DIR_PIN, direction);
    esp_rom_delay_us(MOTOR_DIR_SETTLE_US);

    ESP_LOGI(TAG, "Moving %d steps in direction %d", steps_to_move, direction);
    int executed_steps = motor_execute_motion(steps_to_move);

    if (executed_steps >= steps_to_move) {
      g_current_position = target_position;
      movement_completed = true;
    } else {
      int direction_sign = (target_steps > current_steps) ? 1 : -1;
      int new_steps = current_steps + (direction_sign * executed_steps);
      if (new_steps < 0) {
        new_steps = 0;
      }
      if (new_steps > total_steps) {
        new_steps = total_steps;
      }

      g_current_position = (uint8_t)((new_steps * 100 + (total_steps / 2)) /
                                     total_steps);
      ESP_LOGW(TAG, "Movement interrupted at %d%%", g_current_position);
    }

    ESP_LOGI(TAG, "Motor positioned at %d%%", g_current_position);
  } while (0);

  if (driver_active) {
    motor_driver_set_active(false);
    ESP_LOGD(TAG, "TMC2208 motion session closed");
  }

  s_motor_state = MOTOR_STATE_IDLE;
  s_stop_requested = false;
  xSemaphoreGive(s_motor_mutex);
  return movement_completed;
}

void motor_stop(void) {
  motor_request_stop();

  if (s_motor_mutex &&
      xSemaphoreTake(s_motor_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (s_motor_state == MOTOR_STATE_IDLE && (s_power_enabled || s_coils_enabled)) {
      motor_driver_set_active(false);
    }
    xSemaphoreGive(s_motor_mutex);
  }

  ESP_LOGI(TAG, "Motor stop requested");
}

void motor_request_stop(void) { s_stop_requested = true; }

bool motor_is_busy(void) { return s_motor_state != MOTOR_STATE_IDLE; }

static esp_err_t motor_driver_ensure_ready(void) {
  if (!s_tmc_driver.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

#if MOTOR_KEEP_DRIVER_READY
  if (s_driver_configured && s_uart_initialized && s_power_enabled) {
    return ESP_OK;
  }
#endif

#if MOTOR_VERIFY_UART_ON_WAKE
  esp_err_t ret = motor_driver_verify_uart_link();
  if (ret != ESP_OK) {
    return ret;
  }
#endif

  if (!s_driver_configured) {
    esp_err_t ret = motor_driver_apply_settings();
    if (ret != ESP_OK) {
      return ret;
    }
    s_driver_configured = true;
  }

  return ESP_OK;
}

void motor_prepare_for_sleep(void) { motor_gpio_hold_enable(); }

void motor_resume_from_sleep(void) { motor_gpio_hold_disable(); }

void motor_set_irun_active(uint8_t motor_irun_active) {
  if (motor_irun_active > 31) {
    motor_irun_active = 31;
  }

  if (!s_motor_mutex) {
    ESP_LOGE(TAG, "Motor mutex not initialized");
    return;
  }

  if (xSemaphoreTake(s_motor_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    ESP_LOGW(TAG, "Motor busy, IRUN update deferred");
    return;
  }

  g_motor_irun_active = motor_irun_active;

  if (!s_tmc_driver.initialized || !s_driver_configured) {
    xSemaphoreGive(s_motor_mutex);
    return;
  }

  if (s_coils_enabled) {
    ESP_LOGW(TAG, "IRUN update deferred (motor moving)");
    xSemaphoreGive(s_motor_mutex);
    return;
  }

  uint8_t ihold_idle = motor_calc_ihold_idle();
  esp_err_t ret = tmc2208_set_ihold_irun(&s_tmc_driver, ihold_idle,
                                         g_motor_irun_active,
                                         MOTOR_IHOLD_DELAY);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to apply IRUN immediately: %s", esp_err_to_name(ret));
    xSemaphoreGive(s_motor_mutex);
    return;
  }

  s_current_ihold = ihold_idle;
  s_current_irun = g_motor_irun_active;
  ESP_LOGI(TAG, "IRUN updated to %u (IHOLD=%u)", s_current_irun,
           s_current_ihold);
  xSemaphoreGive(s_motor_mutex);
}

static esp_err_t motor_driver_apply_settings(void) {
  if (!s_tmc_driver.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  bool use_spreadcycle =
      (g_motor_power_mode == MOTOR_POWER_MODE_SPREADCYCLE);
  uint8_t ihold_idle = motor_calc_ihold_idle();

  ESP_LOGI(TAG, "Configuring TMC2208: mode=%s, speed=%u, IRUN=%u, IHOLD=%u",
           use_spreadcycle ? "SpreadCycle" : "StealthChop",
           g_motor_speed, g_motor_irun_active, ihold_idle);
  g_tmc_mres_requested = MOTOR_MICROSTEP_MODE;

  esp_err_t ret = tmc2208_configure_gconf(
      &s_tmc_driver,
      true,              // pdn_disable
      true,              // mstep_reg_select
      false,             // i_scale_analog
      use_spreadcycle);  // en_spreadcycle
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to configure GCONF: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = tmc2208_set_multistep_filter(&s_tmc_driver, !use_spreadcycle);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to set multistep filter: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // CHOPCONF tuning: use values optimized for the active mode
  uint8_t toff = use_spreadcycle ? 4 : 3;
  uint8_t hstrt = use_spreadcycle ? 4 : 6;
  uint8_t hend = use_spreadcycle ? 2 : 0;

  ret = tmc2208_set_toff(&s_tmc_driver, toff);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to set TOFF: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = tmc2208_set_tbl(&s_tmc_driver, 2);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to set TBL: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = tmc2208_set_hysteresis(&s_tmc_driver, hstrt, hend);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to set hysteresis: %s", esp_err_to_name(ret));
    return ret;
  }

  // Force vsense=0 for full current range (maximize torque)
  ret = tmc2208_set_vsense(&s_tmc_driver, false);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set VSENSE: %s", esp_err_to_name(ret));
  }

  ret = tmc2208_set_intpol(&s_tmc_driver, true);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_CONFIG_FAILED;
    ESP_LOGE(TAG, "Failed to set interpolation: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = tmc2208_set_microstep_resolution(&s_tmc_driver, MOTOR_MICROSTEP_MODE);
  if (ret != ESP_OK) {
    g_tmc_last_error = TMC_DIAG_MRES_WRITE_FAILED;
    ESP_LOGE(TAG, "Failed to set microsteps: %s", esp_err_to_name(ret));
    return ret;
  }

  tmc2208_mres_t actual_mres = TMC2208_MRES_256;
  ret = tmc2208_read_microstep_resolution(&s_tmc_driver, &actual_mres);
  if (ret != ESP_OK) {
    g_tmc_mres_actual = 0xFF;
    g_tmc_last_error = TMC_DIAG_MRES_READ_FAILED;
    ESP_LOGE(TAG, "Failed to verify microsteps: %s", esp_err_to_name(ret));
    return ret;
  }
  g_tmc_mres_actual = actual_mres;
  ESP_LOGI(TAG, "TMC2208 MRES verified: requested=%u actual=%u",
           (unsigned)MOTOR_MICROSTEP_MODE, (unsigned)actual_mres);
  if (actual_mres != MOTOR_MICROSTEP_MODE) {
    g_tmc_last_error = TMC_DIAG_MRES_MISMATCH;
    ESP_LOGE(TAG, "TMC2208 MRES mismatch");
    return ESP_FAIL;
  }
  g_tmc_last_error = TMC_DIAG_OK;
  (void)tmc2208_dump_gconf(&s_tmc_driver);
  (void)tmc2208_dump_chopconf(&s_tmc_driver);

  ret = tmc2208_set_ihold_irun(&s_tmc_driver, ihold_idle,
                               g_motor_irun_active, MOTOR_IHOLD_DELAY);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set motor currents: %s", esp_err_to_name(ret));
    return ret;
  }

  // Set TPOWERDOWN for fast automatic current reduction after standstill
  ret = tmc2208_set_tpowerdown(&s_tmc_driver, 10);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set TPOWERDOWN: %s", esp_err_to_name(ret));
    // Non-critical, continue
  }

  if (!use_spreadcycle) {
    // StealthChop: enable PWM auto-tuning
    ret = tmc2208_set_pwm_autoscale(&s_tmc_driver, true);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to enable PWM autoscale: %s", esp_err_to_name(ret));
    }

    ret = tmc2208_set_pwm_autograd(&s_tmc_driver, true);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to enable PWM autograd: %s", esp_err_to_name(ret));
    }
  }

  s_current_ihold = ihold_idle;
  s_current_irun = g_motor_irun_active;

#if MOTOR_DRIVER_UART_DEBUG
  motor_driver_log_settings();
#endif

  return ESP_OK;
}

#if MOTOR_VERIFY_UART_ON_WAKE
static esp_err_t motor_driver_verify_uart_link(void) {
  if (!s_tmc_driver.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t ifcnt_before = 0;
  esp_err_t ret = tmc2208_read_ifcnt(&s_tmc_driver, &ifcnt_before);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read IFCNT: %s", esp_err_to_name(ret));
    return ret;
  }

  uint32_t drv_status = 0;
  ret = tmc2208_read_drv_status(&s_tmc_driver, &drv_status);
  (void)drv_status;
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read DRV_STATUS for UART check: %s",
             esp_err_to_name(ret));
    return ret;
  }

  ret = tmc2208_clear_gstat(&s_tmc_driver);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to clear GSTAT during UART check: %s",
             esp_err_to_name(ret));
    return ret;
  }

  uint8_t ifcnt_after = 0;
  ret = tmc2208_read_ifcnt(&s_tmc_driver, &ifcnt_after);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to re-read IFCNT: %s", esp_err_to_name(ret));
    return ret;
  }

  uint8_t delta = (uint8_t)(ifcnt_after - ifcnt_before);
  if (delta == 0) {
    ESP_LOGE(TAG, "UART IFCNT did not advance (before=%u, after=%u)",
             ifcnt_before, ifcnt_after);
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "UART link verified (IFCNT %u -> %u)", ifcnt_before,
	           ifcnt_after);
  return ESP_OK;
}
#endif

#if MOTOR_DRIVER_UART_DEBUG
static void motor_driver_log_settings(void) {
  if (!s_tmc_driver.initialized) {
    return;
  }

  esp_err_t ret = tmc2208_dump_gconf(&s_tmc_driver);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Unable to dump GCONF: %s", esp_err_to_name(ret));
  }

  ret = tmc2208_dump_drv_status(&s_tmc_driver);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Unable to dump DRV_STATUS: %s", esp_err_to_name(ret));
  }

  ret = tmc2208_dump_chopconf(&s_tmc_driver);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Unable to dump CHOPCONF: %s", esp_err_to_name(ret));
  }

  bool sc = (g_motor_power_mode == MOTOR_POWER_MODE_SPREADCYCLE);
  ESP_LOGI(TAG, "TMC2208 cfg: %s microstep=%u IRUN=%u IHOLD=%u speed=%u",
           sc ? "SpreadCycle=ON" : "StealthChop=ON",
	           (unsigned int)MOTOR_MICROSTEP_MODE, s_current_irun, s_current_ihold,
		           g_motor_speed);
}
#endif

void motor_set_power_mode(uint8_t mode) {
  if (mode > 1) {
    mode = 0;
  }

  if (!s_motor_mutex) {
    ESP_LOGE(TAG, "Motor mutex not initialized");
    return;
  }

  if (xSemaphoreTake(s_motor_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    ESP_LOGW(TAG, "Motor busy, power mode update deferred");
    return;
  }

  g_motor_power_mode = mode;

  // Force re-configuration on next motion
  s_driver_configured = false;

  ESP_LOGI(TAG, "Power mode set to %s",
           mode == MOTOR_POWER_MODE_SPREADCYCLE ? "SpreadCycle" : "StealthChop");
  xSemaphoreGive(s_motor_mutex);
}

void motor_set_speed(uint8_t speed) {
  if (speed < 1) {
    speed = 1;
  }
  if (speed > 100) {
    speed = 100;
  }

  g_motor_speed = speed;
  ESP_LOGI(TAG, "Motor speed set to %u (delay=%lu us)", speed,
           (unsigned long)motor_calc_target_delay());
}
