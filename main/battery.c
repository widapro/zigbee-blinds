#include "battery.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "zb_network.h"
#include <stdlib.h>

static const char *TAG = "BATTERY";

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_chan_handle = NULL;
static bool do_calibration = false;
static bool adc_needs_reinit = true; // Track if ADC needs re-initialization

esp_err_t battery_init(void) {
  ESP_LOGI(TAG, "Initializing battery monitoring on GPIO1");

  // Initialize ADC1
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  // Configure ADC1 channel
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config));

  // Initialize calibration handle
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = BATTERY_ADC_CHANNEL,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config,
                                                       &adc1_cali_chan_handle);
  if (ret == ESP_OK) {
    do_calibration = true;
    ESP_LOGI(TAG, "ADC calibration curve fitting enabled");
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!do_calibration) {
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config,
                                                        &adc1_cali_chan_handle);
    if (ret == ESP_OK) {
      do_calibration = true;
      ESP_LOGI(TAG, "ADC calibration line fitting enabled");
    }
  }
#endif

  if (!do_calibration) {
    ESP_LOGW(TAG, "ADC calibration not available, using raw readings");
  }

  ESP_LOGI(TAG, "Battery monitoring initialized successfully");
  adc_needs_reinit = false; // ADC is now properly initialized
  return ESP_OK;
}

static esp_err_t reinit_adc_if_needed(void) {
  if (!adc_needs_reinit) {
    return ESP_OK; // ADC is still valid
  }

  ESP_LOGD(TAG, "Reinitializing ADC after light sleep");

  // Clean up existing handles if present
  if (adc1_cali_chan_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(adc1_cali_chan_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(adc1_cali_chan_handle);
#endif
    adc1_cali_chan_handle = NULL;
    do_calibration = false;
  }

  if (adc1_handle) {
    adc_oneshot_del_unit(adc1_handle);
    adc1_handle = NULL;
  }

  // Reinitialize ADC
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC reinit failed: %s", esp_err_to_name(ret));
    return ret;
  }

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };
  ret = adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Recreate calibration handle
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .chan = BATTERY_ADC_CHANNEL,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config,
                                             &adc1_cali_chan_handle);
  if (ret == ESP_OK) {
    do_calibration = true;
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!do_calibration) {
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config,
                                              &adc1_cali_chan_handle);
    if (ret == ESP_OK) {
      do_calibration = true;
    }
  }
#endif

  adc_needs_reinit = false; // ADC is now valid
  return ESP_OK;
}

float battery_get_voltage(void) {
  // Reinitialize ADC only if needed (after light sleep)
  esp_err_t ret = reinit_adc_if_needed();
  if (ret != ESP_OK) {
    return 0.0f;
  }

  // Take multiple readings for better accuracy
  int adc_raw;
  uint32_t adc_sum = 0;
  const int num_samples = 16; // Reduced from 64 for power optimization

  for (int i = 0; i < num_samples; i++) {
    esp_err_t ret =
        adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw);
    if (ret == ESP_OK) {
      adc_sum += adc_raw;
    } else {
      ESP_LOGE(TAG, "ADC read error: %s", esp_err_to_name(ret));
      return 0.0f;
    }
  }

  int adc_average = adc_sum / num_samples;

  // Convert to voltage
  int voltage_mv;
  if (do_calibration && adc1_cali_chan_handle) {
    esp_err_t ret = adc_cali_raw_to_voltage(adc1_cali_chan_handle, adc_average,
                                            &voltage_mv);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ADC calibration error: %s", esp_err_to_name(ret));
      voltage_mv = adc_average * 3300 / 4095; // Fallback to raw conversion
    }
  } else {
    voltage_mv = adc_average * 3300 / 4095; // Raw conversion for 3.3V reference
  }

  // Apply voltage divider factor to get actual battery voltage
  float actual_voltage = (voltage_mv / 1000.0f) * BATTERY_VOLTAGE_DIVIDER;

  ESP_LOGI(
      TAG,
      "ADC raw avg: %d, Voltage divider input: %d mV, Battery voltage: %.2f V",
      adc_average, voltage_mv, actual_voltage);

  return actual_voltage;
}

/**
 * @brief Apply non-linear mapping to battery percentage to match Li-ion
 * discharge curve.
 *
 * Li-ion batteries have a steep voltage drop at the end of discharge.
 * This function remaps the linear percentage to better reflect usable capacity:
 *   - Linear 0-40% maps to reported 0-10% (steep discharge region)
 *   - Linear 40-100% maps to reported 10-100% (stable discharge region)
 *
 * @param linear_percent Raw linear percentage (0-100)
 * @return Mapped percentage (0-100) that better reflects actual capacity
 */
static uint8_t apply_battery_curve_mapping(float linear_percent) {
  if (linear_percent <= 0.0f) {
    return 0;
  }
  if (linear_percent >= 100.0f) {
    return 100;
  }

  float mapped;
  if (linear_percent <= 40.0f) {
    // Linear 0-40% maps to 0-10%
    // mapped = (linear / 40) * 10 = linear * 0.25
    mapped = linear_percent * 0.25f;
  } else {
    // Linear 40-100% maps to 10-100%
    // mapped = 10 + ((linear - 40) / 60) * 90
    mapped = 10.0f + ((linear_percent - 40.0f) / 60.0f) * 90.0f;
  }

  // Round and clamp to 0-100
  uint8_t result = (uint8_t)(mapped + 0.5f);
  if (result > 100) {
    result = 100;
  }
  return result;
}

uint8_t battery_get_percentage(void) {
  float voltage = battery_get_voltage();

  if (voltage <= BATTERY_MIN_VOLTAGE) {
    return 0;
  }
  if (voltage >= BATTERY_MAX_VOLTAGE) {
    return 100;
  }

  float linear_percentage = ((voltage - BATTERY_MIN_VOLTAGE) /
                             (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) *
                            100.0f;

  uint8_t battery_percentage = apply_battery_curve_mapping(linear_percentage);

  ESP_LOGD(TAG, "Battery voltage: %.2f V, linear: %.1f%%, mapped: %d%%",
           voltage, linear_percentage, battery_percentage);

  return battery_percentage;
}

// Cached values from last battery_should_report() call
static float cached_voltage = 0.0f;
static uint8_t cached_percentage = 0;

bool battery_should_report(void) {
  // Each scheduled battery check sends a fresh report.
  float current_voltage = battery_get_voltage();
  if (current_voltage <= 0.0f) {
    // ADC errors use 0 V as a sentinel; keep the last valid report.
    ESP_LOGW(TAG, "Skipping battery report after ADC failure");
    return false;
  }
  uint8_t current_percentage;

  if (current_voltage <= BATTERY_MIN_VOLTAGE) {
    current_percentage = 0;
  } else if (current_voltage >= BATTERY_MAX_VOLTAGE) {
    current_percentage = 100;
  } else {
    float linear_percentage = ((current_voltage - BATTERY_MIN_VOLTAGE) /
                               (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) *
                              100.0f;
    current_percentage = apply_battery_curve_mapping(linear_percentage);
  }

  // Cache the values for getter functions
  cached_voltage = current_voltage;
  cached_percentage = current_percentage;

  ESP_LOGD(TAG, "Battery measured: %.2fV (%d%%) - reporting to coordinator",
           current_voltage, current_percentage);

  // No local thresholding: the Zigbee scheduler controls reporting frequency.
  return true;
}

float battery_get_cached_voltage(void) { return cached_voltage; }

uint8_t battery_get_cached_percentage(void) { return cached_percentage; }

void battery_start_reporting_task(void) {
  ESP_LOGD(TAG, "Battery reporting will be handled via Zigbee callbacks");
  // No separate task - reporting will be done from Zigbee context
}

void battery_mark_adc_invalid(void) {
  adc_needs_reinit = true;
  ESP_LOGD(TAG, "ADC marked as needing re-initialization");
}
