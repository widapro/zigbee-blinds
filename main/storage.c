#include "storage.h"
#include "esp_log.h"
#include "motor.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "STORAGE";

static uint8_t sanitize_position(uint8_t position) {
  if (position > POSITION_MAX) {
    return POSITION_MAX;
  }
  return position;
}

static uint16_t sanitize_motor_steps(uint16_t motor_steps) {
  if (motor_steps < MOTOR_STEPS_MIN) {
    return MOTOR_STEPS_MIN;
  }
  if (motor_steps > MOTOR_STEPS_MAX) {
    return MOTOR_STEPS_MAX;
  }
  return motor_steps;
}

static uint8_t sanitize_motor_mode(uint8_t motor_mode) {
  if (motor_mode == 1 || motor_mode == 2) {
    return motor_mode;
  }
  return DEFAULT_MOTOR_MODE;
}

esp_err_t storage_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "NVS storage initialized");
  return ESP_OK;
}

esp_err_t storage_load_position(uint8_t *position) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *position = DEFAULT_POSITION;
    ESP_LOGI(TAG, "Position not found, using default: %d%%", DEFAULT_POSITION);
    return ESP_OK;
  }

  uint16_t motion_state;
  err = nvs_get_u16(handle, KEY_MOTION_STATE, &motion_state);
  if (err == ESP_OK) {
    *position = sanitize_position((uint8_t)motion_state);
    nvs_close(handle);
    return ESP_OK;
  }
  if (err != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return err;
  }

  size_t required_size = sizeof(uint8_t);
  err = nvs_get_blob(handle, KEY_POSITION, position, &required_size);
  if (err != ESP_OK) {
    *position = DEFAULT_POSITION;
    ESP_LOGI(TAG, "Position not found, using default: %d%%", DEFAULT_POSITION);
  } else {
    uint8_t sanitized = sanitize_position(*position);
    if (sanitized != *position) {
      ESP_LOGW(TAG, "Position out of range (%u), clamped to %u", *position,
               sanitized);
      *position = sanitized;
    }
    ESP_LOGI(TAG, "Position loaded: %d%%", *position);
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_position(uint8_t position) {
  return storage_save_motion_state(position, true);
}

esp_err_t storage_load_position_valid(bool *valid) {
  *valid = true; // Preserve the previous estimate on upgrade from old firmware.
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    return ESP_OK;
  }
  if (err != ESP_OK) {
    return err;
  }
  uint16_t motion_state;
  err = nvs_get_u16(handle, KEY_MOTION_STATE, &motion_state);
  if (err == ESP_OK) {
    *valid = (motion_state & 0x100) != 0;
  }
  nvs_close(handle);
  return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t storage_save_motion_state(uint8_t position, bool valid) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    return err;
  }
  uint16_t motion_state = sanitize_position(position) | (valid ? 0x100 : 0);
  err = nvs_set_u16(handle, KEY_MOTION_STATE, motion_state);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);
  return err;
}

esp_err_t storage_load_motor_steps(uint16_t *motor_steps) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *motor_steps = DEFAULT_MOTOR_STEPS;
    ESP_LOGI(TAG, "Motor steps not found, using default: %d (actual=%u)",
             DEFAULT_MOTOR_STEPS,
             (unsigned int)MOTOR_STEPS_TO_ACTUAL(DEFAULT_MOTOR_STEPS));
    return ESP_OK;
  }

  size_t required_size = sizeof(uint16_t);
  err = nvs_get_blob(handle, KEY_MOTOR_STEPS, motor_steps, &required_size);
  if (err != ESP_OK) {
    *motor_steps = DEFAULT_MOTOR_STEPS;
    ESP_LOGI(TAG, "Motor steps not found, using default: %d (actual=%u)",
             DEFAULT_MOTOR_STEPS,
             (unsigned int)MOTOR_STEPS_TO_ACTUAL(DEFAULT_MOTOR_STEPS));
  } else {
    uint16_t sanitized = sanitize_motor_steps(*motor_steps);
    if (sanitized != *motor_steps) {
      ESP_LOGW(TAG, "Motor steps out of range (%u), clamped to %u", *motor_steps,
               sanitized);
      *motor_steps = sanitized;
    }
    ESP_LOGI(TAG, "Motor steps loaded: %d (actual=%u)", *motor_steps,
             (unsigned int)MOTOR_STEPS_TO_ACTUAL(*motor_steps));
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_motor_steps(uint16_t motor_steps) {
  motor_steps = sanitize_motor_steps(motor_steps);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for motor steps save");
    return err;
  }

  err = nvs_set_blob(handle, KEY_MOTOR_STEPS, &motor_steps, sizeof(uint16_t));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "Motor steps saved: %d (actual=%u)", motor_steps,
             (unsigned int)MOTOR_STEPS_TO_ACTUAL(motor_steps));
  } else {
    ESP_LOGE(TAG, "Failed to save motor steps");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_load_motor_mode(uint8_t *motor_mode) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *motor_mode = DEFAULT_MOTOR_MODE;
    ESP_LOGI(TAG, "Motor mode not found, using default: %d (normal)",
             DEFAULT_MOTOR_MODE);
    return ESP_OK;
  }

  size_t required_size = sizeof(uint8_t);
  err = nvs_get_blob(handle, KEY_MOTOR_MODE, motor_mode, &required_size);
  if (err != ESP_OK) {
    *motor_mode = DEFAULT_MOTOR_MODE;
    ESP_LOGI(TAG, "Motor mode not found, using default: %d (normal)",
             DEFAULT_MOTOR_MODE);
  } else {
    uint8_t sanitized = sanitize_motor_mode(*motor_mode);
    if (sanitized != *motor_mode) {
      ESP_LOGW(TAG, "Motor mode invalid (%u), reset to %u", *motor_mode,
               sanitized);
      *motor_mode = sanitized;
    }
    ESP_LOGI(TAG, "Motor mode loaded: %d (%s)", *motor_mode,
             (*motor_mode == 1) ? "reversed" : "normal");
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_motor_mode(uint8_t motor_mode) {
  motor_mode = sanitize_motor_mode(motor_mode);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for motor mode save");
    return err;
  }

  err = nvs_set_blob(handle, KEY_MOTOR_MODE, &motor_mode, sizeof(uint8_t));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "Motor mode saved: %d (%s)", motor_mode,
             (motor_mode == 1) ? "reversed" : "normal");
  } else {
    ESP_LOGE(TAG, "Failed to save motor mode");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_load_motor_irun_active(uint8_t *motor_irun_active) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *motor_irun_active = DEFAULT_MOTOR_IRUN_ACTIVE;
    ESP_LOGI(TAG, "Motor IRUN not found, using default: %u",
             DEFAULT_MOTOR_IRUN_ACTIVE);
    return ESP_OK;
  }

  size_t required_size = sizeof(uint8_t);
  err = nvs_get_blob(handle, KEY_MOTOR_IRUN_ACTIVE, motor_irun_active,
                     &required_size);
  if (err != ESP_OK) {
    *motor_irun_active = DEFAULT_MOTOR_IRUN_ACTIVE;
    ESP_LOGI(TAG, "Motor IRUN not found, using default: %u",
             DEFAULT_MOTOR_IRUN_ACTIVE);
  } else {
    if (*motor_irun_active > 31) {
      ESP_LOGW(TAG, "Motor IRUN out of range (%u), clamping to 31",
               *motor_irun_active);
      *motor_irun_active = 31;
    }
    ESP_LOGI(TAG, "Motor IRUN loaded: %u", *motor_irun_active);
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_motor_irun_active(uint8_t motor_irun_active) {
  if (motor_irun_active > 31) {
    motor_irun_active = 31;
  }

  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for motor IRUN save");
    return err;
  }

  err = nvs_set_blob(handle, KEY_MOTOR_IRUN_ACTIVE, &motor_irun_active,
                     sizeof(uint8_t));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "Motor IRUN saved: %u", motor_irun_active);
  } else {
    ESP_LOGE(TAG, "Failed to save motor IRUN");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_load_ota_mode(bool *enabled) {
  nvs_handle_t my_handle;
  esp_err_t err;
  uint8_t val = 0;

  err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
  if (err != ESP_OK)
    return err;

  err = nvs_get_u8(my_handle, "ota_mode", &val);
  nvs_close(my_handle);

  if (err == ESP_OK) {
    *enabled = (val == 1);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    *enabled = false;
    return ESP_OK;
  }
  return err;
}

esp_err_t storage_save_ota_mode(bool enabled) {
  nvs_handle_t my_handle;
  esp_err_t err;

  err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;

  err = nvs_set_u8(my_handle, "ota_mode", enabled ? 1 : 0);
  if (err == ESP_OK) {
    err = nvs_commit(my_handle);
  }
  nvs_close(my_handle);
  return err;
}

esp_err_t storage_save_pending_restart_cause(restart_cause_t cause) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for restart cause save");
    return err;
  }

  err = nvs_set_u8(handle, KEY_PENDING_RESTART_CAUSE, (uint8_t)cause);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  } else {
    ESP_LOGE(TAG, "Failed to save pending restart cause");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_record_boot_diagnostics(uint8_t reset_reason,
                                          uint8_t *last_restart_cause,
                                          uint32_t *reboot_count) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for boot diagnostics");
    if (last_restart_cause) {
      *last_restart_cause = RESTART_CAUSE_NONE;
    }
    if (reboot_count) {
      *reboot_count = 0;
    }
    return err;
  }

  uint8_t pending_cause = RESTART_CAUSE_NONE;
  uint32_t count = 0;
  esp_err_t get_err =
      nvs_get_u8(handle, KEY_PENDING_RESTART_CAUSE, &pending_cause);
  if (get_err != ESP_OK && get_err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read pending restart cause: %s",
             esp_err_to_name(get_err));
    pending_cause = RESTART_CAUSE_NONE;
  }

  get_err = nvs_get_u32(handle, KEY_REBOOT_COUNT, &count);
  if (get_err != ESP_OK && get_err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to read reboot count: %s", esp_err_to_name(get_err));
    count = 0;
  }
  count++;

  err = nvs_set_u8(handle, KEY_LAST_RESET_REASON, reset_reason);
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, KEY_LAST_RESTART_CAUSE, pending_cause);
  }
  if (err == ESP_OK) {
    err = nvs_set_u32(handle, KEY_REBOOT_COUNT, count);
  }
  if (err == ESP_OK) {
    err = nvs_set_u8(handle, KEY_PENDING_RESTART_CAUSE, RESTART_CAUSE_NONE);
  }
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to record boot diagnostics: %s",
             esp_err_to_name(err));
  }

  nvs_close(handle);

  if (last_restart_cause) {
    *last_restart_cause = pending_cause;
  }
  if (reboot_count) {
    *reboot_count = count;
  }

  return err;
}

esp_err_t storage_reset_all(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for reset");
    return err;
  }

  ESP_LOGI(TAG, "Resetting all stored parameters...");

  err = nvs_erase_all(handle);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "All stored parameters reset to defaults");
  } else {
    ESP_LOGE(TAG, "Failed to reset stored parameters");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_load_motor_power_mode(uint8_t *motor_power_mode) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
    ESP_LOGI(TAG, "Motor power mode not found, using default: %u (%s)",
             DEFAULT_MOTOR_POWER_MODE,
             DEFAULT_MOTOR_POWER_MODE ? "SpreadCycle" : "StealthChop");
    return ESP_OK;
  }

  size_t required_size = sizeof(uint8_t);
  err = nvs_get_blob(handle, KEY_MOTOR_POWER_MODE, motor_power_mode,
                     &required_size);
  if (err != ESP_OK) {
    *motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
    ESP_LOGI(TAG, "Motor power mode not found, using default: %u (%s)",
             DEFAULT_MOTOR_POWER_MODE,
             DEFAULT_MOTOR_POWER_MODE ? "SpreadCycle" : "StealthChop");
  } else {
    if (*motor_power_mode > 1) {
      ESP_LOGW(TAG, "Motor power mode invalid (%u), reset to %u",
               *motor_power_mode, DEFAULT_MOTOR_POWER_MODE);
      *motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
    }
    ESP_LOGI(TAG, "Motor power mode loaded: %u (%s)", *motor_power_mode,
             *motor_power_mode ? "SpreadCycle" : "StealthChop");
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_motor_power_mode(uint8_t motor_power_mode) {
  if (motor_power_mode > 1) {
    motor_power_mode = DEFAULT_MOTOR_POWER_MODE;
  }

  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for motor power mode save");
    return err;
  }

  err = nvs_set_blob(handle, KEY_MOTOR_POWER_MODE, &motor_power_mode,
                     sizeof(uint8_t));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "Motor power mode saved: %u (%s)", motor_power_mode,
             motor_power_mode ? "SpreadCycle" : "StealthChop");
  } else {
    ESP_LOGE(TAG, "Failed to save motor power mode");
  }

  nvs_close(handle);
  return err;
}

esp_err_t storage_load_motor_speed(uint8_t *motor_speed) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    *motor_speed = DEFAULT_MOTOR_SPEED;
    ESP_LOGI(TAG, "Motor speed not found, using default: %u",
             DEFAULT_MOTOR_SPEED);
    return ESP_OK;
  }

  size_t required_size = sizeof(uint8_t);
  err = nvs_get_blob(handle, KEY_MOTOR_SPEED, motor_speed, &required_size);
  if (err != ESP_OK) {
    *motor_speed = DEFAULT_MOTOR_SPEED;
    ESP_LOGI(TAG, "Motor speed not found, using default: %u",
             DEFAULT_MOTOR_SPEED);
  } else {
    if (*motor_speed < 1) {
      *motor_speed = 1;
    }
    if (*motor_speed > 100) {
      *motor_speed = 100;
    }
    ESP_LOGI(TAG, "Motor speed loaded: %u", *motor_speed);
  }

  nvs_close(handle);
  return ESP_OK;
}

esp_err_t storage_save_motor_speed(uint8_t motor_speed) {
  if (motor_speed < 1) {
    motor_speed = 1;
  }
  if (motor_speed > 100) {
    motor_speed = 100;
  }

  nvs_handle_t handle;
  esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS for motor speed save");
    return err;
  }

  err = nvs_set_blob(handle, KEY_MOTOR_SPEED, &motor_speed, sizeof(uint8_t));
  if (err == ESP_OK) {
    err = nvs_commit(handle);
    ESP_LOGI(TAG, "Motor speed saved: %u", motor_speed);
  } else {
    ESP_LOGE(TAG, "Failed to save motor speed");
  }

  nvs_close(handle);
  return err;
}
