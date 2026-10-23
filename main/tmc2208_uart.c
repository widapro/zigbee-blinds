#include "tmc2208_uart.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TMC2208_UART";

#define TMC2208_UART_TIMEOUT_TICKS pdMS_TO_TICKS(100)
#define TMC2208_UART_READ_RETRIES 2
#define TMC2208_BIT(bit) (1UL << (bit))
#define TMC2208_FIELD_MASK(width, shift)                                      \
  (((1UL << (width)) - 1UL) << (shift))

#define TMC2208_REG_GCONF 0x00
#define TMC2208_REG_GSTAT 0x01
#define TMC2208_REG_IFCNT 0x02
#define TMC2208_REG_IHOLD_IRUN 0x10
#define TMC2208_REG_TPOWERDOWN 0x11
#define TMC2208_REG_CHOPCONF 0x6C
#define TMC2208_REG_DRV_STATUS 0x6F
#define TMC2208_REG_PWMCONF 0x70

#define TMC2208_GCONF_I_SCALE_ANALOG TMC2208_BIT(0)
#define TMC2208_GCONF_EN_SPREADCYCLE TMC2208_BIT(2)
#define TMC2208_GCONF_PDN_DISABLE TMC2208_BIT(6)
#define TMC2208_GCONF_MSTEP_REG_SELECT TMC2208_BIT(7)
#define TMC2208_GCONF_MULTISTEP_FILT TMC2208_BIT(8)

#define TMC2208_CHOPCONF_TOFF_MASK TMC2208_FIELD_MASK(4, 0)
#define TMC2208_CHOPCONF_HSTRT_MASK TMC2208_FIELD_MASK(3, 4)
#define TMC2208_CHOPCONF_HEND_MASK TMC2208_FIELD_MASK(4, 7)
#define TMC2208_CHOPCONF_TBL_MASK TMC2208_FIELD_MASK(2, 15)
#define TMC2208_CHOPCONF_VSENSE TMC2208_BIT(17)
#define TMC2208_CHOPCONF_MRES_MASK TMC2208_FIELD_MASK(4, 24)
#define TMC2208_CHOPCONF_INTPOL TMC2208_BIT(28)

#define TMC2208_PWMCONF_AUTOSCALE TMC2208_BIT(18)
#define TMC2208_PWMCONF_AUTOGRAD TMC2208_BIT(19)

static uint8_t tmc2208_crc8(const uint8_t *data, size_t length_without_crc) {
  uint8_t crc = 0;

  for (size_t byte_index = 0; byte_index < length_without_crc; byte_index++) {
    uint8_t current_byte = data[byte_index];
    for (uint8_t bit_index = 0; bit_index < 8; bit_index++) {
      if (((crc >> 7) ^ (current_byte & 0x01)) != 0) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
      current_byte >>= 1;
    }
  }

  return crc;
}

static uint32_t tmc2208_get_field(uint32_t value, uint32_t mask) {
  return (value & mask) >> __builtin_ctz(mask);
}

static uint32_t tmc2208_set_field(uint32_t value, uint32_t mask,
                                  uint32_t field_value) {
  uint32_t shift = (uint32_t)__builtin_ctz(mask);
  return (value & ~mask) | ((field_value << shift) & mask);
}

static void tmc2208_encode_u32(uint8_t *out, uint32_t value) {
  out[0] = (uint8_t)(value >> 24);
  out[1] = (uint8_t)(value >> 16);
  out[2] = (uint8_t)(value >> 8);
  out[3] = (uint8_t)value;
}

static uint32_t tmc2208_decode_u32(const uint8_t *data) {
  return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

static esp_err_t tmc2208_check_driver(const tmc2208_uart_t *driver) {
  if (!driver || !driver->initialized) {
    return ESP_ERR_INVALID_STATE;
  }
  return ESP_OK;
}

static esp_err_t tmc2208_write_register(tmc2208_uart_t *driver, uint8_t reg,
                                        uint32_t value) {
  esp_err_t ret = tmc2208_check_driver(driver);
  if (ret != ESP_OK) {
    return ret;
  }
  driver->last_register = reg;

  uint8_t datagram[8] = {
      0x05,
      0x00,
      (uint8_t)(reg | 0x80),
      0,
      0,
      0,
      0,
      0,
  };
  tmc2208_encode_u32(&datagram[3], value);
  datagram[7] = tmc2208_crc8(datagram, sizeof(datagram) - 1);

  int written =
      uart_write_bytes(driver->config.uart_port, datagram, sizeof(datagram));
  if (written != (int)sizeof(datagram)) {
    ESP_LOGE(TAG, "UART write failed: %d/%u bytes", written,
             (unsigned)sizeof(datagram));
    return ESP_FAIL;
  }

  ret = uart_wait_tx_done(driver->config.uart_port, TMC2208_UART_TIMEOUT_TICKS);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART TX wait failed: %s", esp_err_to_name(ret));
    return ret;
  }

  uart_flush_input(driver->config.uart_port);
  return ESP_OK;
}

static esp_err_t tmc2208_read_register(tmc2208_uart_t *driver, uint8_t reg,
                                       uint32_t *out_value) {
  if (!out_value) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = tmc2208_check_driver(driver);
  if (ret != ESP_OK) {
    return ret;
  }
  driver->last_register = reg;

  uint8_t request[4] = {0x05, 0x00, (uint8_t)(reg & 0x7F), 0};
  request[3] = tmc2208_crc8(request, sizeof(request) - 1);
  bool saw_invalid_response = false;

  for (uint8_t attempt = 0; attempt < TMC2208_UART_READ_RETRIES; attempt++) {
    uart_flush_input(driver->config.uart_port);

    int written =
        uart_write_bytes(driver->config.uart_port, request, sizeof(request));
    if (written != (int)sizeof(request)) {
      ESP_LOGE(TAG, "UART read request failed: %d/%u bytes", written,
               (unsigned)sizeof(request));
      return ESP_FAIL;
    }

    ret =
        uart_wait_tx_done(driver->config.uart_port, TMC2208_UART_TIMEOUT_TICKS);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "UART read request TX wait failed: %s",
               esp_err_to_name(ret));
      return ret;
    }

    uint8_t reply[8] = {0};
    size_t filled = 0;
    const TickType_t started = xTaskGetTickCount();
    // A reply can arrive while TX-wait is yielding. Do not flush RX here:
    // scan past the single-wire request echo and assemble fragmented bytes.
    // Noise/invalid frames do not extend the absolute per-attempt deadline.
    while (true) {
      const TickType_t elapsed = (TickType_t)(xTaskGetTickCount() - started);
      if (elapsed >= TMC2208_UART_TIMEOUT_TICKS) {
        break;
      }
      TickType_t remaining = TMC2208_UART_TIMEOUT_TICKS - elapsed;
      uint8_t byte;
      int length = uart_read_bytes(driver->config.uart_port, &byte, 1, remaining);
      if (length < 0) return ESP_FAIL;
      if (length != 1) {
        break;
      }
      if (filled == sizeof(reply)) {
        memmove(reply, reply + 1, sizeof(reply) - 1);
        --filled;
      }
      reply[filled++] = byte;
      if (filled == sizeof(reply) && reply[0] == 0x05 && reply[1] == 0xFF) {
        if (reply[2] == (reg & 0x7F) &&
            reply[7] == tmc2208_crc8(reply, sizeof(reply) - 1)) {
          *out_value = tmc2208_decode_u32(&reply[3]);
          return ESP_OK;
        }
        saw_invalid_response = true;
      }
    }
    ESP_LOGW(TAG, "Register 0x%02x: no valid UART reply (attempt %u)",
             reg, (unsigned)attempt + 1);
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  return saw_invalid_response ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

static esp_err_t tmc2208_write_register_checked(tmc2208_uart_t *driver,
                                                uint8_t reg, uint32_t value) {
  uint8_t ifcnt_before = 0;
  esp_err_t ret = tmc2208_read_ifcnt(driver, &ifcnt_before);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = tmc2208_write_register(driver, reg, value);
  if (ret != ESP_OK) {
    return ret;
  }

  uint8_t ifcnt_after = 0;
  ret = tmc2208_read_ifcnt(driver, &ifcnt_after);
  if (ret != ESP_OK) {
    return ret;
  }

  if ((uint8_t)(ifcnt_after - ifcnt_before) != 1) {
    ESP_LOGE(TAG, "Register 0x%02x write not acknowledged: IFCNT %u -> %u",
             reg, ifcnt_before, ifcnt_after);
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t tmc2208_update_register(tmc2208_uart_t *driver, uint8_t reg,
                                         uint32_t mask, uint32_t value) {
  uint32_t current = 0;
  esp_err_t ret = tmc2208_read_register(driver, reg, &current);
  if (ret != ESP_OK) {
    return ret;
  }

  uint32_t updated = (current & ~mask) | (value & mask);
  if (updated == current) {
    return ESP_OK;
  }

  return tmc2208_write_register_checked(driver, reg, updated);
}

esp_err_t tmc2208_uart_init(tmc2208_uart_t *driver,
                            const tmc2208_uart_config_t *config) {
  if (!driver || !config) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(driver, 0, sizeof(*driver));
  driver->config = *config;
  driver->last_register = 0xFF;

  gpio_reset_pin(config->rx_pin);
  gpio_set_direction(config->rx_pin, GPIO_MODE_INPUT);

  gpio_reset_pin(config->tx_pin);
  gpio_set_direction(config->tx_pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_pull_mode(config->tx_pin, GPIO_PULLUP_ONLY);

  uart_config_t uart_config = {
      .baud_rate = config->baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
#if defined(UART_SCLK_DEFAULT)
  uart_config.source_clk = UART_SCLK_DEFAULT;
#elif defined(UART_SCLK_APB)
  uart_config.source_clk = UART_SCLK_APB;
#elif defined(UART_SCLK_REF_TICK)
  uart_config.source_clk = UART_SCLK_REF_TICK;
#endif

  esp_err_t ret = uart_driver_install(config->uart_port,
                                      UART_HW_FIFO_LEN(config->uart_port) * 2,
                                      0, 0, NULL, 0);
  if (ret == ESP_ERR_INVALID_STATE) {
    uart_driver_delete(config->uart_port);
    ret = uart_driver_install(config->uart_port,
                              UART_HW_FIFO_LEN(config->uart_port) * 2, 0, 0,
                              NULL, 0);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_param_config(config->uart_port, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
    uart_driver_delete(config->uart_port);
    return ret;
  }

  ret = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(ret));
    uart_driver_delete(config->uart_port);
    return ret;
  }

  uart_flush_input(config->uart_port);
  driver->initialized = true;
  return ESP_OK;
}

esp_err_t tmc2208_uart_deinit(tmc2208_uart_t *driver) {
  esp_err_t ret = tmc2208_check_driver(driver);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = uart_driver_delete(driver->config.uart_port);
  if (ret == ESP_OK) {
    driver->initialized = false;
  }
  return ret;
}

esp_err_t tmc2208_clear_gstat(tmc2208_uart_t *driver) {
  return tmc2208_write_register_checked(driver, TMC2208_REG_GSTAT, 0x00000007);
}

esp_err_t tmc2208_read_ifcnt(tmc2208_uart_t *driver, uint8_t *out_count) {
  if (!out_count) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = 0;
  esp_err_t ret = tmc2208_read_register(driver, TMC2208_REG_IFCNT, &value);
  if (ret != ESP_OK) {
    return ret;
  }

  *out_count = (uint8_t)(value & 0xFF);
  return ESP_OK;
}

esp_err_t tmc2208_read_drv_status(tmc2208_uart_t *driver,
                                  uint32_t *out_status) {
  return tmc2208_read_register(driver, TMC2208_REG_DRV_STATUS, out_status);
}

esp_err_t tmc2208_configure_gconf(tmc2208_uart_t *driver, bool pdn_disable,
                                  bool mstep_reg_select, bool i_scale_analog,
                                  bool en_spreadcycle) {
  uint32_t value = 0;
  if (pdn_disable) {
    value |= TMC2208_GCONF_PDN_DISABLE;
  }
  if (mstep_reg_select) {
    value |= TMC2208_GCONF_MSTEP_REG_SELECT;
  }
  if (i_scale_analog) {
    value |= TMC2208_GCONF_I_SCALE_ANALOG;
  }
  if (en_spreadcycle) {
    value |= TMC2208_GCONF_EN_SPREADCYCLE;
  }

  const uint32_t mask = TMC2208_GCONF_PDN_DISABLE |
                        TMC2208_GCONF_MSTEP_REG_SELECT |
                        TMC2208_GCONF_I_SCALE_ANALOG |
                        TMC2208_GCONF_EN_SPREADCYCLE;
  return tmc2208_update_register(driver, TMC2208_REG_GCONF, mask, value);
}

esp_err_t tmc2208_set_multistep_filter(tmc2208_uart_t *driver, bool enable) {
  return tmc2208_update_register(driver, TMC2208_REG_GCONF,
                                 TMC2208_GCONF_MULTISTEP_FILT,
                                 enable ? TMC2208_GCONF_MULTISTEP_FILT : 0);
}

esp_err_t tmc2208_set_toff(tmc2208_uart_t *driver, uint8_t toff) {
  if (toff > 15) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = tmc2208_set_field(0, TMC2208_CHOPCONF_TOFF_MASK, toff);
  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_TOFF_MASK, value);
}

esp_err_t tmc2208_set_tbl(tmc2208_uart_t *driver, uint8_t tbl) {
  if (tbl > 3) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = tmc2208_set_field(0, TMC2208_CHOPCONF_TBL_MASK, tbl);
  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_TBL_MASK, value);
}

esp_err_t tmc2208_set_hysteresis(tmc2208_uart_t *driver, uint8_t hstrt,
                                 uint8_t hend) {
  if (hstrt > 7 || hend > 15) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = tmc2208_set_field(0, TMC2208_CHOPCONF_HSTRT_MASK, hstrt);
  value = tmc2208_set_field(value, TMC2208_CHOPCONF_HEND_MASK, hend);

  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_HSTRT_MASK |
                                     TMC2208_CHOPCONF_HEND_MASK,
                                 value);
}

esp_err_t tmc2208_set_vsense(tmc2208_uart_t *driver, bool high_sensitivity) {
  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_VSENSE,
                                 high_sensitivity ? TMC2208_CHOPCONF_VSENSE
                                                  : 0);
}

esp_err_t tmc2208_set_intpol(tmc2208_uart_t *driver, bool enable) {
  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_INTPOL,
                                 enable ? TMC2208_CHOPCONF_INTPOL : 0);
}

esp_err_t tmc2208_set_microstep_resolution(tmc2208_uart_t *driver,
                                           tmc2208_mres_t mres) {
  if (mres > TMC2208_MRES_FULLSTEP) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = tmc2208_set_field(0, TMC2208_CHOPCONF_MRES_MASK, mres);
  return tmc2208_update_register(driver, TMC2208_REG_CHOPCONF,
                                 TMC2208_CHOPCONF_MRES_MASK, value);
}

esp_err_t tmc2208_read_microstep_resolution(tmc2208_uart_t *driver,
                                            tmc2208_mres_t *out_mres) {
  if (!out_mres) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = 0;
  esp_err_t ret = tmc2208_read_register(driver, TMC2208_REG_CHOPCONF, &value);
  if (ret != ESP_OK) {
    return ret;
  }

  uint32_t mres = tmc2208_get_field(value, TMC2208_CHOPCONF_MRES_MASK);
  if (mres > TMC2208_MRES_FULLSTEP) {
    return ESP_FAIL;
  }

  *out_mres = (tmc2208_mres_t)mres;
  return ESP_OK;
}

esp_err_t tmc2208_set_ihold_irun(tmc2208_uart_t *driver, uint8_t ihold,
                                 uint8_t irun, uint8_t ihold_delay) {
  if (ihold > 31 || irun > 31 || ihold_delay > 15) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t value = (uint32_t)ihold | ((uint32_t)irun << 8) |
                   ((uint32_t)ihold_delay << 16);
  return tmc2208_write_register_checked(driver, TMC2208_REG_IHOLD_IRUN, value);
}

esp_err_t tmc2208_set_tpowerdown(tmc2208_uart_t *driver, uint8_t tpowerdown) {
  return tmc2208_write_register_checked(driver, TMC2208_REG_TPOWERDOWN,
                                        tpowerdown);
}

esp_err_t tmc2208_set_pwm_autoscale(tmc2208_uart_t *driver, bool enable) {
  return tmc2208_update_register(driver, TMC2208_REG_PWMCONF,
                                 TMC2208_PWMCONF_AUTOSCALE,
                                 enable ? TMC2208_PWMCONF_AUTOSCALE : 0);
}

esp_err_t tmc2208_set_pwm_autograd(tmc2208_uart_t *driver, bool enable) {
  return tmc2208_update_register(driver, TMC2208_REG_PWMCONF,
                                 TMC2208_PWMCONF_AUTOGRAD,
                                 enable ? TMC2208_PWMCONF_AUTOGRAD : 0);
}

esp_err_t tmc2208_dump_gconf(tmc2208_uart_t *driver) {
  uint32_t value = 0;
  esp_err_t ret = tmc2208_read_register(driver, TMC2208_REG_GCONF, &value);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG,
             "GCONF=0x%08" PRIx32
             " pdn_disable=%" PRIu32 " mstep_reg_select=%" PRIu32
             " spreadcycle=%" PRIu32 " multistep_filt=%" PRIu32,
             value,
             (value & TMC2208_GCONF_PDN_DISABLE) ? 1u : 0u,
             (value & TMC2208_GCONF_MSTEP_REG_SELECT) ? 1u : 0u,
             (value & TMC2208_GCONF_EN_SPREADCYCLE) ? 1u : 0u,
             (value & TMC2208_GCONF_MULTISTEP_FILT) ? 1u : 0u);
  }
  return ret;
}

esp_err_t tmc2208_dump_drv_status(tmc2208_uart_t *driver) {
  uint32_t value = 0;
  esp_err_t ret = tmc2208_read_register(driver, TMC2208_REG_DRV_STATUS, &value);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "DRV_STATUS=0x%08" PRIx32, value);
  }
  return ret;
}

esp_err_t tmc2208_dump_chopconf(tmc2208_uart_t *driver) {
  uint32_t value = 0;
  esp_err_t ret = tmc2208_read_register(driver, TMC2208_REG_CHOPCONF, &value);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG,
             "CHOPCONF=0x%08" PRIx32
             " toff=%" PRIu32 " hstrt=%" PRIu32 " hend=%" PRIu32
             " tbl=%" PRIu32 " vsense=%u mres=%" PRIu32 " intpol=%u",
             value, tmc2208_get_field(value, TMC2208_CHOPCONF_TOFF_MASK),
             tmc2208_get_field(value, TMC2208_CHOPCONF_HSTRT_MASK),
             tmc2208_get_field(value, TMC2208_CHOPCONF_HEND_MASK),
             tmc2208_get_field(value, TMC2208_CHOPCONF_TBL_MASK),
             (value & TMC2208_CHOPCONF_VSENSE) ? 1 : 0,
             tmc2208_get_field(value, TMC2208_CHOPCONF_MRES_MASK),
             (value & TMC2208_CHOPCONF_INTPOL) ? 1 : 0);
  }
  return ret;
}
