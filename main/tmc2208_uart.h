#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

typedef enum {
  TMC2208_MRES_256 = 0,
  TMC2208_MRES_128 = 1,
  TMC2208_MRES_64 = 2,
  TMC2208_MRES_32 = 3,
  TMC2208_MRES_16 = 4,
  TMC2208_MRES_8 = 5,
  TMC2208_MRES_4 = 6,
  TMC2208_MRES_2 = 7,
  TMC2208_MRES_FULLSTEP = 8,
} tmc2208_mres_t;

typedef struct {
  uart_port_t uart_port;
  gpio_num_t rx_pin;
  gpio_num_t tx_pin;
  uint32_t baud_rate;
} tmc2208_uart_config_t;

typedef struct {
  tmc2208_uart_config_t config;
  bool initialized;
  uint8_t last_register;
} tmc2208_uart_t;

esp_err_t tmc2208_uart_init(tmc2208_uart_t *driver,
                            const tmc2208_uart_config_t *config);
esp_err_t tmc2208_uart_deinit(tmc2208_uart_t *driver);

esp_err_t tmc2208_clear_gstat(tmc2208_uart_t *driver);
esp_err_t tmc2208_read_ifcnt(tmc2208_uart_t *driver, uint8_t *out_count);
esp_err_t tmc2208_read_drv_status(tmc2208_uart_t *driver,
                                  uint32_t *out_status);

esp_err_t tmc2208_configure_gconf(tmc2208_uart_t *driver, bool pdn_disable,
                                  bool mstep_reg_select, bool i_scale_analog,
                                  bool en_spreadcycle);
esp_err_t tmc2208_set_multistep_filter(tmc2208_uart_t *driver, bool enable);
esp_err_t tmc2208_set_toff(tmc2208_uart_t *driver, uint8_t toff);
esp_err_t tmc2208_set_tbl(tmc2208_uart_t *driver, uint8_t tbl);
esp_err_t tmc2208_set_hysteresis(tmc2208_uart_t *driver, uint8_t hstrt,
                                 uint8_t hend);
esp_err_t tmc2208_set_vsense(tmc2208_uart_t *driver, bool high_sensitivity);
esp_err_t tmc2208_set_intpol(tmc2208_uart_t *driver, bool enable);
esp_err_t tmc2208_set_microstep_resolution(tmc2208_uart_t *driver,
                                           tmc2208_mres_t mres);
esp_err_t tmc2208_read_microstep_resolution(tmc2208_uart_t *driver,
                                            tmc2208_mres_t *out_mres);
esp_err_t tmc2208_set_ihold_irun(tmc2208_uart_t *driver, uint8_t ihold,
                                 uint8_t irun, uint8_t ihold_delay);
esp_err_t tmc2208_set_tpowerdown(tmc2208_uart_t *driver, uint8_t tpowerdown);
esp_err_t tmc2208_set_pwm_autoscale(tmc2208_uart_t *driver, bool enable);
esp_err_t tmc2208_set_pwm_autograd(tmc2208_uart_t *driver, bool enable);

esp_err_t tmc2208_dump_gconf(tmc2208_uart_t *driver);
esp_err_t tmc2208_dump_drv_status(tmc2208_uart_t *driver);
esp_err_t tmc2208_dump_chopconf(tmc2208_uart_t *driver);
