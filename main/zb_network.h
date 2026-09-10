#pragma once

#include "esp_err.h"
#include "esp_zigbee_core.h"

// Zigbee network configuration defines
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN

// Performance profile:
// 1 = keep radio awake for maximum command responsiveness (higher power)
// 0 = sleepy end device with short polling for battery-friendly responsiveness
#define ZB_RESPONSIVE_MODE 0

#if ZB_RESPONSIVE_MODE
#define ED_KEEP_ALIVE 1000
#define ZB_SLEEP_ENABLE 0
#define ZB_RX_ON_WHEN_IDLE 1
#define ZB_PM_MAX_FREQ_MHZ 96
#define ZB_PM_MIN_FREQ_MHZ 96
#define ZB_TX_POWER_DBM 20
#else
#define ED_KEEP_ALIVE 1000
#define ZB_SLEEP_ENABLE 1
#define ZB_RX_ON_WHEN_IDLE 0
#define ZB_PM_MAX_FREQ_MHZ 64
#define ZB_PM_MIN_FREQ_MHZ 32
#define ZB_TX_POWER_DBM 20
#endif

// Zigbee configuration macros
#define ESP_ZB_ZED_CONFIG()                                                    \
  {                                                                            \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                                      \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,                          \
    .nwk_cfg.zed_cfg = {                                                       \
        .ed_timeout = ED_AGING_TIMEOUT,                                        \
        .keep_alive = ED_KEEP_ALIVE,                                           \
    },                                                                         \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                                          \
  { .radio_mode = ZB_RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                                           \
  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, }

// Function declarations
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                            const void *message);
void zb_network_init(void);
void esp_zb_task(void *pvParameters);

// Power management and sleep functions
esp_err_t zb_network_init_power_management(void);
