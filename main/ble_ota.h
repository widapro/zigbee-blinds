#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Starts the BLE OTA mode.
 *
 * This function should be called to switch the device from Zigbee mode
 * into Bluetooth LE OTA mode. It will:
 * 1. Initialize the NimBLE stack.
 * 2. Start advertising the OTA service.
 * 3. Start a safety timer (3 minutes) to reboot if no connection is made or
 *    if the update doesn't complete.
 */
void start_ble_ota_mode(void);
