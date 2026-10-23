#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BLE_OTA_TRANSFER_BUFFER_SIZE (64 * 244)

typedef bool (*ble_ota_transfer_writer_t)(void *context, const uint8_t *data,
                                         size_t len);

typedef struct {
  uint32_t size;
  uint32_t crc_expected;
  uint32_t crc;
  uint32_t received;
  uint32_t committed;
  size_t pending;
  bool failed;
  uint8_t buffer[BLE_OTA_TRANSFER_BUFFER_SIZE];
} ble_ota_transfer_t;

void ble_ota_transfer_reset(ble_ota_transfer_t *state);
void ble_ota_transfer_begin(ble_ota_transfer_t *state, uint32_t size,
                            uint32_t crc_expected);
void ble_ota_transfer_discard(ble_ota_transfer_t *state);
bool ble_ota_transfer_receive(ble_ota_transfer_t *state, const uint8_t *data,
                              size_t len, ble_ota_transfer_writer_t writer,
                              void *context);
bool ble_ota_transfer_flush(ble_ota_transfer_t *state,
                            ble_ota_transfer_writer_t writer, void *context);
bool ble_ota_transfer_finish(ble_ota_transfer_t *state,
                             ble_ota_transfer_writer_t writer, void *context);
