#include "ble_ota_transfer.h"
#include <string.h>

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
  }
  return crc;
}

void ble_ota_transfer_reset(ble_ota_transfer_t *state) {
  state->size = 0;
  state->crc_expected = 0;
  state->crc = 0xffffffffU;
  state->received = 0;
  state->committed = 0;
  state->pending = 0;
  state->failed = false;
}

void ble_ota_transfer_begin(ble_ota_transfer_t *state, uint32_t size,
                            uint32_t crc_expected) {
  ble_ota_transfer_reset(state);
  state->size = size;
  state->crc_expected = crc_expected;
  state->failed = size == 0;
}

void ble_ota_transfer_discard(ble_ota_transfer_t *state) {
  state->pending = 0;
  state->failed = true;
}

bool ble_ota_transfer_flush(ble_ota_transfer_t *state,
                            ble_ota_transfer_writer_t writer, void *context) {
  if (state->failed) return false;
  if (state->pending == 0) return true;
  if (!writer(context, state->buffer, state->pending)) {
    state->failed = true;
    return false;
  }
  // Advertise progress only after the flash writer has succeeded.
  state->committed += state->pending;
  state->pending = 0;
  return true;
}

bool ble_ota_transfer_receive(ble_ota_transfer_t *state, const uint8_t *data,
                              size_t len, ble_ota_transfer_writer_t writer,
                              void *context) {
  if (state->failed || len == 0 || len > sizeof(state->buffer) ||
      len > state->size - state->received) {
    state->failed = true;
    return false;
  }
  if (len > sizeof(state->buffer) - state->pending &&
      !ble_ota_transfer_flush(state, writer, context)) {
    return false;
  }
  memcpy(state->buffer + state->pending, data, len);
  state->pending += len;
  state->received += len;
  state->crc = crc32_update(state->crc, data, len);
  return true;
}

bool ble_ota_transfer_finish(ble_ota_transfer_t *state,
                             ble_ota_transfer_writer_t writer, void *context) {
  if (state->failed || state->received != state->size ||
      (state->crc ^ 0xffffffffU) != state->crc_expected) {
    state->failed = true;
    return false;
  }
  return ble_ota_transfer_flush(state, writer, context);
}
