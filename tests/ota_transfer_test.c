#include "ble_ota_transfer.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t small_image[] = {0xe9, 1, 2, 3};
static uint8_t image[20000];
// Reference CRCs generated independently with Python zlib.crc32.
#define SMALL_CRC 0x20ce88a1U
#define IMAGE_CRC 0x4e442524U

typedef struct {
  uint8_t data[20000];
  size_t size;
  unsigned calls;
  unsigned fail_on_call;
} flash_t;

static bool write_flash(void *context, const uint8_t *data, size_t len) {
  flash_t *flash = context;
  ++flash->calls;
  if (flash->calls == flash->fail_on_call) return false;
  assert(flash->size + len <= sizeof(flash->data));
  memcpy(flash->data + flash->size, data, len);
  flash->size += len;
  return true;
}

static void receive_range(ble_ota_transfer_t *state, flash_t *flash,
                           size_t offset, size_t end) {
  while (offset < end) {
    size_t len = end - offset < 244 ? end - offset : 244;
    assert(ble_ota_transfer_receive(state, image + offset, len, write_flash, flash));
    offset += len;
  }
}

static void block_is_written_before_progress_is_committed(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, sizeof(image), IMAGE_CRC);
  receive_range(&state, &flash, 0, BLE_OTA_TRANSFER_BUFFER_SIZE);
  assert(flash.calls == 0 && state.committed == 0);
  assert(state.received == BLE_OTA_TRANSFER_BUFFER_SIZE);
  assert(ble_ota_transfer_flush(&state, write_flash, &flash));
  assert(flash.calls == 1 && state.committed == flash.size);
  receive_range(&state, &flash, BLE_OTA_TRANSFER_BUFFER_SIZE, sizeof(image));
  assert(state.committed == BLE_OTA_TRANSFER_BUFFER_SIZE);
  assert(ble_ota_transfer_flush(&state, write_flash, &flash));
  assert(ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(flash.calls == 2 && state.committed == sizeof(image));
  assert(memcmp(flash.data, image, sizeof(image)) == 0);
}

static void finish_flushes_the_final_short_block(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, sizeof(small_image), SMALL_CRC);
  assert(ble_ota_transfer_receive(&state, small_image, sizeof(small_image), write_flash, &flash));
  assert(flash.calls == 0);
  assert(ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(state.committed == sizeof(small_image) && flash.calls == 1);
  assert(memcmp(flash.data, small_image, sizeof(small_image)) == 0);
}

static void capacity_is_bounded_even_without_status_reads(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, sizeof(image), IMAGE_CRC);
  receive_range(&state, &flash, 0, sizeof(image));
  assert(flash.calls == 1 && state.pending == sizeof(image) - BLE_OTA_TRANSFER_BUFFER_SIZE);
  assert(ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(memcmp(flash.data, image, sizeof(image)) == 0);
}

static void failed_flash_write_is_never_acknowledged_or_retried(void) {
  ble_ota_transfer_t state;
  flash_t flash = {.fail_on_call = 1};
  ble_ota_transfer_begin(&state, sizeof(small_image), SMALL_CRC);
  assert(ble_ota_transfer_receive(&state, small_image, sizeof(small_image), write_flash, &flash));
  assert(!ble_ota_transfer_flush(&state, write_flash, &flash));
  assert(state.committed == 0 && state.failed);
  assert(!ble_ota_transfer_flush(&state, write_flash, &flash));
  assert(!ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(flash.calls == 1);
}

static void failure_preserves_the_last_successful_offset(void) {
  ble_ota_transfer_t state;
  flash_t flash = {.fail_on_call = 2};
  ble_ota_transfer_begin(&state, sizeof(image), IMAGE_CRC);
  receive_range(&state, &flash, 0, 7808);
  assert(ble_ota_transfer_flush(&state, write_flash, &flash));
  receive_range(&state, &flash, 7808, 15616);
  assert(!ble_ota_transfer_flush(&state, write_flash, &flash));
  assert(state.committed == 7808 && flash.size == 7808);
}

static void incomplete_and_overlong_images_are_rejected(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, sizeof(small_image), SMALL_CRC);
  assert(ble_ota_transfer_receive(&state, small_image, 3, write_flash, &flash));
  assert(!ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(flash.calls == 0);
  ble_ota_transfer_begin(&state, 3, SMALL_CRC);
  assert(!ble_ota_transfer_receive(&state, small_image, 4, write_flash, &flash));
  assert(state.received == 0 && state.committed == 0);
}

static void corrupt_or_reordered_data_cannot_finish(void) {
  const uint8_t corrupt[] = {0xe9, 1, 2, 4};
  const uint8_t reordered[] = {1, 0xe9, 2, 3};
  const uint8_t *cases[] = {corrupt, reordered};
  for (size_t i = 0; i < 2; ++i) {
    ble_ota_transfer_t state;
    flash_t flash = {0};
    ble_ota_transfer_begin(&state, sizeof(small_image), SMALL_CRC);
    assert(ble_ota_transfer_receive(&state, cases[i], 4, write_flash, &flash));
    assert(!ble_ota_transfer_finish(&state, write_flash, &flash));
    assert(flash.calls == 0);
  }
}

static void interrupted_buffer_is_discarded_before_restart(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, sizeof(image), IMAGE_CRC);
  receive_range(&state, &flash, 0, 5000);
  ble_ota_transfer_discard(&state);
  assert(state.pending == 0 && state.failed);
  assert(!ble_ota_transfer_flush(&state, write_flash, &flash));
  ble_ota_transfer_begin(&state, sizeof(small_image), SMALL_CRC);
  assert(state.received == 0 && state.committed == 0);
  assert(ble_ota_transfer_receive(&state, small_image, 4, write_flash, &flash));
  assert(ble_ota_transfer_finish(&state, write_flash, &flash));
  assert(flash.size == 4 && memcmp(flash.data, small_image, 4) == 0);
}

static void empty_image_and_oversized_packets_are_rejected(void) {
  ble_ota_transfer_t state;
  flash_t flash = {0};
  ble_ota_transfer_begin(&state, 0, 0);
  assert(!ble_ota_transfer_finish(&state, write_flash, &flash));
  ble_ota_transfer_begin(&state, sizeof(image), IMAGE_CRC);
  assert(!ble_ota_transfer_receive(&state, image, BLE_OTA_TRANSFER_BUFFER_SIZE + 1,
                                  write_flash, &flash));
  assert(flash.calls == 0 && state.pending == 0);
}

int main(void) {
  for (size_t i = 0; i < sizeof(image); ++i) image[i] = i % 251;
  image[0] = 0xe9;
  block_is_written_before_progress_is_committed();
  finish_flushes_the_final_short_block();
  capacity_is_bounded_even_without_status_reads();
  failed_flash_write_is_never_acknowledged_or_retried();
  failure_preserves_the_last_successful_offset();
  incomplete_and_overlong_images_are_rejected();
  corrupt_or_reordered_data_cannot_finish();
  interrupted_buffer_is_discarded_before_restart();
  empty_image_and_oversized_packets_are_rejected();
  puts("9 OTA transfer scenarios passed");
  return 0;
}
