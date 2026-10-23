#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tmc2208_uart.h"

enum { EARLY_REPLY, ECHO_REPLY, NOISY_REPLY, CORRUPT_THEN_VALID,
       WRONG_REGISTER_THEN_VALID, RETRY_REPLY, NO_REPLY, BAD_ACK,
       CORRUPT_ONLY, ECHO_ONLY };
static int scenario, requests, writes, flushes;
static TickType_t now;
static uint8_t rx[80], count;
static uint8_t ifcnt;

static uint8_t crc8(const uint8_t *data, size_t size) {
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    uint8_t byte = data[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (uint8_t)(((crc >> 7) ^ (byte & 1)) ? (crc << 1) ^ 7 : crc << 1);
      byte >>= 1;
    }
  }
  return crc;
}

static void append(const uint8_t *data, size_t size) {
  assert(count + size <= sizeof(rx));
  memcpy(rx + count, data, size);
  count += size;
}

static void reply(uint8_t reg, bool corrupt) {
  uint8_t frame[] = {5, 0xff, reg, 0, 0, 0, ifcnt, 0};
  frame[7] = crc8(frame, 7) ^ (corrupt ? 1 : 0);
  append(frame, sizeof(frame));
}

int uart_write_bytes(uart_port_t port, const void *data, size_t size) {
  (void)port;
  const uint8_t *frame = data;
  assert(frame[size - 1] == crc8(frame, size - 1));
  if (size == 8) {
    ++writes;
    if (scenario != BAD_ACK) ++ifcnt;
    append(frame, size); // Single-wire TX echo.
  } else {
    assert(size == 4);
    ++requests;
    if (scenario == NO_REPLY || (scenario == RETRY_REPLY && requests == 1)) return size;
    if (scenario == ECHO_ONLY) { append(frame, size); return size; }
    if (scenario == CORRUPT_ONLY) { reply(frame[2], true); return size; }
    if (scenario == ECHO_REPLY || scenario == NOISY_REPLY) append(frame, size);
    if (scenario == NOISY_REPLY) {
      const uint8_t noise[] = {0xff, 0, 5, 0xff, 0x70, 0, 1};
      append(noise, sizeof(noise));
    }
    if (scenario == CORRUPT_THEN_VALID) reply(frame[2], true);
    if (scenario == WRONG_REGISTER_THEN_VALID) reply(0x70, false);
    reply(frame[2], false); // Answer arrives before TX-wait returns.
  }
  return size;
}

esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t timeout) {
  (void)port; (void)timeout;
  ++now;
  return ESP_OK;
}
esp_err_t uart_flush_input(uart_port_t port) {
  (void)port;
  count = 0;
  ++flushes;
  return ESP_OK;
}
int uart_read_bytes(uart_port_t port, void *data, uint32_t size, TickType_t timeout) {
  (void)port;
  if (!count) { now += timeout; return 0; }
  // Deliver fragmented bytes; a receiver must assemble its response.
  uint32_t received = size > 2 ? 2 : size;
  if (received > count) received = count;
  memcpy(data, rx, received);
  memmove(rx, rx + received, count - received);
  count -= received;
  return received;
}
TickType_t xTaskGetTickCount(void) { return now; }
void vTaskDelay(TickType_t ticks) { now += ticks; }
esp_err_t gpio_reset_pin(gpio_num_t pin) { (void)pin; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t pin, int mode) { (void)pin; (void)mode; return ESP_OK; }
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int mode) { (void)pin; (void)mode; return ESP_OK; }
esp_err_t uart_driver_install(uart_port_t p, int rx_size, int tx, int q, void *h, int f) {
  (void)p; (void)rx_size; (void)tx; (void)q; (void)h; (void)f; return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t p) { (void)p; return ESP_OK; }
esp_err_t uart_param_config(uart_port_t p, const uart_config_t *c) { (void)p; (void)c; return ESP_OK; }
esp_err_t uart_set_pin(uart_port_t p, int tx, int rx_pin, int rts, int cts) {
  (void)p; (void)tx; (void)rx_pin; (void)rts; (void)cts; return ESP_OK;
}

int main(void) {
  tmc2208_uart_t driver = {.initialized = true};
  for (scenario = EARLY_REPLY; scenario <= NO_REPLY; ++scenario) {
    now = 0; count = 0; requests = 0; flushes = 0; ifcnt = 42;
    uint8_t actual = 0;
    esp_err_t result = tmc2208_read_ifcnt(&driver, &actual);
    if (scenario == NO_REPLY) {
      assert(result == ESP_ERR_TIMEOUT && actual == 0 && requests == 2 && now <= 210);
    } else {
      assert(result == ESP_OK && actual == 42);
      assert(requests == (scenario == RETRY_REPLY ? 2 : 1));
      assert(flushes == requests); // No RX discard after sending a request.
    }
  }
  // A successful UART transmit without a driver acknowledgement still fails.
  scenario = BAD_ACK; count = 0; requests = 0; writes = 0;
  assert(tmc2208_set_ihold_irun(&driver, 8, 31, 10) != ESP_OK);
  assert(writes == 1 && requests == 2);
  scenario = ECHO_REPLY; count = 0; requests = 0; writes = 0; ifcnt = 255;
  assert(tmc2208_set_ihold_irun(&driver, 8, 31, 10) == ESP_OK);
  assert(writes == 1 && requests == 2 && ifcnt == 0);
  uint8_t actual;
  scenario = CORRUPT_ONLY; count = 0; requests = 0;
  assert(tmc2208_read_ifcnt(&driver, &actual) == ESP_ERR_INVALID_RESPONSE);
  assert(requests == 2);
  scenario = ECHO_ONLY; count = 0; requests = 0;
  assert(tmc2208_read_ifcnt(&driver, &actual) == ESP_ERR_TIMEOUT);
  assert(requests == 2);
  puts("11 UART scenarios passed");
  return 0;
}
