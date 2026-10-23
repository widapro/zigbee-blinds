const {test} = require('node:test');
const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');

test('actual TMC UART handles early replies, echo, corrupt frames and bounded timeouts', () => {
    const root = path.resolve(__dirname, '..');
    const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'blinds-uart-test-'));
    try {
        const headers = {
            'esp_err.h': `#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
static inline const char *esp_err_to_name(esp_err_t e) { (void)e; return "mock"; }
`,
            'freertos/FreeRTOS.h': `#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
`,
            'freertos/task.h': `#pragma once
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
`,
            'esp_log.h': `#pragma once
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
#define ESP_LOGD(tag, ...) ((void)(tag))
#define ESP_LOG_DEBUG 0
#define ESP_LOG_BUFFER_HEX_LEVEL(tag, ...) ((void)(tag))
`,
            'driver/gpio.h': `#pragma once
#include "esp_err.h"
typedef int gpio_num_t;
#define GPIO_MODE_INPUT 1
#define GPIO_MODE_OUTPUT_OD 2
#define GPIO_PULLUP_ONLY 1
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_set_direction(gpio_num_t pin, int mode);
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int mode);
`,
            'driver/uart.h': `#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef int uart_port_t;
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk; } uart_config_t;
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_PIN_NO_CHANGE -1
#define UART_HW_FIFO_LEN(port) 128
int uart_write_bytes(uart_port_t port, const void *data, size_t size);
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t timeout);
esp_err_t uart_flush_input(uart_port_t port);
int uart_read_bytes(uart_port_t port, void *data, uint32_t size, TickType_t timeout);
esp_err_t uart_driver_install(uart_port_t port, int rx, int tx, int queue, void *handle, int flags);
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
`,
        };
        for (const [name, content] of Object.entries(headers)) {
            const file = path.join(temp, name);
            fs.mkdirSync(path.dirname(file), {recursive: true});
            fs.writeFileSync(file, content);
        }
        const executable = path.join(temp, 'tmc-uart');
        const build = spawnSync('cc', ['-std=c11', '-Wall', '-Wextra', '-Werror',
            '-I', temp, '-I', path.join(root, 'main'), path.join(root, 'main/tmc2208_uart.c'),
            path.join(__dirname, 'tmc_uart_test.c'), '-o', executable], {encoding: 'utf8'});
        assert.equal(build.status, 0, build.stderr || String(build.error));
        const run = spawnSync(executable, [], {encoding: 'utf8'});
        assert.equal(run.status, 0, run.stderr || String(run.error));
        assert.match(run.stdout, /11 UART scenarios passed/);
    } finally {
        fs.rmSync(temp, {recursive: true, force: true});
    }
});
