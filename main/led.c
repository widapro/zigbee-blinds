#include "led.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "main.h"

static const char *TAG = "LED";

// WS2812 LED strip handle
static led_strip_handle_t led_strip = NULL;

esp_err_t led_init(void) {
  ESP_LOGI(TAG, "Initializing WS2812 LED on GPIO%d...", WS2812_GPIO);

  led_strip_config_t strip_config = {
      .strip_gpio_num = WS2812_GPIO,
      .max_leds = WS2812_LED_COUNT,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB,
      .led_model = LED_MODEL_WS2812,
      .flags.invert_out = false,
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };

  esp_err_t ret =
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
    return ret;
  }

  // Clear LED
  led_strip_clear(led_strip);
  ESP_LOGI(TAG, "WS2812 LED initialized successfully");
  return ESP_OK;
}

void led_blink_green(int times) {
  if (!led_strip) {
    ESP_LOGE(TAG, "WS2812 LED strip not initialized");
    return;
  }

  ESP_LOGI(TAG, "Blinking WS2812 LED GREEN %d times", times);

  for (int i = 0; i < times; i++) {
    ESP_LOGI(TAG, "LED GREEN ON (blink %d/%d)", i + 1, times);
    led_strip_set_pixel(led_strip, 0, 0, 255, 0); // Green (GRB format)
    led_strip_refresh(led_strip);
    vTaskDelay(600 / portTICK_PERIOD_MS); // Slower startup blink for visibility

    ESP_LOGI(TAG, "LED OFF");
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);

    if (i < times - 1) {                    // No delay after the last blink
      vTaskDelay(600 / portTICK_PERIOD_MS); // Match the on-time gap
    }
  }
}

void led_blink_blue(int times) {
  if (!led_strip) {
    ESP_LOGE(TAG, "WS2812 LED strip not initialized");
    return;
  }

  ESP_LOGI(TAG, "Blinking WS2812 LED BLUE %d times", times);

  for (int i = 0; i < times; i++) {
    ESP_LOGI(TAG, "LED BLUE ON (blink %d/%d)", i + 1, times);
    led_strip_set_pixel(led_strip, 0, 0, 0, 255); // Blue (GRB format)
    led_strip_refresh(led_strip);
    vTaskDelay(300 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "LED OFF");
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);

    if (i < times - 1) { // No delay after the last blink
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }
  }
}

void led_solid_blue(void) {
  if (!led_strip) {
    ESP_LOGE(TAG, "WS2812 LED strip not initialized");
    return;
  }

  ESP_LOGI(TAG, "LED BLUE ON (solid)");
  led_strip_set_pixel(led_strip, 0, 0, 0, 255); // Blue (GRB format)
  led_strip_refresh(led_strip);
}
