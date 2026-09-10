#pragma once

#include "esp_err.h"

// LED control functions
esp_err_t led_init(void);
void led_blink_green(int times);
void led_blink_blue(int times);
void led_solid_blue(void);