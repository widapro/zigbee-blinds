#include "diagnostics.h"
#include "esp_attr.h"

#define DIAG_RTC_MAGIC 0xB11D5D1AUL
#define DIAG_STAGE_MAX DIAG_STAGE_BLE_OTA_WRITE

RTC_NOINIT_ATTR static uint32_t s_diag_magic;
RTC_NOINIT_ATTR static uint8_t s_diag_stage;

uint8_t g_last_crash_stage = DIAG_STAGE_UNKNOWN;

static bool is_fault_reset(esp_reset_reason_t reset_reason) {
  return reset_reason == ESP_RST_PANIC ||
         reset_reason == ESP_RST_INT_WDT ||
         reset_reason == ESP_RST_TASK_WDT ||
         reset_reason == ESP_RST_WDT ||
         reset_reason == ESP_RST_BROWNOUT;
}

void diagnostics_init(esp_reset_reason_t reset_reason) {
  if (s_diag_magic == DIAG_RTC_MAGIC && s_diag_stage <= DIAG_STAGE_MAX &&
      is_fault_reset(reset_reason)) {
    g_last_crash_stage = s_diag_stage;
  } else {
    g_last_crash_stage = DIAG_STAGE_UNKNOWN;
  }

  s_diag_magic = DIAG_RTC_MAGIC;
  s_diag_stage = DIAG_STAGE_APP_START;
}

void diagnostics_mark_stage(diag_stage_t stage) {
  s_diag_magic = DIAG_RTC_MAGIC;
  s_diag_stage = (uint8_t)stage;
}
