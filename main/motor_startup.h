#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_STARTUP_MAX_ATTEMPTS 3

typedef struct {
  void *context;
  int (*configure)(void *context);
  void (*recover)(void *context, uint8_t failed_attempt);
  bool (*cancelled)(void *context);
} motor_startup_ops_t;

typedef enum {
  MOTOR_STARTUP_READY,
  MOTOR_STARTUP_FAILED,
  MOTOR_STARTUP_CANCELLED,
} motor_startup_status_t;

typedef struct {
  motor_startup_status_t status;
  uint8_t attempts;
  int error;
} motor_startup_result_t;

// Only driver configuration is retried. The caller must keep coils disabled
// and issue no STEP pulses until READY, while holding its motor/PM locks.
motor_startup_result_t motor_startup_run(const motor_startup_ops_t *ops);
