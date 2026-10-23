#include "motor_startup.h"

motor_startup_result_t motor_startup_run(const motor_startup_ops_t *ops) {
  motor_startup_result_t result = {MOTOR_STARTUP_FAILED, 0, -1};
  for (uint8_t attempt = 1; attempt <= MOTOR_STARTUP_MAX_ATTEMPTS; ++attempt) {
    if (ops->cancelled(ops->context)) {
      result.status = MOTOR_STARTUP_CANCELLED;
      return result;
    }
    result.attempts = attempt;
    result.error = ops->configure(ops->context);
    if (ops->cancelled(ops->context)) {
      result.status = MOTOR_STARTUP_CANCELLED;
      return result;
    }
    if (result.error == 0) {
      result.status = MOTOR_STARTUP_READY;
      return result;
    }
    if (attempt < MOTOR_STARTUP_MAX_ATTEMPTS) {
      ops->recover(ops->context, attempt);
    }
  }
  return result;
}
