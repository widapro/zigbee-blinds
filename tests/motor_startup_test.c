#include <assert.h>
#include <stdio.h>
#include "motor_startup.h"

typedef struct {
  int failures, calls, recoveries;
  bool stop, stop_in_configure, stop_in_recover;
} fixture_t;

static int configure(void *context) {
  fixture_t *f = context;
  ++f->calls;
  // Every configuration after a failure must have passed through recovery.
  assert(f->recoveries == f->calls - 1);
  if (f->stop_in_configure) f->stop = true;
  return f->calls <= f->failures ? 0x107 : 0;
}
static void recover(void *context, uint8_t failed_attempt) {
  fixture_t *f = context;
  ++f->recoveries;
  assert(f->calls == failed_attempt);
  if (f->stop_in_recover) f->stop = true;
}
static bool cancelled(void *context) { return ((fixture_t *)context)->stop; }
static motor_startup_result_t run(fixture_t *f) {
  motor_startup_ops_t ops = {f, configure, recover, cancelled};
  return motor_startup_run(&ops);
}

int main(void) {
  for (int failures = 0; failures <= 3; ++failures) {
    fixture_t f = {.failures = failures};
    motor_startup_result_t result = run(&f);
    if (failures < 3) {
      assert(result.status == MOTOR_STARTUP_READY && result.error == 0);
      assert(result.attempts == failures + 1 && f.recoveries == failures);
    } else {
      assert(result.status == MOTOR_STARTUP_FAILED && result.error == 0x107);
      assert(result.attempts == 3 && f.calls == 3 && f.recoveries == 2);
    }
  }
  fixture_t before = {.stop = true};
  assert(run(&before).status == MOTOR_STARTUP_CANCELLED && before.calls == 0);
  fixture_t during = {.stop_in_configure = true};
  assert(run(&during).status == MOTOR_STARTUP_CANCELLED && during.calls == 1 && during.recoveries == 0);
  fixture_t between = {.failures = 3, .stop_in_recover = true};
  assert(run(&between).status == MOTOR_STARTUP_CANCELLED && between.calls == 1 && between.recoveries == 1);
  puts("7 motor-startup scenarios passed");
  return 0;
}
