#include "command_control.h"
#include <assert.h>
#include <stdio.h>

static motor_cmd_t move(uint32_t id, uint8_t target) {
  return (motor_cmd_t){.type = MOTOR_CMD_MOVE, .endpoint = 1,
                       .id = id, .target_position = target};
}

static void unaligned_wire_request_decodes_without_word_access(void) {
  // Attribute payload starts at an odd address inside this test packet.
  const uint8_t packet[] = {0xff, 0x65, 0x56, 0x34, 0x12};
  assert(command_control_unpack_request(packet + 1) == 0x12345665U);
}

static void accept(command_control_t *state, motor_cmd_t cmd, bool interrupt) {
  bool actual;
  assert(command_control_submit(state, cmd, &actual) == COMMAND_SUBMIT_NEW);
  assert(actual == interrupt);
}

static void pending_blocks_sleep_and_latest_wins(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  accept(&state, move(1, 10), false);
  assert(command_control_busy(&state));
  accept(&state, move(2, 90), false);
  assert(command_control_take(&state, &cmd));
  assert(cmd.id == 2 && cmd.target_position == 90);
  assert(command_control_busy(&state));
  command_control_finish(&state, COMMAND_COMPLETED);
  assert(!command_control_busy(&state));
  assert(command_control_result(&state) == (2 << 8 | COMMAND_COMPLETED));
}

static void duplicate_never_restarts_motor(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  bool interrupt;
  accept(&state, move(1, 40), false);
  assert(command_control_submit(&state, move(1, 40), &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(!interrupt);
  assert(command_control_take(&state, &cmd));
  assert(command_control_submit(&state, move(1, 40), &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(!interrupt && !state.pending);
  command_control_finish(&state, COMMAND_COMPLETED);
  assert(command_control_submit(&state, move(1, 40), &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(!command_control_busy(&state));
  assert(state.completed == 1 && state.duplicates == 3);
}

static void new_id_same_target_does_not_preempt(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  accept(&state, move(1, 40), false);
  command_control_take(&state, &cmd);
  accept(&state, move(2, 40), false);
  command_control_finish(&state, COMMAND_COMPLETED);
  // Finishing the old execution must not falsely complete the new request.
  assert(command_control_result(&state) == (2 << 8 | COMMAND_ACCEPTED));
  assert(command_control_take(&state, &cmd) && cmd.id == 2);
  command_control_finish(&state, COMMAND_COMPLETED);
}

static void stop_followed_by_new_target_preserves_new_target(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  accept(&state, move(1, 90), false);
  command_control_take(&state, &cmd);
  motor_cmd_t stop = {.type = MOTOR_CMD_STOP, .endpoint = 1,
                      .id = 2, .target_position = COMMAND_STOP_TARGET};
  accept(&state, stop, true);
  accept(&state, move(3, 20), true);
  command_control_finish(&state, COMMAND_SUPERSEDED);
  assert(command_control_take(&state, &cmd));
  assert(cmd.id == 3 && cmd.target_position == 20);
  command_control_finish(&state, COMMAND_COMPLETED);
  bool interrupt;
  assert(command_control_submit(&state, stop, &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(!command_control_busy(&state));
}

static void stop_taken_before_new_target_cannot_overwrite_it(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  motor_cmd_t stop = {.type = MOTOR_CMD_STOP, .endpoint = 1,
                      .id = 1, .target_position = COMMAND_STOP_TARGET};
  accept(&state, stop, false);
  command_control_take(&state, &cmd);
  accept(&state, move(2, 20), true);
  command_control_finish(&state, COMMAND_STOPPED);
  assert(command_control_take(&state, &cmd) && cmd.target_position == 20);
}

static void unseen_late_command_and_evicted_duplicates_are_rejected(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  bool interrupt;
  accept(&state, move(100, 10), false);
  command_control_take(&state, &cmd);
  for (unsigned id = 101; id < 150; ++id) {
    accept(&state, move(id, 90), true);
  }
  assert(command_control_submit(&state, move(100, 10), &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(command_control_submit(&state, move(99, 80), &interrupt) == COMMAND_SUBMIT_STALE);
  assert(command_control_submit(&state, move(110, 90), &interrupt) == COMMAND_SUBMIT_STALE);
  assert(!interrupt && state.pending_command.id == 149);
}

static void sequence_wrap_is_safe(void) {
  command_control_t state = {0};
  bool interrupt;
  accept(&state, move(0xFFFFFE, 10), false);
  accept(&state, move(1, 90), false);
  assert(command_control_submit(&state, move(0xFFFFFD, 80), &interrupt) == COMMAND_SUBMIT_STALE);
  assert(state.pending_command.id == 1);
}

static void invalid_requests_do_not_cancel_accepted_work(void) {
  command_control_t state = {0};
  bool interrupt;
  accept(&state, move(1, 10), false);
  assert(command_control_submit(&state, move(1, 90), &interrupt) == COMMAND_SUBMIT_INVALID);
  assert(command_control_submit(&state, move(2, 255), &interrupt) == COMMAND_SUBMIT_INVALID);
  assert(state.pending_command.target_position == 10 && state.failed == 2);
}

static void standard_commands_and_failures_are_observable(void) {
  command_control_t state = {0};
  motor_cmd_t cmd;
  bool interrupt;
  accept(&state, move(0, 10), false);
  command_control_take(&state, &cmd);
  assert(command_control_submit(&state, move(0, 10), &interrupt) == COMMAND_SUBMIT_DUPLICATE);
  assert(!interrupt);
  command_control_finish(&state, COMMAND_DRIVER_ERROR);
  assert(state.failed == 1 && command_control_result(&state) == COMMAND_DRIVER_ERROR);
  assert(!command_control_busy(&state));
  // A separate standard command can intentionally try again after failure.
  accept(&state, move(0, 10), false);
}

int main(void) {
  unaligned_wire_request_decodes_without_word_access();
  pending_blocks_sleep_and_latest_wins();
  duplicate_never_restarts_motor();
  new_id_same_target_does_not_preempt();
  stop_followed_by_new_target_preserves_new_target();
  stop_taken_before_new_target_cannot_overwrite_it();
  unseen_late_command_and_evicted_duplicates_are_rejected();
  sequence_wrap_is_safe();
  invalid_requests_do_not_cancel_accepted_work();
  standard_commands_and_failures_are_observable();
  puts("9 command-control scenarios passed");
  return 0;
}
