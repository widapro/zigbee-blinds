#include "command_control.h"

uint32_t command_control_unpack_request(const void *value) {
  const uint8_t *bytes = value;
  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static bool same_goal(motor_cmd_t a, motor_cmd_t b) {
  return a.type == MOTOR_CMD_MOVE && b.type == MOTOR_CMD_MOVE &&
         a.endpoint == b.endpoint && a.target_position == b.target_position;
}

static void remember(command_control_t *state, motor_cmd_t command,
                     command_status_t status) {
  if (!command.id) {
    return;
  }
  for (unsigned i = 0; i < COMMAND_HISTORY_LEN; ++i) {
    if (state->history[i].command.id == command.id) {
      state->history[i].status = status;
      return;
    }
  }
  state->history[state->history_next] =
      (command_record_t){.command = command, .status = status};
  state->history_next = (state->history_next + 1) % COMMAND_HISTORY_LEN;
}

command_submit_t command_control_submit(command_control_t *state,
                                         motor_cmd_t command,
                                         bool *interrupt_active) {
  ++state->received;
  *interrupt_active = false;
  if (command.id > 0xFFFFFF || command.type > MOTOR_CMD_CALIBRATE ||
      (command.type == MOTOR_CMD_MOVE && command.target_position > 100)) {
    ++state->failed;
    return COMMAND_SUBMIT_INVALID;
  }
  if (command.id) {
    // An active request must remain deduplicated even if a burst of newer
    // pending targets wraps the bounded history before the worker stops.
    motor_cmd_t live[] = {state->active_command, state->pending_command,
                         state->latest.command};
    for (unsigned i = 0; i < sizeof(live) / sizeof(live[0]); ++i) {
      if (live[i].id == command.id) {
        if (live[i].type != command.type ||
            live[i].target_position != command.target_position ||
            live[i].endpoint != command.endpoint) {
          ++state->failed;
          return COMMAND_SUBMIT_INVALID;
        }
        ++state->duplicates;
        return COMMAND_SUBMIT_DUPLICATE;
      }
    }
    for (unsigned i = 0; i < COMMAND_HISTORY_LEN; ++i) {
      motor_cmd_t old = state->history[i].command;
      if (old.id != command.id) {
        continue;
      }
      if (old.type != command.type || old.target_position != command.target_position ||
          old.endpoint != command.endpoint) {
        ++state->failed;
        return COMMAND_SUBMIT_INVALID;
      }
      ++state->duplicates;
      return COMMAND_SUBMIT_DUPLICATE;
    }
    uint32_t distance = (command.id - state->reliable_cursor) & 0xFFFFFF;
    if (state->reliable_cursor && (!distance || distance >= 0x800000)) {
      ++state->duplicates;
      return COMMAND_SUBMIT_STALE;
    }
  } else if ((state->pending && same_goal(state->pending_command, command)) ||
             (!state->pending && state->active &&
              same_goal(state->active_command, command))) {
    ++state->duplicates;
    return COMMAND_SUBMIT_DUPLICATE;
  }

  if (state->pending) {
    remember(state, state->pending_command, COMMAND_SUPERSEDED);
  }
  command.generation = ++state->generation;
  if (command.id) {
    state->reliable_cursor = command.id;
  }
  state->pending_command = command;
  state->pending = true;
  state->latest = (command_record_t){command, COMMAND_ACCEPTED};
  remember(state, command, COMMAND_ACCEPTED);
  *interrupt_active = state->active &&
                      !same_goal(state->active_command, command);
  return COMMAND_SUBMIT_NEW;
}

bool command_control_take(command_control_t *state, motor_cmd_t *command) {
  if (!state->pending || state->active) {
    return false;
  }
  *command = state->pending_command;
  state->pending = false;
  state->active = true;
  state->active_command = *command;
  state->latest.status = COMMAND_MOVING;
  remember(state, *command, COMMAND_MOVING);
  return true;
}

void command_control_finish(command_control_t *state,
                            command_status_t status) {
  if (!state->active) {
    return;
  }
  if (status == COMMAND_COMPLETED || status == COMMAND_STOPPED) {
    ++state->completed;
  } else if (status != COMMAND_SUPERSEDED) {
    ++state->failed;
  }
  remember(state, state->active_command, status);
  if (state->latest.command.generation == state->active_command.generation) {
    state->latest.status = status;
  }
  state->active = false;
}

bool command_control_busy(const command_control_t *state) {
  return state->pending || state->active;
}

uint32_t command_control_result(const command_control_t *state) {
  return (state->latest.command.id << 8) | state->latest.status;
}
