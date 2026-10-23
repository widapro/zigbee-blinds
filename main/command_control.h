#pragma once

#include <stdbool.h>
#include <stdint.h>

// Packed request: nonzero 24-bit ID in bits 8..31, target in bits 0..7.
// Targets 0..100 are ZCL tilt percentages; 101 is STOP. ID 0 is reserved
// for standard ZCL commands. Results use the same ID and a status byte.
#define COMMAND_PROTOCOL_VERSION 1
#define COMMAND_STOP_TARGET 101
#define COMMAND_HISTORY_LEN 16

// Decode a ZCL U32 even when the stack supplies an unaligned value pointer.
uint32_t command_control_unpack_request(const void *value);

typedef enum {
  COMMAND_IDLE = 0,
  COMMAND_ACCEPTED = 1,
  COMMAND_MOVING = 2,
  COMMAND_COMPLETED = 3,
  COMMAND_STOPPED = 4,
  COMMAND_SUPERSEDED = 5,
  COMMAND_DRIVER_ERROR = 6,
  COMMAND_PULSE_ERROR = 7,
  COMMAND_TIMEOUT = 8,
  COMMAND_INVALID = 9,
  COMMAND_STORAGE_ERROR = 10,
  COMMAND_POSITION_UNKNOWN = 11,
} command_status_t;

typedef enum {
  MOTOR_CMD_MOVE = 0,
  MOTOR_CMD_STOP,
  MOTOR_CMD_CALIBRATE,
} motor_cmd_type_t;

typedef struct {
  motor_cmd_type_t type;
  uint8_t endpoint;
  uint8_t target_position;
  uint32_t id;
  uint32_t generation;
} motor_cmd_t;

typedef struct {
  motor_cmd_t command;
  command_status_t status;
} command_record_t;

// Callers serialize all access. Pending work is a mailbox: the newest target
// wins, while STOP immediately requests cancellation of the active motion.
typedef struct {
  bool pending;
  bool active;
  motor_cmd_t pending_command;
  motor_cmd_t active_command;
  command_record_t latest;
  command_record_t history[COMMAND_HISTORY_LEN];
  uint8_t history_next;
  uint32_t generation;
  uint32_t reliable_cursor;
  uint32_t received;
  uint32_t completed;
  uint32_t failed;
  uint32_t duplicates;
} command_control_t;

typedef enum {
  COMMAND_SUBMIT_NEW,
  COMMAND_SUBMIT_DUPLICATE,
  COMMAND_SUBMIT_INVALID,
  COMMAND_SUBMIT_STALE,
} command_submit_t;

command_submit_t command_control_submit(command_control_t *state,
                                         motor_cmd_t command,
                                         bool *interrupt_active);
bool command_control_take(command_control_t *state, motor_cmd_t *command);
void command_control_finish(command_control_t *state,
                            command_status_t status);
bool command_control_busy(const command_control_t *state);
uint32_t command_control_result(const command_control_t *state);
