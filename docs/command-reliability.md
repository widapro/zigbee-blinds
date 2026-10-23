# Reliable blind commands

The firmware and `zigbee2mqtt/working_converter.js` must be updated together.
The converter requires command protocol v1 and reports a clear error before
movement if the firmware does not support it. Standard ZCL open, close, stop
and tilt commands remain available for other coordinators.

## Update and use

1. Update each blind using the application image, through the existing BLE OTA
   uploader or ESP-IDF flashing workflow. Do not upload bootloader/partition images
   to the BLE uploader.
2. Install the updated `working_converter.js` in Zigbee2MQTT and restart it.
3. Run **Reconfigure** for each device to verify cluster bindings and read the new
   attributes. Pairing again is normally unnecessary: the sleepy network profile,
   endpoint and manufacturer/model identifiers have not changed.
4. Test a small tilt change, STOP, and the first movement after a long idle period.
   Inspect `command_status`, `command_error` and `position_valid` on each device.

Home Assistant cover groups can remain in use. Verification and retries happen
per member device in the converter. A Zigbee group endpoint cannot provide this
individual confirmation and is rejected by the reliable command converter.

Keep the existing Home Assistant automation. `tilt_position: 100` maps to ZCL 0
(software open); check whether that matches the desired physical direction.
Do not add blind repeated sends to the HA automation on top of converter retries.

## Behavior

- A target is sent with an ID and queried until the device confirms acceptance.
  There are at most three writes with the same ID: the first send, then retries
  after approximately 5 and 10 seconds, plus transaction time. Each radio request
  has a 5-second timeout and uses immediate send policy.
- A lost write response is followed by a result read. Once accepted/moving is
  observed, the converter polls every 3 seconds and does not resend motion.
  Lost result/position responses are read again without another motor movement.
- A new target or STOP cancels the old converter job and its queued retries.
  Actual radio transactions are serialized; an already-sent radio frame cannot
  be recalled. Firmware rejects older request IDs after accepting a newer one.
- The default completion deadline is 180 seconds, including initial reads. Set
  the per-device Zigbee2MQTT option `command_timeout` to 15..300 seconds if needed.
  Short timeouts can expire before a slow full-range movement finishes; they do
  not automatically stop a motion that was already accepted.
- Firmware remembers duplicate IDs, including completed commands. Repeated goals
  do not stop the motor. A newer different target requests cancellation, and a
  mailbox preserves the newest pending target even around STOP.
- `tilt` and `state` come from device reports/reads. `target_tilt` is the requested
  value; it is not an indication of completion. Delivery/motor errors are exposed
  through `command_status` and `command_error`.
- A power interruption during movement or ambiguous RMT completion marks the
  position unknown. The device refuses further absolute moves with
  `position_unknown` until the existing calibration procedure succeeds. Inspect
  the mechanism before calibration: the procedure assumes an end position and
  executes full travel; there is still no physical position sensor.
- Do not automatically replay a motor failure or a command across a detected
  device restart. Inspect the reported failure and position first.

Settings persistence, UART motor configuration and OTA preparation run in the
worker rather than blocking Zigbee callbacks. Pending/active work blocks explicit
sleep. A mutex serializes motor operations with GPIO hold/sleep restoration, and
a PM lock prevents automatic light sleep during the worker session.
The receive callback only records the command and wakes the worker; the worker
updates Zigbee attributes and sends reports after the callback has returned.
The converter polls command results by ID, so no result reports are sent during
motion. Only the final tilt is reported once for other coordinators; its value
and the result also remain readable if that report is lost. Parent-link failures
are no longer reset by sleep events. Recovery
timers are cancelled after a successful join, and repeated recovery attempts do
not reset the overall offline timeout. No polling/role/TX-power tuning was changed.

## Wire protocol

All fields are non-manufacturer-specific attributes in Window Covering, endpoint 1.
Requests and results are a single U32 each; they cannot split into separate ID and
target writes. A request contains a nonzero 24-bit ID in bits 8..31 and a target
in bits 0..7. Targets 0..100 use ZCL percentages; 101 means STOP. Result bits 8..31
contain the accepted ID and bits 0..7 contain the status. ID 0 identifies standard
commands. Results describe the newest accepted request, not an arbitrary historical
request. The cursor survives standard commands but resets on device reboot.

| Attribute | Type/access | Meaning |
| --- | --- | --- |
| `0xF040` | U8, read | Protocol version, 1 |
| `0xF041` | U32, write/read | Packed request |
| `0xF042` | U32, read/report | Packed result |
| `0xF043` | U8, read/report | Latest ZCL target or 101 for STOP |
| `0xF044` | U32, read/report | Received command count |
| `0xF045` | U32, read/report | Executed completed/stopped count |
| `0xF046` | U32, read/report | Failed/invalid command count |
| `0xF047` | U32, read/report | Duplicate/stale command count |
| `0xF048` | U8, read/report | Position valid, 0/1 |
| `0xF049` | U32, read/report | Last accepted reliable 24-bit ID |

Statuses: 0 idle, 1 accepted, 2 moving, 3 completed, 4 stopped, 5 superseded,
6 driver_error, 7 pulse_error, 8 timeout, 9 invalid, 10 storage_error,
11 position_unknown. Settings errors are logged separately.

IDs use modular forward ordering within half of the 24-bit range. The converter
reads the cursor, allocates forward IDs and leaves a small random gap for a new
converter session. Firmware retains 16 recent requests plus current work and
rejects older IDs even after history eviction. Use one command-owning converter
per device. Exact-once execution across device/coordinator resets is not claimed:
IDs/cursor are held in RAM, and in-flight messages can outlive a process. Boot
count checks prevent automatic retries across an observed device reboot.

Position and validity are stored atomically in the new `motion_state` NVS value;
older position settings are read on upgrade. An unknown position is represented
as ZCL 0xFF and is not converted into an invalid HA tilt value. Command counters
and history stay in RAM to avoid flash writes on every received packet.

## Validation

Run `node --test tests/*.test.cjs` and `node --check zigbee2mqtt/working_converter.js`.
The test suite compiles and executes the actual C command controller with `cc`,
and executes the converter with simulated dropped packets, reads, restarts,
timeouts, cancellation and duplicate goals. Firmware builds with ESP-IDF 5.5.2.
These checks do not replace on-device radio/sleep/motor testing.

`driver_error` means the device received the request but failed before emitting
STEP pulses. The converter now reads current TMC diagnostics on this failure;
cached MQTT values from configuration are not proof that the driver is healthy.
Read failures leave the diagnostic fields null and do not replay motor motion.
The primary TMC error code is read alone, with one bounded read retry, and other
failure diagnostics are read in pairs so one lost batch does not hide all values.
`tmc_last_error` codes 1..6 retain their old meaning (UART init, GSTAT, config,
MRES write/read/mismatch); 7 is current-register write/ack failure, 8 driver not
initialized, 9 missing motor mutex, 10 unexpected busy state, 11 UART check.
The read-only attributes `0xF036` (`tmc_uart_register`, U8) and `0xF037`
(`tmc_uart_error`, U32) record the last UART register at failed setup and the
ESP error value. Register 255 means none; error 4294967295 means `ESP_FAIL`.
With the startup recovery build, error 263 (`ESP_ERR_TIMEOUT`) means no complete
driver response, including request echo only; 264 (`ESP_ERR_INVALID_RESPONSE`)
means a received response failed register/CRC validation. An unacknowledged write
still returns `ESP_FAIL`. These distinguish communication failure modes without
proving a hardware power fault.

The UART receiver flushes old data only before the read request. Flushing after
TX-wait could discard a reply received while that task was descheduled. The
receiver now scans past request echo/noise, assembles fragmented frames and
requires the correct register/header/CRC, within the original bounded timeout.
The host test executes the actual UART implementation with early replies,
echo, corrupt frames, no response and lost write acknowledgements. Hardware
testing must still distinguish UART reception errors from unstable driver power.

Driver preparation now has at most three attempts within the same motor command,
before coils are enabled or STEP pulses are emitted. Failed preparation powers
the driver off and reinstalls UART, waits 250 ms before attempt two or 1000 ms
before attempt three, and adds 100 ms of rail settling on recovery attempts.
The worker retains its motor mutex and PM/sleep gate throughout. STOP/new targets
cancel preparation and the backoff; persistent errors finish as `driver_error`.
Preparation recovery is never invoked after motion begins, for RMT/pulse errors,
or by replaying a Zigbee command. Timing is a bounded recovery policy, not a
measured guarantee about this board's supply startup.

Read-only attributes `0xF038` (`motor_startup_attempts`, U8) and `0xF039`
(`motor_startup_recoveries`, U32) show the attempts in the latest move and total
power/UART recovery count since boot. A successful move with attempts=2 or 3
means firmware recovered a startup failure; 1 is normal startup, 0 no startup
(for example an already reached goal). The converter reads these fields on
completion and failure, so recovered faults are visible in ordinary MQTT logs.
The counters are diagnostics and are not persisted to flash.

If a command reports `restarted`, compare `reboot_count` before and after the
command and inspect `last_reset_reason` and `last_crash_stage`. A rising count
with `panic` is a real firmware reset, not a lost radio packet; obtain the serial
panic backtrace if it still happens. Do not automatically replay a command across
that reset.

Recent firmware also records finer crash stages in RTC memory. In the converter,
`zigbee_command_decode` means the request was being decoded;
`zigbee_command_enqueue` means it was entering the mailbox;
`motor_command_take` means the worker was claiming it; `motor_report` means the
worker entered Zigbee synchronization; `motor_state_sync` means it held the lock
and was copying the read-only attribute backing values; `motor_tilt_report`
means it was sending the sole final tilt report; `motor_execute` means the worker was starting
motion, including the pre-motion NVS write; `motor_move` is the motor pulse path;
and `motor_store` is the final NVS write. The stage narrows the search, but a
serial panic backtrace still identifies the faulting instruction.

Before rollout to all blinds, test one device on battery: first command after
overnight idle; simultaneous commands to all room members; STOP/new target during
motion; power interruption; parent-router restart. Measure battery current and
receipt/completion latency. `completed` confirms the calculated pulse sequence,
not the physical angle: that still requires a position sensor.
