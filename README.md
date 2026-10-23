# ESP32-H2 Zigbee Blind Tilt Controller

Firmware for a battery-powered blind tilt controller built on ESP32-H2. The
device joins a Zigbee network as a Window Covering end device, drives a
bipolar-modified 28BYJ-48 through a TMC2208, reports battery state, and can
enter BLE OTA mode for firmware updates without opening the enclosure.

This is a hardware-specific project, not a generic ESP32-H2 example. The pinout
and defaults below match the controller this firmware was written for.

## Current State

- Zigbee Window Covering endpoint with open, close, stop, and tilt percentage.
- Sleepy Zigbee end-device profile enabled by default.
- TMC2208 UART configuration with RMT STEP pulse generation and GPIO fallback.
- Zigbee2MQTT external converter with motor settings and diagnostics.
- Correlated command results, bounded delivery retries and safe duplicate handling.
- BLE OTA updater page in `ota.html`.
- Factory reset through the boot button.

Build baseline: ESP-IDF 5.5.2, target ESP32-H2. The component lock file pins the tested
Espressif Zigbee and LED strip components.

## Hardware

| Function | Pin | Notes |
| --- | --- | --- |
| Motor DIR | GPIO10 | TMC2208 DIR |
| Motor STEP | GPIO11 | RMT-backed STEP output |
| Motor ENABLE | GPIO12 | TMC2208 enable, active low |
| Motor power enable | GPIO3 | External motor rail enable, active low |
| TMC2208 UART TX | GPIO13 | Direct to TMC2208 PDN_UART |
| TMC2208 UART RX | GPIO14 | Through 1 kOhm to TMC2208 PDN_UART |
| Battery ADC | GPIO1 | 47 kOhm from battery to ADC, 100 kOhm from ADC to GND |
| WS2812 LED | GPIO8 | Status LED |
| Factory reset button | GPIO9 | Hold for 5 seconds |

The TMC2208 PDN_UART line is single-wire. Use the resistor on ESP32 RX as shown:

```text
ESP32 TX ----------> TMC2208 PDN_UART
ESP32 RX -- 1 kOhm -> TMC2208 PDN_UART
```

The firmware assumes a 1S Li-ion battery range of 3.0 V to 4.2 V.

## Build And Flash

Install and activate ESP-IDF **5.5.2**, then build from the project directory for ESP32-H2:

```bash
idf.py set-target esp32h2
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Use the serial port name for your machine. On macOS it is often something like
`/dev/cu.usbmodemXXXX`.

The project uses ESP-IDF component manager dependencies from
`main/idf_component.yml`. Keep `dependencies.lock` in version control if you
want other builds to use the same tested component versions.

## Zigbee Behavior

By default the device runs as a sleepy end device:

- `ZB_RESPONSIVE_MODE=0`
- `ED_KEEP_ALIVE=1000` ms
- `ZB_RX_ON_WHEN_IDLE=0`
- light sleep enabled

After movement, the firmware keeps a short awake window so follow-up commands
and reports have a better chance to complete quickly. Battery is measured and
reported every 60 minutes.

Pending and active motor work blocks sleep. The Zigbee2MQTT converter verifies
each device's result, retries an unconfirmed delivery up to twice, and cancels
older retries on a new target or STOP. Update the firmware and converter together;
see [command reliability and update instructions](docs/command-reliability.md).

If you change sleepy-device behavior on a device that was already paired, do a
factory reset and re-pair before judging the result. Coordinators can keep stale
interview or power-source state from the previous firmware.

## Zigbee2MQTT

The custom Zigbee2MQTT converter lives in:

```text
zigbee2mqtt/working_converter.js
```

Install it as an external converter, restart Zigbee2MQTT, then pair the device.
See `zigbee2mqtt/Z2M_INSTALLATION.md` for the full setup.

The converter exposes:

- cover tilt control
- motor steps
- motor direction
- active current (`IRUN`)
- motor power mode, `stealthchop` or `spreadcycle`
- motor speed
- calibration trigger
- BLE OTA trigger
- reset, TMC2208, and RMT diagnostics
- battery voltage and percentage
- target tilt, command result/error, position validity and command counters

The firmware uses ZCL 0% for open and 100% for closed; the converter reverses
this for the Zigbee2MQTT tilt slider (100% open).

Basic Window Covering commands should also be usable from coordinators that do
not support the custom converter, but the extra configuration and diagnostics
are Zigbee2MQTT-specific.

## Calibration

Position is estimated from commanded steps; there is no position sensor or
end-stop detection. Missed steps or power loss during movement can invalidate it.
An interrupted movement with uncertain position blocks absolute moves until
calibration succeeds; `position_valid` and `command_status` expose that condition.
The `calibrate` action assumes a starting position of 100% in firmware, moves a
full range toward 0%, then to 100%, then to 50%. It does **not** measure travel or
learn `motor_steps`. Confirm that your mechanism tolerates this before running it.

Start with the defaults, then tune on hardware:

1. Pair the device.
2. Check motor direction with open and close commands.
3. Adjust `motor_steps` if the blind does not reach its limits or overshoots.
4. Run `calibrate` from Zigbee2MQTT.
5. Tune `motor_speed`, `motor_irun_active`, and power mode only after movement
   direction and step count are correct.

`motor_steps` is stored as a raw value. The firmware currently multiplies it by
8 internally, so the default `100` raw units means 800 STEP pulses for the full
range.

## BLE OTA

BLE OTA is entered from Zigbee2MQTT by setting `ota_mode` to `ON`. The device
stores the OTA flag, reboots, advertises as `Blinds-XXXX`, and waits for the
browser uploader.

Open the [online BLE OTA updater](https://widapro.github.io/zigbee-blinds/)
in a browser with Web Bluetooth (such as desktop Chrome or Edge). GitHub Pages
serves the updater over HTTPS. Select **`build/zigbee_blinds.bin`** and connect
to your device. The firmware file stays in your browser and is sent directly
over Bluetooth; it is not uploaded to GitHub.

Alternatively, serve the updater locally:

```sh
python3 -m http.server 8000 --bind localhost
```

Open [the updater](http://localhost:8000/ota.html) in a browser with Web Bluetooth
(such as desktop Chrome or Edge). Select **`build/zigbee_blinds.bin`**,
connect to `Blinds-XXXX`, and keep the page open until the image is accepted.
Check that the device rejoins Zigbee after reboot. Use HTTPS when hosting the page
remotely. Browser/OS Bluetooth support varies.

The updater detects the firmware's checked transfer mode. New firmware accepts
groups of 16, 32 or 64 BLE packets without a response for every packet, then
reports how many bytes it has written to flash. The updater defaults to 32;
compare 64 with buffered firmware, or select 16 if a larger block fails. Earlier
fast firmware is limited to 32 packets by the updater. Current firmware collects
up to 64 packets in RAM and writes the block before returning its committed byte
count, so packet callbacks no longer write individually to flash. Its six-byte
status starts with version 2; the updater also supports version 1 from earlier
fast firmware. Use updater v2.4 or newer with buffered firmware. The device
checks the image length and CRC32 before accepting it. For 64-packet blocks,
use updater v2.6 or newer: it waits for a data write response after the first 32
packets, then sends the remaining packets and reads the committed flash offset.
The final burst uses this status read without an extra data write response.
This limits queued writes without reducing the flash block to 32 packets.
Older firmware remains compatible with the updater
and uses acknowledged writes for every packet; the first update to firmware
with checked transfer mode therefore runs at the earlier speed.
Firmware with the current GATT layout advertises as `Blinds-XXXX-F2` and uses a
different BLE address from earlier versions. This makes the browser discover the
readable control characteristic instead of reusing cached write-only properties.
After installing this firmware, select the device ending in `-F2` for subsequent
updates. The updater displays the transfer mode and any reason for legacy mode.
In fast mode it also shows the block size, write/acknowledgement timings, status
reads per block and time spent waiting between repeated status reads.

Each OTA slot is 1.5 MiB (`0x180000` bytes). Do not upload a bootloader, partition
table, merged flash image or an image for another board. Interrupted uploads
restart from the beginning. Three minutes without activity returns to Zigbee.

**OTA security:** the BLE service does not require pairing or authentication;
anyone nearby can attempt an update while OTA mode is active. Image validation
is not publisher authentication, and this project does not configure signed
updates or automatic rollback. Enable OTA only in a trusted environment.

## Factory Reset

Hold the boot button for 5 seconds. The firmware clears app NVS settings,
clears Zigbee network credentials, saves reset diagnostics, and restarts.

Use this when moving to a new coordinator or after changing sleepy-device
behavior.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for build and test commands.
GitHub Actions builds the firmware from defaults and runs JavaScript regression
tests. Hardware behavior still needs verification on the actual controller.

| Module | Responsibility |
| --- | --- |
| `main/main.c` | Boot mode, saved configuration, reset button |
| `main/zb_network.c` | Zigbee lifecycle, attributes, motor command worker |
| `main/zb_attributes.h` | Custom attribute IDs shared with the converter |
| `main/motor.c` | Movement, speed profile, RMT pulses, power control |
| `main/tmc2208_uart.c` | UART register protocol and CRC verification |
| `main/storage.c` | NVS settings and restart diagnostics |
| `main/battery.c` | ADC sampling and estimated battery charge |
| `main/ble_ota.c` | BLE service and application image update |
| `main/diagnostics.c`, `main/led.c` | Crash stage tracking and status LED |

## Repository Notes

- `main/` contains the firmware used by the current build.
- `zigbee2mqtt/` contains the external converter and setup notes.
- `ota.html` is the browser-based BLE OTA uploader, available online or locally.
- `.github/workflows/pages.yml` publishes only the updater to GitHub Pages.
- `sdkconfig.defaults` contains portable project defaults.
- `partitions.csv` defines the 4 MB flash layout with two OTA app partitions.

Generated files (`build*/`, `sdkconfig`, `managed_components/`) are ignored.
Keep `sdkconfig.defaults` and `dependencies.lock` committed. The repository does
not include PCB files, enclosure models or a complete wiring schematic; the pin
mapping describes the existing firmware configuration.

For publication, create an empty GitHub repository, initialize Git locally,
review `git status --short` and the staged diff, then commit and push. Do not
upload the whole working directory as an archive: it contains local build data.

## License

GPL-3.0. See `LICENSE`.
