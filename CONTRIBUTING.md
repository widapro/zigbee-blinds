# Contributing

Build with ESP-IDF 5.5.2 and the committed `dependencies.lock`. Keep the existing
GPL-3.0 license. Describe the problem, the change, and how you checked it in each
pull request.

## Development checks

```sh
idf.py build
node --test tests/*.test.cjs
node --check zigbee2mqtt/working_converter.js
```

To verify portable defaults without changing local settings:

```sh
idf.py -B build-public -D SDKCONFIG=build-public/sdkconfig -D IDF_TARGET=esp32h2 build
```

Use `.editorconfig` and `.clang-format` (two spaces for C, four for JavaScript).
If clang-format is installed, run `clang-format -i main/*.c main/*.h`.
Keep comments focused on units, protocol conventions, ownership and hardware
constraints. Do not change tuning constants as part of formatting changes.

Motor movement runs in the motor worker; Zigbee API calls from other tasks need
the Zigbee lock. Avoid blocking callbacks with movement or long peripheral work.
Keep custom attribute IDs in `main/zb_attributes.h`, the converter and the
attribute reference in sync. Preserve NVS key names unless a migration is added.

Before a firmware release, verify pairing, full-range movement, STOP handling,
factory reset, power-cycle recovery, battery readings and interrupted BLE OTA on
the target hardware.
Unit tests mock external services; they do not establish radio or motor behavior.
The command-controller test compiles the actual C module with the host `cc`
compiler; keep a C compiler available when running `node --test tests/*.test.cjs`.
Do not commit `sdkconfig`, build directories, downloaded components, network
credentials or flash dumps. Publish only the application image as an OTA artifact.
