# Zigbee2MQTT Integration

This directory contains files for integrating the ESP32H2 window covering device with Zigbee2MQTT.

## Files

- **`working_converter.js`** - The functional Z2M external converter
- **`Z2M_INSTALLATION.md`** - Complete installation and setup guide

## Quick Setup

1. Update the blind firmware to command protocol v1, then copy `working_converter.js` to your Z2M `data/external_converters/` directory
2. Restart Zigbee2MQTT
3. Pair your ESP32H2 device
4. Device should be recognized as "ESP32H2-WindowCovering"

For existing devices, run **Reconfigure** after installing the firmware/converter.
The converter now confirms each command result and reports an error if the
firmware lacks protocol v1. See [reliability behavior, diagnostics and testing](../docs/command-reliability.md).

## Features

- Position control (0-100%)
- Motor steps configuration (10-1000)
- Motor active current (IRUN) control (0-31)
- Motor direction toggle (normal/reversed)
- Battery monitoring (voltage with 0.01V precision and percentage)
- Tilt-only cover control (open/close/stop plus tilt slider)
- Home Assistant integration ready

See `Z2M_INSTALLATION.md` for detailed instructions.
