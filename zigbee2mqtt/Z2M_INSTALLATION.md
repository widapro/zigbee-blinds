# Zigbee2MQTT Custom Device Template Installation

This guide explains how to install the custom device template for your ESP32H2 window covering controller.

## Installation

1. Copy `working_converter.js` to `external_converters/` next to your
   Zigbee2MQTT `configuration.yaml` (for Docker, use the persistent data volume).
2. Enable external JavaScript if disabled by your Zigbee2MQTT version:

   ```yaml
   advanced:
     enable_external_js: true
   ```

   Merge this into the existing `advanced` section; do not create a duplicate.
3. Restart Zigbee2MQTT and check its logs for converter loading errors.
4. Enable permit join, power on the controller and wait for the interview.
   An already paired controller can be reset by holding BOOT for five seconds.

The frontend also supports Settings > Dev console > External converters.
See the [official external converter documentation](https://www.zigbee2mqtt.io/advanced/more/external_converters.html)
and [external JavaScript setting](https://www.zigbee2mqtt.io/guide/configuration/all-settings.html#enable_external_js).
Do not edit installed `node_modules`. This project retains its existing CommonJS
converter; compatibility must be checked against your installed Zigbee2MQTT
version. The automated tests use mocks, not a running coordinator.

## Device Features

### Controls Available in Z2M/Home Assistant:

**Tilt Control:**
- Open/Close buttons
- Tilt slider (0-100%)
- Current tilt display

**Motor Configuration:**
- **Motor Steps**: Adjustable from 10-1000 steps
- Default: 100 raw units. Firmware multiplies by 8 internally (800 effective steps)
  - Adjust based on your motor and driver microstepping
- **Motor Direction**: Toggle between "normal" and "reversed"
  - Use if your motor moves in wrong direction
- **Motor IRUN (Active current)**: Adjustable from 0-31
  - Controls TMC driver active current setting
- **Motor Power Mode**: Toggle between "stealthchop" and "spreadcycle"
- **Motor Speed**: Adjustable from 1-100

**Battery Monitoring:**
- Battery voltage (V, 0.01 resolution via custom attribute)
- Battery percentage (%)
- Low battery alerts

### Home Assistant Integration:

The device will appear as:
- **Cover entity**: `cover.esp32h2_windowcovering`
- **Battery sensor**: `sensor.esp32h2_windowcovering_battery`
- **Configuration controls**: Available in device settings

**Device Class**: Automatically set to "blind" for proper Home Assistant integration

## Configuration Steps

### Initial Setup:

1. **Pair the device** with Zigbee2MQTT
2. **Verify detection**: Device should appear as "ESP32H2-WindowCovering" by widapro
3. **Test basic operation**: Try open/close commands
4. **Configure motor settings**:
   - Set motor steps for your hardware setup
   - Test direction and reverse if needed

### Motor Steps Calibration:

1. **Start with default**: 100 raw units (800 effective steps)
2. **Test full cycle**: Open -> Close -> Open
3. **Adjust if needed**:
   - Too many steps = overshooting
   - Too few steps = not reaching limits
4. **Common values**:
   - Start around 100 raw units with the current firmware scaling
   - Increase if the blind does not reach the limits
   - Decrease if the blind overshoots the limits

### Direction Configuration:

1. **Test open command**: Should move to "open" position
2. **If wrong direction**: Change motor_direction to "reversed"
3. **Verify**: Both open/close should work correctly

## Troubleshooting

### Device Not Detected:
- Ensure external converter is properly configured
- Check Zigbee2MQTT logs for errors
- Verify device model matches: "esp32h2"

### Motor Configuration Not Working:
- Check device interview completed successfully
- Verify Z2M can read/write window covering cluster
- Try re-pairing the device

### Home Assistant Issues:
- Restart Home Assistant after Z2M restart
- Check entity registry for duplicates
- Verify Z2M integration is working

### Battery Not Updating:
- Battery reports every 60 minutes by default
- Check power configuration cluster in Z2M
- Manually refresh in Z2M exposes tab

## Advanced Configuration

### Reporting Intervals:
The template binds the Window Covering and Power Configuration clusters and
reads the current attributes during interview. Runtime reporting is handled by
the firmware:
- Tilt position: reported after movement/calibration
- Motor config and reset diagnostics: reported after join/reboot
- Battery: reported every 60 minutes

### Custom Modifications:
To modify the template:
1. Adjust value ranges in `exposes` section
2. Add matching custom attributes only after firmware support exists
3. Keep `configure` reads in batches of three attributes or fewer

## Device Attributes Reference

| Attribute | Cluster | ID | Type | Description |
|-----------|---------|----|----- |-------------|
| Tilt Position | Window Covering | 0x0009 | U8 | Current tilt percentage |
| Motor Steps | Window Covering | 0xF004 | U16 | Total motor steps |
| Motor Direction | Window Covering | 0x0017 | 8BITMAP | Direction (1=reversed, 2=normal) |
| Calibration Trigger | Window Covering | 0xF010 | U8 | Write 1 to start calibration |
| Motor IRUN Active | Window Covering | 0xF012 | U8 | Active current setting (0-31) |
| OTA Mode Trigger | Window Covering | 0xF013 | U8 | Write 1 to reboot into BLE OTA mode |
| Motor Power Mode | Window Covering | 0xF014 | U8 | 0=StealthChop, 1=SpreadCycle |
| Motor Speed | Window Covering | 0xF015 | U8 | Speed setting (1-100) |
| Reset Diagnostics | Window Covering | 0xF020-0xF023 | U8/U32 | Reset reason, restart cause, boot count, crash stage |
| TMC/RMT Diagnostics | Window Covering | 0xF030-0xF035 | U8/U32 | Driver and step-pulse diagnostics |
| Battery Voltage | Power Config | 0x0020 | U8 | Voltage in 100mV units |
| Battery % | Power Config | 0x0021 | U8 | Percentage in 0.5% units |
| Battery Voltage Precise | Power Config | 0xF100 | U16 | Voltage in 0.01V units |
