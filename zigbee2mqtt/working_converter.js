const m = require('zigbee-herdsman-converters/lib/modernExtend');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ea = exposes.access;

const resetReasonLookup = {
    0: 'unknown',
    1: 'poweron',
    2: 'external',
    3: 'software',
    4: 'panic',
    5: 'interrupt_watchdog',
    6: 'task_watchdog',
    7: 'watchdog',
    8: 'deep_sleep',
    9: 'brownout',
    10: 'sdio',
};

const restartCauseLookup = {
    0: 'none',
    1: 'factory_reset',
    2: 'ota_mode_trigger',
    3: 'ota_timeout',
    4: 'ota_success',
    5: 'zigbee_offline_stall',
    6: 'zigbee_inactive',
};

const crashStageLookup = {
    0: 'unknown',
    1: 'app_start',
    2: 'led_init',
    3: 'storage_init',
    4: 'boot_diagnostics',
    5: 'ota_mode_check',
    6: 'ble_ota_start',
    7: 'load_config',
    8: 'battery_init',
    9: 'motor_init',
    10: 'zigbee_network_init',
    11: 'start_tasks',
    20: 'zigbee_task_start',
    21: 'zigbee_cluster_setup',
    22: 'zigbee_device_register',
    23: 'zigbee_start',
    24: 'zigbee_running',
    30: 'motor_worker',
    31: 'motor_move',
    32: 'calibration',
    40: 'ble_ota_write',
};

const rmtStatusLookup = {
    0: 'unknown',
    1: 'ready',
    2: 'fallback_init',
    3: 'fallback_tx',
};

const rmtFailStageLookup = {
    0: 'none',
    1: 'new_tx_channel',
    2: 'new_copy_encoder',
    3: 'enable',
    4: 'transmit',
    5: 'wait_done',
};

// Reject malformed MQTT values before they reach a motor command.
function integerInRange(value, min, max, name) {
    const number = typeof value === 'number' ? value :
        typeof value === 'string' && value.trim() !== '' ? Number(value) : NaN;
    if (!Number.isInteger(number) || number < min || number > max) {
        throw new Error(`${name} must be an integer between ${min} and ${max}`);
    }
    return number;
}

const definition = {
    zigbeeModel: ['esp32h2'],
    model: 'ESP32H2-WindowCovering',
    vendor: 'widapro',
    description: 'ESP32H2 Zigbee window covering controller',
    extend: [
        m.battery(),
    ],
    exposes: [
        // Cover entity with tilt control and open/close/stop buttons
        exposes.cover().withTilt(),

        // Battery voltage as separate entity
        exposes.numeric('battery_voltage', ea.STATE)
            .withUnit('V')
            .withValueStep(0.01)
            .withDescription('Battery voltage in volts')
            .withCategory('diagnostic'),

        // Motor configuration
        exposes.numeric('motor_steps', ea.ALL)
            .withValueMin(10)
            .withValueMax(1000)
            .withValueStep(10)
            .withUnit('steps')
            .withDescription('Total motor steps for full open/close cycle')
            .withCategory('config'),
        exposes.enum('motor_direction', ea.ALL, ['normal', 'reversed'])
            .withDescription('Motor rotation direction (normal or reversed)')
            .withCategory('config'),
        exposes.numeric('motor_irun_active', ea.ALL)
            .withValueMin(0)
            .withValueMax(31)
            .withValueStep(1)
            .withDescription('Motor active current (TMC IRUN, 0-31)')
            .withCategory('config'),
        exposes.enum('motor_power_mode', ea.ALL, ['stealthchop', 'spreadcycle'])
            .withDescription('Motor driver mode: stealthchop (quiet) or spreadcycle (high torque)')
            .withCategory('config'),
        exposes.numeric('motor_speed', ea.ALL)
            .withValueMin(1)
            .withValueMax(100)
            .withValueStep(1)
            .withDescription('Motor speed (1=~0.5x default, 50=default, 100=fastest)')
            .withCategory('config'),
        exposes.binary('calibrate', ea.ALL, 'ON', 'OFF')
            .withDescription('Start motor calibration sequence (open -> close -> 50%)')
            .withCategory('config'),
        exposes.binary('ota_mode', ea.ALL, 'ON', 'OFF')
            .withDescription('Reboot device into BLE OTA firmware update mode')
            .withCategory('config'),
        exposes.enum('last_reset_reason', ea.STATE_GET, Object.values(resetReasonLookup))
            .withDescription('ESP reset reason from the last boot')
            .withCategory('diagnostic'),
        exposes.enum('restart_cause', ea.STATE_GET, Object.values(restartCauseLookup))
            .withDescription('Firmware reason saved before the last planned restart')
            .withCategory('diagnostic'),
        exposes.numeric('reboot_count', ea.STATE_GET)
            .withDescription('Total boot count stored in device NVS')
            .withCategory('diagnostic'),
        exposes.enum('last_crash_stage', ea.STATE_GET, Object.values(crashStageLookup))
            .withDescription('Firmware stage recorded before the last panic/WDT/brownout reset')
            .withCategory('diagnostic'),
        exposes.numeric('tmc_mres_requested', ea.STATE_GET)
            .withDescription('Requested TMC2208 MRES value (0=256 microsteps, 8=full-step)')
            .withCategory('diagnostic'),
        exposes.numeric('tmc_mres_actual', ea.STATE_GET)
            .withDescription('Last verified TMC2208 MRES value (0=256 microsteps, 8=full-step, 255=unknown)')
            .withCategory('diagnostic'),
        exposes.numeric('tmc_last_error', ea.STATE_GET)
            .withDescription('Last TMC2208 error code (0=OK, 1=UART init, 2=GSTAT, 3=config, 4=MRES write, 5=MRES read, 6=MRES mismatch)')
            .withCategory('diagnostic'),
        exposes.enum('rmt_status', ea.STATE_GET, Object.values(rmtStatusLookup))
            .withDescription('STEP pulse backend status')
            .withCategory('diagnostic'),
        exposes.enum('rmt_fail_stage', ea.STATE_GET, Object.values(rmtFailStageLookup))
            .withDescription('Last STEP RMT failure stage')
            .withCategory('diagnostic'),
        exposes.numeric('rmt_last_error', ea.STATE_GET)
            .withDescription('Last STEP RMT esp_err_t value')
            .withCategory('diagnostic'),
    ],
    fromZigbee: [
        {
            // Custom attributes and manual tilt/position mirroring for HA
            cluster: 'closuresWindowCovering',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};

                // Map tilt attribute so HA tilt slider updates correctly
                if (msg.data.hasOwnProperty('currentPositionTiltPercentage')) {
                    const zclValue = msg.data['currentPositionTiltPercentage'];
                    const inv = 100 - zclValue; // Z2M inverts tilt vs ZCL
                    result.tilt = inv;
                    result.state = inv === 0 ? 'CLOSE' : 'OPEN';
                }
                if (msg.data.hasOwnProperty(9)) { // 0x0009
                    const zclValue = msg.data[9];
                    const inv = 100 - zclValue;
                    result.tilt = inv;
                    result.state = inv === 0 ? 'CLOSE' : 'OPEN';
                }

                // Handle motor_steps by ID and name (custom attribute 0xF004)
                if (msg.data.hasOwnProperty(61444)) { // 0xF004 = 61444
                    result.motor_steps = msg.data[61444];
                }

                // Handle motor_direction by ID and name
                if (msg.data.hasOwnProperty(23)) { // 0x0017 = mode
                    const mode = msg.data[23];
                    result.motor_direction = mode === 1 ? 'reversed' : 'normal';
                }
                if (msg.data.hasOwnProperty('mode')) {
                    result.motor_direction = msg.data['mode'] === 1 ? 'reversed' : 'normal';
                }
                if (msg.data.hasOwnProperty('windowCoveringMode')) {
                    result.motor_direction = msg.data['windowCoveringMode'] === 1 ? 'reversed' : 'normal';
                }

                // Handle calibration button state (attribute 0xF010)
                if (msg.data.hasOwnProperty(61456)) { // 0xF010 = 61456
                    result.calibrate = msg.data[61456] === 1 ? 'ON' : 'OFF';
                }

                // Handle OTA mode state (attribute 0xF013) - always show as OFF
                if (msg.data.hasOwnProperty(61459)) { // 0xF013 = 61459
                    result.ota_mode = msg.data[61459] === 1 ? 'ON' : 'OFF';
                }

                // Handle motor IRUN active (attribute 0xF012)
                if (msg.data.hasOwnProperty(61458)) { // 0xF012 = 61458
                    result.motor_irun_active = msg.data[61458];
                }

                // Handle motor power mode (attribute 0xF014 = 61460)
                if (msg.data.hasOwnProperty(61460)) {
                    result.motor_power_mode = msg.data[61460] === 1 ? 'spreadcycle' : 'stealthchop';
                }

                // Handle motor speed (attribute 0xF015 = 61461)
                if (msg.data.hasOwnProperty(61461)) {
                    result.motor_speed = msg.data[61461];
                }

                // Handle TMC2208 diagnostics
                if (msg.data.hasOwnProperty(61488)) { // 0xF030
                    result.tmc_mres_requested = msg.data[61488];
                }
                if (msg.data.hasOwnProperty(61489)) { // 0xF031
                    result.tmc_mres_actual = msg.data[61489];
                }
                if (msg.data.hasOwnProperty(61490)) { // 0xF032
                    result.tmc_last_error = msg.data[61490];
                }
                if (msg.data.hasOwnProperty(61491)) { // 0xF033
                    const status = msg.data[61491];
                    result.rmt_status = rmtStatusLookup[status] || `unknown_${status}`;
                }
                if (msg.data.hasOwnProperty(61492)) { // 0xF034
                    const stage = msg.data[61492];
                    result.rmt_fail_stage = rmtFailStageLookup[stage] || `unknown_${stage}`;
                }
                if (msg.data.hasOwnProperty(61493)) { // 0xF035
                    result.rmt_last_error = msg.data[61493];
                }

                // Handle reset/restart diagnostics
                if (msg.data.hasOwnProperty(61472)) { // 0xF020
                    const reason = msg.data[61472];
                    result.last_reset_reason = resetReasonLookup[reason] || `unknown_${reason}`;
                }
                if (msg.data.hasOwnProperty(61473)) { // 0xF021
                    const cause = msg.data[61473];
                    result.restart_cause = restartCauseLookup[cause] || `unknown_${cause}`;
                }
                if (msg.data.hasOwnProperty(61474)) { // 0xF022
                    result.reboot_count = msg.data[61474];
                }
                if (msg.data.hasOwnProperty(61475)) { // 0xF023
                    const stage = msg.data[61475];
                    result.last_crash_stage = crashStageLookup[stage] || 'unknown';
                }

                return result;
            },
        },
        {
            cluster: 'genPowerCfg',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};

                // Handle battery attributes by ID
                if (msg.data.hasOwnProperty(61696)) { // 0xF100 = precise battery voltage (centivolts)
                    result.battery_voltage = msg.data[61696] / 100.0;
                } else if (msg.data.hasOwnProperty(32)) { // 0x0020 = batteryVoltage
                    result.battery_voltage = msg.data[32] / 10.0; // Convert from 100mV units to V
                }
                if (msg.data.hasOwnProperty(33)) { // 0x0021 = batteryPercentageRemaining
                    result.battery = msg.data[33] / 2; // Convert from 0.5% units to %
                }

                // Also handle by name (try all possible variations)
                if (msg.data.hasOwnProperty('batteryVoltage')) {
                    result.battery_voltage = msg.data['batteryVoltage'] / 10.0; // Convert from 100mV units
                }
                if (msg.data.hasOwnProperty('batteryPercentageRemaining')) {
                    // ESP32H2 reports in 0.5% units, convert to percentage
                    result.battery = msg.data['batteryPercentageRemaining'] / 2;
                }

                return result;
            },
        },
    ],
    toZigbee: [
        // Standard ZCL window covering commands (ZHA compatible)
        {
            key: ['state', 'tilt'],
            convertSet: async (entity, key, value, meta) => {
                if (key === 'state') {
                    const state = typeof value === 'string' ? value.toLowerCase() : '';
                    if (state === 'open') {
                        await entity.command('closuresWindowCovering', 'upOpen', {}, {});
                        return { state: { state: 'OPEN', tilt: 100 } };
                    } else if (state === 'close') {
                        await entity.command('closuresWindowCovering', 'downClose', {}, {});
                        return { state: { state: 'CLOSE', tilt: 0 } };
                    } else if (state === 'stop') {
                        await entity.command('closuresWindowCovering', 'stop', {}, {});
                        return { state: { state: 'STOP' } };
                    }
                }

                if (key === 'tilt') {
                    const tilt = integerInRange(value, 0, 100, 'Tilt');
                    const zclValue = 100 - tilt; // Z2M inverts tilt vs ZCL
                    await entity.command('closuresWindowCovering', 'goToTiltPercentage', { percentagetiltvalue: zclValue }, {});
                    return { state: { tilt: tilt, state: tilt === 0 ? 'CLOSE' : 'OPEN' } };
                }
            },
            convertGet: async (entity, key, meta) => {
                if (key === 'tilt' || key === 'state') {
                    await entity.read('closuresWindowCovering', ['currentPositionTiltPercentage']);
                }
            },
        },
        {
            key: ['motor_steps'],
            convertSet: async (entity, key, value, meta) => {
                const val = integerInRange(value, 10, 1000, 'motor_steps');
                await entity.write('closuresWindowCovering', { 0xF004: { value: val, type: 0x21 } }); // U16
                return { readAfterWriteTime: 200, state: { motor_steps: val } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF004]);
            },
        },
        {
            key: ['motor_direction'],
            convertSet: async (entity, key, value, meta) => {
                if (!['normal', 'reversed'].includes(value)) {
                    throw new Error('Motor direction must be "normal" or "reversed"');
                }
                const val = value === 'reversed' ? 1 : 2;
                await entity.write('closuresWindowCovering', { 0x0017: { value: val, type: 0x18 } }); // 8BITMAP
                return { readAfterWriteTime: 200, state: { motor_direction: value } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0x0017]);
            },
        },
        {
            key: ['battery', 'battery_voltage'],
            convertGet: async (entity, key, meta) => {
                if (key === 'battery') {
                    await entity.read('genPowerCfg', [0x0021]); // batteryPercentageRemaining
                } else if (key === 'battery_voltage') {
                    await entity.read('genPowerCfg', [0x0020, 0xF100]); // Standard + precise voltage
                }
            },
        },
        {
            key: ['calibrate'],
            convertSet: async (entity, key, value, meta) => {
                if (value === 'ON') {
                    // Trigger calibration by writing 1 to custom attribute 0xF010
                    await entity.write('closuresWindowCovering', { 0xF010: { value: 1, type: 0x20 } }); // U8
                    // Schedule a delayed read of the tilt position so Z2M picks
                    // up the updated value even if the firmware's autonomous
                    // report was missed (calibration takes ~20-25 seconds).
                    setTimeout(async () => {
                        try {
                            await entity.read('closuresWindowCovering', ['currentPositionTiltPercentage']);
                        } catch (e) {
                            console.log('Post-calibration tilt read failed (device may be sleeping):', e.message);
                        }
                    }, 30000);
                    // Immediately reset the switch back to OFF
                    return { state: { calibrate: 'OFF' } };
                }
                return { state: { calibrate: 'OFF' } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF010]);
            },
        },
        {
            key: ['motor_irun_active'],
            convertSet: async (entity, key, value, meta) => {
                const val = integerInRange(value, 0, 31, 'motor_irun_active');
                await entity.write('closuresWindowCovering', { 0xF012: { value: val, type: 0x20 } }); // U8
                return { readAfterWriteTime: 200, state: { motor_irun_active: val } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF012]);
            },
        },
        {
            key: ['motor_power_mode'],
            convertSet: async (entity, key, value, meta) => {
                if (!['stealthchop', 'spreadcycle'].includes(value)) {
                    throw new Error('Motor power mode must be "stealthchop" or "spreadcycle"');
                }
                const val = value === 'spreadcycle' ? 1 : 0;
                await entity.write('closuresWindowCovering', { 0xF014: { value: val, type: 0x20 } }); // U8
                return { readAfterWriteTime: 200, state: { motor_power_mode: value } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF014]);
            },
        },
        {
            key: ['motor_speed'],
            convertSet: async (entity, key, value, meta) => {
                const val = integerInRange(value, 1, 100, 'motor_speed');
                await entity.write('closuresWindowCovering', { 0xF015: { value: val, type: 0x20 } }); // U8
                return { readAfterWriteTime: 200, state: { motor_speed: val } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF015]);
            },
        },
        {
            key: ['ota_mode'],
            convertSet: async (entity, key, value, meta) => {
                if (value === 'ON') {
                    // Trigger OTA mode by writing 1 to custom attribute 0xF013
                    await entity.write('closuresWindowCovering', { 0xF013: { value: 1, type: 0x20 } }); // U8
                    // Device will reboot into BLE OTA mode
                    return { state: { ota_mode: 'OFF' } };
                }
                return { state: { ota_mode: 'OFF' } };
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('closuresWindowCovering', [0xF013]);
            },
        },
        {
            key: ['last_reset_reason', 'restart_cause', 'reboot_count', 'last_crash_stage', 'tmc_mres_requested', 'tmc_mres_actual', 'tmc_last_error', 'rmt_status', 'rmt_fail_stage', 'rmt_last_error'],
            convertGet: async (entity, key, meta) => {
                const attrByKey = {
                    last_reset_reason: 0xF020,
                    restart_cause: 0xF021,
                    reboot_count: 0xF022,
                    last_crash_stage: 0xF023,
                    tmc_mres_requested: 0xF030,
                    tmc_mres_actual: 0xF031,
                    tmc_last_error: 0xF032,
                    rmt_status: 0xF033,
                    rmt_fail_stage: 0xF034,
                    rmt_last_error: 0xF035,
                };
                await entity.read('closuresWindowCovering', [attrByKey[key]]);
            },
        },
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);

        // Bind clusters so the coordinator accepts autonomous reports from the sleepy end device
        try {
            await reporting.bind(endpoint, coordinatorEndpoint, ['closuresWindowCovering', 'genPowerCfg']);
            console.log('Successfully bound clusters for autonomous reporting');
        } catch (error) {
            console.log('Failed to bind clusters (device may be sleeping):', error.message);
        }

        // Read motor configuration in batches of 3 attributes max
        // ESP32 ZCL stack may truncate Read Attributes Response beyond 3 attrs
        try {
            await endpoint.read('closuresWindowCovering', [
                0xF012, // motor IRUN active
                0xF014, // motor power mode
                0xF015, // motor speed
            ]);
        } catch (error) {
            console.log('Failed to read motor config batch 1:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0xF004, // motor_steps
                0x0009, // currentPositionTiltPercentage
            ]);
        } catch (error) {
            console.log('Failed to read motor config batch 1b:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0x0017, // mode (motor_direction)
                0xF010, // calibrate
                0xF013, // ota_mode
            ]);
        } catch (error) {
            console.log('Failed to read motor config batch 2:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0xF020, // last_reset_reason
                0xF021, // restart_cause
                0xF022, // reboot_count
            ]);
        } catch (error) {
            console.log('Failed to read reset diagnostics:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0xF023, // last_crash_stage
            ]);
        } catch (error) {
            console.log('Failed to read crash stage:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0xF030, // tmc_mres_requested
                0xF031, // tmc_mres_actual
                0xF032, // tmc_last_error
            ]);
        } catch (error) {
            console.log('Failed to read TMC diagnostics:', error.message);
        }

        try {
            await endpoint.read('closuresWindowCovering', [
                0xF033, // rmt_status
                0xF034, // rmt_fail_stage
                0xF035, // rmt_last_error
            ]);
        } catch (error) {
            console.log('Failed to read RMT diagnostics:', error.message);
        }

        // Try to read battery configuration (non-critical)
        try {
            await endpoint.read('genPowerCfg', [
                0x0020, // batteryVoltage
                0x0021, // batteryPercentageRemaining
                0xF100, // precise battery voltage (centivolts)
            ]);
            console.log('Successfully read battery configuration from device');
        } catch (error) {
            console.log('Failed to read battery configuration (device may be sleeping):', error.message);
        }

        console.log('Configuration completed - device will report autonomously');
    },
};

module.exports = definition;
