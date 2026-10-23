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
    25: 'zigbee_command_decode',
    26: 'zigbee_command_enqueue',
    30: 'motor_worker',
    31: 'motor_move',
    32: 'calibration',
    33: 'motor_command_take',
    34: 'motor_report',
    35: 'motor_execute',
    36: 'motor_store',
    37: 'motor_state_sync',
    38: 'motor_tilt_report',
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

const commandAttributes = {
    protocol: 0xF040, request: 0xF041, result: 0xF042, target: 0xF043,
    received: 0xF044, completed: 0xF045, failed: 0xF046, duplicates: 0xF047,
    positionValid: 0xF048, cursor: 0xF049, boot: 0xF022,
};
const commandStatusLookup = {
    0: 'idle', 1: 'accepted', 2: 'moving', 3: 'completed', 4: 'stopped',
    5: 'superseded', 6: 'driver_error', 7: 'pulse_error', 8: 'timeout',
    9: 'invalid', 10: 'storage_error', 11: 'position_unknown',
};
const tmcErrorLookup = {
    0: 'ok', 1: 'uart_init', 2: 'gstat', 3: 'configuration', 4: 'mres_write',
    5: 'mres_read', 6: 'mres_mismatch', 7: 'current_write', 8: 'not_initialized',
    9: 'mutex_missing', 10: 'motor_busy', 11: 'uart_check',
};
const activeCommands = new WeakMap();
const endpointTransactions = new WeakMap();
const commandIds = new WeakMap();
const commandIoOptions = {timeout: 5000, disableRecovery: true, sendPolicy: 'immediate'};

class SupersededCommand extends Error {}
class DeviceCommandError extends Error {}

function requireCurrent(entity, job) {
    if (activeCommands.get(entity) !== job || job.cancelled) {
        throw new SupersededCommand('Command replaced by a newer goal');
    }
}

// Serializing the actual radio transactions prevents an old retry queued in
// herdsman from being sent after a newer target or STOP from this converter.
async function commandTransaction(entity, job, operation) {
    const previous = endpointTransactions.get(entity) || Promise.resolve();
    const current = previous.catch(() => {}).then(async () => {
        requireCurrent(entity, job);
        if (Date.now() >= job.deadline) throw new Error('Command deadline exceeded');
        const result = await operation();
        requireCurrent(entity, job);
        if (Date.now() >= job.deadline) throw new Error('Command deadline exceeded');
        return result;
    });
    endpointTransactions.set(entity, current);
    return current;
}

async function commandDelay(entity, job, milliseconds) {
    requireCurrent(entity, job);
    await new Promise((resolve) => {
        const timer = setTimeout(() => {job.wake = undefined; resolve();}, milliseconds);
        job.wake = () => {clearTimeout(timer); job.wake = undefined; resolve();};
    });
    requireCurrent(entity, job);
}

function commandPublish(meta, payload) {
    if (typeof meta.publish === 'function') meta.publish(payload);
}

function uint32Attribute(data, id) {
    const value = data[id];
    if (!Number.isInteger(value) || value < 0 || value > 0xFFFFFFFF) {
        throw new Error(`Missing or invalid command attribute 0x${id.toString(16)}`);
    }
    return value;
}

function confirmedTilt(data) {
    const value = data.currentPositionTiltPercentage ?? data[9];
    if (!Number.isInteger(value) || value < 0 || value > 100) {
        throw new Error('Device position is unknown or invalid');
    }
    return 100 - value;
}

async function readDriverFailure(entity, job, meta) {
    // MQTT retains old diagnostics. Clear unknown fields and explicitly read
    // current values instead of interpreting a cached 0 as a healthy driver.
    const diagnostics = {tmc_mres_requested: null, tmc_mres_actual: null,
        tmc_last_error: null, tmc_uart_register: null, tmc_uart_error: null,
        motor_startup_attempts: null, motor_startup_recoveries: null};
    const batches = [
        {tmc_last_error: 0xF032},
        {tmc_uart_register: 0xF036, tmc_uart_error: 0xF037},
        {tmc_mres_requested: 0xF030, tmc_mres_actual: 0xF031},
        {motor_startup_attempts: 0xF038, motor_startup_recoveries: 0xF039},
    ];
    for (const batch of batches) {
        Object.assign(diagnostics, await readOptionalDiagnostics(entity, job, batch,
            batch.tmc_last_error !== undefined ? 2 : 1));
    }
    commandPublish(meta, diagnostics);
    const details = [];
    if (diagnostics.motor_startup_attempts !== null) {
        details.push(`startup_attempts=${diagnostics.motor_startup_attempts}`);
    }
    if (diagnostics.tmc_last_error !== null) {
        details.push(`tmc_last_error=${diagnostics.tmc_last_error} ` +
            `(${tmcErrorLookup[diagnostics.tmc_last_error] || 'unknown'})`);
    }
    if (diagnostics.tmc_uart_register !== null) {
        details.push(`uart_register=0x${diagnostics.tmc_uart_register.toString(16)}`);
    }
    if (diagnostics.tmc_uart_error !== null) {
        details.push(`uart_error=0x${diagnostics.tmc_uart_error.toString(16)}`);
    }
    return details.length ? ` (${details.join(', ')})` : ' (driver diagnostics unavailable)';
}

async function readOptionalDiagnostics(entity, job, attributes, attempts = 1) {
    const diagnostics = Object.fromEntries(Object.keys(attributes).map((name) => [name, null]));
    for (let attempt = 0; attempt < attempts; ++attempt) {
        try {
            const data = await commandTransaction(entity, job, () =>
                entity.read('closuresWindowCovering', Object.values(attributes), commandIoOptions));
            for (const [name, id] of Object.entries(attributes)) {
                if (Number.isInteger(data[id]) && data[id] >= 0 && data[id] <= 0xFFFFFFFF) {
                    diagnostics[name] = data[id];
                }
            }
            if (Object.values(diagnostics).some((value) => value !== null)) break;
        } catch (error) {
            if (error instanceof SupersededCommand) throw error;
            // Optional diagnostics must not mask the actual command failure.
        }
        if (attempt + 1 < attempts) await commandDelay(entity, job, 250);
    }
    return diagnostics;
}

async function executeReliableCommand(entity, job, meta) {
    if (typeof entity.read !== 'function' || typeof entity.write !== 'function') {
        throw new Error('Reliable blind commands require individual devices, not a Zigbee group');
    }
    commandPublish(meta, {command_status: 'sending', command_error: '',
        ...(job.target <= 100 ? {target_tilt: 100 - job.target} : {})});
    let initial;
    for (let attempt = 0; attempt < 3; ++attempt) {
        try {
            initial = await commandTransaction(entity, job, () => entity.read('closuresWindowCovering',
                [commandAttributes.protocol, commandAttributes.boot, commandAttributes.cursor], commandIoOptions));
            break;
        } catch (error) {
            if (error instanceof SupersededCommand || attempt === 2) throw error;
            await commandDelay(entity, job, attempt === 0 ? 2000 : 5000);
        }
    }
    if (initial[commandAttributes.protocol] !== 1) {
        throw new Error('Update the blind firmware: reliable command protocol v1 is required');
    }
    const boot = uint32Attribute(initial, commandAttributes.boot);
    const cursor = uint32Attribute(initial, commandAttributes.cursor);
    if (cursor > 0xFFFFFF) throw new Error('Invalid command cursor');
    const previousId = commandIds.get(entity);
    const distance = previousId === undefined ? 0 : (previousId - cursor) & 0xFFFFFF;
    const base = distance > 0 && distance < 0x800000 ? previousId : cursor;
    // A fresh converter session leaves a gap ahead of the device's cursor,
    // reducing reuse of an ID from a lost in-flight packet before restart.
    const increment = previousId === undefined ? 1 + Math.floor(Math.random() * 65535) : 1;
    job.id = (base + increment) & 0xFFFFFF;
    if (!job.id) job.id = 1;
    commandIds.set(entity, job.id);
    const request = (job.id * 256 + job.target) >>> 0;
    let sends = 0;
    let accepted = false;
    let nextSendAt = 0;
    let lastTransportError;
    commandPublish(meta, {command_id: job.id});

    while (Date.now() < job.deadline) {
        requireCurrent(entity, job);
        if (!accepted && sends < 3 && Date.now() >= nextSendAt) {
            ++sends;
            try {
                await commandTransaction(entity, job, () => entity.write('closuresWindowCovering',
                    {[commandAttributes.request]: {value: request, type: 0x23}}, commandIoOptions));
            } catch (error) {
                if (error instanceof SupersededCommand) throw error;
                // The write response can be lost after the device accepted it.
                // Read the correlated result before considering a retransmission.
                lastTransportError = error;
            }
            nextSendAt = Date.now() + (sends === 1 ? 5000 : 10000);
        }
        let snapshot;
        try {
            snapshot = await commandTransaction(entity, job, () => entity.read('closuresWindowCovering',
                [commandAttributes.result, commandAttributes.boot, commandAttributes.positionValid], commandIoOptions));
        } catch (error) {
            if (error instanceof SupersededCommand) throw error;
            lastTransportError = error;
        }
        if (snapshot) {
            const observedBoot = uint32Attribute(snapshot, commandAttributes.boot);
            if (observedBoot !== boot) {
                commandPublish(meta, {command_status: 'restarted', reboot_count: observedBoot});
                throw new DeviceCommandError(`Blind restarted during the command (reboot_count ${boot} → ${observedBoot}); ` +
                    'result is uncertain, inspect position_valid');
            }
            const result = uint32Attribute(snapshot, commandAttributes.result);
            const id = result >>> 8;
            const status = result & 0xFF;
            if (id === job.id) {
                accepted = true;
                commandPublish(meta, {command_status: commandStatusLookup[status] || 'invalid', command_id: id});
                if (status === 3 || (job.target === 101 && status === 4)) {
                    const valid = uint32Attribute(snapshot, commandAttributes.positionValid) === 1;
                    if (!valid && job.target !== 101) throw new Error('Blind position is not valid');
                    const startup = await readOptionalDiagnostics(entity, job,
                        {motor_startup_attempts: 0xF038, motor_startup_recoveries: 0xF039});
                    if (valid) {
                        let position;
                        try {
                            position = await commandTransaction(entity, job, () => entity.read('closuresWindowCovering',
                                ['currentPositionTiltPercentage', commandAttributes.result, commandAttributes.boot], commandIoOptions));
                        } catch (error) {
                            if (error instanceof SupersededCommand) throw error;
                            lastTransportError = error;
                            await commandDelay(entity, job, 1000);
                            continue;
                        }
                        const confirmedBoot = uint32Attribute(position, commandAttributes.boot);
                        if (confirmedBoot !== boot) {
                            commandPublish(meta, {command_status: 'restarted', reboot_count: confirmedBoot});
                            throw new DeviceCommandError(`Blind restarted while confirming position (reboot_count ` +
                                `${boot} → ${confirmedBoot})`);
                        }
                        if (uint32Attribute(position, commandAttributes.result) !== result) {
                            throw new SupersededCommand('Device accepted another goal while confirming position');
                        }
                        const tilt = confirmedTilt(position);
                        if (job.target <= 100 && tilt !== 100 - job.target) {
                            throw new Error('Completed command does not match the confirmed position');
                        }
                        commandPublish(meta, {...startup, tilt, state: tilt === 0 ? 'CLOSE' : 'OPEN', position_valid: true});
                    } else {
                        commandPublish(meta, {...startup, position_valid: false});
                    }
                    return;
                }
                if (status !== 1 && status !== 2) {
                    const details = status === 6 ? await readDriverFailure(entity, job, meta) : '';
                    throw new DeviceCommandError(`Blind command failed: ${commandStatusLookup[status] || status}${details}`);
                }
            } else if (accepted || (id !== 0 && id !== cursor)) {
                throw new SupersededCommand('Device accepted a different command');
            }
        }
        if (!accepted && sends === 3 && Date.now() >= nextSendAt) {
            throw new Error(`No command acceptance after 3 attempts${lastTransportError ? `: ${lastTransportError.message}` : ''}`);
        }
        await commandDelay(entity, job, accepted ? 3000 : 1000);
    }
    throw new Error('No confirmed completion before command deadline');
}

async function setReliableCommand(entity, target, meta) {
    const previous = activeCommands.get(entity);
    if (previous && previous.target === target && !previous.cancelled) {
        return previous.promise;
    }
    if (previous) {
        previous.cancelled = true;
        if (previous.wake) previous.wake();
    }
    const timeout = integerInRange(meta.options?.command_timeout ?? 180, 15, 300, 'command_timeout');
    const job = {target, deadline: Date.now() + timeout * 1000, cancelled: false};
    activeCommands.set(entity, job);
    job.promise = executeReliableCommand(entity, job, meta).catch((error) => {
        if (error instanceof SupersededCommand) return;
        if (activeCommands.get(entity) === job) commandPublish(meta, {
            ...(error instanceof DeviceCommandError ? {} : {command_status: 'delivery_error'}),
            command_error: error.message,
        });
        throw error;
    }).finally(() => {
        if (activeCommands.get(entity) === job) activeCommands.delete(entity);
    });
    return job.promise;
}

const definition = {
    zigbeeModel: ['esp32h2'],
    model: 'ESP32H2-WindowCovering',
    vendor: 'widapro',
    description: 'ESP32H2 Zigbee window covering controller',
    extend: [
        m.battery(),
    ],
    options: [
        exposes.numeric('command_timeout', ea.SET)
            .withValueMin(15).withValueMax(300).withUnit('s')
            .withDescription('Deadline for confirmed motion (default 180 seconds)'),
    ],
    exposes: [
        // Cover entity with tilt control and open/close/stop buttons
        exposes.cover().withTilt(),
        exposes.numeric('target_tilt', ea.STATE).withValueMin(0).withValueMax(100)
            .withDescription('Requested tilt; confirmed position is tilt'),
        exposes.enum('command_status', ea.STATE_GET, [...Object.values(commandStatusLookup),
            'sending', 'delivery_error', 'restarted']).withCategory('diagnostic'),
        exposes.numeric('command_id', ea.STATE_GET).withCategory('diagnostic'),
        exposes.text('command_error', ea.STATE).withCategory('diagnostic'),
        exposes.binary('position_valid', ea.STATE_GET, true, false).withCategory('diagnostic'),
        ...['commands_received', 'commands_completed', 'commands_failed', 'command_duplicates']
            .map((name) => exposes.numeric(name, ea.STATE_GET).withCategory('diagnostic')),

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
            .withDescription('TMC error (0=OK, 1=UART init, 2=GSTAT, 3=config, 4=MRES write, 5=MRES read, 6=MRES mismatch, 7=current write, 8=not initialized, 9=mutex missing, 10=busy, 11=UART check)')
            .withCategory('diagnostic'),
        exposes.numeric('tmc_uart_register', ea.STATE_GET)
            .withDescription('UART register at failed driver setup, decimal address (255=none)')
            .withCategory('diagnostic'),
        exposes.numeric('tmc_uart_error', ea.STATE_GET)
            .withDescription('ESP setup error (0=OK, 263=no reply, 264=invalid reply, 4294967295=ESP_FAIL)')
            .withCategory('diagnostic'),
        exposes.numeric('motor_startup_attempts', ea.STATE_GET)
            .withDescription('Driver startup attempts used by the latest move (0=no startup, 1=normal, 2..3=recovery)')
            .withCategory('diagnostic'),
        exposes.numeric('motor_startup_recoveries', ea.STATE_GET)
            .withDescription('Driver power/UART recovery count since boot, before any motion pulses')
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

                if (msg.data.currentPositionTiltPercentage !== undefined || msg.data[9] !== undefined) {
                    try {
                        const tilt = confirmedTilt(msg.data);
                        result.tilt = tilt;
                        result.state = tilt === 0 ? 'CLOSE' : 'OPEN';
                    } catch { /* Ignore ZCL unknown (0xFF) and malformed positions. */ }
                }
                if (msg.data[commandAttributes.result] !== undefined) {
                    const packed = msg.data[commandAttributes.result] >>> 0;
                    result.command_id = packed >>> 8;
                    result.command_status = commandStatusLookup[packed & 0xFF] || 'invalid';
                    if ((packed & 0xFF) === 3 || (packed & 0xFF) === 4) result.command_error = '';
                }
                if (Number.isInteger(msg.data[commandAttributes.target]) &&
                    msg.data[commandAttributes.target] >= 0 && msg.data[commandAttributes.target] <= 100) {
                    result.target_tilt = 100 - msg.data[commandAttributes.target];
                }
                for (const [name, id] of Object.entries({
                    commands_received: commandAttributes.received,
                    commands_completed: commandAttributes.completed,
                    commands_failed: commandAttributes.failed,
                    command_duplicates: commandAttributes.duplicates,
                })) {
                    if (msg.data[id] !== undefined) result[name] = msg.data[id];
                }
                if (msg.data[commandAttributes.positionValid] !== undefined) {
                    result.position_valid = msg.data[commandAttributes.positionValid] === 1;
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
                if (msg.data.hasOwnProperty(61494)) { // 0xF036
                    result.tmc_uart_register = msg.data[61494];
                }
                if (msg.data.hasOwnProperty(61495)) { // 0xF037
                    result.tmc_uart_error = msg.data[61495];
                }
                if (msg.data.hasOwnProperty(61496)) { // 0xF038
                    result.motor_startup_attempts = msg.data[61496];
                }
                if (msg.data.hasOwnProperty(61497)) { // 0xF039
                    result.motor_startup_recoveries = msg.data[61497];
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
        {
            key: ['command_status', 'command_id', 'position_valid', 'commands_received',
                'commands_completed', 'commands_failed', 'command_duplicates'],
            convertGet: async (entity, key) => {
                const attrs = {
                    command_status: commandAttributes.result, command_id: commandAttributes.result,
                    position_valid: commandAttributes.positionValid, commands_received: commandAttributes.received,
                    commands_completed: commandAttributes.completed, commands_failed: commandAttributes.failed,
                    command_duplicates: commandAttributes.duplicates,
                };
                await entity.read('closuresWindowCovering', [attrs[key]], commandIoOptions);
            },
        },
        // Standard ZCL window covering commands (ZHA compatible)
        {
            key: ['state', 'tilt'],
            convertSet: async (entity, key, value, meta) => {
                if (key === 'tilt') {
                    const tilt = integerInRange(value, 0, 100, 'Tilt');
                    await setReliableCommand(entity, 100 - tilt, meta);
                } else {
                    const state = typeof value === 'string' ? value.toLowerCase() : '';
                    const targets = {open: 0, close: 100, stop: 101};
                    if (!Object.hasOwn(targets, state)) throw new Error('State must be OPEN, CLOSE or STOP');
                    await setReliableCommand(entity, targets[state], meta);
                }
                // Actual state is published only from device reports/reads.
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
            key: ['last_reset_reason', 'restart_cause', 'reboot_count', 'last_crash_stage', 'tmc_mres_requested', 'tmc_mres_actual', 'tmc_last_error', 'tmc_uart_register', 'tmc_uart_error', 'motor_startup_attempts', 'motor_startup_recoveries', 'rmt_status', 'rmt_fail_stage', 'rmt_last_error'],
            convertGet: async (entity, key, meta) => {
                const attrByKey = {
                    last_reset_reason: 0xF020,
                    restart_cause: 0xF021,
                    reboot_count: 0xF022,
                    last_crash_stage: 0xF023,
                    tmc_mres_requested: 0xF030,
                    tmc_mres_actual: 0xF031,
                    tmc_last_error: 0xF032,
                    tmc_uart_register: 0xF036,
                    tmc_uart_error: 0xF037,
                    motor_startup_attempts: 0xF038,
                    motor_startup_recoveries: 0xF039,
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
        await reporting.bind(endpoint, coordinatorEndpoint, ['closuresWindowCovering', 'genPowerCfg']);
        await endpoint.read('closuresWindowCovering', [
            commandAttributes.protocol, commandAttributes.result, commandAttributes.positionValid,
        ]);
        try {
            await endpoint.read('closuresWindowCovering', [
                commandAttributes.received, commandAttributes.completed, commandAttributes.failed,
            ]);
            await endpoint.read('closuresWindowCovering', [commandAttributes.duplicates]);
        } catch (error) {
            console.log('Failed to read command counters:', error.message);
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
            await endpoint.read('closuresWindowCovering', [0xF036, 0xF037]);
        } catch (error) {
            console.log('Failed to read UART failure diagnostics:', error.message);
        }
        try {
            await endpoint.read('closuresWindowCovering', [0xF038, 0xF039]);
        } catch (error) {
            console.log('Failed to read motor startup diagnostics:', error.message);
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
