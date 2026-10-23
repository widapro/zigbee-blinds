const {test} = require('node:test');
const assert = require('node:assert/strict');
const {A, loadConverter, makeBlind, coverConverter} = require('./converter_harness.cjs');

function setup(options) {
    const {definition, now} = loadConverter({fastClock: true, ...options});
    const updates = [];
    return {definition, now, updates, converter: coverConverter(definition),
        meta: {publish: (value) => updates.push(value), options: {}}};
}

test('confirmed tilt is never published optimistically', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({status: 2, onRead: (requested, attrs, reads) => {
        if (reads.length === 4) {
            attrs[A.result] = ((attrs[A.result] & 0xFFFFFF00) | 3) >>> 0;
            attrs.currentPositionTiltPercentage = 20;
        }
    }});
    const returned = await converter.convertSet(entity, 'tilt', 80, meta);
    assert.equal(returned, undefined);
    assert.equal(entity.writes.length, 1);
    assert.ok(updates.slice(0, -1).every((value) => value.tilt === undefined));
    assert.equal(updates.at(-1).tilt, 80);
});

test('lost write response is recovered by reading the result, without resend', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind({onWrite: (values, attrs) => {
        attrs[A.result] = ((values[A.request].value & 0xFFFFFF00) | 3) >>> 0;
        attrs.currentPositionTiltPercentage = values[A.request].value & 0xFF;
        throw new Error('lost write ACK');
    }});
    await converter.convertSet(entity, 'tilt', 75, meta);
    assert.equal(entity.writes.length, 1);
});

test('a lost command is retried with the identical ID and target', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind({onWrite: (values, attrs, writes) => {
        if (writes.length === 1) throw new Error('radio packet lost');
        attrs[A.result] = ((values[A.request].value & 0xFFFFFF00) | 3) >>> 0;
        attrs.currentPositionTiltPercentage = values[A.request].value & 0xFF;
    }});
    await converter.convertSet(entity, 'tilt', 75, meta);
    assert.equal(entity.writes.length, 2);
    assert.equal(entity.writes[0].values[A.request].value, entity.writes[1].values[A.request].value);
});

test('lost completion and position read responses do not replay the movement', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind({onRead: (requested, attrs, reads) => {
        if (reads.length === 2 || reads.length === 4) throw new Error('lost read response');
    }});
    await converter.convertSet(entity, 'tilt', 25, meta);
    assert.equal(entity.writes.length, 1);
});

test('delivery retries stop after three attempts and expose an error', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({onWrite: () => {throw new Error('offline');}});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /3 attempts/);
    assert.equal(entity.writes.length, 3);
    assert.equal(updates.at(-1).command_status, 'delivery_error');
    assert.ok(updates.every((value) => value.tilt === undefined));
});

test('accepted long motion is polled to its deadline without resending', async () => {
    const {converter, meta, updates, now} = setup();
    meta.options.command_timeout = 15;
    const entity = makeBlind({status: 2});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /deadline/);
    assert.equal(entity.writes.length, 1);
    assert.ok(now() >= 15000 && now() <= 18000);
    assert.ok(updates.every((value) => value.tilt === undefined));
});

test('motor failure is explicit and is never automatically replayed', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({status: 6});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /driver_error/);
    assert.equal(entity.writes.length, 1);
    assert.ok(updates.some((value) => value.command_status === 'driver_error'));
    assert.match(updates.at(-1).command_error, /driver_error/);
});

test('driver failure refreshes UART diagnostics and describes the failing register', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({status: 6});
    Object.assign(entity.attrs, {0xF030: 7, 0xF031: 7, 0xF032: 7,
        0xF036: 2, 0xF037: 0xFFFFFFFF});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta),
        /driver_error.*current_write.*uart_register=0x2.*uart_error=0xffffffff/);
    assert.equal(entity.writes.length, 1);
    assert.ok(entity.reads.every((read) => read.requested.length <= 3));
    assert.ok(updates.some((value) => value.tmc_last_error === 7 && value.tmc_uart_register === 2));
});

test('a lost diagnostic read is retried individually without replaying motion', async () => {
    const {converter, meta, updates} = setup();
    let errorsRead = 0;
    const entity = makeBlind({status: 6, onRead: (requested) => {
        if (requested.includes(0xF032) && ++errorsRead === 1) throw new Error('lost diagnostic read');
    }});
    Object.assign(entity.attrs, {0xF032: 3, 0xF036: 0, 0xF037: 0xFFFFFFFF,
        0xF038: 3, 0xF039: 2});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta),
        /driver_error.*startup_attempts=3.*configuration.*uart_register=0x0/);
    assert.equal(errorsRead, 2);
    assert.equal(entity.writes.length, 1);
    assert.ok(updates.some((value) => value.tmc_last_error === 3 && value.motor_startup_attempts === 3));
});

test('successful pre-motion recovery is exposed with confirmed completion', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind();
    Object.assign(entity.attrs, {0xF038: 2, 0xF039: 4});
    await converter.convertSet(entity, 'tilt', 25, meta);
    assert.equal(entity.writes.length, 1);
    assert.equal(updates.at(-1).tilt, 25);
    assert.equal(updates.at(-1).motor_startup_attempts, 2);
    assert.equal(updates.at(-1).motor_startup_recoveries, 4);
});

test('reboot during startup diagnostic reads cannot publish stale confirmed position', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({onRead: (requested, attrs) => {
        if (requested.includes(0xF038)) ++attrs[A.boot];
    }});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /restarted/);
    assert.ok(updates.every((value) => value.tilt === undefined));
    assert.equal(entity.writes.length, 1);
});

test('missing diagnostic responses clear cached data without masking driver failure', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({status: 6, onRead: (requested) => {
        if (requested.includes(0xF030) || requested.includes(0xF036)) throw new Error('radio timeout');
    }});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta),
        /driver_error.*diagnostics unavailable/);
    assert.equal(entity.writes.length, 1);
    assert.ok(updates.some((value) => value.tmc_last_error === null && value.tmc_uart_error === null));
});

test('a new goal cancels failure-diagnostic reads and old error publications', async () => {
    const {converter, meta, updates} = setup();
    let unblock;
    let reading;
    const started = new Promise((resolve) => {reading = resolve;});
    const entity = makeBlind({status: 6, onRead: async (requested) => {
        if (requested.includes(0xF030)) {
            reading();
            await new Promise((resolve) => {unblock = resolve;});
        }
    }});
    const old = converter.convertSet(entity, 'tilt', 25, meta);
    await started;
    const stop = converter.convertSet(entity, 'state', 'STOP', meta);
    unblock();
    await Promise.all([old, stop]);
    assert.ok(!updates.some((value) => value.command_error?.includes('driver_error')));
    assert.equal(entity.writes.length, 2);
});

test('device reboot invalidates confirmation without another movement', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({onRead: (requested, attrs, reads) => {
        if (reads.length === 2) ++attrs[A.boot];
    }});
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /restarted/);
    assert.equal(entity.writes.length, 1);
    assert.ok(updates.some((value) => value.command_status === 'restarted'));
    assert.ok(updates.some((value) => value.reboot_count === 8));
    assert.match(updates.at(-1).command_error, /reboot_count 7 → 8/);
    assert.ok(updates.every((value) => value.tilt === undefined));
});

test('new STOP cancels an older retry even when its read is still in flight', async () => {
    const {converter, meta, updates} = setup();
    let unblock;
    let snapshotStarted;
    const started = new Promise((resolve) => {snapshotStarted = resolve;});
    const entity = makeBlind({onWrite: (values, attrs) => {
        if ((values[A.request].value & 0xFF) === 101) {
            attrs[A.result] = ((values[A.request].value & 0xFFFFFF00) | 4) >>> 0;
        }
    }, onRead: async (requested, attrs, reads) => {
        if (reads.length === 2) {
            snapshotStarted();
            await new Promise((resolve) => {unblock = resolve;});
        }
    }});
    const old = converter.convertSet(entity, 'tilt', 25, meta);
    await started;
    const stop = converter.convertSet(entity, 'state', 'STOP', meta);
    unblock();
    await Promise.all([old, stop]);
    assert.deepEqual(entity.writes.map((write) => write.values[A.request].value & 0xFF), [75, 101]);
    assert.ok(!updates.some((value) => value.tilt === 25));
});

test('concurrent requests for the same goal share one command', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind();
    await Promise.all([converter.convertSet(entity, 'tilt', 75, meta),
        converter.convertSet(entity, 'tilt', 75, meta)]);
    assert.equal(entity.writes.length, 1);
});

test('retrying one room member does not replay commands to the others', async () => {
    const {converter, meta} = setup();
    const healthy = [makeBlind(), makeBlind()];
    const missed = makeBlind({onWrite: (values, attrs, writes) => {
        if (writes.length === 1) throw new Error('first packet lost');
        attrs[A.result] = ((values[A.request].value & 0xFFFFFF00) | 3) >>> 0;
        attrs.currentPositionTiltPercentage = values[A.request].value & 0xFF;
    }});
    await Promise.all([...healthy, missed].map((entity) =>
        converter.convertSet(entity, 'tilt', 100, meta)));
    assert.deepEqual([...healthy, missed].map((entity) => entity.writes.length), [1, 1, 2]);
});

test('STOP remains available when the position estimate is unknown', async () => {
    const {converter, meta, updates} = setup();
    const entity = makeBlind({position: 255});
    entity.attrs[A.valid] = 0;
    await converter.convertSet(entity, 'state', 'STOP', meta);
    assert.equal(entity.writes.length, 1);
    assert.equal(updates.at(-1).position_valid, false);
    assert.ok(updates.every((value) => value.tilt === undefined));
});

test('allocated IDs move forward across the 24-bit wrap without using zero', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind();
    await converter.convertSet(entity, 'tilt', 25, meta);
    entity.attrs[A.cursor] = 0xFFFFFF;
    entity.attrs[A.result] = 0xFFFFFF03;
    await converter.convertSet(entity, 'tilt', 75, meta);
    const id = entity.writes.at(-1).values[A.request].value >>> 8;
    assert.ok(id > 0 && id < 0x800000);
    const distance = (id - 0xFFFFFF) & 0xFFFFFF;
    assert.ok(distance > 0 && distance < 0x800000);
});

test('stale completed state with another ID cannot confirm a new goal', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind({onWrite() {}});
    entity.attrs[A.result] = 0x123403;
    entity.attrs[A.cursor] = 0x1234;
    entity.attrs.currentPositionTiltPercentage = 75;
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /3 attempts/);
    assert.equal(entity.writes.length, 3);
});

test('old firmware fails clearly before transmitting a movement', async () => {
    const {converter, meta} = setup();
    const entity = makeBlind();
    delete entity.attrs[A.protocol];
    await assert.rejects(converter.convertSet(entity, 'tilt', 25, meta), /Update the blind firmware/);
    assert.equal(entity.writes.length, 0);
});

test('unknown ZCL position is ignored instead of publishing negative tilt', () => {
    const {definition} = setup();
    const converter = definition.fromZigbee.find((entry) => entry.cluster === 'closuresWindowCovering');
    const converted = converter.convert({}, {data: {currentPositionTiltPercentage: 255}}, null, {}, {});
    assert.equal(converted.tilt, undefined);
});

test('binding failure propagates so Zigbee2MQTT can retry configuration', async () => {
    const {definition} = setup({bind: async () => {throw new Error('binding failed');}});
    await assert.rejects(definition.configure({getEndpoint: () => makeBlind()}, {}, {}), /binding failed/);
});
