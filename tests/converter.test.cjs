const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

// Only the host's expose builders are stubbed; execute the actual converter.
const builder = new Proxy(function () {}, {
    get: () => builder,
    apply: () => builder,
});
const context = {
    module: {exports: {}},
    require: () => builder,
    console,
    setTimeout,
};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,
    '../zigbee2mqtt/working_converter.js'), 'utf8'), context);
const definition = context.module.exports;

for (const [key, min, max] of [
    ['tilt', 0, 100], ['motor_steps', 10, 1000],
    ['motor_irun_active', 0, 31], ['motor_speed', 1, 100],
]) {
    const converter = definition.toZigbee.find((entry) => entry.key.includes(key));
    test(`${key}: invalid values never send Zigbee writes`, async () => {
        let writes = 0;
        const entity = {write: async () => writes++, command: async () => writes++};
        for (const value of [NaN, Infinity, '', 'bad', '12oops', 12.5, null, true, [], min - 1, max + 1]) {
            await assert.rejects(converter.convertSet(entity, key, value, {}));
        }
        assert.equal(writes, 0);
    });
    test(`${key}: accepts bounds and numeric strings`, async () => {
        let writes = 0;
        const entity = {write: async () => writes++, command: async () => writes++};
        for (const value of [min, max, String(min)]) {
            await converter.convertSet(entity, key, value, {});
        }
        assert.equal(writes, 3);
    });
}

test('tilt maps Z2M open=100 to ZCL open=0', async () => {
    const converter = definition.toZigbee.find((entry) => entry.key.includes('tilt'));
    let payload;
    const entity = {command: async (cluster, command, value) => { payload = value; }};
    await converter.convertSet(entity, 'tilt', 100, {});
    assert.equal(payload.percentagetiltvalue, 0);
    await converter.convertSet(entity, 'tilt', 0, {});
    assert.equal(payload.percentagetiltvalue, 100);
});
