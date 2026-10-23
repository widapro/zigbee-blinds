const {test} = require('node:test');
const assert = require('node:assert/strict');
const {A, loadConverter, makeBlind} = require('./converter_harness.cjs');
const {definition} = loadConverter();

for (const [key, min, max] of [
    ['tilt', 0, 100], ['motor_steps', 10, 1000],
    ['motor_irun_active', 0, 31], ['motor_speed', 1, 100],
]) {
    const converter = definition.toZigbee.find((entry) => entry.key.includes(key));
    test(`${key}: invalid values never send Zigbee writes`, async () => {
        let writes = 0;
        const entity = makeBlind({onWrite: () => writes++});
        for (const value of [NaN, Infinity, '', 'bad', '12oops', 12.5, null, true, [], min - 1, max + 1]) {
            await assert.rejects(converter.convertSet(entity, key, value, {}));
        }
        assert.equal(writes, 0);
    });
    test(`${key}: accepts bounds and numeric strings`, async () => {
        const entity = makeBlind();
        for (const value of [min, max, String(min)]) {
            await converter.convertSet(entity, key, value, {});
        }
        assert.equal(entity.writes.length, 3);
    });
}

test('tilt maps Z2M open=100 to ZCL open=0', async () => {
    const converter = definition.toZigbee.find((entry) => entry.key.includes('tilt'));
    const entity = makeBlind();
    await converter.convertSet(entity, 'tilt', 100, {});
    assert.equal(entity.writes[0].values[A.request].value & 0xFF, 0);
    await converter.convertSet(entity, 'tilt', 0, {});
    assert.equal(entity.writes[1].values[A.request].value & 0xFF, 100);
});
