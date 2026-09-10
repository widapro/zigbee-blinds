const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const script = fs.readFileSync(path.join(__dirname, '../ota.html'), 'utf8')
    .match(/<script>([\s\S]*?)<\/script>/)[1];

function uploader(bytes, failChunk = false) {
    const elements = new Map();
    function element(id) {
        if (!elements.has(id)) elements.set(id, {
            style: {}, files: [], disabled: false,
            addEventListener(event, fn) { this[event] = fn; },
        });
        return elements.get(id);
    }
    const writes = [];
    const control = {writeValueWithResponse: async (value) => writes.push(['control', ...value])};
    const data = {writeValueWithResponse: async (value) => {
        if (failChunk) throw new Error('Flash write failed');
        writes.push(['data', ...value]);
    }};
    const service = {getCharacteristic: async (uuid) => uuid.includes('0002-') ? control : data};
    const gatt = {
        connected: false,
        async connect() { this.connected = true; return this; },
        async getPrimaryService() { return service; },
        disconnect() { this.connected = false; },
    };
    const device = {gatt, addEventListener() {}};
    vm.runInNewContext(script, {
        document: {getElementById: element}, window: {isSecureContext: true},
        navigator: {bluetooth: {requestDevice: async () => device}},
        Uint8Array, performance, console: {error() {}},
    });
    element('fileInput').files = [{arrayBuffer: async () => Uint8Array.from(bytes).buffer}];
    return {elements, element, writes, gatt};
}

test('OTA sends start, exact image chunks, end and releases UI', async () => {
    const bytes = [0xE9, ...Array.from({length: 499}, (_, i) => i % 256)];
    const u = uploader(bytes);
    await u.element('connectBtn').click();
    assert.deepEqual(u.writes[0], ['control', 0]);
    assert.deepEqual(u.writes.at(-1), ['control', 1]);
    assert.deepEqual(u.writes.filter((w) => w[0] === 'data').flatMap((w) => w.slice(1)), bytes);
    assert.equal(u.element('fileInput').disabled, false);
    assert.equal(u.gatt.connected, false);
    assert.match(u.element('status').innerText, /Image accepted/);
});

test('OTA rejects empty and non-image files before start', async () => {
    for (const bytes of [[], [0, 1, 2]]) {
        const u = uploader(bytes);
        await u.element('connectBtn').click();
        assert.equal(u.writes.length, 0);
        assert.match(u.element('status').innerText, /Error/);
    }
});

test('OTA never retries ambiguous writes or finalizes a failed upload', async () => {
    const u = uploader([0xE9, 1, 2], true);
    await u.element('connectBtn').click();
    assert.deepEqual(u.writes, [['control', 0]]);
    assert.match(u.element('status').innerText, /Flash write failed/);
    assert.equal(u.element('connectBtn').disabled, false);
});
