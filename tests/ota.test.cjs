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

function fastUploader(bytes, failAfterChunks = Infinity, delayStatus = false, options = {}) {
    const elements = new Map();
    function element(id) {
        if (!elements.has(id)) elements.set(id, {
            style: {}, files: [], disabled: false,
            addEventListener(event, fn) { this[event] = fn; },
        });
        return elements.get(id);
    }
    const writes = [];
    const reads = [];
    const dataChunksAtRead = [];
    const writeModes = [];
    let committed = 0;
    let chunks = 0;
    let lastReported = 0;
    let queuedCommands = 0;
    let maxQueuedCommands = 0;
    let responsePending = false;
    const control = {
        properties: {read: options.readable ?? true},
        writeValueWithResponse: async (value) => writes.push(['control', ...value]),
        readValue: async () => {
            if (responsePending) throw new Error('Read issued before flow response');
            if (options.readError) throw options.readError;
            queuedCommands = 0;
            const reported = delayStatus && lastReported !== committed
                ? lastReported : committed;
            lastReported = committed;
            reads.push(reported);
            dataChunksAtRead.push(chunks);
            const status = new DataView(new ArrayBuffer(6));
            status.setUint8(0, options.statusVersion ?? 1);
            status.setUint8(1, chunks >= failAfterChunks ? 1 : 0);
            status.setUint32(2, reported, true);
            return status;
        },
    };
    const writeData = async (value, withResponse) => {
        if (responsePending) throw new Error('Write issued before flow response');
        writes.push(['data', ...value]);
        writeModes.push(withResponse ? 'response' : 'command');
        chunks++;
        const queueFull = !withResponse && queuedCommands >= (options.noResponseCapacity ?? Infinity);
        if (withResponse) {
            if (options.responseError) throw options.responseError;
            if (options.pauseResponses) {
                responsePending = true;
                await new Promise(resolve => setImmediate(resolve));
                responsePending = false;
            }
            queuedCommands = 0;
        } else {
            queuedCommands++;
            maxQueuedCommands = Math.max(maxQueuedCommands, queuedCommands);
        }
        if (!queueFull && chunks < failAfterChunks && chunks !== options.dropChunk) committed += value.length;
    };
    const data = {
        properties: {writeWithoutResponse: true},
        writeValueWithoutResponse: (value) => writeData(value, false),
        writeValueWithResponse: (value) => writeData(value, true),
    };
    const service = {getCharacteristic: async (uuid) => uuid.includes('0002-') ? control : data};
    const gatt = {
        connected: false,
        async connect() { this.connected = true; return this; },
        async getPrimaryService() { return service; },
        disconnect() { this.connected = false; },
    };
    const device = {name: options.name ?? 'Blinds-1234-F2', gatt, addEventListener() {}};
    vm.runInNewContext(script, {
        document: {getElementById: element}, window: {isSecureContext: true},
        navigator: {bluetooth: {requestDevice: async () => device}},
        Uint8Array, DataView,
        performance: options.performance ?? performance,
        setTimeout: options.setTimeout ?? setTimeout,
        console: {error() {}},
    });
    element('fileInput').files = [{arrayBuffer: async () => Uint8Array.from(bytes).buffer}];
    if (options.blockPackets !== undefined) element('blockPackets').value = String(options.blockPackets);
    return {element, writes, reads, dataChunksAtRead, writeModes, gatt,
        get maxQueuedCommands() { return maxQueuedCommands; }};
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
    assert.match(u.element('transferMode').innerText, /legacy mode/);
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

test('fast OTA sends checked start and acknowledges committed blocks', async () => {
    const bytes = [0xE9, ...Array.from({length: 7999}, (_, i) => i % 256)];
    const u = fastUploader(bytes);
    await u.element('connectBtn').click();
    const start = u.writes[0].slice(1);
    assert.equal(start[0], 2);
    assert.equal(new DataView(Uint8Array.from(start).buffer).getUint32(1, true), bytes.length);
    assert.deepEqual(u.reads, [0, 7808, 8000]);
    assert.deepEqual(u.writes.filter(w => w[0] === 'data').flatMap(w => w.slice(1)), bytes);
    assert.deepEqual(u.writes.at(-1), ['control', 1]);
    assert.match(u.element('status').innerText, /Image accepted/);
    assert.match(u.element('transferMode').innerText, /v2\.6.*fast mode; block 32 packets/);
    assert.match(u.element('transferMode').innerText, /reads 1\.0\/block, poll wait 0 ms\/block/);
    assert.equal(u.element('blockPackets').disabled, false);
});

test('fast OTA honors selected 16 and 64 packet blocks without changing image data', async () => {
    const bytes = [0xE9, ...Array.from({length: 19999}, (_, i) => i % 256)];
    for (const [blockPackets, offsets] of [
        [16, [0, 3904, 7808, 11712, 15616, 19520, 20000]],
        [64, [0, 15616, 20000]],
    ]) {
        const u = fastUploader(bytes, Infinity, false, {blockPackets, statusVersion: 2});
        await u.element('connectBtn').click();
        assert.deepEqual(u.reads, offsets);
        const packets = u.writes.filter(w => w[0] === 'data');
        assert.ok(packets.every(w => w.length - 1 <= 244));
        assert.deepEqual(packets.flatMap(w => w.slice(1)), bytes);
        assert.deepEqual(u.writes.at(-1), ['control', 1]);
        assert.match(u.element('transferMode').innerText, new RegExp(`block ${blockPackets} packets`));
        assert.match(u.element('transferMode').innerText, /buffered flash/);
    }
});

test('unbuffered firmware uses 32 packets even when 64 was selected', async () => {
    const bytes = [0xE9, ...Array.from({length: 19999}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, false, {blockPackets: 64});
    await u.element('connectBtn').click();
    assert.deepEqual(u.reads, [0, 7808, 15616, 20000]);
    assert.match(u.element('transferMode').innerText, /block 32 packets.*64 requires newer firmware/);
    assert.match(u.element('status').innerText, /Image accepted/);
});

test('64-packet blocks wait for flow responses before exceeding the command queue', async () => {
    const bytes = [0xE9, ...Array.from({length: 19999}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, false, {
        blockPackets: 64, statusVersion: 2, noResponseCapacity: 32, pauseResponses: true,
    });
    await u.element('connectBtn').click();
    assert.equal(u.maxQueuedCommands, 32);
    assert.deepEqual(u.writeModes.flatMap((mode, index) => mode === 'response' ? [index] : []), [31]);
    assert.deepEqual(u.reads, [0, 15616, 20000]);
    assert.deepEqual(u.dataChunksAtRead, [0, 64, 82]);
    assert.deepEqual(u.writes.filter(w => w[0] === 'data').flatMap(w => w.slice(1)), bytes);
    assert.deepEqual(u.writes.at(-1), ['control', 1]);
    assert.match(u.element('transferMode').innerText, /v2\.6.*block 64 packets; buffered flash; flow 32 packets/);
    assert.match(u.element('transferMode').innerText, /flow ack .* ms\/burst/);
    assert.match(u.element('status').innerText, /Image accepted/);
});

test('final burst and short tail wait for committed flash without a redundant write response', async () => {
    const bytes = [0xE9, ...Array.from({length: 19999}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, true, {
        blockPackets: 64, statusVersion: 2, noResponseCapacity: 32,
    });
    await u.element('connectBtn').click();
    assert.deepEqual(u.reads, [0, 0, 15616, 15616, 20000]);
    assert.deepEqual(u.dataChunksAtRead, [0, 64, 64, 82, 82]);
    assert.equal(u.writeModes.filter(mode => mode === 'response').length, 1);
    assert.deepEqual(u.writes.at(-1), ['control', 1]);
    assert.match(u.element('status').innerText, /Image accepted/);
});

test('failed flow response stops the upload without retries or finalization', async () => {
    const bytes = [0xE9, ...Array.from({length: 15999}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, false, {
        blockPackets: 64, statusVersion: 2, responseError: new Error('Flow response failed'),
    });
    await u.element('connectBtn').click();
    assert.equal(u.writeModes.length, 32);
    assert.equal(u.writeModes.at(-1), 'response');
    assert.deepEqual(u.reads, [0]);
    assert.equal(u.writes.at(-1)[0], 'data');
    assert.match(u.element('status').innerText, /Flow response failed/);
    assert.equal(u.gatt.connected, false);
    assert.equal(u.element('blockPackets').disabled, false);
});

test('OTA rejects unsupported block sizes before starting the transfer', async () => {
    const u = fastUploader([0xE9, 1, 2, 3], Infinity, false, {blockPackets: 128});
    await u.element('connectBtn').click();
    assert.equal(u.writes.length, 0);
    assert.match(u.element('status').innerText, /Select a block size/);
    assert.equal(u.element('blockPackets').disabled, false);
});

test('fast OTA stops on a device flash error without finalizing', async () => {
    const u = fastUploader([0xE9, 1, 2, 3], 1, false, {statusVersion: 2});
    await u.element('connectBtn').click();
    assert.equal(u.writes[0][1], 2);
    assert.equal(new DataView(Uint8Array.from(u.writes[0].slice(1)).buffer).getUint32(5, true), 0x20CE88A1);
    assert.equal(u.writes.at(-1)[0], 'data');
    assert.match(u.element('status').innerText, /failed to write firmware/);
    assert.equal(u.gatt.connected, false);
    assert.equal(u.element('blockPackets').disabled, false);
});

test('missing packets time out with the committed offset and never finalize', async () => {
    let now = 0;
    const bytes = [0xE9, ...Array.from({length: 499}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, false, {
        statusVersion: 2, dropChunk: 1,
        performance: {now: () => now},
        setTimeout: (fn) => { now += 1000; fn(); },
    });
    await u.element('connectBtn').click();
    assert.match(u.element('status').innerText, /Timed out.*256\/500 bytes committed/);
    assert.equal(u.writes.at(-1)[0], 'data');
    assert.equal(u.gatt.connected, false);
    assert.equal(u.element('blockPackets').disabled, false);
});

test('fast OTA waits until all queued data has reached flash', async () => {
    const bytes = [0xE9, ...Array.from({length: 4999}, (_, i) => i % 256)];
    const u = fastUploader(bytes, Infinity, true, {blockPackets: 16});
    await u.element('connectBtn').click();
    assert.deepEqual(u.reads, [0, 0, 3904, 3904, 5000]);
    assert.deepEqual(u.dataChunksAtRead, [0, 16, 16, 21, 21]);
    assert.deepEqual(u.writes.at(-1), ['control', 1]);
});

test('cached write-only metadata reports why OTA used legacy mode', async () => {
    const readError = new Error('GATT operation not permitted');
    readError.name = 'NotSupportedError';
    const u = fastUploader([0xE9, 1, 2, 3], Infinity, false, {
        name: 'Blinds-1234', readable: false, readError,
    });
    await u.element('connectBtn').click();
    assert.deepEqual(u.writes[0], ['control', 0]);
    assert.match(u.element('transferMode').innerText, /legacy mode.*NotSupportedError/);
    assert.match(u.element('status').innerText, /Image accepted/);
});

test('advertised fast firmware never silently falls back when status is blocked', async () => {
    const u = fastUploader([0xE9, 1, 2, 3], Infinity, false, {
        readable: false, readError: new Error('GATT operation not permitted'),
    });
    await u.element('connectBtn').click();
    assert.equal(u.writes.length, 0);
    assert.match(u.element('status').innerText, /Fast OTA device could not be verified/);
});
