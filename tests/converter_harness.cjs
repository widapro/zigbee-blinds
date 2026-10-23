const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

const A = {protocol: 0xF040, request: 0xF041, result: 0xF042, target: 0xF043,
    received: 0xF044, completed: 0xF045, failed: 0xF046, duplicates: 0xF047,
    valid: 0xF048, cursor: 0xF049, boot: 0xF022};

function loadConverter({fastClock = false, bind = async () => {}} = {}) {
    const builder = new Proxy(function () {}, {get: () => builder, apply: () => builder});
    let now = 0;
    const context = {
        module: {exports: {}},
        require: (name) => name.endsWith('/reporting') ? {bind} : builder,
        console: {log() {}},
        setTimeout: fastClock ? (fn, ms) => setImmediate(() => {now += ms; fn();}) : setTimeout,
        clearTimeout: fastClock ? clearImmediate : clearTimeout,
        Date: fastClock ? {now: () => now} : Date,
    };
    vm.runInNewContext(fs.readFileSync(path.join(__dirname,
        '../zigbee2mqtt/working_converter.js'), 'utf8'), context);
    return {definition: context.module.exports, now: () => now};
}

function makeBlind({position = 50, status = 3, onWrite, onRead} = {}) {
    const attrs = {[A.protocol]: 1, [A.boot]: 7, [A.result]: 0, [A.cursor]: 0,
        [A.valid]: 1, currentPositionTiltPercentage: position};
    const writes = [];
    const reads = [];
    const entity = {
        attrs, writes, reads,
        async write(cluster, values, options) {
            writes.push({cluster, values, options});
            if (onWrite) return onWrite(values, attrs, writes);
            if (values[A.request]) {
                const packed = values[A.request].value >>> 0;
                attrs[A.cursor] = packed >>> 8;
                const target = packed & 0xFF;
                attrs[A.result] = ((packed & 0xFFFFFF00) | (target === 101 ? 4 : status)) >>> 0;
                if (target <= 100 && status === 3) attrs.currentPositionTiltPercentage = target;
            }
        },
        async read(cluster, requested, options) {
            reads.push({cluster, requested, options});
            if (onRead) await onRead(requested, attrs, reads);
            return Object.fromEntries(requested.map((id) => [id, attrs[id]]));
        },
    };
    return entity;
}

function coverConverter(definition) {
    return definition.toZigbee.find((entry) => entry.key.includes('tilt'));
}

module.exports = {A, loadConverter, makeBlind, coverConverter};
