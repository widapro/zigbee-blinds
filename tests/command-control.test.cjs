const {test} = require('node:test');
const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');

test('firmware command controller handles cancellation, duplicates and sleep readiness', () => {
    const root = path.resolve(__dirname, '..');
    const temp = fs.mkdtempSync(path.join(os.tmpdir(), 'blinds-command-test-'));
    try {
        const executable = path.join(temp, 'command-control');
        const build = spawnSync('cc', ['-std=c11', '-Wall', '-Wextra', '-Werror',
            '-I', path.join(root, 'main'), path.join(root, 'main/command_control.c'),
            path.join(__dirname, 'command_control_test.c'), '-o', executable], {encoding: 'utf8'});
        assert.equal(build.status, 0, build.stderr || String(build.error));
        const run = spawnSync(executable, [], {encoding: 'utf8'});
        assert.equal(run.status, 0, run.stderr || String(run.error));
        assert.match(run.stdout, /9 command-control scenarios passed/);
    } finally {
        fs.rmSync(temp, {recursive: true, force: true});
    }
});
