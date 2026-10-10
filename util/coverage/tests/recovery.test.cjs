// Copyright (c) 2026 The Regents of The University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

// Run with: node --test util/coverage/tests/recovery.test.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');

const workflow = fs.readFileSync(path.resolve(__dirname,
    '../../../.github/workflows/codecov.yaml'), 'utf8');
function scriptBetween(start, end) {
    const block = workflow.split(`- name: ${start}`)[1]
        .split(`- name: ${end}`)[0].split('script: |\n')[1];
    return block.split('\n').map(line => line.slice(22)).join('\n');
}
const checkRun = scriptBetween('Check the source coverage run',
    'Checkout trusted reporting tools');
const checkManifest = scriptBetween('Verify the admitted source metadata',
    'Checkout source for offline browsing');
const revision = 'a'.repeat(40);
const context = {
    eventName: 'workflow_dispatch', ref: 'refs/heads/stable', runId: 9,
    repo: {owner: 'gem5', repo: 'gem5'},
    payload: {repository: {default_branch: 'stable'}},
};
const producer = {
    path: '.github/workflows/codecov.yaml', event: 'workflow_run',
    head_repository: {full_name: 'gem5/gem5'}, status: 'completed',
};
const manifest = {
    commit: revision, branch: 'develop',
    daily: {id: 1, attempt: 2}, weekly: {id: 2, attempt: 1},
};
async function execute(script, overrides = {}) {
    const outputs = {};
    const run = overrides.run || producer;
    const metadata = overrides.manifest || manifest;
    const github = {rest: {actions: {
        getWorkflowRun: async () => ({data: run}),
        getWorkflowRunAttempt: async args => ({data: {
            path: `.github/workflows/${args.run_id === 1 ? 'daily' : 'weekly'}-tests.yaml`,
            head_sha: revision, head_branch: 'develop',
            event: 'workflow_dispatch', conclusion: 'success',
            head_repository: {full_name: 'gem5/gem5'},
            ...overrides.attempt,
        }}),
    }}};
    const mockRequire = name => name === 'fs' ? {
        readdirSync: () => Array.from({length: overrides.manifests ?? 1}, (_, i) => ({
            name: `coverage-source-${i}`, isDirectory: () => true,
        })),
        readFileSync: () => JSON.stringify(metadata),
    } : require(name);
    const fn = new Function('context', 'github', 'core', 'process', 'require',
        `return (async () => {${script}})()`);
    await fn({...context, ...overrides.context}, github,
        {setOutput: (key, value) => {outputs[key] = value;}},
        {env: {REQUESTED_RUN: overrides.id || '12'}}, mockRequire);
    return outputs;
}

test('report-only recovery accepts the original trusted campaign', async () => {
    assert.equal((await execute(checkRun))['run-id'], '12');
});
test('normal campaign may report its own running workflow', async () => {
    assert.equal((await execute(checkRun, {
        context: {eventName: 'workflow_run'},
        run: {...producer, status: 'in_progress'},
    }))['run-id'], '9');
});
test('recovery rejects non-default-branch invocation', async () => {
    await assert.rejects(execute(checkRun, {context: {ref: 'refs/heads/topic'}}));
});
test('recovery rejects malformed run IDs', async () => {
    await assert.rejects(execute(checkRun, {id: '12; echo bad'}));
});
test('recovery rejects PR or unrelated artifacts', async () => {
    for (const override of [
        {event: 'pull_request'}, {path: '.github/workflows/ci-tests.yaml'},
        {head_repository: {full_name: 'fork/gem5'}}, {status: 'in_progress'},
    ]) {
        await assert.rejects(execute(checkRun, {run: {...producer, ...override}}));
    }
});
test('source metadata resolves the original passing attempts', async () => {
    assert.equal((await execute(checkManifest))['source-ref'], revision);
});
test('source metadata rejects mismatched revision or repository', async () => {
    for (const attempt of [
        {head_sha: 'b'.repeat(40)}, {head_repository: {full_name: 'fork/gem5'}},
        {conclusion: 'failure'}, {event: 'pull_request'}, {head_branch: 'topic'},
    ]) {
        await assert.rejects(execute(checkManifest, {attempt}));
    }
});
test('source metadata requires exactly one admission artifact', async () => {
    await assert.rejects(execute(checkManifest, {manifests: 0}));
    await assert.rejects(execute(checkManifest, {manifests: 2}));
});
test('source metadata rejects malformed identities and commits', async () => {
    await assert.rejects(execute(checkManifest, {
        manifest: {...manifest, commit: '../source'},
    }));
    await assert.rejects(execute(checkManifest, {
        manifest: {...manifest, daily: {id: '1', attempt: 2}},
    }));
});
