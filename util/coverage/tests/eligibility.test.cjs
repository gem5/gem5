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

// Exercise the actual eligibility script without API calls or artifact writes.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const workflow = fs.readFileSync(path.resolve(__dirname,
    '../../../.github/workflows/codecov.yaml'), 'utf8');
const script = workflow.split('- name: Require passing Daily and Weekly tests')[1]
    .split('script: |\n')[1].split('\n            - ')[0]
    .split('\n').map(line => line.slice(22)).join('\n');
const revision = 'a'.repeat(40);
const common = {
    head_repository: {full_name: 'gem5/gem5'}, head_branch: 'develop',
    event: 'workflow_dispatch', head_sha: revision, status: 'completed',
    conclusion: 'success', run_attempt: 1, created_at: '2026-09-25T19:30:00Z',
};
const daily = {...common, id: 10, path: '.github/workflows/daily-tests.yaml'};
const weekly = {...common, id: 20, path: '.github/workflows/weekly-tests.yaml'};
async function check(options = {}) {
    const outputs = {}, files = [], notices = [];
    const trigger = options.trigger || weekly;
    const api = {
        listWorkflowRuns: async () => {}, listArtifactsForRepo: async () => {},
        getWorkflowRun: async ({run_id}) => ({data:
            run_id === trigger.id ? trigger : options.producer}),
    };
    const github = {rest: {actions: api}, paginate: async (method, args) => {
        if (method === api.listArtifactsForRepo) return options.artifacts || [];
        return args.workflow_id === 'daily-tests.yaml' ?
            (options.dailyRuns || [daily]) : (options.weeklyRuns || [weekly]);
    }};
    const summary = {addHeading() { return this; }, addRaw() { return this; },
        addCodeBlock() { return this; }, async write() {}};
    const core = {summary, notice: value => notices.push(value),
        setOutput: (key, value) => {outputs[key] = value;}};
    const context = {repo: {owner: 'gem5', repo: 'gem5'}, runId: 30,
        payload: {workflow_run: {id: trigger.id}}};
    const mockRequire = name => name === 'fs' ? {
        writeFileSync: (_filename, contents) => files.push(JSON.parse(contents)),
    } : require(name);
    const execute = new Function('context', 'github', 'core', 'process', 'require',
        `return (async () => {${script}})()`);
    await execute(context, github, core, {env: {
        RUNNER_TEMP: '/unused', COVERAGE_DISABLED: options.disabled || '',
    }}, mockRequire);
    return {outputs, files, notices};
}
test('Weekly completion admits the passing source and original attempts', async () => {
    const result = await check();
    assert.equal(result.outputs.allowed, 'true');
    assert.equal(result.outputs['source-ref'], revision);
    assert.equal(result.files[0].week, '2026-09-21');
    assert.deepEqual(result.files[0].daily, {id: 10, attempt: 1});
});
test('Daily completion can admit Weekly when it finishes last', async () => {
    assert.equal((await check({trigger: daily})).outputs.allowed, 'true');
});
test('a newer failed or unfinished Daily prevents fallback to success', async () => {
    for (const override of [
        {conclusion: 'failure'}, {status: 'in_progress', conclusion: null},
    ]) {
        const result = await check({dailyRuns: [daily, {...daily, id: 11, ...override}]});
        assert.equal(result.outputs.allowed, undefined);
        assert.equal(result.files.length, 0);
    }
});
test('Daily does not fall back to an older matching Weekly', async () => {
    const result = await check({trigger: daily, weeklyRuns: [weekly,
        {...weekly, id: 21, head_sha: 'b'.repeat(40)}]});
    assert.equal(result.outputs.allowed, undefined);
});
test('foreign repositories and non-dispatched tests cannot admit collection', async () => {
    for (const override of [
        {head_repository: {full_name: 'fork/gem5'}}, {event: 'pull_request'},
        {head_branch: 'topic'}, {path: '.github/workflows/other.yaml'},
    ]) {
        assert.equal((await check({trigger: {...weekly, ...override}})).files.length, 0);
    }
});
test('trusted weekly marker blocks duplicate runs but permits original reruns', async () => {
    const producer = {path: '.github/workflows/codecov.yaml', event: 'workflow_run',
        head_repository: {full_name: 'gem5/gem5'}};
    const artifacts = [{workflow_run: {id: 31}, expired: false}];
    assert.equal((await check({artifacts, producer})).outputs.allowed, undefined);
    assert.equal((await check({artifacts: [{workflow_run: {id: 30}}]}))
        .outputs.allowed, 'true');
    assert.equal((await check({artifacts, producer: {...producer,
        event: 'pull_request'}})).outputs.allowed, 'true');
});
test('administrative pause leaves no admission marker', async () => {
    const result = await check({disabled: 'true'});
    assert.equal(result.outputs.allowed, undefined);
    assert.equal(result.files.length, 0);
    assert.match(result.notices[0], /disabled/);
});
