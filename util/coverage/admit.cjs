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

// Admit a weekly campaign only from passing trusted Daily and Weekly runs.
module.exports = async ({github, context, core}) => {
    const repo = context.repo;
    core.setOutput('allowed', 'false');
    const trigger = (await github.rest.actions.getWorkflowRun({
        ...repo, run_id: context.payload.workflow_run.id
    })).data;
    const skip = async reason => {
        core.notice(reason);
        await core.summary.addHeading('Coverage skipped')
            .addRaw(reason).write();
    };
    if (process.env.COVERAGE_DISABLED === 'true') {
        return skip('Collection is disabled by GEM5_COVERAGE_DISABLED.');
    }
    const trusted = run =>
        run.head_repository?.full_name ===
            `${repo.owner}/${repo.repo}` &&
        run.head_branch === 'develop' &&
        run.event === 'workflow_dispatch';
    if (!trusted(trigger) || trigger.status !== 'completed' ||
            trigger.conclusion !== 'success' || ![
        '.github/workflows/daily-tests.yaml',
        '.github/workflows/weekly-tests.yaml'
    ].includes(trigger.path)) {
        return skip('The completed run is not a trusted test workflow.');
    }
    // Use the latest Weekly run even when an older completion was queued.
    // A newer failure or unfinished run must not fall back to an older success.
    const weeklyRuns = await github.paginate(
        github.rest.actions.listWorkflowRuns, {
            ...repo, workflow_id: 'weekly-tests.yaml',
            branch: 'develop', event: 'workflow_dispatch', per_page: 100
        });
    const weekly = weeklyRuns.sort((a, b) => b.id - a.id)[0];
    if (!weekly || weekly.head_sha !== trigger.head_sha) {
        return skip('No matching latest Weekly run for this test commit.');
    }
    if (!trusted(weekly) ||
            weekly.path !== '.github/workflows/weekly-tests.yaml' ||
            weekly.status !== 'completed' ||
            weekly.conclusion !== 'success') {
        return skip('Weekly tests did not pass on trusted develop code.');
    }
    const dailyRuns = await github.paginate(
        github.rest.actions.listWorkflowRuns, {
            ...repo, workflow_id: 'daily-tests.yaml',
            branch: 'develop', head_sha: weekly.head_sha,
            event: 'workflow_dispatch', per_page: 100
        });
    const daily = dailyRuns.sort((a, b) => b.id - a.id)[0];
    if (!daily || !trusted(daily) ||
            daily.path !== '.github/workflows/daily-tests.yaml' ||
            daily.head_sha !== weekly.head_sha ||
            daily.status !== 'completed' ||
            daily.conclusion !== 'success') {
        return skip('The latest Daily run for the Weekly commit has not passed.');
    }
    const week = new Date(weekly.created_at);
    week.setUTCDate(week.getUTCDate() -
        (week.getUTCDay() + 6) % 7);
    week.setUTCHours(0, 0, 0, 0);
    const marker = `coverage-source-${week.toISOString().slice(0, 10)}`;
    // An artifact records admission, even if tests or uploads
    // subsequently fail. Rerunning this campaign is allowed.
    const artifacts = await github.paginate(
        github.rest.actions.listArtifactsForRepo, {
            ...repo, name: marker, per_page: 100
        });
    for (const artifact of artifacts) {
        const id = artifact.workflow_run?.id;
        if (artifact.expired || !id ||
                String(id) === String(context.runId)) continue;
        // PR and unrelated workflows can use the same name.
        // Only this trusted completion workflow can admit a run.
        const producer = (await github.rest.actions.getWorkflowRun({
            ...repo, run_id: id
        })).data;
        if (producer.path === '.github/workflows/codecov.yaml' &&
                producer.event === 'workflow_run' &&
                producer.head_repository?.full_name ===
                    `${repo.owner}/${repo.repo}`) {
            return skip('A coverage campaign for this UTC week already exists.');
        }
    }
    const manifest = {
        commit: weekly.head_sha, branch: 'develop',
        week: week.toISOString().slice(0, 10),
        daily: {id: daily.id, attempt: daily.run_attempt,
            url: daily.html_url},
        weekly: {id: weekly.id, attempt: weekly.run_attempt,
            url: weekly.html_url}
    };
    const fs = require('fs');
    fs.writeFileSync(`${process.env.RUNNER_TEMP}/coverage-source.json`,
        JSON.stringify(manifest, null, 2) + '\n');
    core.setOutput('source-ref', weekly.head_sha);
    core.setOutput('revision', weekly.head_sha);
    core.setOutput('marker', marker);
    core.setOutput('allowed', 'true');
    await core.summary.addHeading('Coverage source')
        .addCodeBlock(JSON.stringify(manifest, null, 2), 'json').write();
};
