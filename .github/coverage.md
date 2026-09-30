# Scheduled code coverage

Ordinary Quick, Daily and Weekly jobs call the same test commands as coverage:
`util/ci/testlib.py` handles TestLib discovery and execution, and
`util/ci/native.py` handles unit and integration tests. Ordinary workflows
keep their own build caches, dependencies and parallelism. They do not
collect coverage or select coverage phases.

New TestLib suites are discovered automatically by duration. Add a native
workload to `util/ci/native.json`, implement its commands in the shared
executor and call it from ordinary CI. Coverage discovers that catalog
entry without changing its matrix scheduling or runner limit.

`codecov.yaml` owns the coverage schedule:

1. Require passing Weekly tests and the latest Daily run for the same
   `develop` commit. Record at most one admitted campaign per UTC week.
   A Daily completion can admit collection if it finishes after Weekly.
2. Pin container images, discover the expected workloads and retain a plan.
3. Build each TestLib binary once, with GCC coverage and test objects.
4. Run the six native unit/integration workloads.
5. Run quick, long and very-long TestLib workloads.
6. Generate reports and upload complete results to Codecov.

Steps 3–5 are sequential matrices. Each has `max-parallel: 4`; one campaign
runs at a time. Together these enforce a maximum of four self-hosted
coverage jobs, using the existing `[self-hosted, linux, x64]` pool.
Available capacity is shared with other testing. This cap does not reserve
capacity for ordinary jobs or restrict CPU use within an assigned runner.
Reporting and manual recovery use GitHub-hosted runners.

The passing-test gate verifies repository, branch, workflow, event and
revision. A newer failed or unfinished test run prevents fallback to an
older success. The admitted commit is used for discovery, compilation,
execution, indexing and Codecov uploads. Instrumented binaries carry
matching coverage notes, generated sources, compiler/container metadata
and checksums; consumers verify them before installation.

## Results and limits

The `coverage-report-*` artifact contains compressed LCOV reports, an
`index.sqlite3` database and a completeness summary. The database supports
test-to-line and line-to-test queries with the two commands documented in
[the reporting guide](../util/coverage/README.md).

TestLib attributes coverage to each gem5 invocation and its suite UID.
Unit and integration coverage identifies the aggregate workload group.
Python tracing is separate from native GCC coverage; child processes and
Python startup/shutdown are outside its scope. Aggregate exports and
queries cover source lines; branch records remain retained for later work.

The known very-long x86 boot instrumentation failure is listed as an
explicit exclusion in the plan and summary. Ordinary Weekly tests still
run those suites without instrumentation. A measured zero-hit executable
line is preserved; a missing profile is reported as incomplete data.
Failed tests and extraction failures remain visible and prevent publishing
a complete campaign.

## Recovery and rollout

Retained plans and extracted profiles allow reports to be rebuilt without
rerunning tests. Use the separate **Recover Coverage Reports** workflow,
provide the original **Code Coverage** run ID, and run it from the default
branch. It shares the reporting workflow with normal collection. Partial
reports are retained for diagnosis and are not uploaded as complete results.
Artifacts expire after 30 days. Compressed raw counters and matching build
artifacts are also retained; automated raw-counter re-extraction is outside
this PR.

GitHub completion triggers load workflows from the default branch, which
is `stable` in gem5. Deploy the coverage/recovery/report workflow definitions
and admission helper there when activating this infrastructure. Test and
report helpers are checked out from the admitted `develop` revision; that
revision must contain this infrastructure too. Merging to `develop` alone
does not activate collection. Shared runners require Docker and the existing
`/gem5-resource-cache` mount, and Codecov uploads require `CODECOV_TOKEN`.
Set repository variable `GEM5_COVERAGE_DISABLED=true` to pause admission
without changing ordinary tests.

Before activation, measure a complete campaign's runtime, disk, memory and
artifact sizes, and verify hosted Codecov ingestion. Native/Python fixture
checks and discovery validate the infrastructure, but do not establish
production campaign sizing.

This is a focused alternative to [PR #3540](https://github.com/gem5/gem5/pull/3540).
A source browser, coverage comparisons, changed-code test suggestions,
exclusive-coverage analysis and automated extraction recovery are deferred.
