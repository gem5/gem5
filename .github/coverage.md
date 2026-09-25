# Code coverage

`codecov.yaml` runs the shared quick, Daily, and Weekly test plans with GCC
coverage enabled. Ordinary PR, Daily, and Weekly runs do not enable coverage.
The coverage workflow is independent: an instrumentation or upload failure
does not change the result of the ordinary tests or the required PR check.

## When collection runs

A successful `Weekly Tests` completion on `develop` starts the eligibility
check. The latest dispatched Daily run **for that same commit** must also have
completed successfully. A missing, running, cancelled, or failed Daily run
skips collection, with the reason in the workflow summary. A successful Daily
completion also rechecks the latest Weekly run, so collection can start when
Daily finishes last. Both must pass at the same commit; an older passing Weekly
run cannot substitute for the latest Weekly run on a Daily-triggered check.
Rerun the admitted coverage workflow from its Actions page to retry collection.

The coverage workflow checks out that tested commit throughout and explicitly
attributes Codecov uploads to it on `develop`. It records the SHA and the
qualifying Daily/Weekly run URLs and attempts in `coverage-source.json`.

Only one campaign is admitted for each UTC week (Monday through Sunday) in
which Weekly tests were started. A 30-day `coverage-source-YYYY-MM-DD` artifact
records admission, including campaigns that later fail. Keep this artifact to
preserve deduplication. Rerunning the same coverage run is allowed. Coverage
campaigns are serialized without cancelling an active campaign, and quick,
Daily, and Weekly workloads run sequentially to retain the five-job TestLib
matrix limit. Later workloads still collect coverage if an earlier one fails.
Coverage builds and tests target the dedicated `gem5-coverage` runner group.
Reporting and upload recovery run on GitHub-hosted runners.

## Enabling the completion trigger

GitHub requires a `workflow_run` workflow on the repository's default branch,
which is `stable` for gem5. Relative reusable workflow calls also resolve from
that branch. To activate this design, install `codecov.yaml` and its matching
`quick-tests.yaml`, `daily-tests.yaml`, and `weekly-tests.yaml` definitions on
`stable`, together with the `scheduler.yaml` change that removes the old
coverage dispatch. Remove `ci-daily-codecov.yaml` there as well. Install the
matching `util/coverage/index.py` and `report.py` reporting tools on `stable`
as well. Keep these definitions and tools synchronized when
their interfaces, test plans, or report schemas change.
Merging only to `develop` does not activate the completion trigger.

The source-ref input is necessary because the completion workflow itself has
`stable` as its GitHub ref/SHA. The test source and Codecov attribution use the
qualifying Weekly run's commit instead. The gate does not execute or download
code from the completed run; it admits only same-repository `develop` runs
started through the ordinary dispatch workflows.

## Runner isolation and shared builds

Provision the `gem5-coverage` group before activating collection. Its runners
must be Linux x86_64 machines with Docker and the `/gem5-resource-cache`
mount. Register them without default labels and give them only the
`gem5-coverage` label, so ordinary jobs selecting `self-hosted`, `linux`, and
`x64` cannot consume this pool. Group membership alone does not prevent those
ordinary label-based jobs from using a runner.

Restrict the group's workflow access to these definitions at
`refs/heads/stable`: `gem5/gem5/.github/workflows/codecov.yaml`,
`quick-tests.yaml`, `daily-tests.yaml`, and `weekly-tests.yaml` (use the full
repository/workflow path for each). GitHub applies this restriction to the
workflows directly defining jobs, including reusable workflows. Runner
provisioning and group access settings are external to this PR.

The coordinator resolves the all-dependencies image to an immutable Linux
x86_64 digest, discovers the union of TestLib build targets across all three
lengths, and builds each target once with test objects and GCC coverage.
Workers download only the binaries needed by their suites and verify the
revision, container identity, and checksums before installation. Matching
notes, generated sources, and configuration accompany the binary; build-time
counters and object files are omitted. Ordinary caches are not used by these
coverage builds. Integration and GTest builds remain separate because they
use different targets or environments.

The existing x86 boot exception builds its ordinary binary separately. A
failed shared build does not prevent unrelated successfully built targets
from collecting coverage; the final accounting reports the resulting gaps.
The collector supports parallel invocations, but scheduled coverage keeps its
existing per-job execution limits until gem5 memory/runtime costs are measured.

## Reading the results

The `coverage-report-<run-id>` artifact contains a summary, explicit grouped
Codecov reports, and `index/index.html` with a test selector and source-line
lookup. Open the HTML locally; it does not need Codecov or a web server.
The accompanying `index.json` supports command-line queries through
`util/coverage/index.py`. See `util/coverage/README.md` for examples.

TestLib's `--gcov=per-test` mode retains a separate record for each gem5
invocation, identified by its SuiteUID and invocation ID. The profile records
include the source revision, build identity, outcome, and collection status.
The combined report and the test-to-line and line-to-test lookups come from
these same records. The browser index shares repeated membership sets to
avoid embedding thousands of copies of the same baseline and identifiers.

The summary accounts for expected, completed, failed, skipped, unfinished,
excluded, and missing-profile TestLib suites. Execution outcome and profile
availability are separate: a failed invocation may still have valid coverage,
while a process terminated before flushing counters may have none. Invalid or
missing discovery manifests and incomplete collections fail final accounting;
valid partial reports are still retained and uploaded.

C++ GTests and the SST, SystemC, and DRAMSys integrations retain aggregate
reports and existing Codecov flags. These groups do not yet provide individual
in-process GTest-case attribution in the TestLib index. Native GCC coverage
does not measure Python source execution.

The very-long `gem5/x86_boot_tests` group retains its uninstrumented fallback
because of a known gcov segmentation fault. It still runs, and its omission
from coverage is explicit in the accounting. Collection accounting can be
complete within its declared scope while these exclusions remain; this must
not be described as coverage from every test or every source language.

## Retention and report-only recovery

Coverage data artifacts retain individual records, matching notes, raw
counters, test results, and exported aggregate reports for 30 days. Raw
profiles are packed into a compressed tar archive to preserve shared hard
links instead of duplicating compiler metadata for each test. Coverage result
artifact names differ from ordinary test artifacts.

To retry reporting or uploads without rebuilding or rerunning tests, dispatch
**Code Coverage** on the default branch with `source-run-id` set to the
original coverage campaign's Actions run ID. Recovery verifies the original
workflow identity, admission manifest, source revision, and qualifying passing
test attempts before rebuilding the index and grouped reports. It uses trusted
reporting tools and does not execute code or unpack raw profiles from the
artifacts. Valid groups retain their original Codecov flags when reuploaded.

Recovery needs the retained artifacts. It cannot manufacture missing counters
or repair an instrumented test failure; those require rerunning the relevant
coverage jobs. New schemas may also require compatible reporting tools.
