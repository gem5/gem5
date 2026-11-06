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
uses GitHub's larger queue so later Daily completions do not replace waiting
Weekly checks or recovery requests. Up to 100 requests can wait; requests
beyond that platform limit are cancelled. Campaigns are serialized without
cancelling an active campaign. A campaign occupies at most four shared
self-hosted runners at once. Later phases still collect coverage if an earlier
phase fails. Reporting and upload recovery run on GitHub-hosted runners and
do not occupy the self-hosted pool.


## Enabling the completion trigger

Use this deployment order:

1. Merge the coverage changes to `develop`. Set the repository Actions
   variable `GEM5_COVERAGE_DISABLED` to `true` while preparing deployment.
2. Check the shared runner requirements described below.
3. Install the matching workflows, configuration, and runtime tools on
   `stable`; remove the old coverage workflow and scheduler dispatch.
4. Fetch both branches and run the deployment comparison below. Review the
   effective Codecov report-age configuration as well.
5. Set `GEM5_COVERAGE_DISABLED` to `false` or remove it. Let a qualifying
   Weekly/Daily pair start collection, inspect the landing page and missing
   groups, then exercise a report-only recovery while artifacts are retained.

Setting the variable back to `true` stops new automatic collections without
changing ordinary tests or disabling report-only recovery. It does not cancel
an active campaign. Runner capacity checks and branch installation are
external deployment steps, not actions performed by merging this PR.

GitHub requires a `workflow_run` workflow on the repository's default branch,
which is `stable` for gem5. Relative reusable workflow calls also resolve from
that branch. To activate this design, install `codecov.yaml` and its matching
`quick-tests.yaml`, `daily-tests.yaml`, and `weekly-tests.yaml` definitions on
`stable`, together with the `scheduler.yaml` change that removes the old
coverage dispatch. Remove `ci-daily-codecov.yaml` there as well. Install the
matching runtime tools under `util/coverage/` and the native decoder
`ext/testlib/coverage.py` on `stable` as well. Keep these definitions and
tools synchronized when
their interfaces, test plans, or report schemas change.
Merging only to `develop` does not activate the completion trigger.

Before recording weekly admission or building, the coordinator compares the
deployed workflows,
configuration, and runtime coverage tools with the tested revision. A
difference stops collection and lists the files to synchronize in the Actions
summary. This prevents silently using a different test plan from `stable`.
After repairing a deployment mismatch, a new completion event can admit the
campaign; the failed preflight does not consume its weekly slot.
Report-only recovery deliberately does not require byte-identical tools:
current reporting tools can read compatible retained schemas.

To check a deployment locally after fetching the reference:

```sh
python3 util/coverage/deployment.py --reference origin/stable
```

The comparison includes the matching files under `util/coverage`, the native
decoder, `.github/coverage-native.json`, and `.github/codecov.yml`. Runtime tools added
later are discovered automatically; tests and prose documentation are excluded.

The source-ref input is necessary because the completion workflow itself has
`stable` as its GitHub ref/SHA. The test source and Codecov attribution use the
qualifying Weekly run's commit instead. The gate does not execute or download
code from the completed run; it admits only same-repository `develop` runs
started through the ordinary dispatch workflows.

## Runner requirements and shared builds

Coverage uses the same `[self-hosted, linux, x64]` selection as ordinary
Linux tests. Those runners need Docker and the `/gem5-resource-cache`
mount. No additional runner group or coverage label is required.
Coverage shares available capacity with development and scheduled testing.

The coordinator runs these phases sequentially:

| Phase | Maximum self-hosted runners |
| --- | ---: |
| Shared TestLib builds | 4 |
| Quick unit tests (fast and opt) | 2 |
| Quick TestLib | 4 |
| Daily unit and integration tests (debug, SST, SystemC, DRAMSys) | 4 |
| Daily TestLib | 4 |
| Weekly TestLib | 4 |

Quick and Daily reusable workflows expose a `coverage-phase` input. Coverage
callers select `native` or `testlib`; the coordinator invokes both separately.
The default is `testlib`. Ordinary runs ignore this input and keep their
existing test selection, dependencies, and parallelism. The phase split
prevents native jobs from adding runners alongside a TestLib matrix; setting
only a matrix's `max-parallel` would not enforce the campaign-wide bound.
Keep new self-hosted coverage jobs within this budget when extending a phase.

Coverage waits for available runners and may occupy four for a long period.
This bounds contention but does not give ordinary jobs priority or reclaim
runners from an active coverage job. Individual jobs can still use all CPU
cores on their assigned runner.

The coordinator resolves the all-dependencies image to an immutable Linux
x86_64 digest, discovers the union of TestLib build targets across all three
lengths, and builds each target once with test objects and GCC coverage.
Workers download only the binaries needed by their suites and verify the
revision, container identity, and checksums before installation. Matching
notes, generated sources, and configuration accompany the binary; build-time
counters and object files are omitted. Ordinary caches are not used by these
coverage builds. Integration and GTest builds remain separate because they
use different targets or environments. Coverage integration jobs also start
with fresh build directories: restoring old notes can count sources no longer
in the build. SystemC uses separate executable and library build roots so its
configuration change cannot overwrite the executable's coverage graph.

The existing x86 boot exception builds its ordinary binary separately. A
failed shared build does not prevent unrelated successfully built targets
from collecting coverage; the final accounting reports the resulting gaps.
The collector supports parallel invocations, but scheduled coverage keeps its
existing per-job execution limits until gem5 memory/runtime costs are measured.

## Reading the results

The `coverage-report-<run-id>` artifact opens at `index.html`. It combines
collection status, native line coverage from TestLib and aggregate jobs,
and links to grouped reports and the test/line browser at `index/index.html`.
Download and unpack the whole artifact before opening it locally; it does
not need Codecov or a web server.
The `index/index.json` file and its `index/branches/` sidecars support
command-line queries through `util/coverage/index.py`. Keep the complete
artifact directory together. See `util/coverage/README.md` for examples.

TestLib's `--gcov=per-test` mode retains a separate record for each gem5
invocation, identified by its SuiteUID and invocation ID. The profile records
include the source revision, build identity, outcome, and collection status.
The combined report and the test-to-line and line-to-test lookups come from
these same records. Native profiles store the executable baseline once per
build and retain only positive counts for each invocation. The browser
index also shares repeated memberships instead of duplicating test IDs.
GCC branch observations retain build and translation-unit identities;
branches from different compiler graphs are not treated as one branch.

The summary accounts for expected, completed, failed, skipped, unfinished,
excluded, and missing-profile TestLib suites. Execution outcome and profile
availability are separate: a failed invocation may still have valid coverage,
while a process terminated before flushing counters may have none. Invalid or
missing discovery manifests and incomplete collections fail final accounting;
valid partial reports are still retained and uploaded.

C++ GTests and the SST, SystemC, and DRAMSys integrations retain aggregate
reports and existing Codecov flags. These groups do not yet provide individual
in-process GTest-case attribution in the TestLib index. Their required
build/run stages and runtime counters must be present before collection can
be complete. `.github/coverage-native.json` defines the expected groups,
flags, and stages, with a regression check against the actual workflows.

Scheduled TestLib collection also enables `--python-coverage`, using a
pinned coverage.py tracer in the embedded Python interpreter. Native and
Python inventories remain separate in reports and queries. Python coverage
begins at configuration execution: it does not include gem5 startup or
shutdown, unimported modules, separate interpreters, or individual PyUnit
cases within an invocation. The Python companion record for each native
invocation is checked, so a missing tracer result remains a visible gap.
This check does not require profiles from separately spawned interpreters.

Source browsing uses committed files from the tested revision and retained
generated sources. Missing, oversized, or conflicting generated files are
labelled unavailable. Source retention for the browser is capped at 2 MiB
per file and 128 MiB in total; these limits do not change coverage counts.

The command-line tools can suggest tests for changed files or a verified
diff, compare compatible observations, and identify lines covered by only
one observed test. These are aids for investigation, not a rule for skipping
CI: new code and missing tests cannot be assigned coverage from old data.
See `util/coverage/README.md` for commands and comparison limits.

The very-long `gem5/x86_boot_tests` group retains its uninstrumented fallback
because of a known gcov segmentation fault. It still runs, and its omission
from coverage is explicit in the accounting. Collection accounting can be
complete within its declared scope while these exclusions remain; this must
not be described as coverage from every test or every source language.

## Retention and report-only recovery

`coverage-data-*` artifacts retain invocation records, compressed shared
baselines, test results, aggregate XML, and generated-source snapshots.
Routine reporting downloads these lightweight inputs together with the
discovery and admission manifests. It does not download raw counters.

The matching `coverage-raw-*` artifacts retain native notes and counters in
compressed tar archives, preserving shared hard links instead of duplicating
compiler metadata for each test. Download both matching artifact types for
a local extraction retry. Both types are retained for 30 days and have names
distinct from ordinary test artifacts.

The resulting `coverage-report-<run-id>` artifact retains the offline browser,
collection summary, aggregate XML, and compressed `.info.gz` LCOV reports.
Each upload job decompresses its selected LCOV report before sending it to
Codecov.

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

`.github/codecov.yml` accepts reports up to 90 days old so the original
timestamps survive a long campaign and recovery within the 30-day artifact
retention period. This does not extend artifact retention. Install this
configuration on the tested branch and verify the effective Codecov settings
during deployment; do not refresh timestamps to disguise old measurements.

A local extraction retry can also rerun the original gcov against retained
native notes and counters, without running gem5. This requires the matching
toolchain and writes a separate recovered directory; see
[`util/coverage/RECOVERY.md`](../util/coverage/RECOVERY.md). Report recovery
cannot repair missing execution data, and native extraction recovery does
not regenerate Python tracer output.

## Maintaining and validating the infrastructure

The hosted `coverage-tools` PR job runs the collector, report, index,
workflow-admission and command-selection regression tests, including on
draft PRs. These checks use small real GCC programs and recorded workflow
commands; they do not start the weekly campaign or require coverage runners.

Keep normal workload edits in the shared quick, Daily, or Weekly definition.
Update the native plan when adding or removing an aggregate group or required
stage. When changing profile schemas, retain support for artifacts still
within the retention window or document the recovery boundary.
