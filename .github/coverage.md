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
These jobs still share the self-hosted runner pool with normal CI.

## Enabling the completion trigger

GitHub requires a `workflow_run` workflow on the repository's default branch,
which is `stable` for gem5. Relative reusable workflow calls also resolve from
that branch. To activate this design, install `codecov.yaml` and its matching
`quick-tests.yaml`, `daily-tests.yaml`, and `weekly-tests.yaml` definitions on
`stable`, together with the `scheduler.yaml` change that removes the old
coverage dispatch. Remove `ci-daily-codecov.yaml` there as well. Keep these
workflow definitions synchronized when their interfaces or test plans change.
Merging only to `develop` does not activate the completion trigger.

The source-ref input is necessary because the completion workflow itself has
`stable` as its GitHub ref/SHA. The test source and Codecov attribution use the
qualifying Weekly run's commit instead. The gate does not execute or download
code from the completed run; it admits only same-repository `develop` runs
started through the ordinary dispatch workflows.

## Reading the results

Existing Codecov flags identify test directories, lengths, and integration or
unit-test groups. They are aggregate groups, not an individual-test-to-line
index. Test result artifact names include `gcov` during coverage runs, and coverage
build caches are separate from ordinary quick/Daily caches.

The very-long `gem5/x86_boot_tests` group retains its existing uninstrumented
fallback because of a known gcov segmentation fault. It still runs, but its
coverage upload is skipped and the omission is reported in the job summary.
Consequently, these reports do not represent coverage from every test.
GCC coverage also measures native code, not execution of Python source.

Codecov upload errors fail the coverage job. An interrupted process may not
flush counters, and reports collected after a test failure can be partial.
The current workflow does not yet publish expected-versus-collected test
counts or retain every individual test's profile for later reprocessing.
