# Coverage reports and test attribution

The weekly coverage workflow collects C++ and Python coverage separately.
Its report artifact contains:

- `coverage-native.info.gz` and `coverage-python.info.gz`: aggregate line
  coverage, including executable lines with zero hits.
- `index.sqlite3`: a portable database of executable lines, test identities,
  invocation identities and positive hit counts.
- `summary.json`: the revision, coverage totals, explicit exclusions,
  missing workloads and collection errors. `complete` must be true before
  a report is published to Codecov.

Coverage shows which code executed; it does not establish that the tests
check its behavior. A line with no hits in this campaign may be exercised
by a workload outside the coverage selection.

## Find tests for a line

Download and extract the report artifact. Use repository-relative paths
and the source revision recorded in `summary.json`:

```sh
python3 util/coverage/report.py tests index.sqlite3 src/sim/simulate.cc:100
python3 util/coverage/report.py tests index.sqlite3 configs/example.py:20 \
    --language python
```

The JSON result lists test UIDs, their languages, total hit counts and the
invocations that reached the line. An empty result means no recorded
invocation covered that line. The executable-line inventory remains in
the database and aggregate reports even when no test reached a line.

## Find lines for a test

```sh
python3 util/coverage/report.py lines index.sqlite3 'SuiteUID:...'
python3 util/coverage/report.py lines index.sqlite3 group:sst
```

Use the full UID returned by the first command or TestLib discovery.
Repeated gem5 invocations belonging to the same suite are combined by UID,
while their individual invocation IDs remain available in each result.
The reported counts are summed across those invocations.

TestLib attribution identifies a **suite's gem5 invocation**, rather than
individual assertions inside that process. Python attribution covers
instrumented gem5 Python modules and configuration execution associated
with the same invocation; it does not cover arbitrary child processes.
Unit tests and integrations have aggregate labels such as `group:sst`
and `group:unittests-opt`. They identify the workload group, rather than
individual GoogleTest cases or integration assertions.

The SQLite queries and aggregate reports currently cover **lines**.
Raw records retain their branch data for later analysis. Branch queries,
test recommendations, change comparisons and a source browser are outside
this change's scope.

## Rebuild retained reports

Download the campaign plan, TestLib coverage-data artifacts and native
coverage artifacts into separate subdirectories under `inputs/`. Preserve
their internal paths, including baseline directories:

```sh
python3 util/coverage/report.py build inputs --output report \
    --revision FULL_COMMIT_SHA
```

This uses only Python's standard library. The command saves the database,
LCOV files and summary, then returns a failure status if the campaign is
incomplete. It rejects mismatched revisions, altered baselines, invalid
source paths and invalid counts. The summary makes absent task artifacts,
native groups, suite profiles and native/Python pairs visible. Failed
workloads can still contribute their collected data to an incomplete
report, but that report is not published.

The plan contains `revision`, `tests` entries with `id` and `suites`, and
`native` entries with `group`. Each workload supplies a `status.json` with
the same revision, its `task_id` or `group`, and named `stages` outcomes.
Native artifacts also supply gcovr 8.3 `native.json.gz` and an `extraction`
status. TestLib artifacts contain `coverage.json` and
`python-coverage.json` per invocation (both may be gzip compressed). Sparse native profiles refer to
shared `baselines/HASH/baseline.json` or `baseline.json.gz` files; the hash
identifies their canonical JSON contents, revision and instrumented build.

Rebuilding reports consumes the retained extracted records. It does not
recompile gem5, rerun tests, or re-extract raw counters. The campaign's
compressed counter and build artifacts are retained separately for
diagnosis; they can be large.

Native extraction runs in the original build environment with gcovr 8.3
and the matching compiler's gcov. The workflow invokes:

```sh
python3 util/coverage/report.py native --group sst \
    --status workload-status.json --output native-report
```

The output is a compressed gcovr JSON report and a status record. Embedded Python
coverage notes are excluded from this native extraction; their counts
belong in the separate Python report.
