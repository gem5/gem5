# TestLib coverage index

Build a bidirectional index from TestLib's native `coverage.json` and
optional `python-coverage.json` invocation records:

```sh
python3 util/coverage/index.py build downloaded-profiles --output coverage-index
```

The input directory is searched recursively for those two profile filenames.
Other JSON files, raw counters, and ordinary test results are not interpreted
as profiles. The command writes `index.json` and a standalone `index.html`.
Open the HTML file directly in a browser; no server, network connection, or
JavaScript packages are needed to use the index. Following source or result
links requires access to their host unless a retained source map is supplied.
The Python commands use only the standard
library.

The viewer lets you select a TestLib suite, inspect its invocations and build
details, filter its covered source lines, and find tests that covered a
`path:line`. Source and test-definition links point to the recorded revision
in `https://github.com/gem5/gem5`. Supply `--repository-url` when building an
index for another GitHub repository. A record's optional `results_url` appears
as a result link when it is an absolute HTTP(S) URL. Input values are rendered
as text; embedded profile data cannot introduce HTML or script elements.
Test-definition links accept UID paths beginning with either `tests/gem5/`
or `gem5/`, because the relative UID path depends on TestLib's invocation.

Query the same data without a browser:

```sh
python3 util/coverage/index.py tests-for-line coverage-index/index.json \
    src/cpu/base.cc:100
python3 util/coverage/index.py lines-for-test coverage-index/index.json \
    'SuiteUID:gem5/example/test.py:example-suite'
```

Both query commands print JSON. `tests-for-line` reports the covering suite
UIDs and individual invocation IDs. A line absent from complete profiles has
`measured: false` and `count: null`; a measured, uncovered line has
`measured: true`, `count: 0`, and an empty test list. `lines-for-test` reports
positive-hit lines only, with their contributing invocation IDs,
plus collection status counts. A suite with no complete profile has
`coverage_known: false`. An unknown suite UID is an error.

## Input records and identity

The following version 1 example describes one native gem5 invocation. New
native collection uses sparse version 2 records, described below:

```json
{
  "schema_version": 1,
  "test_uid": "SuiteUID:gem5/example/test.py:example-suite",
  "invocation_id": "unique-invocation-id",
  "revision": "0123456789abcdef0123456789abcdef01234567",
  "build": {"binary": "build/ALL/gem5.opt"},
  "outcome": "passed",
  "collection": "complete",
  "files": [
    {"path": "src/example.cc", "lines": {"10": 2, "11": 0}}
  ]
}
```

- `test_uid` identifies the TestLib suite, not a case inside GTest or a Python
  unittest run. Multiple gem5 invocations or retries can share a suite UID.
- `invocation_id` must uniquely identify an execution, including across CI
  attempts and shards. Identical duplicate records are collapsed, as when the
  same artifact was downloaded twice. Conflicting records with one ID are an
  error; a newer attempt never silently overwrites an older one.
- All records must have the same nonempty source `revision`. Mixing revisions
  is an error even when one of the profiles is missing or empty.
- `outcome` is `passed`, `failed`, or `interrupted`. `collection` is `complete`,
  `missing`, or `error`; these describe separate properties. A failed test can
  still have a complete extracted profile.
- File paths are canonical repository-relative POSIX paths. Line keys are
  positive decimal integers and hit counts are nonnegative integers. Duplicate
  paths in a record are rejected. A complete profile may have no measured
  lines, and individual measured lines may have zero hits.
- `build` and additional record metadata are retained, including diagnostics
  or links supplied by the collector. Full file data and per-invocation hit
  counts remain in the original profile artifacts, which must be retained
  alongside the index.

## How the union is computed

The builder reads one record at a time. It retains invocation metadata and
accumulates line counts with integer bitmasks, rather than keeping a full
baseline or a list of UUIDs for every invocation and source line. Identical
memberships are shared in the output, including the empty membership for
uncovered lines.

`index.json` has `format: "gem5-coverage-index"` and `schema_version: 1`:

- `invocations` contains metadata sorted by invocation ID, a SHA-256 digest
  of each normalized record (including its baseline reference for version 2),
  and counts of its measured/covered lines.
  The array position is that invocation's ordinal.
- `tests` maps each suite UID to its invocation ordinals.
- `memberships` contains distinct lists of covering invocation ordinals.
- `files[path][line]` is `[total_hit_count, membership_id]`. The membership
  ID is an array position in `memberships`. Only positive hits add a member.
- `summary` contains invocation outcomes, collection statuses, suite counts,
  and measured/covered line counts.

These fields and their serialization are deterministic regardless of artifact
traversal order. The CLI translates compact memberships into ordinary suite
UIDs and invocation IDs in query results. The browser does the same locally.
Only campaign-wide total hits are stored in the compact index; use the raw
records when individual invocation hit counts are needed.

Only records with `collection: complete` contribute to this measured union.
Metadata from `missing` and `error` records remains in `invocations`, but
partial file data from those records does not create zero-hit lines or add to
the denominator. Complete profiles from failed or interrupted tests do
contribute their observed coverage. A line is covered
when at least one contributing invocation has a positive hit count. Counts
sum executions and retries; they are not a performance comparison between
tests. No per-invocation identity is lost during aggregation.

The denominator contains lines present in complete profiles. It is **not** a
claim that every gem5 source file or every test has been included. A missing
record cannot be inferred from the index alone; campaign completeness needs
the separate expected-test manifest and reporting checks. A process that
never flushed its counters may leave missing or partial coverage. This index
does not infer missing Python execution or distinguish individual cases
within a gem5 invocation. Python appears only when the separate tracer
records are present; native and Python denominators remain separate.

The standalone page embeds the compact index. Keep the JSON alongside it for
repeatable command-line queries, and retain the raw profile artifacts for
reprocessing. Covered-line tables initially show 200 rows and can be expanded
or filtered. A focused size test covers 2,000 invocations sharing a 1,000-line
baseline and requires the standalone HTML to stay below 2 MB. Actual campaign
size depends on its source-line count and distinct coverage memberships.

## Verification

From the repository root:

```sh
python3 -m unittest discover -s util/coverage -p 'test_index.py'
```

The focused tests cover merged and reverse queries, multiple attempts,
revision and identity conflicts, unknown versus measured-zero coverage,
validation, safe HTML data embedding, and command-line build/query behavior.

## Shared record validation and sparse profiles

`schema.py` is the common validation boundary for reporting and indexing.
`load_record(path, root=input_directory)` returns a normalized record;
`iter_records(directory)` yields `(path, record)` pairs and `read_records`
yields records alone. Both `coverage.json` and `python-coverage.json` are read.
Call `load_record` separately when a reporting application must retain valid
records after rejecting another file.

Version 1 profiles remain supported. Native version 2 profiles contain only
positive line and branch counts, referencing a shared
`baselines/<baseline_id>/baseline.json`. The loader checks the baseline's
canonical JSON SHA-256, revision, language, and referenced lines and branches.
It restores zero-count executable lines and branch metadata in memory.
Missing/error records created before baseline preparation may have no baseline
and no files. They remain visibly incomplete.

`load_sparse_record` and `read_index_records` avoid expanding the baseline per
invocation. They return a `SparseProfile(record, baseline)` for version 2 and
ordinary normalized dictionaries for version 1. The index seeds each baseline
once and processes positive counts thereafter. Consumers must treat shared
baseline objects as immutable.

The index keeps native and Python inventories separate. Its language selector
changes both lookup directions; source-line lookup also lists matching branch
identities and their covering suites. Native branch identities distinguish
build graphs and translation units; Python branches describe directed arcs,
including negative destination lines for function exits. Branch counts are
compiler/interpreter observations, not proof of every semantic condition.

```sh
python3 util/coverage/index.py tests-for-line coverage-index/index.json \
  src/python/gem5/resources/resource.py:100 --language python
python3 util/coverage/index.py tests-for-branch coverage-index/index.json \
  src/example.cc BRANCH_ID --language native
```

## Suggesting tests for a change

Suggestions are prior observations, not permission to skip CI. They include
all observed covering suites, unknown changes, collection limitations and the
exact tested revision. Added lines have no prior coordinates: suggestions
broaden to suites covering the old file and explicitly leave the new lines
unproven. A completely new file may have no evidence at all.

Prefer a repository-backed comparison so the tool verifies both commits and
obtains the old-side diff itself:

```sh
python3 util/coverage/analysis.py suggest coverage-index/index.json \
  --repository /path/to/gem5 --target-revision HEAD
python3 util/coverage/analysis.py suggest coverage-index/index.json \
  --file src/cpu/base.cc
```

A supplied `--diff change.patch` requires `--base-revision` equal to the index
revision. Its provenance remains caller-supplied; the tool does not pretend it
verified an arbitrary patch against a repository. Mismatched bases and malformed
hunks are rejected. New-side line numbers are never looked up in old coverage.
`--language python` selects the separate Python inventory. Evidence examples
are capped at 20 locations per suggestion while match counts retain the total.
