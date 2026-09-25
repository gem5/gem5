# Report and extraction retries

A **report retry** rebuilds the summary, grouped uploads, landing page and
index from retained `coverage.json`, `python-coverage.json`, baselines and
aggregate XML. Routine reporting downloads only `coverage-data-*`,
`coverage-plan-*` and `coverage-source-*` artifacts. Shared baselines in the
reporting artifacts are compressed as `baseline.json.gz`; generated source
snapshots are packaged separately from runtime counters. It does not run
gcov or tests. Use `report.py summarize`,
`report.py landing` and `index.py build` against the recovered artifacts.

An **extraction retry** reruns gcov against retained native notes and counters.
Download both matching `coverage-data-*` and `coverage-raw-*` artifacts from
the original run into one directory, preserving their artifact directory
names (for example, `coverage-data-quick-gem5-example` alongside
`coverage-raw-quick-gem5-example`). Both types are retained for 30 days.
Recovery pairs their name suffixes and verifies their common revision and
archive checksum. Older artifacts with colocated raw archives also work.

Then run:

```sh
python3 util/coverage/report.py reextract coverage-input \
    --output coverage-reextracted --revision FULL_TESTED_SHA \
    --gcov /path/to/original/gcov
python3 util/coverage/report.py summarize coverage-reextracted \
    --output coverage-report --revision FULL_TESTED_SHA --campaign
```

Run the checked-in recovery tool and collector from this change, with a gcov
executable whose complete version string matches the original collection.
The decoder also checks the compiler versions recorded by each invocation.
No simulator, test, build command or code from the artifacts is executed.
Input artifacts remain unchanged; the output directory must be new and
separate. Native extraction retries need the raw archives, so retain those
original artifacts for any further retry.

Recovery preserves invocation IDs, build identities and execution outcomes.
A failed test stays failed after its coverage is successfully extracted.
Shared schema 2 baselines are required for invocation retries. Native
aggregate retries require the recorded build root and gcov version. Older
artifacts missing those fields can still support a report retry when their
extracted reports are available.

Recovery cannot create counters for a process that never ran, restore a
missing baseline, or reconstruct a killed process's unflushed counters.
Python tracer records are carried through unchanged: this command does not
regenerate them from the tracer's database. Missing or failed Python
collection remains visible in the report.

`reextraction.json` records recovered counts and errors. Errors produce a
nonzero exit code while retaining usable results, and the summary includes
those errors rather than declaring the campaign complete. The default
archive expansion limit is 100 GiB per archive; `--max-bytes` changes it.
Restoration rejects symbolic links, unsafe paths, duplicate members and
special files, while retaining internal hard links to shared notes.

Grouped LCOV uploads are retained as deterministic `.info.gz` files. Report
retries and the offline source browser read them directly. Each upload job
decompresses only its selected report before sending ordinary LCOV to
Codecov; aggregate XML and older uncompressed reports remain supported.
