# Per-invocation GCC coverage

From `tests`, run a selected TestLib suite or directory with
`./main.py run <selection> --gcov=per-test`. The mode builds with the existing
GCC coverage flags. Use `--skip-build` only with an already instrumented build
and its matching `.gcno` files. `--gcov-tool gcov-13`, for example, selects the
gcov executable matching the build's compiler. GCC 9 or newer is required for
JSON output. Ordinary TestLib runs are unchanged.

Use one binary variant per run. `--gcov=per-test --variant=opt,debug` is
rejected before testing because GCC uses the same `.gcno` names for gem5's
`.o`, `.do`, and `.fo` object variants. To compare variants locally, use
separate clean build roots, for example `--build-dir build/coverage-opt`
and `--build-dir build/coverage-debug`, with one `--variant` each. Do not
reuse an instrumented directory after changing its variant or compiler.

The native denominator is the executable lines described by the retained
notes in that build directory. It is not a link-map-derived list of only
code in the selected executable: build helper and stale object notes can
also contribute zero-count lines. A clean dedicated coverage build avoids
stale configurations; the campaign build artifact records its exact notes.

`-t` still controls parallel test execution. Each gem5 invocation writes to a
unique `testing-results/coverage/<invocation-id>/raw` directory through
`GCOV_PREFIX`. The original object paths are retained beneath that directory.
The collector snapshots coverage notes once per build fixture and hardlinks
those immutable notes beside each invocation's counters (copying when linking
is unavailable). Build manifests are also snapshotted once and hardlinked into
each invocation directory. No invocation deletes another invocation's counters. Archive
the coverage tree with a format preserving hardlinks, such as tar, to avoid
storing repeated copies of the notes.

Every invocation has an atomically written `coverage.json` with:

- `schema_version`: `2` (`1` remains readable by reporting tools).
- `language`: `native`.
- `baseline_id`: the SHA-256 of the shared canonical baseline document.
- `test_uid`: the complete stable `SuiteUID`, including its prefix.
- `invocation_id`: a UUID unique to this execution and retry.
- `revision`: the source checkout's Git revision, or `unknown` when unavailable.
- `build`: target, executable, gcov tool/version, and GCC versions from notes.
- `outcome`: `passed`, `failed`, or `interrupted` for the gem5 process.
- `collection`: `complete`, `missing`, or `error`.
- `files`: repository-relative POSIX paths with a `lines` mapping containing
  positive hit counts only. Executable zero-count lines are stored once in
  `coverage/baselines/<baseline_id>/baseline.json`, not repeated per test.

Retain the `baselines` directory with the invocation records when downloading
or reprocessing a report. Consumers verify the baseline's canonical JSON hash,
revision, and language before expanding sparse records. A preparation failure
or early interrupted record can have no baseline and empty files. Such a
record remains missing/error, never a measured zero.

`build_id` hashes the normalized coverage graph, source contents, compiler,
and configuration. `compatibility_id` identifies configuration/compiler/
variant settings independently of source revision for comparison checks.
Neither identity includes invocation UUIDs or absolute build paths.

An initial `interrupted`/`missing` record is written before execution. Normal
completion or an exception updates the outcome and collects available data
without replacing the process's failure. A hard-killed harness leaves the
initial record detectable. `missing` means no counters were found; it must not
be treated as measured zero coverage. `error` includes an explanatory `error`
field. `complete` means all available counters were processed successfully;
it does not prove that an abnormally terminated process flushed every counter.
The process outcome is separate from subsequent TestLib output verifiers.

If `<target-directory>/<configuration>-gem5.<variant>.json` is present, its
revision must match the checkout. The collector retains this manifest as
`build.json`, records its SHA-256, compiler, image and instrumentation fields,
and uses its `build_root` to relocate the original compiler paths. This permits
build artifacts to run in another absolute checkout directory. Without that
manifest, the collector assumes compilation used the current checkout path.

The raw `.gcda` files and matching `.gcno` notes remain after extraction, even
when a process fails. For manual reprocessing, restore the source revision,
use the recorded matching gcov executable, and run `gcov --json-format
--preserve-paths --hash-filenames <absolute-path-to-raw-counter.gcda>` from a
separate output directory. Convert original compiler paths using the recorded
build root. Notes can also be passed to gcov to recover executable lines with
zero counters. Generated source files are supplied by the original build
artifact; the per-invocation directory retains coverage notes, not a second
copy of the entire build.

The unit is one gem5 invocation, including any subprocesses inheriting its
profile environment. It does not distinguish individual Python unittest cases
run inside that process, measure Python source execution, or fix limitations
of instrumented gem5 itself (including the existing x86 boot coverage issue).
Sources outside the compilation checkout and generated `.py` embedding notes
are excluded. Optimized GCC line counts retain GCC's documented limitations.

Run the focused native checks with GNU GCC and matching gcov installed:

```
python3 -m unittest discover -s ext/testlib/tests -v
```

These checks compile small native fixtures, compare serial and concurrent
profiles, retain counters after failed processes, test relocated build
artifacts, and exercise TestLib with a tiny stand-in executable. They do not
build or validate the gem5 simulator.
