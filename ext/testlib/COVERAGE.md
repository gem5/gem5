# Per-invocation coverage

From `tests`, run a selected TestLib suite or directory with
`./main.py run <selection> --gcov=per-test`. This uses gem5's existing GCC
instrumentation. Select matching gcov with `--gcov-tool gcov-13`, for example;
GCC 9 or newer is required for JSON output. Use `--skip-build` only when the
instrumented binary and matching `.gcno` notes already exist. Ordinary runs
and the older `--gcov` modes keep their behavior.

Use one binary variant per run and a clean build directory. GCC reuses note
filenames across gem5's opt/debug/fast objects, so multiple variants are
rejected in per-test mode. Separate variant measurements need separate clean
`--build-dir` directories. Changing compiler or variant requires a fresh build.

## Native records

Each gem5 process receives a unique `GCOV_PREFIX` directory beneath
`testing-results/coverage/<invocation-id>/raw`. Parallel tests do not share
counters. The collector snapshots immutable notes once per build fixture and
hardlinks them beside each invocation's counters, falling back to copying.
Archive the coverage tree with tar to preserve these hardlinks.

Each invocation writes `coverage.json` before execution and updates it after
completion, including failed processes. The record contains:

- `schema_version: 2`, `language: native`, source `revision`, full `test_uid`
  (`SuiteUID`) and a unique `invocation_id`.
- `build`: target, executable, compiler/gcov versions and configuration IDs.
- `baseline_id`: the hash of a shared executable-line baseline.
- `outcome`: `passed`, `failed` or `interrupted` for the gem5 process.
- `collection`: `complete`, `missing` or `error`, with an error explanation.
- `files`: repository-relative paths with positive line and branch counts.

Zero-hit executable lines and branch descriptors are stored once in
`coverage/baselines/<baseline_id>/baseline.json`. Keep these baselines with
invocation records. Native branches describe GCC control-flow edges; their
IDs are valid within the exact build. Generated sources under `build/` are
retained beside the baseline when available.

The denominator includes executable lines described by the build directory's
notes, including build helpers and stale objects if present. It is not a
link-map-derived list of only sources in the chosen executable. Clean builds
avoid stale configurations. Embedded `.py` object notes and sources outside
the compilation checkout are excluded.

An initial record is `interrupted` and `missing`. A hard-killed harness leaves
that record detectable. Missing counters are never measured zero coverage.
Collection errors do not replace a simulator failure. `complete` means the
available counters were extracted; an abnormal exit may still lose unflushed
counters. The process outcome does not include later TestLib output verifiers.

When `<target-directory>/<configuration>-gem5.<variant>.json` is present, its
revision must match the checkout. The collector retains the manifest and uses
its `build_root` to map profiles from a different absolute checkout location.
Without it, compilation is assumed to have used the current checkout path.
Raw `.gcda` counters and matching `.gcno` notes remain available after failures.

Attribution is to a TestLib gem5 invocation, including subprocesses inheriting
its native profile environment. It does not identify individual Python
unittest cases inside that process. GCC's optimized-line limitations and the
existing x86 boot instrumentation issue still apply.

## Python records

Add `--python-coverage` after installing `coverage==7.10.7` in the Python
environment embedded by gem5. This requires `--gcov=per-test`. The wrapper
preserves file configuration arguments, globals and import paths, and module
entry points (`gem5 -m package.module ...`). Interactive/debugger and string
entry points are unsupported. Ordinary runs do not import coverage.py.

A separate `python-coverage.json` links to the native invocation through
`parent_invocation_id`. Native and Python percentages must remain separate.
Python counts are 0/1 execution observations; branches are coverage.py arcs.
The retained `.coverage`, `.coverage-mapped` and `python-contexts.json` files
preserve raw observations, relocated source paths and invocation contexts.

Python coverage begins when the configuration starts. Its denominator covers
executable statements in measured repository files, excluding unimported
modules, earlier startup, later shutdown, individual unittest cases and
separate interpreter processes. Forked children stop the inherited tracer so
they cannot overwrite the parent record. Multisim's controller is measured;
its worker interpreters are outside this Python scope. Missing dependencies,
extraction errors and abrupt exits remain visible in collection status without
replacing the configuration's exit status.

## Collector checks

With GNU GCC, matching gcov and coverage.py installed, run:

```
python3 -m unittest discover -s ext/testlib/tests -v
```

These checks use small compiled fixtures and a gem5 stand-in. They cover
concurrency, failed-profile retention, relocation, sparse baselines, actual
TestLib integration and file/module/fork semantics without building gem5.
