# Test Agent Notes

This directory contains gem5 test entry points, TestLib suites, PyUnit tests,
fixtures, and test resources.

## Test Families

C++ GTests are generally declared near the C++ code under `src/` and built with
SCons as `*.test.opt` binaries. Python PyUnit tests live under `tests/pyunit`
and `tests/gem5/pyunit` and are run with `tests/run_pyunit.py`. Broader
regression tests use TestLib through `tests/main.py`.

## TestLib Selection

TestLib suites are tagged as `quick`, `long`, or `very-long`. The default
`./main.py run` selects quick tests. Use `./main.py list -q --suites` before
running expensive tests, and prefer focused `--uid <SuiteUID> --skip-build`
runs when validating a specific suite.

Host and ISA filters are part of suite selection. For GPU tests, `--host
gcn_gpu` is not enough by itself; include `--isa=VEGA_X86` when that is the
intended build target.

## Validation Examples

```sh
./main.py list --length=long -q --suites
./main.py list --host gcn_gpu --isa=VEGA_X86 -q
./main.py run --uid <SuiteUID> --skip-build
```

When estimating rerun cost from artifacts, prefer per-test metadata in
`results.xml` over total suite wall time.
