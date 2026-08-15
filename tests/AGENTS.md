# Test Agent Notes

This directory contains gem5 test entry points, TestLib suites, PyUnit tests,
fixtures, and test resources.

See [`../TESTING.md`](../TESTING.md) for broader background on the repository's
test infrastructure. Use the commands below for the test layout in this
checkout.

## Test Families

C++ GTests are generally declared near the C++ code under `src/` and built with
SCons as `*.test.opt` binaries. Python PyUnit tests live under `tests/pyunit`.
Run them from the `tests` directory so `run_pyunit.py` finds its default
`pyunit` directory:

```sh
cd tests
../build/ALL/gem5.opt run_pyunit.py
```

`tests/gem5/pyunit/test_run.py` is a quick TestLib suite which invokes that
runner; it is not a second PyUnit test directory. Broader regression tests use
TestLib through `tests/main.py`.

## TestLib Selection

TestLib suites are tagged as `quick`, `long`, or `very-long`. The default
`./main.py run` selects quick tests. Use `./main.py list -q --suites` before
running expensive tests, and prefer focused `--uid <SuiteUID>` runs when
validating a specific suite. Add `--skip-build` only when all required gem5 and
unit-test binaries already exist.

Host and ISA filters are part of suite selection. In the current suite set,
`--host gcn_gpu` selects GPU suites whose declared ISA is `VEGA_X86`, so an
explicit `--isa=VEGA_X86` is optional. Supplying both can make the intended
intersection explicit, but the container image alone does not select tests.

## Validation Examples

```sh
./main.py list --length=long -q --suites
./main.py list --host gcn_gpu --isa=VEGA_X86 -q
./main.py run --uid <SuiteUID> --skip-build
```

When estimating rerun cost from artifacts, prefer per-test metadata in
`results.xml` (including each JUnit testcase's `time` attribute) over total
suite wall time.
