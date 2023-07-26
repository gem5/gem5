This file explains how to use gem5's updated testing infrastructure. Running
tests before submitting a patch is *incredibly important* so unexpected bugs
don't creep into gem5.

gem5's testing infrastructure has the following goals:
 * Simple for *all* users to run
 * Fast execution in the simple case
 * High coverage of gem5 code

## Running the CPP unit tests

gem5 comes with unit tests for CPP, created using the Google Test framework. These can
be built through SCons.

To build and run all the unit tests:

```shell
scons build/ALL/unittests.opt
```

All unit tests should be run prior to creating a pull request at
https://github.com/gem5/gem5/pulls/

To compile and run just one set of tests (e.g. those declared within
`src/base/bitunion.test.cc`):

```shell
scons build/ALL/base/bitunion.test.opt
./build/ALL/base/bitunion.test.opt
```

To list the available test functions from a test file:

```shell
./build/ALL/base/bitunion.test.opt --gtest_list_tests
```

To run a specific test function (e.g., BitUnionData.NormalBitfield):

```shell
./build/ALL/base/bitunion.test.opt --gtest_filter=BitUnionData.NormalBitfield
```

## Running the Python unit tests

gem5 comes with Python unit tests.
These are built using the [Python unit testing framework](https://docs.python.org/3/library/unittest.html).
These tests can be found in "tests/gem5/pyunit".

To run these tests a gem5 binary must first be compiled.
We recommend, `build/ALL/gem5.opt`:

```sh
scons build/ALL/gem5.opt -j {number of compilation threads}
```

Then the Pyunit tests may be executed using:

```sh
./build/ALL/gem5.opt tests/run_pyunit.py
```

**Note**: These tests are also run via the 'quick' system-level tests, explained below.

## Running system-level tests

Within the "tests/gem5" directory we have system-level tests. These tests run
the gem5 framework against various hardware configurations, with different
ISAs, then verify the simulations execute correctly. These should be seen as
high-level, coarse-grained tests to compliment the unit-tests.

Below is the most common way the tests are run. This will run all of the
"quick" tests for X86, ARM, and RISC-V. These tests make up our best-supported
platforms and use cases. When running these tests, you will likely want to us
the option `-j <CPUs>` where `CPUs` is as large as you can make it.
Additionally, it is often a good idea to run longer tests (e.g., linux boot)
before submitting your patch.

```shell
cd tests
./main.py run
```

The above is the *minumum* you should run before posting a pull request to
https://github.com/gem5/gem5/pulls/

## Running tests from multiple directories

The command line above will walk the directory tree starting from the cwd
(tests), and it will run every test it encounters in its path. It is possible
to specify multiple root directories by providing several positional
arguments:

```shell
./main.py run <directory1> <directory2> [...]
```

This will load every test in directory1 and directory2 (and their
subdirectories).

### 'quick', 'long', and 'very-long' tests

There are three categoties of tests which may be run from the "tests" directory:

1. **'quick' tests**. This suite of tests are designed to finish execution in a few hours, inclusive of compilation of gem5.
We run these as part of our continuous integration tests on pull requests made to our repository.
These tests all utilize a binary build `scons build/ALL/gem5.opt`, and thus only rely on a single compilation for the tests to run.
2. **'long' tests**. This suite of tests are designed to finish execution in around 12 hours.
They incorporate longer running tests which are unsuitable to run as part of the 'quick' tests.
We run these daily via a scheduled job.
3. **'very-long' tests**. This suite of tests are designed to finish execution in days.
They incorporate tests which are too long to run frequntly
We run these daily via a scheduled job.

When executing `./main.py run` the 'quick' tests are executed.
To run the 'long' tests execute:

```sh
./main.py run --length=long
```

and to run the 'very-long' tests execute:

```sh
./main.py run --length=very-long
```

In most cases we recommend running the 'quick' tests for most changes.
Only in some cases, such as contributions which significantly change the codebase, do we recommend running the 'long' or 'very-long' suite.

### Running tests in batch

The testing infrastructure provides the two needed methods to run tests in
batch. First, you can list all of the tests based on the same tags as above in
a machine-readable format by passing the `-q` flag. This will list all of the
*suites* that match the given tag(s).

```shell
./main.py list -q --suites
SuiteUID:tests/gem5/hello_se/test_hello_se.py:testhello64-static-X86-opt
SuiteUID:tests/gem5/hello_se/test_hello_se.py:testhello64-dynamic-X86-opt
SuiteUID:tests/gem5/hello_se/test_hello_se.py:testhello32-static-X86-opt
SuiteUID:tests/gem5/hello_se/test_hello_se.py:testhello64-static-ARM-opt
SuiteUID:tests/gem5/hello_se/test_hello_se.py:testhello32-static-ARM-opt
SuiteUID:tests/gem5/m5_util/test_exit.py:m5_exit_test-X86-opt
SuiteUID:tests/gem5/test_build/test_build.py:build-X86-opt
SuiteUID:tests/gem5/test_build/test_build.py:build-RISCV-opt
SuiteUID:tests/gem5/test_build/test_build.py:build-ARM-opt
```

Next, you can run a single *suite* from the command line by passing the option
`--uid`. For instance,

```shell
./main.py run --skip-build \
    --uid SuiteUID:tests/gem5/m5_util/test_exit.py:m5_exit_test-X86-opt
```

With this method, you can only run a *single* suite at a time. If you want to
run more than one uid, you must call `./main.py` multiple times.

Currently, you must specify `--skip-build` if you want to run a single suite or
run in batch mode. Otherwise, you will build gem5 for all architectures.

### Rerunning failed tests

While developing software a common practice is to run tests, make a change, and
assert that the tests still pass. If tests fail you'll likely want to
rerun and fix those specific tests without running redundant ones. The testing
infrastructure allows you to rerun tests which failed in the last execution by
using the `rerun` command.

```shell
./main.py run
#
#  Some tests fail...
#

# Rerun only the failed test suites (not the ones which passed).
./main.py rerun
```

### If something goes wrong

The first step is to turn up the verbosity of the output using `-v`. This will
allow you to see what tests are running and why a test is failing.

If a test fails, the temporary directory where the gem5 output was saved is kept
and the path to the directory is printed in the terminal.

### Debugging the testing infrastructure

Every command takes an option for the verbosity. `-v`, `-vv`, `-vvv` will
increase the verbosity level. If something isn't working correctly, you can
start here.

Most of the code for the testing infrastructure is in ext/testlib. This code
contains the base code for tests, suites, fixtures, etc. The code in tests/gem5
is *gem5-specific* code. For the most part, the code in tests/gem5 extends the
structures in ext/testlib.

### Common errors

You may see a number of lines of output during test discovery that look like
the following:

```shell
    Tried to load tests from ... but failed with an exception.
    Tried to load tests from ... but failed with an exception.
    ...
```

The testing library searches all python files in the `tests/` directory. The
test library executes each python file it finds searching for tests. It's okay
if the file causes an exception. This means there are no tests in that file
(e.g., it's not a new-style test).

### Running Tests in Parallel

Whimsy has support for parallel testing baked in. This system supports
running multiple suites at the same time on the same computer. To run
suites in parallel, supply the `-t <number-tests>` flag to the run command.

For example, to run up to three test suites at the same time::

    ./main.py run --skip-build -t 3

### Testing resources

By default binaries and testing resources are obtained via the [gem5 resources infrastructure](https://www.gem5.org/documentation/general_docs/gem5_resources/).
The downloaded resources are cached in "tests/gem5/resources".
The resources are cached to avoid re-downloading when tests are run multiple times, though some of these resources, such as disk images, are large.
It is therefore recommended you remove the "tests/gem5/resources" directory when you are done testing.

## Running Tests within GitHub Actions

These tests outlined here are run as part of [GitHub Actions](https://github.com/features/actions).
These are outlined in [workflow files](https://docs.github.com/en/actions/using-workflows/about-workflows), which can be found in the repo's ".github" directory.
Each workflow is made up of individual jobs where ecch job consists of a series of steps which are executed within a [GitHub Runner](https://docs.github.com/en/actions/using-github-hosted-runners/about-github-hosted-runners).

### Adding Tests to GitHub Actions

To ensure tests added are run in GitHub Actions you may need to modify the worklfow files.
For tests run via `./main.py` we split up the tests via the subdirectories in "tests/gem5".
For example, all tests under "test/gem5/cpu_tests" are run as one job.
Therefore tests added to existing directories are likely to be included, but modifications to the workflow files may be needed if new directories are added.

We strongly recommend that when adding or ammending tests, that contributors check the ".github/workflows" files to ensure the tests they specify will be run as intended.
