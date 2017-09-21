:Authors: Jason Lowe-Power
          Sean Wilson

This file explains how to use gem5's updated testing infrastructure. Running
tests before submitting a patch is *incredibly important* so unexpected bugs
don't creep into gem5.

gem5's testing infrastructure has the following goals:
 * Simple for *all* users to run
 * Fast execution in the simple case
 * High coverage of gem5 code

# Running tests

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

The above is the *minumum* you should run before posting a patch to 
https://gem5-review.googlesource.com

## Specifying a subset of tests to run

You can use the tag query interface to specify the exact tests you want to run.
For instance, if you want to run only with `gem5.opt`, you can use

```shell
./main.py run --variant opt
```

Or, if you want to just run X86 tests with the `gem5.opt` binary:

```shell
./main.py run --length quick --variant opt --isa X86
```


To view all of the available tags, use

```shell
./main.py list --all-tags
```

The output is split into tag *types* (e.g., isa, variant, length) and the
tags for each type are listed after the type name.

You can specify "or" between tags within the same type by using the tag flag
multiple times. For instance, to run everything that is tagged "opt" or "fast"
use

```shell
./main.py run --variant opt --variant fast
```

You can also specify "and" between different types of tags by specifying more
than one type on the command line. For instance, this will only run tests with
both the "X86" and "opt" tags.

```shell
./main.py run --isa X86 --variant opt
```

## Running tests in batch

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

## Rerunning failed tests

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

# If something goes wrong

The first step is to turn up the verbosity of the output using `-v`. This will
allow you to see what tests are running and why a test is failing.

If a test fails, the temporary directory where the gem5 output was saved is kept
and the path to the directory is printed in the terminal.

## Debugging the testing infrastructure

Every command takes an option for the verbosity. `-v`, `-vv`, `-vvv` will
increase the verbosity level. If something isn't working correctly, you can
start here.

Most of the code for the testing infrastructure is in ext/testlib. This code
contains the base code for tests, suites, fixtures, etc. The code in tests/gem5
is *gem5-specific* code. For the most part, the code in tests/gem5 extends the
structures in ext/testlib.

## Common errors

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


# Binary test applications

The code for test binaries that are run in the gem5 guest during testing are
found in `tests/test-progs`.
There's one directory per test application.
The source code is under the `source` directory.

You may have a `bin` directory as well.
The `bin` directory is automatically created when running the test case that
uses the test binary. The binary is downloaded from the gem5 servers the first
time it is referenced by a test.

## Updating the test binaries

The test infrastructure should check with the gem5 servers to ensure you have
the latest binaries. However, if you believe your binaries are out of date,
simply delete the `bin` directory and they will be re-downloaded to your local
machine.

## Building (new-style) test binaries

In each `src/` directory under `tests/test-progs`, there is a Makefile.
This Makefile downloads a docker image and builds the test binary for some ISA
(e.g., Makefile.x86 builds the binary for x86). Additionally, if you run `make
upload` it will upload the binaries to the gem5 server, if you have access to
modify the binaries. *If you need to modify the binaries for updating a test or
adding a new test and you don't have access to the gem5 server, contact a
maintainer (see MAINTAINERS).*


# Running Tests in Parallel

Whimsy has support for parallel testing baked in. This system supports
running multiple suites at the same time on the same computer. To run 
suites in parallel, supply the `-t <number-tests>` flag to the run command.

For example, to run up to three test suites at the same time::

    ./main.py run --skip-build -t 3

