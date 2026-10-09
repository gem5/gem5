# How to get code coverage for gem5 using gcov/gcovr

This document outlines the three ways of getting code coverage for the C++ side
of gem5's codebase:

1. Directly using gcovr
2. Using gcovr through TestLib
3. Using Codecov through GitHub Actions

## 0. Tools for code coverage in gem5

The utility that underlies all of the methods above is `gcov`, which is a tool
that can be run on code compiled with GCC in order to get code coverage.
When running `gcov` directly, users must pass in a list of files for which they
wish to get code coverage metrics. However, for a project with a large codebase
such as `gem5`, it is infeasible to pass in all filepaths. As such, other tools,
which manage the use of `gcov`, are used instead. The two that gem5 uses are:

1. `gcovr`: This is a command line utility that manages calls to `gcov` and
provides various options for combining and outputting code coverage.
2. `Codecov`: This is a service that allows for code coverage data to be
generated while running CI or other GitHub Actions workflows, then to be
uploaded to, analyzed, and displayed on the Codecov website.

### How `gcov` records code coverage

The tool `gcovr` manages calls to `gcov`, so it can be helpful to understand how
`gcov` records code coverage information.

Code coverage for `gcov` is stored in `.gcda` and `.gcno` files. The `.gcno`
files are generated during compilation, and `.gcda` files are generated
and updated when a program is run. See [here](https://gcc.gnu.org/onlinedocs/gcc/Gcov-Data-Files.html)
for more information.

Some `.gcda` files are generated during gem5's build process. However, because
we typically don't want to include the coverage of the build process, we remove
the `.gcda` files in the instructions below.

Similarly, you can clear the `.gcda` files to restore the directory to a blank
slate, to get separate code coverage for another test/program. Instructions for
this are also included below.


## 1. Getting code coverage using gcovr

- 0. Set up the Python environment needed for `gcovr`. When running it on
`gem5`, `gcovr` version 7.1 or later is required, as previous versions contain a
bug that will cause `gcovr` to fail when [source files longer than 9,999 lines
are encountered](https://github.com/gcovr/gcovr/issues/882).

```bash
python3 -m venv gcovr-env
source gcovr-env/bin/activate
pip3 install 'gcovr>=7.1' # installing the latest gcovr version should be sufficient
```

- 1. Build gem5 using the `--gcov` flag. Either `gem5.opt` or `gem5.debug` may
be used.
  - When building gem5 for code coverage, it's suggested to keep the `build`
  directory in the `gem5` root directory.
  - Additionally, the `--gcov` option currently only works on X86 machines; on
  Arm machines, you may encounter an error related to the code model being too
  small.
  - gem5 must be built using `gcc` in order to use `gcov`. The `--gcov` option
  will be ignored if Clang is used.

```bash
scons build/ALL/gem5.debug --gcov
```

- 2. Next, remove all files ending with `.py.gcno` and `.gcda` from the
`build/{ISA}` directory.
  - Code coverage files ending with `.py.gcno` or `.py.gcda` will cause
errors upon running `gcovr`.
  - Files ending with `.gcda` record the code coverage obtained from running
  tests or programs. If these files aren't removed after building, the code
  coverage from the build process will be combined into the code coverage of
  the tests, which may be undesirable.

```bash
# If the `build` directory is inside the `gem5` directory, the `cd` isn't
# necessary
cd build/ALL
find . -name "*.py.gcno" -delete
find . -name "*.gcda" -delete
```

- 3. Next, run the test(s) that you would like to get code coverage for, using
the gem5 binary compiled with `--gcov`. The TestLib tests may be used, as well
as any gem5 simulation.

```bash
# Running from a configuration script
path/to/build/gem5.debug path/to/gem5/config.py

# Example; run this from the `gem5` directory:
build/ALL/gem5.debug configs/example/gem5_library/arm-hello.py
```

```bash
# for running from TestLib
cd tests
./main.py run gem5/{test suite to run} --build-dir=path/to/build --skip-build \
--length {quick, long, very-long} -t {threads to use for running tests}

# Example:
cd tests
./main.py run gem5/gem5_resources --build-dir=../build --skip-build \
--length=very-long -t 10 -vvv
```

  - Note that the TestLib tests use the `gem5.opt` binary, so they will fail if
  only `gem5.debug` is built.
  - `-v`, `-vv`, and `-vvv` can be passed to see more verbose output from
  TestLib.

- 4. Remove code coverage files ending with `.py.gcda` from the `build/{ISA}`
directory; these files will cause errors upon running `gcovr`.

```bash
# If the `build` directory is inside the `gem5` directory, the `cd` isn't
# necessary.
cd build/ALL
find . -name "*.py.gcda" -delete
```

- 5. Run `gcovr`. The following command includes suggested options; excluding
them may result in errors when running `gcovr`:

```bash
gcovr \
--verbose \
# There are other options for `merge-mode-functions`; you just need one of them
# to prevent gcovr from failing with an error
--merge-mode-functions separate \
# The `gcov-ignore-parse-errors` options tell gcovr to warn instead of exiting
# when encountering a suspicious or negative hit
--gcov-ignore-parse-errors=suspicious_hits.warn \
--gcov-ignore-parse-errors=negative_hits.warn \
--root /path/to/gem5 \
# {ISA} can be any gem5 isa, e.g. ALL, ARM, NULL, RISCV, X86, etc.
--object-directory /path/to/gem5/build/{ISA} \
# There are various options for how code coverage should be outputted;
# `json-summary` generates an overview of the code coverage for each file, which
# consists of line, function, and branch coverage totals and percentages.
#
# More options can be seen [here](https://gcovr.com/en/stable/output/txt.html)
--json /path/to/json/coverage.json
--json-summary /path/to/summary.json \
# This formats the code coverage report so the whitespace is more human readable
--json-summary-pretty \
# Number of threads to use
-j 10
```

```bash
# Example; run this from the `gem5` directory
gcovr \
--verbose \
--merge-mode-functions separate \
--gcov-ignore-parse-errors=suspicious_hits.warn \
--gcov-ignore-parse-errors=negative_hits.warn \
--root . \
--object-directory ./build/ALL \
--json gcovr-coverage/coverage.json \
--json-summary gcovr-coverage/summary.json \
--json-summary-pretty \
-j 20

```

- 6. Assuming you used the options above, the code coverage and code coverage
summary can be found at `/path/to/json/coverage.json` and `/path/to/summary.json`.

For the specific example above, the coverage can be found in the
`gcovr-coverage` directory.

- 7. To get separate code coverage for another test using the same gem5 build,
remove the `.gcda` files in the `build/{ISA}` directory of that build:

```bash
# If the `build` directory is inside the `gem5` directory, the `cd` isn't
# necessary.
cd build/ALL
find . -name "*.gcda" -delete
```

### 1.1. Merging gcovr code coverage files

Two code coverage files generated using `gcovr` can also be merged using
`gcovr` to obtain the combined code coverage of the tests covered by each file.
A command of the following format can be used. Note that summary code coverage
files cannot be used as inputs, as they do not include the detailed information
on which lines are covered or not.

```bash
gcovr \
-a first/code/coverage/file.json \
-a second/code/coverage/file.json \
-a third/code/coverage/file.json \
# etc.
--merge-mode-functions separate \
--json-summary path/to/summary/name.json
```

## 2. Getting code coverage using gcovr through TestLib's `--gcov` options

In addition to manually running gcovr, TestLib also has three options for
running gcov and gcovr, which automate the steps involved in running gcovr to
varying extents.

These can be used by passing the `--gcov` flag in the TestLib command line.
The options are:

- `--gcov=test-only`: This command builds the gem5 binary with the `--gcov`
option to include the flags needed to run `gcov`, and clears the `gcda` files
to avoid including the code coverage of the build process.

Next, it runs the specified TestLib tests on the binary, generating `.gcda` and
`.gcno` files. It does not run `gcovr`, and allows the user to process the code
coverage files as they wish (e.g. using a tool other than `gcovr`, or with
`gcovr` options other than the ones built into TestLib).

- `--gcov=all-test-and-gcov`: This command builds the gem5 binary with the
`--gcov` option, runs **all** of the specified TestLib tests, then runs `gcovr`
after all of the tests have finished. This option is useful for getting the
overall test coverage of a test suite or entire set of tests. This option runs
`gcovr` once.

- `--gcov=ind-test-and-gcov`: This command builds the gem5 binary with the
`--gcov` option, runs `gcovr` after **each** test, and clears the code coverage
specific to that test before starting the next one. This option is useful for
getting separate code coverage of a number of individual tests. This option runs
`gcovr` the same number of times as there are tests; as such, it is very time
consuming.

If you want to get the code coverage of a TestLib test suite, it is recommended
to use the `all-test-and-gcov` option. It is possible to use the `ind-test-and-gcov`
option and combine the results afterward (see section 1.1), but it is *very*
time-consuming to obtain code coverage for each individual test and to combine
the results.

## 3. Getting code coverage using Codecov in GitHub Actions

This method has been implemented for gem5's test workflows, located in
`.github/workflows`. Specifically, the workflow for the Weekly tests
(`.github/workflows/weekly-tests.yaml`) was modified, and a new workflow that
runs the Daily and CI tests on a weekly basis was introduced
(`.github/workflows/ci-daily-codecov.yaml`).

The changes that were made to get code coverage using Codecov were as follows:

1. Modifying the gem5 build commands to include the `--gcov` option
2. Running TestLib tests with the `--gcov=test-only` option. Although the binary
has already been built, including this option is still required, as it removes
the `gcda` files that are generated when building. These files represent the code
coverage from the build process itself, and we don't want to include it as code
coverage for tests.
3. Adding a step to uploade code coverage files to Codecov at the end of each
relevant job.

The code coverage results for the main `gem5/gem5` repository can be seen
[on the Codecov page](https://app.codecov.io/gh/gem5/gem5/tree/develop).
