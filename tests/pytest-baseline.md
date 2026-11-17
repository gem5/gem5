# Test migration baseline

This is a point-in-time record of what the pytest migration starts from.
Recollect against the checkout and runner environment used for each cutover.

At `7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3`, the TestLib suite
inventory for an x86-64 selection was:

| Schedule | Length | Suite IDs | Source directories | Build targets |
| --- | --- | ---: | ---: | --- |
| PR CI | quick | 1,895 | 29 | ALL |
| Daily | long | 107 | 17 | ALL, NULL |
| Weekly | very-long | 21 | 5 | ALL, ARM, MIPS, POWER, RISCV, SPARC, X86 |

These counts are from a clean checkout with the x86-64 host filter. They
describe TestLib registrations, including gem5 variants and verifier groups.
Suite registration can depend on the collection host and generated files,
so repeat the comparison in the workflow container with its actual inputs.
Capture the IDs, build targets, assertions, and exclusions relevant to the
cutover. From `tests/`:

```sh
for length in quick long very-long; do
    ./main.py list gem5 --host x86_64 --length="$length" --suites -q \
        | LC_ALL=C sort > "suites-$length.txt"
    ./main.py list gem5 --host x86_64 --length="$length" --build-targets -q
done
```

Workflow policy also includes checks outside this inventory:

- [CI] runs native fast/opt unit tests, formatting, compiler and platform
  builds, and embeds PyUnit in the quick TestLib selection. Quick jobs
  consume prebuilt simulator artifacts.
- [Daily] runs debug unit tests, SystemC, SST and DRAMSys in addition to long
  simulations, and warms the ALL build cache.
- [Weekly] runs very-long tests, with coverage disabled for the
  `gem5/x86_boot_tests` execution due to a documented segfault.
- The separately dispatched [coverage] workflow runs quick and long
  simulations, instrumented native unit variants, and instrumented
  integration jobs. Adding `--gcov=test-only` to the inventory command
  produces the same suite IDs. Instrumentation is a build and run policy.
- [Compiler checks] build supported toolchain and dependency variants.
  The sanitizer matrix is explicitly disabled in that workflow.

In [one successful x86-64 CI run], the cached ALL build step took 6m 13s and
the upload took 2m 20s. The [m5_util job] downloaded the build in 1m 11s and
ran the suite in 14s. The build and test jobs started 1s and 2s after
their respective job creation timestamps. These are measurements from one
run. Repeat them with comparable inputs and cache state during the pilot.

The first port maps
`SuiteUID:tests/gem5/m5_util/test_exit.py:m5-inst-exit-riscv-se-mode-ALL-x86_64-opt`
to `tests/gem5/m5_util/m5_exit_pytest.py::test_riscv_m5_exit`. Both use
the existing configuration with RISC-V, its pinned workload, and the same
line match on simulator stdout or stderr. Keep the legacy registration
through the comparison. The dedicated pytest config opts into only ported
files during coexistence. The preview plan describes selected cases and
declared build targets; build identity and scheduling belong to later
changes. SCons GoogleTest targets and embedded PyUnit keep their existing
entry points until their adapters are available.

The native discovery probe built `build/NULL/base/bitunion.test.opt` with
SCons, enumerated its cases with `--gtest_list_tests`, and ran
`BitUnionData.NormalBitfield` with a native filter and XML output. This
shows what the adapter can enumerate once a binary is available. A
build-free native catalog is still needed.

The embedded Python probe ran `tests/run_pyunit.py --directory` on the
conversion checks with an existing simulator. It produced case names and a
status for 18 passing methods through `unittest.TextTestRunner`. The later
adapter needs a structured result channel.

[CI]: https://github.com/gem5/gem5/blob/7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3/.github/workflows/ci-tests.yaml
[Daily]: https://github.com/gem5/gem5/blob/7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3/.github/workflows/daily-tests.yaml
[Weekly]: https://github.com/gem5/gem5/blob/7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3/.github/workflows/weekly-tests.yaml
[coverage]: https://github.com/gem5/gem5/blob/7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3/.github/workflows/ci-daily-codecov.yaml
[Compiler checks]: https://github.com/gem5/gem5/blob/7bdf09a0f75f2f57857aa6303ad1c45eabb6a4b3/.github/workflows/compiler-tests.yaml
[one successful x86-64 CI run]: https://github.com/gem5/gem5/actions/runs/41409200592/job/125158706217
[m5_util job]: https://github.com/gem5/gem5/actions/runs/41409200592/job/125162431579
