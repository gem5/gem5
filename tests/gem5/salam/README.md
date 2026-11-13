# SALAM regression tests

Install the normal gem5 dependencies, matching Clang/LLVM, PyYAML, and the
ARM bare-metal toolchain and libraries (`gcc-arm-none-eabi`,
`libnewlib-arm-none-eabi`, `libstdc++-arm-none-eabi-newlib`). Tests build
workloads in temporary directories.

From the repository root, list the selected suites before running them:

```bash
cd tests
./main.py list gem5/salam --include-tags salam -q --suites \
    > /tmp/salam-suites.txt
./main.py run gem5/salam --include-tags salam -j 8
python3 gem5/salam/check_salam_results.py /tmp/salam-suites.txt \
    testing-results/results.xml gem5/salam
```

The result checker rejects missing suites, skipped required tests, failures,
and zero-test results. Use `--skip-build` only when the required binary
already exists and matches the selected build configuration.

Custom-profile tests use the `fu-remap`, `profile-singleton`, and
`profile-reordered` tags and rebuild the ARM binary. Run each tag in a
separate TestLib process, then rebuild the default profile before ordinary
SALAM suites. Use separate output directories when retaining their results.
