# RISC-V Bare-Metal Unit Tests

This directory contains gem5 testlib tests for small RISC-V bare-metal
programs. Each test program should exit through RISC-V semihosting with status
0 on success and a non-zero status on failure.

The shared `configs/riscv_baremetal.py` config accepts the binary path as its
first argument, so new tests can reuse the same system setup. Add new prebuilt
binaries under `tests/test-progs/<name>/bin/riscv/baremetal/`, then add a
`(test_name, binary_path)` entry to `test.py`.
