gem5 Specifc RISC-V tests
=========================

About
-----

This work provides assembly testing infrastructure including single-threaded
and multi-threaded tests for RISC-V ISA in gem5. Each test targets an
individual RISC-V instruction or a Linux system call. This work targets
system call emulation (SE) mode in gem5.

This work is based on the riscv-tests project.

Link to the orignal riscv-tests projects can be found here:
  https://github.com/riscv/riscv-tests

Link to the original riscv-tests project's LICENSE and README can be found
here:
  https://github.com/riscv/riscv-tests/blob/master/LICENSE
  https://github.com/riscv/riscv-tests/blob/master/README.md

Specific commit ID that this work is based off:
  68cad7baf3ed0a4553fffd14726d24519ee1296a

Changes from the orignal riscv-tests project
--------------------------------------------

1. Only rv64 tests are imported into this work

The original project offers both rv64 and rv32 tests. Since the current
implementation of RISC-V in gem5 is focused on its 64-bit version, only
64-bit tests (rv64) are imported from the original project. Future work
on 32-bit can easily integrate all 32-bit tests into gem5.

2. New testing environment for gem5

Since the original riscv-tests project is designed for bare-metal system (i.e.,
without OS support), it offers several environments to control how a test
interacts with a host machine (to-host communication). However, in gem5 SE
mode, gem5 emulates an OS, and there is no host machine. Therefore, we
developed a new testing environment called `ps` for gem5.

This testing environment uses system call `exit` to return test results as an
exit code of a particular test instead of writing them to a host machine. This
environment requires the testing platform to implement/emulate at least `exit`
system call.

3. Minimal threading library written in assembly (`isa/macros/mt`)

To simplify debugging multi-threading systems, we developed a minimal threading
library that supports very basic threading functionality including creating a
thread, exiting a thread, waiting for some thread(s) on a condition, and waking
up some thread(s).

Multi-threaded tests can rely on this library to manage multiple threads.

4. RISC-V AMO, LR, and SC instruction tests (`isa/rv64uamt`)

This is a set of assembly tests that target multi-core systems and test AMO
instructions. This test set uses a minimal number of system calls (i.e., clone,
mmap, munmap and exit) to create and manage threads.  It does not use any
complex sleep/wakeup mechanism to manage and synchronize threads to avoid
adding extra unnecessary complexity. The major goal of this test set is to
stress AMO instructions. Threads only synchronize at the end of their
execution. The master thread does a spin-wait to wait for all threads to
complete before it checks final results.

5. Thread-related system call tests (`isa/rv64samt`)

This is a set of assembly tests that target thread-related system calls and
thread wait/wakeup behaviors. This set reuses some of the tests in
`isa/rv64uamt` but uses more advanced futex system call operations to make
threads wait and wake up in certain cases. This test set also checks functional
behaviors of threads after a wait/wakeup operation.

How to compile this test suite
------------------------------

1. Install RISC-V GNU toolchain. Source code and instruction on how to install
it can be found here: https://github.com/riscv/riscv-gnu-toolchain

2. Run `make`

3. Test binaries are in `$GEM5/tests/test-progs/asmtest/bin/riscv/` ($GEM5 is
your gem5 directory)

How to run all tests
--------------------

1. Run `./run-tests.py`

2. Test outputs are in ./test-summary.out
