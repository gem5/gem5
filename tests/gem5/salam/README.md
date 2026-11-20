<!--
Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
(University of Wisconsin-Madison)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met: redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer;
redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution;
neither the name of the copyright holders nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-->

# SALAM regression tests

Install the normal gem5 dependencies, matching Clang/LLVM, PyYAML, and the
ARM bare-metal toolchain and libraries (`gcc-arm-none-eabi`,
`libnewlib-arm-none-eabi`, `libstdc++-arm-none-eabi-newlib`). Tests build
workloads in temporary directories.

Kernel workloads keep their boot code, linker script, interrupt handler,
syscall support, and build rules in `harness/`. The fixture copies tracked
harness sources into each workload and passes the selected Clang and ARM
cross compiler to Make through `CLANG` and `CROSS_COMPILE`.

The current baseline has 103 default suites. Keep a suite for a distinct
behavior or regression rather than every combination of its parameters.

| Family | Suites | Retained coverage |
| --- | ---: | --- |
| LLVM kernels | 48 | LLVM semantics, scheduling, clocks, stats windows |
| Devices | 25 | Register/SPM/stream protocols, DMA, invalid accesses |
| Stdlib | 17 | Board wiring, BFS parity, source isolation, DMA/FIFOs |
| BFS | 3 | Checked workload, checker negative control, and build cleanup |
| Profiles | 6 | Opcodes, latency rules, ownership, device construction |
| Power reporting | 1 | Equal results and timing with reporting on/off |
| Storage | 1 | Unaligned byte-enable traffic and payload correctness |
| Stream kernel | 1 | Repeated payload roundtrip and byte accounting |
| Debug writes | 1 | Packet-sized trace dumps for short writes |

Four custom-profile suites run separately under the tags below. Focused
Python and C++ unit tests retain the lower-level validation and arithmetic
contracts.

The reduction removes repeated port/delay/width/lockstep products,
same-site diagnostic variants, default-profile inventory tables, exact
scheduler pass tables, the SPM 10/11 absorption boundary, and printed
runtime precision checks. DMA checks use explicit payload lengths and
completed endpoint accounting. Power/area diagnostic and coefficient
permutations are outside this end-to-end baseline.

Some scheduling and ordering checks still use operation timestamps from
existing traces. Broader reference workloads, targeted queued nonvolatile
alias/unresolved-address/store hazards, relaxed-volatile progress, and
invalid instruction-to-FU mapping diagnostics remain later work, along
with the planned quick/long split.

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

Keep test registration and result checks in `test_*.py`. Put new standalone
gem5 configurations in `configs/` and pass command-line arguments or paths
to JSON parameter files. Keep executable Python in those scripts so normal
formatters and syntax checks can inspect it. The host-side `profile_data.py`
helper reads profile expectations in a separate Python process.

Fixed LLVM kernels belong in `harness/kernels/*.ll.in`. The shared harness
substitutes pointer syntax and memory-map addresses when placing them in a
temporary workload. Generate only the fragments needed for a test, such as
the long control-flow chain and constant expressions; keep the surrounding
kernel in a template. Profile-data assignments, C header definitions, and
JSON/YAML parameter files are intentional generated data.
