# Copyright (c) 2026 Akanksha Chaudhari
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# SALAM kernel tests on the single-accelerator harness.
#
# Each suite builds harness/ around one kernel from harness/kernels,
# runs it, and checks the OUT words the host prints. Run explicitly:
#   ./main.py run gem5/salam --include-tags salam

import re

import salam_testlib as st
from testlib import *

KERNEL_WALL = 180
KERNEL_TICKS = 1000000000


def out_regex(index, value, run=0):
    return re.compile(rf"^RUN\[{run}\] OUT\[{index}\]={value} ")


def kernel_suite(
    name,
    kernel,
    inputs,
    expect,
    forbid=(),
    n_out=None,
    runs=1,
    hw_config=None,
    ir_patterns=(),
    fs_args=(),
    cache_size=None,
):
    # expect/forbid: {index: value} checked for every run.
    # fs_args: extra fs.py options for the run.
    # cache_size: give the accelerator a Cache var of that size.
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    n_out = n_out or max(list(expect) + [i for i, _ in forbid]) + 1
    harness = st.SalamHarnessFixture(
        "salam-workload",
        gem5,
        tempdir,
        kernel,
        inputs,
        n_out,
        runs=runs,
        hw_config=hw_config,
        ir_patterns=ir_patterns,
        cache_size=cache_size,
    )
    tests = [
        TestFunction(
            st.run_fs("salam-workload", KERNEL_WALL, KERNEL_TICKS, fs_args),
            name=name,
        )
    ]
    verifiers = {
        "m5-exit": verifier.MatchFileRegex(
            re.compile(st.M5_EXIT_REGEX), ["simout.txt"]
        )
    }
    for run in range(runs):
        for index, value in expect.items():
            verifiers[f"run{run}-out{index}"] = verifier.MatchFileRegex(
                out_regex(index, value, run), ["system.terminal"]
            )
        for index, value in forbid:
            verifiers[f"run{run}-not-out{index}-{value}"] = (
                st.NoMatchFileRegex(
                    out_regex(index, value, run), ["system.terminal"]
                )
            )
    tests += [
        v.instantiate_test(f"{name}-{label}") for label, v in verifiers.items()
    ]
    return TestSuite(
        name=name,
        fixtures=[gem5, tempdir, harness],
        tags=[st.SALAM_TAG],
        tests=tests,
    )


kernel_suite(
    "salam-kernel-sanity",
    "sanity.c",
    inputs=[2, 3],
    expect={0: 5},
    forbid=[(0, 6)],
    ir_patterns=[r"^\s*%\S+ = add nsw i32"],
)


def rejected_suite(name, kernel, inputs, hw_config, regex):
    # The run must stop with a diagnostic instead of executing.
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    harness = st.SalamHarnessFixture(
        "salam-workload", gem5, tempdir, kernel, inputs, 1, hw_config=hw_config
    )
    return TestSuite(
        name=name,
        fixtures=[gem5, tempdir, harness],
        tags=[st.SALAM_TAG],
        tests=[
            TestFunction(
                st.run_fs_rejected(
                    "salam-workload", KERNEL_WALL, KERNEL_TICKS, regex
                ),
                name=name,
            )
        ],
    )


# A nonzero load or store cycle count used to hang the accelerator.
rejected_suite(
    "salam-kernel-load-latency-rejected",
    "load_store.c",
    inputs=[41],
    hw_config={"instructions": {"load": {"runtime_cycles": 1}}},
    regex=r"nonzero runtime_cycles for 'load' is not supported",
)


# IN[0] = 0x1234abcd: bytes cd ab 34 12 in memory.
kernel_suite(
    "salam-kernel-narrow-load",
    "narrow_load.c",
    inputs=[0x1234ABCD],
    expect={0: 0x1234, 1: -85, 2: 0x12},
    ir_patterns=[r"load volatile i16", r"load volatile i8"],
)


kernel_suite(
    "salam-kernel-call-return",
    "call_return.c",
    inputs=[5],
    expect={0: 16, 1: 17},
    ir_patterns=[r"= (tail )?call i32 @triple_plus_one\("],
)

kernel_suite(
    "salam-kernel-call-nested",
    "call_nested.c",
    inputs=[5, 2],
    expect={0: 35, 1: 7},
    ir_patterns=[r"call i32 @g\(", r"call i32 @f\("],
)

# Phis at a block entry read their incoming values simultaneously.
PHI_PATTERN = r"= phi i32 \[ %\d+, %\d+ \], \[ %\d+, %\d+ \]"
for n, expect in ((3, {0: 2, 1: 1}), (2, {0: 1, 1: 2})):
    kernel_suite(
        f"salam-kernel-phi-swap-{n}",
        "phi_swap.c",
        inputs=[n, 1, 2],
        expect=expect,
        ir_patterns=[PHI_PATTERN],
    )

kernel_suite(
    "salam-kernel-phi-rotate",
    "phi_rotate.c",
    inputs=[4, 1, 2, 3],
    expect={0: 2, 1: 3, 2: 1},
    ir_patterns=[PHI_PATTERN],
)

# i1 values: representation and signed interpretation.
kernel_suite(
    "salam-kernel-i1-trunc",
    "i1_trunc.ll.in",
    inputs=[3],
    expect={0: 7, 1: 1},
    ir_patterns=[r"trunc i32 %v to i1", r"zext i1 %b to i32"],
)

kernel_suite(
    "salam-kernel-i1-sext",
    "i1_sext.ll.in",
    inputs=[3],
    expect={0: -1, 1: 0},
    ir_patterns=[r"sext i1 %t to i32"],
)

# Pointers in memory have the data layout's width (32 bits on ARM).
POINTER_LOAD = r"= load volatile (i32\*|ptr), (i32\*\*|ptr) "
kernel_suite(
    "salam-kernel-pointer-load",
    "pointer_load.c",
    inputs=["IN + 8", "IN + 12", 111, 222],
    expect={0: 111, 1: 222},
    ir_patterns=[POINTER_LOAD],
)

kernel_suite(
    "salam-kernel-pointer-store",
    "pointer_store.c",
    inputs=["IN + 4", 333],
    expect={0: 333},
    ir_patterns=[POINTER_LOAD, r"store volatile (i32\*|ptr) %\d+, "],
)

# A load waits for every in-flight store whose bytes it overlaps.
# Without lockstep, the load can issue while a store is in flight.
NO_LOCKSTEP = [
    "--param=system.harness_clstr.kernel.llvm_interface.lockstep_mode=False"
]
kernel_suite(
    "salam-kernel-store-twice",
    "store_twice.c",
    inputs=[1, 2],
    expect={0: 2},
    fs_args=NO_LOCKSTEP,
    ir_patterns=[
        r"store volatile i32 %\d+, (?:i32\*|ptr) (inttoptr \(i32 \d+ to "
        r"(?:i32\*|ptr)\)),.*\n.*\n\s*store volatile i32 %\d+, (?:i32\*|ptr) "
        r"\1,.*\n\s*%\d+ = load volatile i32, (?:i32\*|ptr) \1,"
    ],
)

kernel_suite(
    "salam-kernel-store-overlap",
    "store_overlap.c",
    inputs=[0x11223344],
    expect={0: 0x11223344, 1: 0x33},
    ir_patterns=[r"= load volatile i8, (i8\*|ptr) "],
)

# Memory accesses keep program order where they may overlap: a slow
# division leaves an older access's address or data pending while a
# younger access to the same word is ready.
SLOW_UDIV = {"instructions": {"udiv": {"runtime_cycles": 20}}}
UDIV = r"= udiv i32 "
for name, kernel, inputs, expect in (
    ("raw", "order_raw.c", [6, 2], {0: 3}),
    ("war", "order_war.c", [0, 1, 5], {0: 0, 1: 5}),
    ("waw", "order_waw.c", [6, 2, 5, 0], {0: 5}),
    ("unresolved", "order_unresolved.c", [0, 1, 9], {0: 9}),
):
    kernel_suite(
        f"salam-kernel-order-{name}",
        kernel,
        inputs=inputs,
        expect=expect,
        hw_config=SLOW_UDIV,
        fs_args=NO_LOCKSTEP,
        ir_patterns=[UDIV],
    )

# Calls order memory accesses between the caller and the callee.
kernel_suite(
    "salam-kernel-call-load",
    "call_load.c",
    inputs=[6, 2, 0],
    expect={0: 3},
    hw_config=SLOW_UDIV,
    fs_args=NO_LOCKSTEP,
    ir_patterns=[UDIV, r"= (tail )?call i32 @get\("],
)

kernel_suite(
    "salam-kernel-call-void",
    "call_void.c",
    inputs=[5],
    expect={0: 10, 1: 11},
    fs_args=NO_LOCKSTEP,
    ir_patterns=[r"call void @put_double\("],
)

# A Cache var connects to its accelerator, whose name ("Kernel") the
# cluster stores in lower case.
kernel_suite(
    "salam-kernel-cache",
    "sanity.c",
    inputs=[2, 3],
    expect={0: 5},
    cache_size=1024,
)
