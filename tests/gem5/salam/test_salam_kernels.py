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

import json
import os
import re
import shutil
import subprocess

import salam_testlib as st
from testlib import *

KERNEL_WALL = 180
KERNEL_TICKS = 1000000000


def out_regex(index, value, run=0):
    return re.compile(rf"^RUN\[{run}\] OUT\[{index}\]={value} ")


def check_invocation_duration(params):
    outdir, out = st.read_simout(params)
    starts = [
        tuple(map(int, values))
        for values in re.findall(
            r"launch accepted at (\d+); first compute pass at (\d+) "
            r"\(period (\d+)\)",
            out,
        )
    ]
    finishes = [
        tuple(map(int, values))
        for values in re.findall(
            r"finished at (\d+) after (\d+) cycles",
            out,
        )
    ]
    printed = [
        float(value)
        for value in re.findall(
            r"^\s*Runtime:\s+([0-9.e+-]+) us$",
            out,
            re.M,
        )
    ]
    if (
        not starts
        or len(starts) != len(finishes)
        or len(starts) != len(printed)
    ):
        raise AssertionError("missing invocation boundary or runtime report")
    frequency = st.read_stats(outdir)[-1]["simFreq"]
    records = [
        dict(
            accepted=start,
            first_pass=first,
            period=period,
            completed=end,
            passes=passes,
            elapsed_ticks=end - start,
            printed_us=us,
        )
        for (start, first, period), (end, passes), us in zip(
            starts, finishes, printed
        )
    ]
    with open(os.path.join(outdir, "invocation-duration.json"), "w") as f:
        json.dump(records, f, indent=2)
    for record in records:
        # Match the summary's six significant digits, not a tolerance.
        expected = float(f"{record['elapsed_ticks'] / (frequency / 1e6):.6g}")
        if record["printed_us"] != expected:
            raise AssertionError(
                f"elapsed invocation runtime {record['printed_us']} us != "
                f"{expected} us from accepted/completed events {record}"
            )
    ticks = [
        int(n)
        for n in re.findall(
            r"^\s*Invocation duration:\s+(\d+) ticks$",
            out,
            re.M,
        )
    ]
    passes = [
        int(n)
        for n in re.findall(
            r"^\s*Compute passes:\s+(\d+)$",
            out,
            re.M,
        )
    ]
    if ticks != [r["elapsed_ticks"] for r in records]:
        raise AssertionError("reported ticks differ from invocation events")
    if passes != [r["passes"] for r in records]:
        raise AssertionError("compute-pass count changed in the report")


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
    simout_counts=(),
    gem5_args=(),
    gem5=None,
    profile=None,
    tags=(st.SALAM_TAG,),
    clocks=None,
    ticks=KERNEL_TICKS,
    wrapper=None,
    host_options=None,
    extra_tests=(),
):
    if any("SALAMLaunch" in arg for arg in gem5_args):
        extra_tests = tuple(extra_tests) + (
            ("invocation-duration", check_invocation_duration),
        )
        tags = tuple(tags) + ("salam-invocation-duration",)
    # expect/forbid: {index: value} checked for every run.
    # fs_args, gem5_args: extra fs.py and gem5 options for the run.
    # cache_size: give the accelerator a Cache var of that size.
    # simout_counts: (label, regex, count): count matching simout lines.
    # profile: a SalamProfileFixture to write before gem5 builds it.
    gem5 = gem5 or st.SalamGem5Fixture()
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
        clocks=clocks,
        host_options=host_options,
    )
    tests = [
        TestFunction(
            st.run_fs(
                "salam-workload",
                KERNEL_WALL,
                ticks,
                fs_args,
                gem5_args,
                wrapper,
            ),
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
    for label, regex, count in simout_counts:
        verifiers[label] = st.CountFileRegex(
            re.compile(regex), ["simout.txt"], count
        )
    tests += [
        v.instantiate_test(f"{name}-{label}") for label, v in verifiers.items()
    ]
    tests += [
        TestFunction(check, name=f"{name}-{label}")
        for label, check in extra_tests
    ]
    fixtures = [gem5, tempdir, harness]
    if profile is not None:
        fixtures.insert(0, profile)
    return TestSuite(
        name=name,
        fixtures=fixtures,
        tags=list(tags),
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


def rejected_suite(
    name,
    kernel,
    inputs,
    regex,
    hw_config=None,
    fs_args=(),
    config_regex=None,
    ir_patterns=(),
    tags=(st.SALAM_TAG,),
):
    # The run must stop with a diagnostic instead of executing.
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    harness = st.SalamHarnessFixture(
        "salam-workload",
        gem5,
        tempdir,
        kernel,
        inputs,
        1,
        hw_config=hw_config,
        ir_patterns=ir_patterns,
    )
    return TestSuite(
        name=name,
        fixtures=[gem5, tempdir, harness],
        tags=list(tags),
        tests=[
            TestFunction(
                st.run_fs_rejected(
                    "salam-workload",
                    KERNEL_WALL,
                    KERNEL_TICKS,
                    regex,
                    fs_args=fs_args,
                    config_regex=config_regex,
                ),
                name=name,
            )
        ],
    )


class FloatConstantHarness(st.SalamHarnessFixture):
    def __init__(self, gem5, tempdir, operation):
        super().__init__(
            "salam-workload",
            gem5,
            tempdir,
            f"float-constant-{operation}.ll.in",
            [],
            3,
        )
        self.operation = operation

    def _place_kernel(self, tools):
        header = os.path.join(self.accpath, "harness_clstr_hw_defines.h")
        with open(header) as stream:
            defines = dict(
                re.findall(r"#define (\w+) (0x[0-9a-fA-F]+)", stream.read())
            )
        out = int(defines["OUT"], 16)
        int_ptr = "i32*" if tools.llvm_major < 15 else "ptr"
        float_ptr = "float*" if tools.llvm_major < 15 else "ptr"
        offset = -1 if self.operation == "sitofp" else 1
        # Equal 64-bit pointer/index widths make the negative GEP yield -4
        # after ptrtoint. The GEP is deliberately not inbounds.
        integer = (
            f"ptrtoint ({int_ptr} getelementptr "
            f"(i32, {int_ptr} null, i64 {offset}) to i64)"
        )
        expression = {
            "uitofp": f"uitofp (i64 {integer} to float)",
            "sitofp": f"sitofp (i64 {integer} to float)",
            "fptrunc": (
                "fptrunc (double uitofp "
                f"(i64 {integer} to double) to float)"
            ),
            "literal": "4.000000e+00",
        }[self.operation]
        ir = (
            'target datalayout = "e-p:64:64-i64:64-n32:64-S128"\n'
            'target triple = "armv7-pc-none-eabi"\n'
            "define void @top() {\n"
            f"  store volatile float {expression}, {float_ptr} "
            f"inttoptr (i64 {out} to {float_ptr})\n"
            f"  store volatile i32 305419896, {int_ptr} "
            f"inttoptr (i64 {out + 4} to {int_ptr})\n"
            f"  store volatile i32 2309737967, {int_ptr} "
            f"inttoptr (i64 {out + 8} to {int_ptr})\n"
            "  ret void\n}\n"
        )
        with open(os.path.join(self.accpath, "hw", "kernel.ll"), "w") as f:
            f.write(ir)


def check_float_constant(operation):
    def check(params):
        if (
            params.fixtures["salam-gem5"].tools.llvm_major >= 18
            and operation != "literal"
        ):
            # LLVM 18 removed these ConstantExpr forms. Check the parser
            # diagnostic rather than claiming execution coverage for them.
            st.run_fs_rejected(
                "salam-workload",
                KERNEL_WALL,
                KERNEL_TICKS,
                r"(?:uitofp|sitofp|fptrunc) constexprs are no longer supported",
            )(params)
            return
        st.run_fs("salam-workload", KERNEL_WALL, KERNEL_TICKS)(params)
        outdir, simout = st.read_simout(params)
        if not re.search(st.M5_EXIT_REGEX, simout, re.M):
            raise AssertionError("float constant kernel did not exit normally")
        with open(os.path.join(outdir, "system.terminal")) as stream:
            words = re.findall(
                r"^RUN\[0\] OUT\[(\d+)\]=-?\d+ 0x([0-9a-f]{8})$",
                stream.read(),
                re.M,
            )
        expected = [
            ("0", "c0800000" if operation == "sitofp" else "40800000"),
            ("1", "12345678"),
            ("2", "89abcdef"),
        ]
        if words != expected:
            raise AssertionError(f"float constant words {words} != {expected}")

    return check


for operation in ("uitofp", "sitofp", "fptrunc", "literal"):
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    name = f"salam-kernel-float-constant-{operation}"
    TestSuite(
        name=name,
        fixtures=[
            gem5,
            tempdir,
            FloatConstantHarness(gem5, tempdir, operation),
        ],
        tags=[st.SALAM_TAG, "salam-correctness", "salam-float-constant"],
        tests=[TestFunction(check_float_constant(operation), name=name)],
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
    tags=(st.SALAM_TAG, "salam-correctness"),
)


def check_call_handoff(latency):
    def check(params):
        outdir, out = st.read_simout(params)
        returns = list(
            map(int, re.findall(r"^(\d+): .*issue control ret$", out, re.M))
        )
        calls = list(
            map(int, re.findall(r"^(\d+): .*complete call$", out, re.M))
        )
        period = int(re.search(r"period (\d+)\)", out)[1])
        # Each run has one callee return, then its caller's return. The
        # caller processes completion on subsequent passes: one handoff
        # pass plus the configured number of call-latency passes.
        if len(returns) != 4 or len(calls) != 2:
            raise AssertionError("call/return did not complete once per run")
        for returned, completed in zip(returns[::2], calls):
            if completed - returned != (latency + 1) * period:
                raise AssertionError("call completion violated return handoff")
        stats = st.read_stats(outdir)[-1]
        for field in ("issued::call", "completed::call"):
            if (
                stats["system.harness_clstr.kernel.llvm_interface." + field]
                != 2
            ):
                raise AssertionError("call accounting differs from trace")

    return check


for call_latency in (0, 1, 3):
    for call_lockstep in (False, True):
        kernel_suite(
            f"salam-kernel-call-handoff-{call_latency}-{call_lockstep}",
            "call_return.c",
            inputs=[5],
            expect={0: 16, 1: 17},
            runs=2,
            hw_config={
                "instructions": {"call": {"runtime_cycles": call_latency}}
            },
            fs_args=[
                "--param=system.harness_clstr.kernel.llvm_interface."
                f"lockstep_mode={call_lockstep}",
            ],
            gem5_args=["--debug-flags=SALAMIssue,SALAMLaunch"],
            extra_tests=[("return-handoff", check_call_handoff(call_latency))],
            tags=(st.SALAM_TAG, "salam-correctness"),
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

# An instruction whose functional unit the profile does not define is
# rejected when the IR is read. The mapping comes from the generated
# InstConfig parameters; override one so it names enum 99.
INST_CONFIG = "system.harness_clstr.kernel.hw_interface.inst_config"
rejected_suite(
    "salam-kernel-unknown-functional-unit",
    "phi_swap.c",
    inputs=[3, 1, 2],
    fs_args=[f"--param={INST_CONFIG}.icmp.functional_unit=99"],
    config_regex=(
        r"^\[" + re.escape(INST_CONFIG) + r"\.icmp\]\n(?:\w+=.*\n)*?"
        r"functional_unit=99$"
    ),
    regex=r"instruction 'icmp' uses functional unit 99, which the "
    r"hardware profile does not define",
    ir_patterns=[r"= icmp "],
)

# Functional-unit instances. With limit 0 a unit has one instance per
# static instruction that uses it in top and its direct callees: the
# twice-called helper() counts once, the uncalled unused() not at all.
# Every invocation resolves the same inventory.
FU_UNITS = "system.harness_clstr.kernel.hw_interface.functional_units"
INVENTORY_IR = [
    r"call i32 @helper\(.*\n.*call i32 @helper\(",
    r"define dso_local (?:noundef )?i32 @unused\(.*\n.*= shl ",
]


def inventory(alias, enum, instances, uses, runs):
    return (
        f"inventory-{alias}",
        rf"llvm_interface: {alias} \({enum}\): {instances} instances, "
        rf"{uses} static instructions$",
        runs,
    )


kernel_suite(
    "salam-kernel-fu-inventory",
    "fu_inventory.c",
    inputs=[9, 3],
    expect={0: 6, 1: 10},
    runs=3,
    gem5_args=["--debug-flags=SALAMFUInventory"],
    ir_patterns=INVENTORY_IR,
    simout_counts=[
        inventory("integer_adder", 1, 2, 2, 3),
        inventory("integer_multiplier", 2, 1, 1, 3),
        inventory("bitwise_operations", 4, 1, 1, 3),
        inventory("bit_shifter", 3, 0, 0, 3),
        inventory("float_adder", 5, 0, 0, 3),
    ],
)

# A nonzero limit is the instance count.
kernel_suite(
    "salam-kernel-fu-limit",
    "fu_inventory.c",
    inputs=[9, 3],
    expect={0: 6, 1: 10},
    gem5_args=["--debug-flags=SALAMFUInventory"],
    fs_args=[f"--param={FU_UNITS}.integer_adder.limit=5"],
    ir_patterns=INVENTORY_IR,
    simout_counts=[inventory("integer_adder", 1, 5, 2, 1)],
)


def multiply_schedule(latency, issue_cycles, runs):
    def check(params):
        outdir, out = st.read_simout(params)
        starts = [
            tuple(map(int, values))
            for values in re.findall(
                r"launch accepted at (\d+); first compute pass at (\d+) "
                r"\(period (\d+)\)",
                out,
            )
        ]
        if len(starts) != runs:
            raise AssertionError(f"expected {runs} traced invocations")
        events = []
        for position, line in enumerate(out.splitlines()):
            match = re.match(r"^\s*(\d+): .*\.llvm_interface: (.*)$", line)
            if match:
                events.append((position, int(match[1]), match[2]))
        records = []
        for run, (start, first, period) in enumerate(starts):
            end = starts[run + 1][0] if run + 1 < runs else float("inf")
            invocation = [e for e in events if start <= e[1] < end]
            issues = [
                t for _, t, text in invocation if text == "issue compute mul"
            ]
            completes = [
                t for _, t, text in invocation if text == "complete compute"
            ]
            expected_issues = [
                start + cycle * period for cycle in issue_cycles
            ]
            expected_completes = [
                t + latency * period for t in expected_issues
            ]
            if (
                first != start
                or issues != expected_issues
                or completes != expected_completes
            ):
                raise AssertionError(
                    f"multiply timing: issues={issues}, completes={completes}, "
                    f"expected issues={expected_issues}, "
                    f"expected completions={expected_completes}"
                )
            stores = []
            memory_completes = {}
            for position, tick, text in invocation:
                match = re.fullmatch(
                    r"memory issue store uid (\d+) address (0x[0-9a-f]+) "
                    r"volatile (\d)",
                    text,
                )
                if match:
                    stores.append(
                        (
                            position,
                            tick,
                            int(match[1]),
                            int(match[2], 16),
                            int(match[3]),
                        )
                    )
                match = re.fullmatch(r"memory complete uid (\d+)", text)
                if match:
                    memory_completes[int(match[1])] = (position, tick)
            if len(stores) != 3:
                raise AssertionError(f"expected three output stores: {stores}")
            previous_completion = -1
            previous_tick = start
            observed = []
            for index, store in enumerate(stores):
                position, tick, uid, address, volatile = store
                completion = memory_completes.get(uid)
                expected_issue = max(expected_completes[index], previous_tick)
                if (
                    (tick, address, volatile)
                    != (expected_issue, 0x2F000100 + 4 * index, 1)
                    or completion is None
                    or completion[1] != expected_issue + period
                    or completion[0] <= position
                    or position <= previous_completion
                ):
                    raise AssertionError(
                        f"volatile store {index}: issue={store}, "
                        f"completion={completion}, expected tick={expected_issue}"
                    )
                previous_completion, previous_tick = completion
                observed.append(
                    dict(
                        uid=uid,
                        address=address,
                        issued=tick,
                        completed=completion[1],
                    )
                )
            records.append(
                dict(
                    latency=latency,
                    period=period,
                    issues=issues,
                    completes=completes,
                    stores=observed,
                )
            )
        with open(os.path.join(outdir, "multiply-schedule.json"), "w") as f:
            json.dump(records, f, indent=2)

    return check


def zero_latency_accounting(instances, operations):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        prefix = "system.harness_clstr.kernel.hw_interface.integer_multiplier."
        expected = {
            "instances": instances,
            "admissions": operations,
            "inFlight": 0,
            "busyInstances": 0,
            "utilization": 0,
        }
        found = {name: stats[prefix + name] for name in expected}
        if found != expected:
            raise AssertionError(f"zero-latency FU accounting {found}")

    return check


# Three independent multiplies are ready together. Each physical instance
# admits at most once per compute edge, including zero-latency operations.
# Volatile output stores complete in order, one per later pass. The trace
# checks individual issues and completions because the store tail can hide
# admission differences in the final invocation duration.
MULTIPLIER = FU_UNITS + ".integer_multiplier"
CYCLE_COUNTS = "system.harness_clstr.kernel.hw_interface.cycle_counts"
LOCKSTEP = "system.harness_clstr.kernel.llvm_interface.lockstep_mode"
for name, latency, limit, interval, lockstep, runtime, runs, issues in (
    ("l4-three", 4, 3, None, False, 8, 1, (0, 0, 0)),
    ("l4-one", 4, 1, None, False, 14, 3, (0, 4, 8)),
    ("l4-ii4", 4, 1, 4, False, 14, 1, (0, 4, 8)),
    ("l4-ii1", 4, 1, 1, False, 8, 1, (0, 1, 2)),
    ("l4-ii1-lockstep", 4, 1, 1, True, 14, 1, (0, 4, 8)),
    ("l1-ii5", 1, 1, 5, False, 13, 1, (0, 5, 10)),
    ("l0-one", 0, 1, None, False, 4, 3, (0, 1, 2)),
    ("l0-one-lockstep", 0, 1, None, True, 4, 1, (0, 1, 2)),
    ("l0-two", 0, 2, None, False, 4, 1, (0, 0, 1)),
    ("l0-three", 0, 3, None, False, 4, 1, (0, 0, 0)),
    ("l0-ii5", 0, 1, 5, False, 12, 1, (0, 5, 10)),
):
    fs_args = [
        f"--param={LOCKSTEP}={lockstep}",
        f"--param={CYCLE_COUNTS}.mul={latency}",
        f"--param={MULTIPLIER}.limit={limit}",
    ]
    if interval is not None:
        fs_args.append(f"--param={MULTIPLIER}.initiation_interval={interval}")
    checks = [("multiply-schedule", multiply_schedule(latency, issues, runs))]
    if latency == 0:
        checks.append(
            ("unit-accounting", zero_latency_accounting(limit, 3 * runs))
        )
    kernel_suite(
        f"salam-kernel-fu-sched-{name}",
        "fu_sched.c",
        inputs=[3, 4, 5],
        expect={0: 9, 1: 16, 2: 25},
        runs=runs,
        fs_args=fs_args,
        ir_patterns=[r"(?s)(= mul i32 .*){3}"],
        simout_counts=[
            ("runtime", rf"^\s*Compute passes:\s+{runtime}$", runs)
        ],
        gem5_args=["--debug-flags=SALAMIssue,SALAMLaunch"],
        extra_tests=checks,
        tags=(st.SALAM_TAG, "salam-correctness"),
    )

# An explicitly unbound operation has no physical-instance admission limit.
kernel_suite(
    "salam-kernel-fu-sched-unbound",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=[
        f"--param={LOCKSTEP}=False",
        f"--param={CYCLE_COUNTS}.mul=0",
        f"--param={INST_CONFIG}.mul.functional_unit=0",
        f"--param={MULTIPLIER}.limit=1",
    ],
    ir_patterns=[r"(?s)(= mul i32 .*){3}"],
    gem5_args=["--debug-flags=SALAMIssue,SALAMLaunch"],
    extra_tests=[("multiply-schedule", multiply_schedule(0, (0, 0, 0), 1))],
    tags=(st.SALAM_TAG, "salam-correctness"),
)


def nested_zero_admission(params):
    _outdir, out = st.read_simout(params)
    period = int(
        re.search(r"first compute pass at \d+ \(period (\d+)\)", out)[1]
    )
    issues = [
        int(tick)
        for tick in re.findall(
            r"^\s*(\d+): .*\.llvm_interface: issue compute (?:add|mul)$",
            out,
            re.M,
        )
    ]
    completes = [
        int(tick)
        for tick in re.findall(
            r"^\s*(\d+): .*\.llvm_interface: complete compute$", out, re.M
        )
    ]
    # Three calls to f issue one mul and one add each; g adds its second
    # argument and then the two results. All eight use the same instance.
    if (
        len(issues) != 8
        or completes != issues
        or any(b - a < period for a, b in zip(issues, issues[1:]))
    ):
        raise AssertionError(
            f"nested zero-latency admission {issues}, {completes}"
        )
    zero_latency_accounting(1, 8)(params)


for lockstep in (False, True):
    kernel_suite(
        f"salam-kernel-fu-sched-nested-zero-{lockstep}",
        "call_nested.c",
        inputs=[5, 2],
        expect={0: 35, 1: 7},
        fs_args=[
            f"--param={LOCKSTEP}={lockstep}",
            f"--param={CYCLE_COUNTS}.mul=0",
            f"--param={CYCLE_COUNTS}.add=0",
            f"--param={INST_CONFIG}.add.functional_unit=2",
            f"--param={MULTIPLIER}.limit=1",
        ],
        ir_patterns=[r"call i32 @g\(", r"call i32 @f\("],
        gem5_args=["--debug-flags=SALAMIssue,SALAMLaunch"],
        extra_tests=[("shared-unit", nested_zero_admission)],
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


# Latency comes from CycleCounts (and workload overrides) only. The
# profile's descriptive fields do not change it: with all of them set
# to 9, the multiplies still take the default 1 cycle.
kernel_suite(
    "salam-kernel-fu-descriptive-fields",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=[
        f"--param={MULTIPLIER}.cycles=9",
        f"--param={MULTIPLIER}.stages=9",
        f"--param={INST_CONFIG}.mul.runtime_cycles=9",
        f"--param={INST_CONFIG}.mul.functional_unit_limit=9",
    ],
    ir_patterns=[r"(?s)(= mul i32 .*){3}"],
    simout_counts=[("runtime", r"^\s*Compute passes:\s+5$", 1)],
    gem5_args=["--debug-flags=SALAMIssue,SALAMLaunch"],
    extra_tests=[("multiply-schedule", multiply_schedule(1, (0, 0, 0), 1))],
    tags=(st.SALAM_TAG, "salam-correctness"),
)

# A remap profile moves fsub from double_adder to the float_trig_sine
# example unit. It is built from a copy of the default profile, so it
# replaces the default build: run its tag on its own,
#   ./main.py run gem5/salam --include-tags fu-remap
# and rebuild the default profile afterwards.
FSUB_INPUTS = [0x40B00000, 0x40100000]  # 5.5, 2.25
FSUB_OUT = 0x40500000  # 3.25
FSUB_IR = [r"= fsub float "]
REMAP_TAG = "fu-remap"

kernel_suite(
    "salam-kernel-fsub",
    "fsub.c",
    inputs=FSUB_INPUTS,
    expect={0: FSUB_OUT},
    gem5_args=["--debug-flags=SALAMFUInventory"],
    ir_patterns=FSUB_IR,
    simout_counts=[inventory("double_adder", 6, 1, 1, 1)],
)


def remap_fsub(profile):
    import yaml

    units = os.path.join(profile, "functional_units")
    sine = os.path.join(units, "float_trig_sine")
    shutil.copytree(
        os.path.join(
            config.base_dir, "src/salam/hw_profiles/examples/float_trig_sine"
        ),
        sine,
    )
    for alias, change in (
        ("double_adder", lambda names: [n for n in names if n != "fsub"]),
        ("float_trig_sine", lambda names: ["fsub"]),
    ):
        path = os.path.join(units, alias, alias + ".yml")
        with open(path) as f:
            document = yaml.safe_load(f)
        params = document["functional_unit"]["parameters"]
        params["instructions"] = change(params["instructions"])
        with open(path, "w") as f:
            yaml.safe_dump(document, f, sort_keys=False)
    path = os.path.join(profile, "salam-hw-config.yml")
    with open(path) as f:
        document = yaml.safe_load(f)
    document["functional_unit_order"].append("float_trig_sine")
    with open(path, "w") as f:
        yaml.safe_dump(document, f, sort_keys=False)


REMAP_CONFIG = os.path.join(
    config.build_dir, "salam-fu-remap-profile", "salam-hw-config.yml"
)
kernel_suite(
    "salam-kernel-fsub-remap",
    "fsub.c",
    inputs=FSUB_INPUTS,
    expect={0: FSUB_OUT},
    gem5_args=["--debug-flags=SALAMFUInventory"],
    ir_patterns=FSUB_IR,
    gem5=st.SalamGem5Fixture(REMAP_CONFIG),
    profile=st.SalamProfileFixture(REMAP_CONFIG, remap_fsub),
    tags=(REMAP_TAG,),
    simout_counts=[
        inventory("float_trig_sine", 17, 1, 1, 1),
        inventory("double_adder", 6, 0, 0, 1),
    ],
)

# Launch timing. The run bit is checked on the next edge of the
# cluster's compute clock (100 MHz by default) and the first compute
# pass runs on that edge, for every invocation. At 50 MHz the edges are
# 20 ns apart and the kernel takes the same number of cycles.
COMPUTE_CLOCK = "system.harness_clstr.compute_clk_domain.clock"
for name, fs_args, period, edge in (
    ("launch-100mhz", [], 10000, r"\d*0000"),
    (
        "launch-50mhz",
        [f"--param={COMPUTE_CLOCK}='50MHz'"],
        20000,
        r"\d*[02468]0000",
    ),
):
    kernel_suite(
        f"salam-kernel-{name}",
        "sanity.c",
        inputs=[2, 3],
        expect={0: 5},
        runs=2,
        fs_args=fs_args,
        gem5_args=["--debug-flags=SALAMLaunch"],
        simout_counts=[
            ("checked-on-edge", rf"run bit written; checked at {edge}$", 2),
            (
                "first-pass-on-acceptance",
                rf"launch accepted at ({edge}); first compute pass at \1 "
                rf"\(period {period}\)$",
                2,
            ),
            ("cycles", r"^\s*Compute passes:\s+3$", 2),
        ],
    )

# CommInterface service passes. A scratchpad load issued while the
# CommInterface is idle is serviced process_cycles compute cycles
# later, answered 2 ns after that and seen by the next compute pass;
# the add and the register-bank store take 3 more cycles: 4 + n cycles.
COMM = "system.harness_clstr.kernel"
for cycles in (1, 2, 4):
    kernel_suite(
        f"salam-kernel-service-{cycles}",
        "spm_load.c",
        inputs=[0],
        expect={0: 1},
        fs_args=[f"--param={COMM}.process_cycles={cycles}"],
        ir_patterns=[r"= load volatile i32, (i32\*|ptr) inttoptr"],
        simout_counts=[
            ("runtime", rf"^\s*Compute passes:\s+{4 + cycles}$", 1)
        ],
    )

# Scratchpad latency variation. Each scratchpad keeps one generator,
# seeded by gem5, and draws once per serviced access: the extra cycles
# are the default-seeded generator's successive values (4, 0, 2 over
# [0, 5]; see latency_sample.test). A slow scratchpad rejects the second
# ordinary load once; the rejection draws nothing, and a second invocation
# continues the sequence. No variation, no draws.
SPM = "system.harness_clstr.mem"
SLOW_SPM = f"--param={SPM}.bandwidth='200MiB/s'"
SAMPLE = r"mem: latency sample \d+ \(extra \d+\) cycles$"
REJECTED = r"mem: port 1 busy: request rejected$"
# Require exactly two ordinary scalar loads: volatile loads serialize and
# vector loads change both the packet count and the sampler sequence.
SPM_LOADS_IR = [
    r"(?ms)\A(?!(?:.*?^\s*%\S+ = load ){3})"
    r"(?:.*?^\s*%\S+ = load i32, (?:i32\*|ptr) inttoptr){2}"
]
for name, variation, runs, counts in (
    (
        "spm-latency-var",
        5,
        1,
        [
            ("samples", SAMPLE, 2),
            ("first", r"latency sample 6 \(extra 4\) cycles$", 1),
            ("second", r"latency sample 2 \(extra 0\) cycles$", 1),
            ("rejected", REJECTED, 1),
        ],
    ),
    (
        "spm-latency-var-runs",
        5,
        2,
        [
            ("samples", SAMPLE, 4),
            ("third", r"latency sample 4 \(extra 2\) cycles$", 1),
        ],
    ),
    (
        "spm-latency-fixed",
        0,
        1,
        [("samples", SAMPLE, 0), ("rejected", REJECTED, 1)],
    ),
):
    kernel_suite(
        f"salam-kernel-{name}",
        "spm_two_loads.c",
        inputs=[0],
        expect={0: 0},
        runs=runs,
        fs_args=[f"--param={SPM}.access_cycles_var={variation}", SLOW_SPM],
        gem5_args=["--debug-flags=SALAMScratchpad"],
        ir_patterns=SPM_LOADS_IR,
        simout_counts=counts,
    )

# Scratchpad access time. The response is ready access_cycles fabric
# cycles (1 ns) after the request arrives on a compute edge, and the next
# compute pass (10 ns later) sees it: up to 10 cycles costs nothing more
# than the default 2, 11 costs one compute cycle.
for access, runtime in ((2, 5), (10, 5), (11, 6)):
    kernel_suite(
        f"salam-kernel-spm-access-{access}",
        "spm_load.c",
        inputs=[0],
        expect={0: 1},
        fs_args=[f"--param={SPM}.access_cycles={access}"],
        simout_counts=[("runtime", rf"^\s*Compute passes:\s+{runtime}$", 1)],
    )

# A register-bank store completes only after its batch becomes visible.
# A dependent load must read the stored value with a shorter, equal, or
# longer visibility delay than the ten-cycle minimum acknowledgment.
REGS = "system.harness_clstr.out"
for name, write_cycles, value in (
    ("regbank-visible-early", 5, 9),
    ("regbank-visible", 10, 9),
    ("regbank-visible-late", 30, 9),
):
    kernel_suite(
        f"salam-kernel-{name}",
        "regbank_rw.c",
        inputs=[9],
        expect={0: value},
        fs_args=[f"--param={REGS}.write_cycles={write_cycles}"],
        ir_patterns=[r"store volatile i32 %\d+, (i32\*|ptr) inttoptr"],
    )

# Cluster clocks from the workload's Clocks item. A controlled compute
# interval, one multiplier taking three 4-cycle multiplies one after
# another, takes the same 14 cycles at any compute clock: 13 periods
# from the first pass to the last, 130000 ticks at 100 MHz, twice that
# at 50 MHz, and ten times as many 0.1 ps ticks at a 10 THz tick
# resolution.
ONE_MULTIPLIER = [
    f"--param={LOCKSTEP}=False",
    f"--param={CYCLE_COUNTS}.mul=4",
    f"--param={MULTIPLIER}.limit=1",
]
for name, clocks, ticks, wrapper, limit in (
    ("clocks-compute-100mhz", {"Compute": "100MHz"}, 130000, None, None),
    ("clocks-compute-50mhz", {"Compute": "50MHz"}, 260000, None, None),
    (
        "clocks-tick-resolution",
        None,
        1300000,
        os.path.join(os.path.dirname(__file__), "tick_resolution.py"),
        100000000000,
    ),
):
    kernel_suite(
        f"salam-kernel-{name}",
        "fu_sched.c",
        inputs=[3, 4, 5],
        expect={0: 9, 1: 16, 2: 25},
        fs_args=ONE_MULTIPLIER,
        gem5_args=["--debug-flags=SALAMLaunch"],
        clocks=clocks,
        wrapper=wrapper,
        ticks=limit or KERNEL_TICKS,
        simout_counts=[
            (
                "elapsed",
                rf"finished at \d+ after 14 cycles \({ticks} ticks\)$",
                1,
            )
        ],
    )

# A slower fabric clock lengthens the idle scratchpad load: at 100 MHz
# the scratchpad's 2 cycles take 20 ns (one more compute cycle) and the
# register bank's 10-cycle write acknowledgement takes 100 ns (nine
# more): 5 + 1 + 9 cycles.
kernel_suite(
    "salam-kernel-clocks-fabric-100mhz",
    "spm_load.c",
    inputs=[0],
    expect={0: 1},
    clocks={"Fabric": "100MHz"},
    simout_counts=[("runtime", r"^\s*Compute passes:\s+15$", 1)],
)


# Execution statistics report the current statistics window. A reset
# clears them and changes nothing in the accelerator: the kernel takes
# the same cycles, and printResults (per-invocation totals) still shows
# all of them.
LLVM_STATS = "system.harness_clstr.kernel.llvm_interface."


def check_reset_during_compute(params):
    # The reset lands while the one-multiplier kernel runs. Passes run
    # on edges first + k * period; those at or before the reset tick are
    # outside the window (the reset event runs after a pass on the same
    # tick).
    outdir, out = st.read_simout(params)
    first, period = map(
        int,
        re.search(
            r"first compute pass at (\d+) \(period (\d+)\)", out
        ).groups(),
    )
    reset = int(re.search(r"^(\d+): .*resetstats", out, re.M).group(1))
    total = int(re.search(r"after (\d+) cycles", out).group(1))
    before = sum(1 for k in range(total) if first + k * period <= reset)
    stats = st.read_stats(outdir)[-1]
    if not 0 < before < total:
        raise AssertionError(f"reset at {reset} is not during compute")
    if stats[LLVM_STATS + "cycles"] != total - before:
        raise AssertionError(
            f"cycles {stats[LLVM_STATS + 'cycles']}, expected {total - before}"
        )
    if stats[LLVM_STATS + "invocations"] != 0:
        raise AssertionError("the invocation started before the reset")


def check_reset_with_load_outstanding(params):
    # A slow scratchpad load is in flight when the reset lands: it was
    # issued before the window and completes, once, inside it.
    outdir, out = st.read_simout(params)
    stats = st.read_stats(outdir)[-1]
    issued = stats[LLVM_STATS + "issued::load"]
    completed = stats[LLVM_STATS + "completed::load"]
    if (issued, completed) != (0, 1):
        raise AssertionError(f"loads issued {issued}, completed {completed}")
    # The memory interface counts its response, and its bytes, once.
    comm = "system.harness_clstr.kernel."
    found = tuple(
        stats[comm + k]
        for k in ("requests::read", "responses::read", "bytes::read")
    )
    if found != (0, 1, 4):
        raise AssertionError(f"read requests/responses/bytes {found}")


def check_repeated_dumps(params):
    # Dumps with no activity between them report the same counts. A
    # time-weighted average covers the whole window, so with nothing in
    # flight its value times the window length stays the same.
    outdir, _out = st.read_simout(params)
    dumps = st.read_stats(outdir)
    if len(dumps) != 3:
        raise AssertionError(f"{len(dumps)} dumps, not 3")
    averages = ("reservedOps", "computeOps", "readOps", "writeOps")
    first = dumps[0]
    for dump in dumps[1:]:
        for key, value in first.items():
            # power_state residency is gem5's time accounting, not ours.
            if not key.startswith(LLVM_STATS) or ".power_state." in key:
                continue
            if key[len(LLVM_STATS) :] in averages:
                # Within the last printed digit (stats print 6 decimals).
                expected = value * first["simTicks"] / dump["simTicks"]
                if abs(dump[key] - expected) > 1e-6:
                    raise AssertionError(
                        f"{key}: {dump[key]}, expected {expected}"
                    )
            elif dump[key] != value:
                raise AssertionError(f"{key}: {value} then {dump[key]}")


# Three 20-cycle multiplies on one multiplier, 2 + 3 * 20 cycles, so
# the host's reset, issued after its run-bit write is acknowledged, lands
# while they run.
kernel_suite(
    "salam-kernel-stats-reset-compute",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=[
        f"--param={LOCKSTEP}=False",
        f"--param={CYCLE_COUNTS}.mul=20",
        f"--param={MULTIPLIER}.limit=1",
    ],
    gem5_args=["--debug-flags=SALAMLaunch,PseudoInst"],
    host_options={"STATS_RESET_AFTER_START": 1},
    simout_counts=[("runtime", r"^\s*Compute passes:\s+62$", 1)],
    extra_tests=[("window", check_reset_during_compute)],
)

kernel_suite(
    "salam-kernel-stats-reset-load",
    "spm_load.c",
    inputs=[0],
    expect={0: 1},
    fs_args=[f"--param={SPM}.access_cycles=150"],
    host_options={"STATS_RESET_AFTER_START": 1},
    extra_tests=[("load", check_reset_with_load_outstanding)],
)

kernel_suite(
    "salam-kernel-stats-dumps",
    "sanity.c",
    inputs=[2, 3],
    expect={0: 5},
    host_options={"STATS_DUMPS_AT_END": 2},
    extra_tests=[("dumps", check_repeated_dumps)],
)


# Cycle categories, derived pass by pass. Three 4-cycle multiplies:
# with three multipliers, pass 1 issues the loads and multiplies,
# passes 2-4 wait for them, passes 5-7 each issue one volatile store,
# and pass 8 returns after the final store completes: five issuing
# passes and three compute waits. With one multiplier the waiting
# multiplies are denied it in passes 2-4 and 6-8, and the last one
# computes alone in 10-12. An
# idle scratchpad load waits for memory while it is in flight: one
# pass at the default 2 cycles, 15 passes when the scratchpad takes 150.
def categories(expected):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        found = {
            name: stats[LLVM_STATS + "cycleCategories::" + name]
            for name in expected
        }
        if found != expected:
            raise AssertionError(f"cycle categories {found}, not {expected}")

    return check


ONE_PASS = dict(fuStall=0, memoryWait=0, computeWait=0, other=0)
for name, kernel, fs_args, expected in (
    (
        "three-multipliers",
        "fu_sched.c",
        [
            f"--param={LOCKSTEP}=False",
            f"--param={CYCLE_COUNTS}.mul=4",
            f"--param={MULTIPLIER}.limit=3",
        ],
        dict(ONE_PASS, issue=5, computeWait=3),
    ),
    (
        "one-multiplier",
        "fu_sched.c",
        ONE_MULTIPLIER,
        dict(ONE_PASS, issue=5, fuStall=6, computeWait=3),
    ),
    ("spm", "spm_load.c", [], dict(ONE_PASS, issue=4, memoryWait=1)),
    (
        "slow-spm",
        "spm_load.c",
        [f"--param={SPM}.access_cycles=150"],
        dict(ONE_PASS, issue=4, memoryWait=15),
    ),
):
    kernel_suite(
        f"salam-kernel-categories-{name}",
        kernel,
        inputs=[3, 4, 5] if kernel == "fu_sched.c" else [0],
        expect={0: 9, 1: 16, 2: 25} if kernel == "fu_sched.c" else {0: 1},
        fs_args=fs_args,
        extra_tests=[("categories", categories(expected))],
        tags=(
            (st.SALAM_TAG, "salam-correctness")
            if name == "three-multipliers"
            else (st.SALAM_TAG,)
        ),
    )


# Functional-unit statistics. Three 4-cycle multiplies on one
# multiplier keep operations in flight for 3 * 4 cycles (120000 ticks)
# either way. The instance is busy for those 12 cycles when it is not
# pipelined, but only from the first admission to the last completion,
# 6 cycles, when it admits one per cycle. A reset while 20-cycle
# multiplies run leaves the instance count alone and counts the two
# admissions and the busy time after it.
MULTIPLIER_STATS = (
    "system.harness_clstr.kernel.hw_interface.integer_multiplier."
)


def unit_stats(admissions, denials, in_flight, busy, start_tick=None):
    def check(params):
        outdir, out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        window = stats["simTicks"]
        expected = {"instances": 1, "admissions": admissions}
        if denials is not None:
            expected["denials"] = denials
        found = {k: stats[MULTIPLIER_STATS + k] for k in expected}
        if found != expected:
            raise AssertionError(f"unit counts {found}, not {expected}")
        busy_area = busy
        if start_tick is not None:
            reset = int(re.search(r"^(\d+): .*resetstats", out, re.M).group(1))
            busy_area = busy - reset
        # Averages are printed with 6 decimals.
        slack = 1e-6 * window
        for name, area in (
            ("inFlight", in_flight),
            ("busyInstances", busy_area),
        ):
            if (
                area is not None
                and abs(stats[MULTIPLIER_STATS + name] * window - area) > slack
            ):
                raise AssertionError(
                    f"{name} area {stats[MULTIPLIER_STATS + name] * window}, "
                    f"expected {area}"
                )
        utilization = stats[MULTIPLIER_STATS + "utilization"]
        if abs(utilization - stats[MULTIPLIER_STATS + "busyInstances"]) > 1e-6:
            raise AssertionError(f"utilization {utilization}")

    return check


for name, interval, denials in (
    ("pipelined", 1, 3),
    ("unpipelined", None, 12),
):
    fs_args = [
        f"--param={LOCKSTEP}=False",
        f"--param={CYCLE_COUNTS}.mul=4",
        f"--param={MULTIPLIER}.limit=1",
    ]
    if interval:
        fs_args.append(f"--param={MULTIPLIER}.initiation_interval={interval}")
    kernel_suite(
        f"salam-kernel-fu-stats-{name}",
        "fu_sched.c",
        inputs=[3, 4, 5],
        expect={0: 9, 1: 16, 2: 25},
        fs_args=fs_args,
        extra_tests=[
            (
                "unit",
                unit_stats(3, denials, 120000, 60000 if interval else 120000),
            )
        ],
    )


def busy_until_last_completion(params):
    # The 20-cycle multiplies end 60 compute cycles after the first pass.
    _outdir, out = st.read_simout(params)
    first = int(re.search(r"first compute pass at (\d+)", out).group(1))
    return first + 60 * 10000


def reset_unit_check(params):
    end = busy_until_last_completion(params)
    unit_stats(2, None, None, end, start_tick=True)(params)


kernel_suite(
    "salam-kernel-fu-stats-reset",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=[
        f"--param={LOCKSTEP}=False",
        f"--param={CYCLE_COUNTS}.mul=20",
        f"--param={MULTIPLIER}.limit=1",
    ],
    gem5_args=["--debug-flags=SALAMLaunch,PseudoInst"],
    host_options={"STATS_RESET_AFTER_START": 1},
    extra_tests=[("unit", reset_unit_check)],
)


# Memory-interface statistics, counted by hand. A request is one load
# or store; attempts include a packet offered again after a rejection;
# accepted packets, responses and bytes are counted once each. Register
# reads complete during the send and are still counted once.
COMM_STATS = "system.harness_clstr.kernel."


def comm_stats(expected):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        found = {k: stats[COMM_STATS + k] for k in expected}
        if found != expected:
            raise AssertionError(f"memory interface {found}, not {expected}")

    return check


def directions(**counts):
    # requests=(reads, writes), ... -> {"requests::read": reads, ...}
    out = {}
    for name, (reads, writes) in counts.items():
        out[f"{name}::read"] = reads
        out[f"{name}::write"] = writes
    return out


for name, kernel, inputs, expect, fs_args, runs, counts in (
    (
        "spm",
        "spm_load.c",
        [0],
        {0: 1},
        [],
        1,
        directions(
            requests=(1, 1),
            attempts=(1, 1),
            accepted=(1, 1),
            responses=(1, 1),
            bytes=(4, 4),
        ),
    ),
    (
        "rejected",
        "spm_two_loads.c",
        [0],
        {0: 0},
        [SLOW_SPM],
        1,
        directions(
            requests=(2, 1),
            attempts=(3, 1),
            accepted=(2, 1),
            responses=(2, 1),
            bytes=(8, 4),
        ),
    ),
    (
        "registers",
        "fu_sched.c",
        [3, 4, 5],
        {0: 9, 1: 16, 2: 25},
        [],
        1,
        directions(
            requests=(3, 3),
            attempts=(3, 3),
            accepted=(3, 3),
            responses=(3, 3),
            bytes=(12, 12),
        ),
    ),
    ("runs", "sanity.c", [2, 3], {0: 5}, [], 2, {"starts": 2, "finishes": 2}),
):
    if "starts" not in counts:
        counts = dict(counts, starts=1, finishes=1)
    kernel_suite(
        f"salam-kernel-comm-stats-{name}",
        kernel,
        inputs=inputs,
        expect=expect,
        runs=runs,
        fs_args=fs_args,
        ir_patterns=SPM_LOADS_IR if name == "rejected" else (),
        extra_tests=[("memory-interface", comm_stats(counts))],
    )


# The end-of-invocation summary comes from the invocation's totals.
# Each of three invocations prints the same area (the opcode counts no
# longer accumulate across invocations), issues the same 10 operations
# (3 loads, 3 multiplies, 3 stores, the return), and its cycle
# categories add up to its runtime.
def summaries_per_invocation(params):
    _outdir, out = st.read_simout(params)
    blocks = out.split("Uncalibrated area/power estimate:")[1:]
    if len(blocks) != 3:
        raise AssertionError(f"{len(blocks)} summaries, not 3")
    areas = {re.search(r"Total Area: (\S+)", b).group(1) for b in blocks}
    if len(areas) != 1:
        raise AssertionError(f"area changes between invocations: {areas}")
    for block in blocks:
        runtime = int(re.search(r"Compute passes:\s+(\d+)", block).group(1))
        parts = re.findall(r"^\s+[a-z ]+:\s+(\d+) cycles$", block, re.M)
        if sum(map(int, parts)) != runtime or len(parts) != 5:
            raise AssertionError(f"categories {parts} for {runtime} cycles")
        if not re.search(r"Operations Issued:\s+10$", block, re.M):
            raise AssertionError("not 10 operations issued")


kernel_suite(
    "salam-kernel-summary",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    runs=3,
    extra_tests=[("summaries", summaries_per_invocation)],
)


# Tracing changes no statistics: the same kernel run again with the
# SALAMIssue debug flag reports the same values, apart from host time.
HOST_STATS = re.compile(r"^(host|sim(Seconds|Freq))|\.power_state\.")


def same_stats_with_tracing(params):
    outdir, _out = st.read_simout(params)
    command, env, _ = st._fs_command(
        params, "salam-workload", KERNEL_WALL, KERNEL_TICKS, ONE_MULTIPLIER
    )
    traced = os.path.join(outdir, "traced")
    i = command.index("-d")
    command[i + 1] = traced
    command.insert(i, "--debug-flags=SALAMIssue")
    subprocess.run(command, env=env, check=True)
    with open(os.path.join(traced, "simout.txt")) as f:
        if "issue compute mul" not in f.read():
            raise AssertionError("SALAMIssue printed no issues")
    plain = st.read_stats(outdir)[-1]
    with_flag = st.read_stats(traced)[-1]

    def same(a, b):
        return a == b or (a != a and b != b)  # NaN matches NaN

    differ = {
        k: (plain.get(k), with_flag.get(k))
        for k in set(plain) | set(with_flag)
        if not HOST_STATS.search(k)
        and not same(plain.get(k), with_flag.get(k))
    }
    if differ:
        raise AssertionError(f"statistics differ with tracing: {differ}")


kernel_suite(
    "salam-kernel-issue-trace",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=ONE_MULTIPLIER,
    extra_tests=[("same-stats", same_stats_with_tracing)],
)


# Observe volatile execution order, not just final RAM contents. Ordinary
# independent accesses remain able to issue during a stalled volatile op.
def check_volatile_order(params):
    _outdir, out = st.read_simout(params)
    issues = re.compile(
        r"memory issue (load|store) uid (\d+) address (0x[0-9a-f]+) "
        r"volatile ([01])"
    )
    completes = re.compile(r"memory complete uid (\d+)")
    outstanding = set()
    volatile_addresses = []
    all_addresses = []
    for line in out.splitlines():
        match = completes.search(line)
        if match:
            outstanding.discard(int(match[1]))
        match = issues.search(line)
        if match:
            address = int(match[3], 16)
            all_addresses.append(address)
            if match[4] == "1":
                if outstanding:
                    raise AssertionError("volatile operation overtook a peer")
                outstanding.add(int(match[2]))
                volatile_addresses.append(address)
    expected = [
        0x2F000080,
        0x2F000100,
        0x2F000084,
        0x2F000088,
        0x2F000104,
        0x2F00008C,
        0x2F000108,
        0x2F00010C,
    ]
    if volatile_addresses != expected or outstanding:
        raise AssertionError(f"volatile execution order: {volatile_addresses}")
    if all_addresses.index(0x2F000120) >= all_addresses.index(0x2F000100):
        raise AssertionError("independent ordinary store was serialized")


for lockstep in (True, False):
    kernel_suite(
        f"salam-kernel-volatile-order-{lockstep}",
        "volatile_order.ll.in",
        inputs=[6, 7, 9, 17],
        expect={0: 3, 1: 16, 2: 17, 3: 42, 8: 99},
        hw_config=SLOW_UDIV,
        fs_args=[
            "--param=system.harness_clstr.kernel.llvm_interface."
            f"lockstep_mode={lockstep}"
        ],
        gem5_args=["--debug-flags=SALAMIssue"],
        extra_tests=[("memory-order", check_volatile_order)],
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


def check_ordinary_raw_wait(params):
    outdir, out = st.read_simout(params)
    # The ordinary store and load have constant, equal addresses and no
    # SSA dependency. Exactly one ready-load observation creates a
    # runtime dependency; it is not a count of elapsed wait cycles.
    waits = re.findall(r"wait: older memory access", out)
    stats = st.read_stats(outdir)[-1]
    if len(waits) != 1 or stats[LLVM_STATS + "memoryOrderWaits"] != 1:
        raise AssertionError("expected one in-flight RAW wait observation")
    events = re.findall(
        r"^(\d+): .*memory issue (load|store) uid (\d+) address "
        r"(0x[0-9a-f]+) volatile 0",
        out,
        re.M,
    )
    writes = [event for event in events if event[1] == "store"]
    reads = [event for event in events if event[1] == "load"]
    if len(writes) != 2 or len(reads) != 1 or writes[0][3] != reads[0][3]:
        raise AssertionError(f"unexpected RAW access sequence: {events}")
    completed = re.search(
        rf"^(\d+): .*memory complete uid {writes[0][2]}$", out, re.M
    )
    if completed is None or int(reads[0][0]) < int(completed[1]):
        raise AssertionError("dependent load issued before store completion")


kernel_suite(
    "salam-kernel-ordinary-raw-wait",
    "volatile_order.ll.in",
    inputs=[0],
    expect={0: 99, 1: 99},
    fs_args=NO_LOCKSTEP
    + [
        "--param=system.harness_clstr.kernel.llvm_interface."
        'top_name="ordinary_raw"',
    ],
    gem5_args=["--debug-flags=SALAMIssue"],
    extra_tests=[("wait-observation", check_ordinary_raw_wait)],
    tags=(st.SALAM_TAG, "salam-correctness"),
)


for operation in ("load", "store", "fence", "rmw", "cmpxchg"):
    rejected_suite(
        f"salam-kernel-atomic-{operation}-rejected",
        f"atomic_{operation}.ll.in",
        inputs=[1],
        regex=r"SALAM does not support LLVM atomic instructions:",
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


for width, index in ((32, 32), (64, 64)):
    kernel_suite(
        f"salam-kernel-pointer-width-{width}-{index}",
        f"pointer_{width}_{index}.ll.in",
        inputs=[],
        expect={0: 1},
        tags=(st.SALAM_TAG, "salam-correctness"),
    )

# Constant address arithmetic creates no extra dynamic GEP operation.
# One runtime GEP indexes IN[1 + IN[0]]; numeric addresses only test folding.
for width in (32, 64):
    kernel_suite(
        f"salam-kernel-constant-gep-{width}",
        f"constant_gep_{width}.ll.in",
        inputs=[1, 23, 37],
        expect={0: 37, 1: 1},
        ir_patterns=[r"getelementptr \(\s*i32"],
        gem5_args=["--debug-flags=SALAMIssue"],
        simout_counts=[
            ("runtime-gep", r"llvm_interface: issue compute getelementptr$", 1)
        ],
        tags=(st.SALAM_TAG, "salam-correctness"),
    )

rejected_suite(
    "salam-kernel-constant-gep-symbolic-global",
    "constant_gep_global.ll.in",
    inputs=[],
    regex=r"symbolic global base '@base' in constant GEP:",
    tags=(st.SALAM_TAG, "salam-correctness"),
)


kernel_suite(
    "salam-kernel-pointer-unused-address-space",
    "pointer_unused_space.ll.in",
    inputs=[],
    expect={0: 1},
    tags=(st.SALAM_TAG, "salam-correctness"),
)

# Unequal pointer and index widths are unsupported. These fixtures cover
# rejection in instructions, constant expressions, globals and callees.
for case, index, context in (
    ("64_32", 32, "top"),
    ("64_16", 16, "top"),
    ("unequal_instruction", 32, "top"),
    ("unequal_constexpr", 32, "<global>"),
    ("unequal_runtime", 32, "top"),
    ("unequal_callee", 32, "adjust"),
):
    rejected_suite(
        f"salam-kernel-pointer-{case}-rejected",
        f"pointer_{case}.ll.in",
        inputs=[],
        regex=(
            r"SALAM does not support unequal pointer and GEP index widths: "
            rf"address space 0 has a 64-bit pointer and a {index}-bit index"
            rf"\nIR file: .*kernel\.ll; function: {context}; context: "
        ),
        tags=(st.SALAM_TAG, "salam-correctness"),
    )

for case, diagnostic in (
    ("address_space", r"pointer address space 1"),
    ("non_integral", r"non-integral LLVM pointers"),
    ("wide", r"pointer/index widths 128/128"),
):
    rejected_suite(
        f"salam-kernel-pointer-{case}-rejected",
        f"pointer_{case}.ll.in",
        inputs=[],
        regex=diagnostic,
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


for width in (32, 64):
    kernel_suite(
        f"salam-kernel-pointer-comparisons-{width}",
        f"pointer_cmp_{width}.ll.in",
        inputs=[],
        expect={0: 1},
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


for kind in ("direct", "mutual"):
    rejected_suite(
        f"salam-kernel-recursion-{kind}-rejected",
        f"recursion_{kind}.ll.in",
        inputs=[],
        regex=r"SALAM does not support recursive LLVM calls:",
        tags=(st.SALAM_TAG, "salam-correctness"),
    )

kernel_suite(
    "salam-kernel-recursion-unreachable",
    "recursion_unreachable.ll.in",
    inputs=[],
    expect={0: 7},
    tags=(st.SALAM_TAG, "salam-correctness"),
)


kernel_suite(
    "salam-kernel-scalar-matrix-120",
    "scalar_matrix.ll.in",
    inputs=[],
    expect={0: 1},
    tags=(st.SALAM_TAG, "salam-correctness"),
)
