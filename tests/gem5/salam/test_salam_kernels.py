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
import threading

import salam_testlib as st
from testlib import *

KERNEL_WALL = 180
KERNEL_TICKS = 1000000000
COMM = "system.harness_clstr.kernel"
LLVM_STATS = COMM + ".llvm_interface."
MULTIPLIER_STATS = COMM + ".hw_interface.integer_multiplier."
COMM_STATS = COMM + "."


def out_regex(index, value, run=0):
    return re.compile(rf"^RUN\[{run}\] OUT\[{index}\]={value} ")


def native_stats(expected, prefix=COMM_STATS):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        found = {k: stats[prefix + k] for k in expected}
        if found != expected:
            raise AssertionError(f"statistics {found}, not {expected}")

    return check


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
    gem5_args=(),
    gem5=None,
    profile=None,
    tags=(st.SALAM_TAG,),
    clocks=None,
    ticks=KERNEL_TICKS,
    wrapper=None,
    host_options=None,
    extra_tests=(),
    harness_class=st.SalamHarnessFixture,
):
    # expect/forbid: {index: value} checked for every run.
    # fs_args, gem5_args: extra fs.py and gem5 options for the run.
    # cache_size: give the accelerator a Cache var of that size.
    # profile: a SalamProfileFixture to write before gem5 builds it.
    gem5 = gem5 or st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    n_out = n_out or max(list(expect) + [i for i, _ in forbid]) + 1
    harness = harness_class(
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
    runs=2,
    ir_patterns=[r"^\s*%\S+ = add nsw i32"],
    extra_tests=[
        ("memory-interface", native_stats({"starts": 2, "finishes": 2}))
    ],
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
            "float_constant.ll.in",
            [],
            3,
        )
        self.operation = operation

    def _place_kernel(self, tools):
        super()._place_kernel(tools)
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
        path = os.path.join(self.accpath, "hw", "kernel.ll")
        with open(path) as stream:
            ir = stream.read()
        ir = ir.replace("{EXPRESSION}", expression)
        ir = ir.replace("{PTR_FLOAT}", float_ptr)
        with open(path, "w") as stream:
            stream.write(ir)


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


# IN[0] = 0x1234abcd: bytes cd ab 34 12 in memory.
kernel_suite(
    "salam-kernel-narrow-load",
    "narrow_load.c",
    inputs=[0x1234ABCD],
    expect={0: 0x1234, 1: -85, 2: 0x12},
    ir_patterns=[r"load volatile i16", r"load volatile i8"],
)


kernel_suite(
    "salam-kernel-call-nested",
    "call_nested.c",
    inputs=[5, 2],
    expect={0: 35, 1: 7},
    ir_patterns=[r"call i32 @g\(", r"call i32 @f\("],
    tags=(st.SALAM_TAG, "salam-correctness"),
)


# Phis at a block entry read their incoming values simultaneously.
PHI_PATTERN = r"= phi i32 \[ %\d+, %\d+ \], \[ %\d+, %\d+ \]"
kernel_suite(
    "salam-kernel-phi-swap-3",
    "phi_swap.c",
    inputs=[3, 1, 2],
    expect={0: 2, 1: 1},
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

# Delay a producer so the caller's memory access is still pending.
SLOW_UDIV = {"instructions": {"udiv": {"runtime_cycles": 20}}}
UDIV = r"= udiv i32 "

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

INST_CONFIG = "system.harness_clstr.kernel.hw_interface.inst_config"

FU_UNITS = "system.harness_clstr.kernel.hw_interface.functional_units"
INVENTORY_IR = [
    r"call i32 @helper\(.*\n.*call i32 @helper\(",
    r"define dso_local (?:noundef )?i32 @unused\(.*\n.*= shl ",
]


# A nonzero limit is the instance count.
kernel_suite(
    "salam-kernel-fu-limit",
    "fu_inventory.c",
    inputs=[9, 3],
    expect={0: 6, 1: 10},
    fs_args=[f"--param={FU_UNITS}.integer_adder.limit=5"],
    ir_patterns=INVENTORY_IR,
    extra_tests=[
        (
            "inventory",
            native_stats(
                {"integer_adder.instances": 5}, COMM + ".hw_interface."
            ),
        )
    ],
)


def check_multiply_latency(latency, frequency, pipelined=False):
    def check(params):
        outdir, out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        period = stats["simFreq"] / frequency
        issues = [
            int(tick)
            for tick in re.findall(
                r"^\s*(\d+): .*\.llvm_interface: issue compute mul$",
                out,
                re.M,
            )
        ]
        completions = [
            int(tick)
            for tick in re.findall(
                r"^\s*(\d+): .*\.llvm_interface: complete compute$",
                out,
                re.M,
            )
        ]
        if len(issues) != 3 or len(completions) != 3:
            raise AssertionError("expected three multiply operations")
        if any(
            completed - issued != latency * period
            for issued, completed in zip(issues, completions)
        ):
            raise AssertionError("multiply latency differs from configuration")
        overlap = any(
            issued < completed
            for issued, completed in zip(issues[1:], completions)
        )
        if overlap != pipelined:
            raise AssertionError(
                "FU initiation interval did not control overlap"
            )
        if any(b - a < period for a, b in zip(issues, issues[1:])):
            raise AssertionError(
                "one instance admitted twice on a compute edge"
            )

    return check


def zero_latency_accounting(instances, operations):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        expected = {
            "instances": instances,
            "admissions": operations,
            "inFlight": 0,
            "busyInstances": 0,
            "utilization": 0,
        }
        found = {name: stats[MULTIPLIER_STATS + name] for name in expected}
        if found != expected:
            raise AssertionError(f"zero-latency FU accounting {found}")

    return check


# Keep one non-pipelined and one pipelined instance. Check configured
# operation latency rather than a table of scheduler issue passes.
MULTIPLIER = FU_UNITS + ".integer_multiplier"
CYCLE_COUNTS = "system.harness_clstr.kernel.hw_interface.cycle_counts"
LOCKSTEP = "system.harness_clstr.kernel.llvm_interface.lockstep_mode"
for name, interval in (("l4-one", 0), ("l4-ii1", 1)):
    kernel_suite(
        f"salam-kernel-fu-sched-{name}",
        "fu_sched.c",
        inputs=[3, 4, 5],
        expect={0: 9, 1: 16, 2: 25},
        fs_args=[
            f"--param={LOCKSTEP}=False",
            f"--param={CYCLE_COUNTS}.mul=4",
            f"--param={MULTIPLIER}.limit=1",
            f"--param={MULTIPLIER}.initiation_interval={interval}",
        ],
        clocks={"Compute": "100MHz"},
        ir_patterns=[r"(?s)(= mul i32 .*){3}"],
        gem5_args=["--debug-flags=SALAMIssue"],
        extra_tests=[
            ("latency", check_multiply_latency(4, 100000000, interval > 0)),
            (
                "admissions",
                native_stats(
                    {"instances": 1, "admissions": 3}, MULTIPLIER_STATS
                ),
            ),
        ],
        tags=(st.SALAM_TAG, "salam-correctness"),
    )


# Exercise token acquisition and release across nested calls when
# operations complete immediately on one shared functional unit.
kernel_suite(
    "salam-kernel-fu-sched-nested-zero-False",
    "call_nested.c",
    inputs=[5, 2],
    expect={0: 35, 1: 7},
    fs_args=[
        f"--param={LOCKSTEP}=False",
        f"--param={CYCLE_COUNTS}.mul=0",
        f"--param={CYCLE_COUNTS}.add=0",
        f"--param={INST_CONFIG}.add.functional_unit=2",
        f"--param={MULTIPLIER}.limit=1",
    ],
    ir_patterns=[r"call i32 @g\(", r"call i32 @f\("],
    extra_tests=[("shared-unit", zero_latency_accounting(1, 8))],
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
    ir_patterns=FSUB_IR,
)


def characterize_bitwise(profile):
    import yaml

    # Exercise the generated profile's established ownership aliases while
    # the characterization records use actual LLVM operation names.
    units = os.path.join(profile, "functional_units")
    path = os.path.join(units, "bitwise_operations", "bitwise_operations.yml")
    with open(path) as stream:
        document = yaml.safe_load(stream)
    unit = document["functional_unit"]
    if unit["parameters"]["instructions"] != [
        "and_inst",
        "or_inst",
        "xor_inst",
    ]:
        raise AssertionError("bitwise profile ownership aliases changed")
    unit["parameters"]["limit"] = 1
    unit.setdefault("power_model", {})[
        "characterization"
    ] = "synthetic-ppa.json"
    with open(path, "w") as stream:
        yaml.safe_dump(document, stream, sort_keys=False)

    source = {
        "revision": "synthetic-test",
        "path": "tests/gem5/salam/test_salam_kernels.py",
        "row": "characterize_bitwise",
        "sha256": st.sha256_file(
            os.path.join(
                config.base_dir, "tests/gem5/salam/test_salam_kernels.py"
            )
        ),
        "library": "synthetic",
        "datatype": "32-bit integer",
        "implementation": "controlled test coefficients only",
        "synthetic": True,
    }

    def quantity(value, unit):
        return {"value": value, "unit": unit, "status": "characterized"}

    records = [
        {
            "key": "fu:bitwise_operations",
            "id": "synthetic-bitwise",
            "source": source,
            "signatures": ["i32,i32->i32"],
            "area": quantity(10, "um2"),
            "leakage": quantity(2, "uW"),
            "constraints": {
                "minimum_latency_s": 0,
                "minimum_issue_interval_s": 0,
                "pipelined": False,
            },
        }
    ]
    for operation, energy in (("and", 5), ("or", 7), ("xor", 11)):
        records.append(
            {
                "key": f"op:bitwise_operations:{operation}:i32,i32->i32",
                "id": "synthetic-bitwise",
                "source": source,
                "event": quantity(energy, "pJ"),
            }
        )
    records.append(
        {
            "key": "register:i32",
            "id": "synthetic-ssa-i32",
            "source": source,
            "area": quantity(2, "um2"),
            "leakage": quantity(0.5, "uW"),
            "read": quantity(3, "pJ"),
            "write": quantity(7, "pJ"),
        }
    )
    with open(
        os.path.join(os.path.dirname(path), "synthetic-ppa.json"), "w"
    ) as stream:
        json.dump({"version": 1, "records": records}, stream, indent=2)


def check_generated_bitwise_ppa(params):
    import math

    outdir, text = st.read_simout(params)
    model_path = "system.harness_clstr.kernel.hw_interface.salam_power_model"
    comm = "system.harness_clstr.kernel"
    hw = comm + ".hw_interface"

    def read(path):
        with open(path) as stream:
            return stream.read()

    config_document = json.loads(read(os.path.join(outdir, "config.json")))
    hardware = config_document["system"]["harness_clstr"]["kernel"][
        "hw_interface"
    ]
    selected = hardware["salam_power_model"]
    expected_keys = [
        "fu:bitwise_operations",
        "op:bitwise_operations:and:i32,i32->i32",
        "op:bitwise_operations:or:i32,i32->i32",
        "op:bitwise_operations:xor:i32,i32->i32",
        "register:i32",
    ]
    if selected["keys"] != expected_keys:
        raise AssertionError(
            "generated profile selected different PPA records"
        )
    # Check binding and unit conversion; runtime arithmetic below checks
    # the combined contributions rather than snapshotting every zero slot.
    expected_coefficients = (
        ("area_um2", 0, 10),
        ("leakage_w", 0, 2e-6),
        ("event_j", 1, 5e-12),
        ("event_j", 2, 7e-12),
        ("event_j", 3, 11e-12),
        ("area_um2", 4, 2),
        ("leakage_w", 4, 0.5e-6),
        ("read_j", 4, 3e-12),
        ("write_j", 4, 7e-12),
    )
    for field, index, expected in expected_coefficients:
        if not math.isclose(selected[field][index], expected, rel_tol=1e-12):
            raise AssertionError(f"generated {field}[{index}] differs")
    if not selected["enabled"] or not selected["strict"]:
        raise AssertionError(
            "generated-profile test requires strict enabled PPA"
        )
    if not all(
        json.loads(value)["synthetic"] for value in selected["sources"]
    ):
        raise AssertionError("generated synthetic provenance was lost")
    if hardware["functional_units"]["bitwise_operations"]["limit"] != 1:
        raise AssertionError("test must use one shared physical bitwise FU")

    entries = []
    for block in text.split("   PPA model: ")[1:]:
        if block.splitlines()[0] != model_path:
            continue
        values = dict(re.findall(r"^   PPA ([^:]+): (.+)$", block, re.M))
        if values.get("scope", "").startswith("invocation:"):
            entries.append(values)
    if len(entries) != 1:
        raise AssertionError(
            "expected one generated-profile invocation report"
        )
    entry = entries[0]
    for key, expected in {
        "coverage area/dynamic/leakage": "1/1/1; approximate=0",
        "FU timing compatibility": "compatible",
        "operand reads/bits": "5/160",
        "result writes/bits": "5/160",
        "missing event count": "0",
        "SSA slots": "i32=5",
    }.items():
        if entry.get(key) != expected:
            raise AssertionError(
                f"generated-profile {key}: {entry.get(key)} != {expected}"
            )
    ticks = re.findall(r"^\s*Invocation duration:\s+(\d+) ticks$", text, re.M)
    if len(ticks) != 1:
        raise AssertionError("expected one invocation duration")
    elapsed = int(ticks[0]) / 1e12
    # One and/or/xor event; five materialized i32 slots, reads, and writes.
    dynamic = (5 + 7 + 11 + 5 * 3 + 5 * 7) * 1e-12
    area = 10 + 5 * 2
    leakage = (2 + 5 * 0.5) * 1e-6 * elapsed
    expected_quantities = {
        "known area (um^2)": (area, 1e-12),
        "known dynamic energy (J)": (dynamic, 1e-24),
        "known leakage energy (J)": (leakage, 1e-24),
        "known total energy (J)": (dynamic + leakage, 1e-24),
        "elapsed seconds": (elapsed, 1e-20),
    }
    if elapsed <= 0:
        raise AssertionError(
            "the nonzero-latency bitwise graph must take time"
        )
    expected_quantities["known average power (W)"] = (
        (dynamic + leakage) / elapsed,
        1e-18,
    )
    for key, (expected, absolute) in expected_quantities.items():
        if not math.isclose(
            float(entry[key]), expected, rel_tol=1e-12, abs_tol=absolute
        ):
            raise AssertionError(
                f"generated-profile {key}: {entry[key]} != {expected}"
            )

    stats = st.read_stats(outdir)
    if not stats:
        raise AssertionError("missing generated-profile statistics")
    counts = {
        comm + ".llvm_interface.opcodesIssued::" + op: 1
        for op in ("and", "or", "xor")
    }
    counts[hw + ".bitwise_operations.admissions"] = 3
    counts[hw + ".bitwise_operations.instances"] = 1
    for key, expected in counts.items():
        if stats[-1].get(key) != expected:
            raise AssertionError(
                f"generated-profile count {key}: {stats[-1].get(key)} != {expected}"
            )

    workload = params.fixtures["salam-workload"]
    artifacts = {
        "binary": params.fixtures["salam-gem5"].path,
        "elf": workload.elf,
        "ir": workload.ir,
        "yaml": os.path.join(workload.accpath, "config.yml"),
    }
    before = {name: st.sha256_file(path) for name, path in artifacts.items()}
    if (
        before["binary"] != workload.manifest["gem5_sha256"]
        or before["elf"] != workload.manifest["outputs"]["sw/main.elf"]
        or before["ir"] != workload.manifest["outputs"]["hw/kernel.ll"]
    ):
        raise AssertionError("enabled run's fixed artifacts changed")
    disabled_args = [
        f"--param={model_path}.enabled=False",
        f"--param={model_path}.strict=True",
        f"--param={model_path}.report_details=True",
    ]
    disabled_args += [
        f"--param=getattr(system.harness_clstr, {name!r}).ppa.enabled=False"
        for name in ("in", "out", "mem")
    ]
    command, env, disabled = st._fs_command(
        params,
        "salam-workload",
        KERNEL_WALL,
        KERNEL_TICKS,
        disabled_args,
        ["--debug-flags=SALAMLaunch"],
        subdir="disabled",
    )
    completed = subprocess.run(command, env=env)
    if completed.returncode:
        raise AssertionError(
            f"disabled generated-profile run failed: {completed.returncode}"
        )
    after = {name: st.sha256_file(path) for name, path in artifacts.items()}
    if before != after:
        raise AssertionError("on/off runs did not retain identical artifacts")
    disabled_config = json.loads(read(os.path.join(disabled, "config.json")))

    def without_enabled(value):
        if isinstance(value, dict):
            return {
                key: without_enabled(child)
                for key, child in value.items()
                if not (
                    key == "enabled" and value.get("type") == "SALAMPowerModel"
                )
            }
        if isinstance(value, list):
            return [without_enabled(child) for child in value]
        return value

    if without_enabled(config_document) != without_enabled(disabled_config):
        raise AssertionError(
            "PPA mode changed effective hardware configuration"
        )
    disabled_text = read(os.path.join(disabled, "simout.txt"))
    if not re.search(st.M5_EXIT_REGEX, disabled_text, re.M):
        raise AssertionError("disabled generated-profile run did not finish")
    if "   PPA model: " in disabled_text:
        raise AssertionError("disabled PPA emitted a report")
    terminal = read(os.path.join(outdir, "system.terminal"))
    if terminal != read(os.path.join(disabled, "system.terminal")):
        raise AssertionError("generated PPA changed payloads")
    if not re.search(out_regex(0, 22).pattern, terminal, re.M) or re.search(
        out_regex(0, 23).pattern, terminal, re.M
    ):
        raise AssertionError("bitwise output or negative control failed")
    events = r"launch accepted at .+|finished at .+|Exiting @ tick .+"
    enabled_events = re.findall(events, text)
    if not enabled_events or enabled_events != re.findall(
        events, disabled_text
    ):
        raise AssertionError("generated PPA changed launch/completion timing")
    other = st.read_stats(disabled)
    if len(stats) != len(other):
        raise AssertionError("generated PPA changed statistics dump count")
    for left, right in zip(stats, other):
        for key in left.keys() | right.keys():
            if key.startswith("host") or any(
                part in key for part in (".salam_power_model.", ".ppa.")
            ):
                continue
            a, b = left.get(key), right.get(key)
            if a != b and not (
                isinstance(a, float)
                and isinstance(b, float)
                and math.isnan(a)
                and math.isnan(b)
            ):
                raise AssertionError(
                    f"generated PPA changed {key}: {a} != {b}"
                )
    evidence = {
        "keys": expected_keys,
        "ids": selected["ids"],
        "expected": {
            "output": 22,
            "area_um2": area,
            "dynamic_j": dynamic,
            "leakage_w": 4.5e-6,
            "opcode_events": {op: 1 for op in ("and", "or", "xor")},
        },
        "observed_invocation": entry,
        "expected_counts": counts,
        "artifact_sha256": before,
        "selected_coefficients": expected_coefficients,
        "on_off_observations_equal": True,
        "on_off_effective_configuration_equal": True,
    }
    with open(
        os.path.join(outdir, "generated-profile-ppa.json"), "w"
    ) as stream:
        json.dump(evidence, stream, indent=2)


def remap_fsub(profile):
    import yaml

    units = os.path.join(profile, "functional_units")
    sine = os.path.join(units, "float_trig_sine")
    shutil.copytree(
        os.path.join(
            config.base_dir,
            "configs/SALAM/hw_profiles/examples/float_trig_sine",
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

    characterize_bitwise(profile)


REMAP_CONFIG = os.path.join(
    config.build_dir, "salam-fu-remap-profile", "salam-hw-config.yml"
)


# Both custom-profile suites share one input snapshot and one built binary.
# A repeated suite setup must not recreate inputs underneath that binary.
class RemapProfile(st.SalamProfileFixture):
    def __init__(self):
        super().__init__(REMAP_CONFIG, remap_fsub)
        self._profile_lock = threading.Lock()
        self._profile_ready = False

    def setup(self, testitem):
        with self._profile_lock:
            if not self._profile_ready:
                super().setup(testitem)
                self._profile_ready = True


REMAP_PROFILE = RemapProfile()
kernel_suite(
    "salam-kernel-fsub-remap",
    "fsub.c",
    inputs=FSUB_INPUTS,
    expect={0: FSUB_OUT},
    ir_patterns=FSUB_IR,
    gem5=st.SalamGem5Fixture(REMAP_CONFIG),
    profile=REMAP_PROFILE,
    tags=(REMAP_TAG,),
    extra_tests=[
        (
            "inventory",
            native_stats(
                {
                    "float_trig_sine.instances": 1,
                    "float_trig_sine.admissions": 1,
                    "double_adder.instances": 0,
                    "double_adder.admissions": 0,
                },
                COMM + ".hw_interface.",
            ),
        )
    ],
)

kernel_suite(
    "salam-kernel-bitwise-generated-ppa",
    "bitwise_ppa.ll.in",
    inputs=[85, 15],
    expect={0: 22},
    forbid=[(0, 23)],
    ir_patterns=[r"= and i32 ", r"= or i32 ", r"= xor i32 "],
    fs_args=[
        "--param=system.harness_clstr.kernel.hw_interface.salam_power_model.strict=True",
        "--param=system.harness_clstr.kernel.hw_interface.salam_power_model.report_details=True",
    ],
    gem5_args=["--debug-flags=SALAMLaunch"],
    gem5=st.SalamGem5Fixture(REMAP_CONFIG),
    profile=REMAP_PROFILE,
    tags=(REMAP_TAG, "ppa-profile"),
    extra_tests=[
        ("characterization-and-equivalence", check_generated_bitwise_ppa)
    ],
)


# A longer compute period must lengthen each configured multiply latency.
ONE_MULTIPLIER = [
    f"--param={LOCKSTEP}=False",
    f"--param={CYCLE_COUNTS}.mul=4",
    f"--param={MULTIPLIER}.limit=1",
    f"--param={MULTIPLIER}.initiation_interval=0",
]
kernel_suite(
    "salam-kernel-clocks-compute-50mhz",
    "fu_sched.c",
    inputs=[3, 4, 5],
    expect={0: 9, 1: 16, 2: 25},
    fs_args=ONE_MULTIPLIER,
    clocks={"Compute": "50MHz"},
    ir_patterns=[r"(?s)(= mul i32 .*){3}"],
    gem5_args=["--debug-flags=SALAMIssue"],
    extra_tests=[("latency", check_multiply_latency(4, 50000000))],
)

# Keep the delayed-publication case; device tests cover the exact
# register-bank completion delay.
REGS = "system.harness_clstr.out"
kernel_suite(
    "salam-kernel-regbank-visible-late",
    "regbank_rw.c",
    inputs=[9],
    expect={0: 9},
    fs_args=[f"--param={REGS}.write_cycles=30"],
    ir_patterns=[r"store volatile i32 %\d+, (i32\*|ptr) inttoptr"],
)

SPM = "system.harness_clstr.mem"


# Execution statistics report the current statistics window. A reset
# clears them and changes nothing in the accelerator: the kernel takes
# the same cycles, and printResults (per-invocation totals) still shows
# all of them.


def check_reset_during_compute(params):
    outdir, out = st.read_simout(params)
    total = int(re.search(r"Compute passes:\s+(\d+)", out).group(1))
    stats = st.read_stats(outdir)[-1]
    if not 0 < stats[LLVM_STATS + "cycles"] < total:
        raise AssertionError("reset did not leave a partial compute window")
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
    host_options={"STATS_RESET_AFTER_START": 1},
    extra_tests=[
        ("window", check_reset_during_compute),
    ],
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


kernel_suite(
    "salam-kernel-volatile-order-False",
    "volatile_order.ll.in",
    inputs=[6, 7, 9, 17],
    expect={0: 3, 1: 16, 2: 17, 3: 42, 8: 99},
    hw_config=SLOW_UDIV,
    fs_args=NO_LOCKSTEP,
    gem5_args=["--debug-flags=SALAMIssue"],
    extra_tests=[("memory-order", check_volatile_order)],
    tags=(st.SALAM_TAG, "salam-correctness"),
)


def check_ordinary_raw_wait(params):
    outdir, out = st.read_simout(params)
    # The ordinary store and load have constant, equal addresses and no
    # SSA dependency. Exactly one ready-load observation creates a
    # runtime dependency; it is not a count of elapsed wait cycles.
    stats = st.read_stats(outdir)[-1]
    if stats[LLVM_STATS + "memoryOrderWaits"] != 1:
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


rejected_suite(
    "salam-kernel-atomic-rmw-rejected",
    "atomic_rmw.ll.in",
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
        extra_tests=[
            (
                "runtime-gep",
                native_stats({"opcodesIssued::getelementptr": 1}, LLVM_STATS),
            )
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

# One representative for each unsupported pointer-layout condition.
rejected_suite(
    "salam-kernel-pointer-unequal_instruction-rejected",
    "pointer_unequal_instruction.ll.in",
    inputs=[],
    regex=r"SALAM does not support unequal pointer and GEP index widths:",
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


rejected_suite(
    "salam-kernel-recursion-mutual-rejected",
    "recursion_mutual.ll.in",
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


# Each eager traversal follows only unconditional edges. A conditional loop
# must remain valid on repeated visits and on a new invocation.
rejected_suite(
    "salam-kernel-cfg-cycle-call_cycle-rejected",
    "cfg_cycles.ll.in",
    inputs=[],
    regex=r"SALAM cannot eagerly schedule an unconditional CFG cycle",
    fs_args=[
        "--param=system.harness_clstr.kernel.llvm_interface."
        'top_name="call_cycle"',
    ],
    tags=(st.SALAM_TAG, "salam-correctness", "salam-cfg"),
)


def cfg_loop_control_count(iterations, runs):
    def check(params):
        outdir, _out = st.read_simout(params)
        stats = st.read_stats(outdir)[-1]
        # Each invocation makes iterations + 1 conditional decisions and
        # returns once. Eager unconditional edges do not issue operations.
        expected = runs * (iterations + 2)
        for field in ("issued::control", "completed::control"):
            if stats[LLVM_STATS + field] != expected:
                raise AssertionError("conditional loop control count changed")

    return check


kernel_suite(
    "salam-kernel-cfg-conditional-3-False",
    "cfg_cycles.ll.in",
    inputs=[3],
    expect={0: 3},
    runs=2,
    fs_args=[
        "--param=system.harness_clstr.kernel.llvm_interface."
        'top_name="conditional_loop"',
        f"--param={LOCKSTEP}=False",
    ],
    extra_tests=[("control-count", cfg_loop_control_count(3, 2))],
    tags=(st.SALAM_TAG, "salam-correctness", "salam-cfg"),
)


class LongCfgChainHarness(st.SalamHarnessFixture):
    def _place_kernel(self, tools):
        super()._place_kernel(tools)
        path = os.path.join(self.accpath, "hw", "kernel.ll")
        with open(path) as stream:
            text = stream.read()
        blocks = 4096
        chain = []
        for index in range(blocks):
            target = f"chain_{index + 1}" if index + 1 < blocks else "exit"
            chain.append(f"chain_{index}:\n  br label %{target}")
        if text.count("{CHAIN}") != 1:
            raise AssertionError("missing long CFG chain template marker")
        with open(path, "w") as stream:
            stream.write(text.replace("{CHAIN}", "\n".join(chain)))


kernel_suite(
    "salam-kernel-cfg-long-chain",
    "cfg_chain.ll.in",
    inputs=[],
    expect={0: 7},
    runs=2,
    harness_class=LongCfgChainHarness,
    tags=(st.SALAM_TAG, "salam-correctness", "salam-cfg"),
)
