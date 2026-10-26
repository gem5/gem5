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

import os
import re
import shutil

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
    simout_counts=(),
    gem5_args=(),
    gem5=None,
    profile=None,
    tags=(st.SALAM_TAG,),
):
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
    )
    tests = [
        TestFunction(
            st.run_fs(
                "salam-workload",
                KERNEL_WALL,
                KERNEL_TICKS,
                fs_args,
                gem5_args,
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
        tags=[st.SALAM_TAG],
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
    r"define dso_local i32 @unused\(.*\n.*= shl ",
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

# Functional-unit scheduling: three independent multiplies are ready
# together. With three multiplier instances all issue in the first
# cycle (the base runtime for that latency L: 2, 3 and 6 cycles for L
# 0, 1 and 4). With one instance the last issue is later by the
# admission schedule: 2L when the unit is not pipelined or its
# interval is L, 2 cycles with interval 1, 10 cycles with interval 5.
# An interval shorter than the latency only overlaps without lockstep.
MULTIPLIER = FU_UNITS + ".integer_multiplier"
CYCLE_COUNTS = "system.harness_clstr.kernel.hw_interface.cycle_counts"
LOCKSTEP = "system.harness_clstr.kernel.llvm_interface.lockstep_mode"
for name, latency, limit, interval, lockstep, runtime, runs in (
    ("l4-three", 4, 3, None, False, 6, 1),
    ("l4-one", 4, 1, None, False, 6 + 8, 3),
    ("l4-ii4", 4, 1, 4, False, 6 + 8, 1),
    ("l4-ii1", 4, 1, 1, False, 6 + 2, 1),
    ("l4-ii1-lockstep", 4, 1, 1, True, 6 + 8, 1),
    ("l1-ii5", 1, 1, 5, False, 3 + 10, 1),
    ("l0-one", 0, 1, None, False, 2, 1),
    ("l0-ii5", 0, 1, 5, False, 2 + 10, 1),
):
    fs_args = [
        f"--param={LOCKSTEP}={lockstep}",
        f"--param={CYCLE_COUNTS}.mul={latency}",
        f"--param={MULTIPLIER}.limit={limit}",
    ]
    if interval is not None:
        fs_args.append(f"--param={MULTIPLIER}.initiation_interval={interval}")
    kernel_suite(
        f"salam-kernel-fu-sched-{name}",
        "fu_sched.c",
        inputs=[3, 4, 5],
        expect={0: 9, 1: 16, 2: 25},
        runs=runs,
        fs_args=fs_args,
        ir_patterns=[r"(?s)(= mul i32 .*){3}"],
        simout_counts=[
            ("runtime", rf"^\s*Runtime:\s+{runtime} cycles$", runs)
        ],
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
    simout_counts=[("runtime", r"^\s*Runtime:\s+3 cycles$", 1)],
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
