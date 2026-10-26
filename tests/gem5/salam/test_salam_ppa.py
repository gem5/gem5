# Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
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

# Controlled coefficients test arithmetic, never production defaults.

import json
import math
import os
import re
import subprocess

import salam_testlib as st
from testlib import *

WALL = 180
TICKS = 1000000000
COMM = "system.harness_clstr.kernel"
MODEL = COMM + ".hw_interface.salam_power_model"
HW = COMM + ".hw_interface"
# The console emits seventeen significant digits. These tolerances cover
# roundoff of fewer than one hundred additions, independently of results.
REL_TOL = 1e-12
ENERGY_ABS = 1e-24
POWER_ABS = 1e-18

BASIC = """
define void @top() {
entry:
  %a = load volatile i32, PTR inttoptr (i32 IN to PTR), align 4
  %b = load volatile i32, PTR inttoptr (i32 IN1 to PTR), align 4
  %sum = add i32 %a, %b
  %product = mul i32 %sum, 3
  store volatile i32 %product, PTR inttoptr (i32 OUT to PTR), align 4
  ret void
}
"""

DIVISION = """
define void @top() {
entry:
  %a = load volatile i32, PTR inttoptr (i32 IN to PTR), align 4
  %quotient = sdiv i32 %a, 3
  store volatile i32 %quotient, PTR inttoptr (i32 OUT to PTR), align 4
  ret void
}
"""

LOOP = """
define void @top() {
entry:
  %v = load volatile i32, PTR inttoptr (i32 IN to PTR), align 4
  br label %loop
loop:
  %i = phi i32 [ 0, %entry ], [ %inc, %loop ]
  %acc = phi i32 [ 0, %entry ], [ %sum, %loop ]
  %sum = add i32 %acc, %v
  %inc = add i32 %i, 1
  %more = icmp ult i32 %inc, 3
  br i1 %more, label %loop, label %done
done:
  store volatile i32 %sum, PTR inttoptr (i32 OUT to PTR), align 4
  ret void
}
"""

CALLEE = """
define i32 @helper(i16 %x) {
entry:
  %wide = zext i16 %x to i32
  %y = add i32 %wide, 1
  ret i32 %y
}
define void @top() {
entry:
  %a = load volatile i32, PTR inttoptr (i32 IN to PTR), align 4
  %n = trunc i32 %a to i16
  %x = call i32 @helper(i16 %n)
  %condition = icmp eq i32 %x, 3
  %selected = select i1 %condition, i32 %x, i32 0
  store volatile i32 %selected, PTR inttoptr (i32 OUT to PTR), align 4
  ret void
}
define i32 @unreachable(i32 %unused) {
entry:
  %product = mul i32 %unused, 17
  ret i32 %product
}
"""

MIXED_FU = """
define void @top() {
entry:
  %a = load volatile i32, PTR inttoptr (i32 IN to PTR), align 4
  %big = add i32 %a, 1
  %small = trunc i32 %big to i16
  %sum = add i16 %small, 1
  %out = zext i16 %sum to i32
  store volatile i32 %out, PTR inttoptr (i32 OUT to PTR), align 4
  ret void
}
"""


class PPAHarness(st.SalamHarnessFixture):
    def __init__(self, gem5, tempdir, ir, inputs, runs, host_options=None):
        super().__init__(
            "salam-workload",
            gem5,
            tempdir,
            "ppa.ll.in",
            inputs,
            1,
            runs=runs,
            host_options=host_options,
        )
        self.template = ir

    def _place_kernel(self, tools):
        header = os.path.join(self.accpath, "harness_clstr_hw_defines.h")
        with open(header) as stream:
            addresses = dict(
                re.findall(r"#define (\w+) (0x[0-9a-fA-F]+)", stream.read())
            )
        ir = self.template.replace(
            "PTR", "i32*" if tools.llvm_major < 15 else "ptr"
        )
        for name, address in (
            ("IN1", int(addresses["IN"], 16) + 4),
            ("IN", int(addresses["IN"], 16)),
            ("OUT", int(addresses["OUT"], 16)),
        ):
            ir = re.sub(r"\b" + name + r"\b", str(address), ir)
        with open(
            os.path.join(self.accpath, "hw", "kernel.ll"), "w"
        ) as stream:
            stream.write(
                'target datalayout = "e-m:e-p:32:32-Fi8-i64:64-'
                'v128:64:128-a:0:32-n32-S64"\n'
                'target triple = "armv7-pc-none-eabi"\n' + ir
            )


def coefficients():
    rows = [
        (
            "fu:integer_adder",
            "adder",
            "i32,i32->i32",
            10,
            2e-6,
            None,
            None,
            None,
        ),
        (
            "op:integer_adder:add:i32,i32->i32",
            "adder",
            "",
            None,
            None,
            5e-12,
            None,
            None,
        ),
        (
            "fu:integer_multiplier",
            "mul",
            "i32,i32->i32",
            30,
            4e-6,
            None,
            None,
            None,
        ),
        (
            "op:integer_multiplier:mul:i32,i32->i32",
            "mul",
            "",
            None,
            None,
            11e-12,
            None,
            None,
        ),
        (
            "fu:float_adder",
            "unused-float",
            "f32,f32->f32",
            13,
            3e-6,
            None,
            None,
            None,
        ),
        ("register:i32", "ssa32", "", 2, 0.5e-6, None, 3e-12, 7e-12),
        ("register:i16", "ssa16", "", 1, 0.25e-6, None, 2e-12, 4e-12),
        ("register:i1", "ssa1", "", 0.25, 0.1e-6, None, 0.2e-12, 0.4e-12),
    ]
    model = {field: [] for field in ("keys", "ids", "sources", "signatures")}
    for field in (
        "timing_available",
        "minimum_latency_s",
        "minimum_issue_interval_s",
        "pipelined",
    ):
        model[field] = []
    quantities = ("area", "leakage", "event", "read", "write")
    fields = ("area_um2", "leakage_w", "event_j", "read_j", "write_j")
    for quantity, field in zip(quantities, fields):
        model[field] = []
        model[quantity + "_status"] = []
    for key, identity, signature, *values in rows:
        model["timing_available"].append(key.startswith("fu:"))
        model["minimum_latency_s"].append(0)
        model["minimum_issue_interval_s"].append(0)
        model["pipelined"].append(False)
        model["keys"].append(key)
        model["ids"].append(identity)
        model["signatures"].append(signature)
        model["sources"].append(
            json.dumps({"synthetic": True, "test": "salam-ppa"})
        )
        for quantity, field, value in zip(quantities, fields, values):
            model[field].append(0 if value is None else value)
            model[quantity + "_status"].append(
                "unavailable" if value is None else "characterized"
            )
    return model


def model_args(enabled=True, strict=True, missing=False):
    model = coefficients()
    if missing:
        index = model["keys"].index("op:integer_multiplier:mul:i32,i32->i32")
        model["event_j"][index] = 0
        model["event_status"][index] = "unavailable"
    args = [
        f"--param={MODEL}.{field}={value!r}" for field, value in model.items()
    ]
    return (
        args
        + [
            f"--param={MODEL}.enabled={enabled}",
            f"--param={MODEL}.strict={strict}",
        ]
        + [
            f"--param=getattr(system.harness_clstr, {memory!r})"
            f".ppa.enabled={enabled}"
            for memory in ("in", "out", "mem")
        ]
    )


def read_text(path):
    with open(path) as stream:
        return stream.read()


def output_check(text, expected, runs):
    observed = re.findall(r"^RUN\[(\d+)\] OUT\[0\]=(-?\d+) ", text, re.M)
    wanted = [(str(run), str(expected)) for run in range(runs)]
    if observed != wanted:
        raise AssertionError(f"payload {observed}, expected {wanted}")


def reports(text, scope="invocation"):
    selected = []
    for block in text.split("   PPA model: ")[1:]:
        if block.splitlines()[0] != MODEL:
            continue
        match = re.search(r"^   PPA scope: (.+)$", block, re.M)
        if not match or not match[1].startswith(scope):
            continue
        values = dict(re.findall(r"^   PPA ([^:]+): (.+)$", block, re.M))
        values["SSA slots"] = {
            typ: int(count)
            for typ, count in re.findall(
                r"^   PPA SSA slots: (\S+)=(\d+)$", block, re.M
            )
        }
        selected.append(values)
    return selected


def near(found, expected, absolute=ENERGY_ABS):
    if not math.isclose(
        float(found), expected, rel_tol=REL_TOL, abs_tol=absolute
    ):
        raise AssertionError(
            f"{found} differs from independent expectation {expected}"
        )


def check_reports(text, expected, runs):
    entries = reports(text)
    if len(entries) != runs:
        raise AssertionError(
            f"expected {runs} invocation PPA reports, got {len(entries)}"
        )
    duration_ticks = [
        int(value)
        for value in re.findall(
            r"^\s*Invocation duration:\s+(\d+) ticks$", text, re.M
        )
    ]
    if len(duration_ticks) != runs:
        raise AssertionError("missing elapsed invocation durations")
    for entry, ticks in zip(entries, duration_ticks):
        coverage = expected.get("coverage", "1/1/1") + "; approximate=0"
        if entry["coverage area/dynamic/leakage"] != coverage:
            raise AssertionError(f"unexpected coverage {entry}")
        if int(entry["missing event count"]) != expected.get(
            "missing_events", 0
        ):
            raise AssertionError("uncharacterized execution count differs")
        for identity in expected.get("missing", ()):
            if identity not in text:
                raise AssertionError(
                    f"missing uncharacterized identity {identity}"
                )
        if entry["FU timing compatibility"] != expected.get(
            "timing", "compatible"
        ):
            raise AssertionError("unexpected FU timing compatibility status")
        if entry["SSA slots"] != expected["slots"]:
            raise AssertionError(
                f"static slots {entry['SSA slots']}, expected {expected['slots']}"
            )
        if entry["operand reads/bits"] != expected["reads"]:
            raise AssertionError(
                f"operand transfers {entry['operand reads/bits']}, expected {expected['reads']}"
            )
        if entry["result writes/bits"] != expected["writes"]:
            raise AssertionError(
                f"result transfers {entry['result writes/bits']}, expected {expected['writes']}"
            )
        near(entry["known area (um^2)"], expected["area"], 1e-12)
        near(entry["known dynamic energy (J)"], expected["dynamic"])
        duration = ticks / 1e12
        near(entry["elapsed seconds"], duration, 1e-20)
        leakage = expected["leakage"] * duration
        near(entry["known leakage energy (J)"], leakage)
        near(entry["known total energy (J)"], expected["dynamic"] + leakage)
        if duration:
            near(
                entry["known average power (W)"],
                (expected["dynamic"] + leakage) / duration,
                POWER_ABS,
            )


def run_variant(params, name, args, tracing=False, rejected=False):
    flags = "SALAMLaunch"
    if tracing:
        flags += ",RuntimeCompute,Runtime"
        args = list(args) + [f"--param={COMM}.enable_debug_msgs=True"]
    command, env, outdir = st._fs_command(
        params,
        "salam-workload",
        WALL,
        TICKS,
        args,
        ["--debug-flags=" + flags],
        subdir=name,
    )
    completed = subprocess.run(command, env=env)
    if rejected:
        if completed.returncode in (0, 124, 137):
            raise AssertionError(
                f"expected coverage rejection, got {completed.returncode}"
            )
        error = read_text(os.path.join(outdir, "simerr.txt"))
        if "strict PPA coverage is incomplete" not in error:
            raise AssertionError(f"unexpected rejection: {error}")
        return outdir, ""
    if completed.returncode:
        raise AssertionError(
            f"PPA run {name} failed ({completed.returncode}): {outdir}"
        )
    text = read_text(os.path.join(outdir, "simout.txt"))
    if not re.search(st.M5_EXIT_REGEX, text, re.M):
        raise AssertionError(f"PPA run {name} did not finish normally")
    return outdir, text


def equal_observations(first, second):
    a, b = st.read_stats(first), st.read_stats(second)
    if len(a) != len(b):
        raise AssertionError("different statistics dump counts")
    for left, right in zip(a, b):
        for key in set(left) | set(right):
            if (
                key.startswith("host")
                or ".power_state." in key
                or ".salam_power_model." in key
                or ".ppa." in key
            ):
                continue
            x, y = left.get(key), right.get(key)
            if x != y and not (
                isinstance(x, float)
                and isinstance(y, float)
                and math.isnan(x)
                and math.isnan(y)
            ):
                raise AssertionError(f"non-PPA statistic {key}: {x} != {y}")
    if read_text(os.path.join(first, "system.terminal")) != read_text(
        os.path.join(second, "system.terminal")
    ):
        raise AssertionError("PPA changed payloads")
    pattern = r"launch accepted at .+|finished at .+|Exiting @ tick .+"
    left = re.findall(pattern, read_text(os.path.join(first, "simout.txt")))
    right = re.findall(pattern, read_text(os.path.join(second, "simout.txt")))
    if left != right or not left:
        raise AssertionError(f"PPA changed scheduling: {left} != {right}")


def analytical(expected, runs, extra=(), equivalence=False, reset=False):
    def check(params):
        strict = expected.get("coverage", "1/1/1") == "1/1/1"
        outdir, text = run_variant(
            params, "enabled", model_args(strict=strict) + list(extra)
        )
        terminal = read_text(os.path.join(outdir, "system.terminal"))
        output_check(terminal, expected["output"], runs)
        # Negative control for this checker family, independent of the model.
        try:
            output_check(terminal, expected["output"] + 1, runs)
        except AssertionError:
            pass
        else:
            raise AssertionError("payload checker accepted a wrong reference")
        check_reports(text, expected, runs)
        if equivalence:
            disabled, _ = run_variant(
                params,
                "disabled",
                model_args(False, strict=strict) + list(extra),
            )
            traced, trace = run_variant(
                params,
                "traced",
                model_args(strict=strict) + list(extra),
                tracing=True,
            )
            equal_observations(outdir, disabled)
            equal_observations(outdir, traced)
            check_reports(trace, expected, runs)
            if "Computing" not in trace:
                raise AssertionError(
                    "register/computation tracing did not execute"
                )
        if reset:
            windows = reports(text, "device")
            if len(windows) < 3:
                raise AssertionError(
                    "required repeated statistics dumps did not execute"
                )
            dumps = st.read_stats(outdir)
            if len(dumps) != len(windows):
                raise AssertionError("missing per-window PPA reports")
            for window, stats in zip(windows, dumps):
                reads = int(window["operand reads/bits"].split("/")[0])
                writes = int(window["result writes/bits"].split("/")[0])
                near(
                    window["known leakage energy (J)"],
                    expected["leakage"] * float(window["elapsed seconds"]),
                )
                add = stats[HW + ".integer_adder.admissions"]
                mul = stats[HW + ".integer_multiplier.admissions"]
                near(
                    window["known dynamic energy (J)"],
                    add * 5e-12
                    + mul * 11e-12
                    + reads * 3e-12
                    + writes * 7e-12,
                )
                near(
                    window["elapsed seconds"],
                    stats["simTicks"] / stats["simFreq"],
                    1e-20,
                )
            # No accelerator activity occurs between the host's final dumps.
            dynamics = [
                window["known dynamic energy (J)"] for window in windows[-3:]
            ]
            if len(set(dynamics)) != 1:
                raise AssertionError("repeated dumps charged activity twice")
        evidence = {
            "expected": expected,
            "invocations": reports(text),
            "elf": st.sha256_file(params.fixtures["salam-workload"].elf),
            "ir": st.sha256_file(params.fixtures["salam-workload"].ir),
        }
        with open(os.path.join(outdir, "analytical-ppa.json"), "w") as stream:
            json.dump(evidence, stream, indent=2)

    return check


BASIC_EXPECTED = dict(
    output=15,
    slots={"i32": 4},
    reads="4/128",
    writes="4/128",
    area=48,
    dynamic=56e-12,
    leakage=8e-6,
)
LOOP_EXPECTED = dict(
    output=6,
    slots={"i32": 5, "i1": 1},
    reads="20/547",
    writes="16/419",
    area=30.25,
    dynamic=173.8e-12,
    leakage=6.6e-6,
    coverage="0/0/0",
    missing_events=3,
    timing="unavailable",
    missing=["fu:unbound:icmp:i32,i32->i1"],
)
CALLEE_EXPECTED = dict(
    output=3,
    slots={"i32": 5, "i16": 2, "i1": 1},
    reads="9/225",
    writes="8/193",
    area=22.25,
    dynamic=70.6e-12,
    leakage=5.1e-6,
    coverage="0/0/0",
    missing_events=4,
    timing="unavailable",
    missing=[
        "fu:unbound:zext:i16->i32",
        "fu:unbound:trunc:i32->i16",
        "fu:unbound:icmp:i32,i32->i1",
        "fu:unbound:select:i1,i32,i32->i32",
    ],
)


def suite(name, ir, check, inputs=(2, 3), runs=1, host_options=None):
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    workload = PPAHarness(gem5, tempdir, ir, inputs, runs, host_options)
    TestSuite(
        name=name,
        fixtures=[gem5, tempdir, workload],
        tags=[st.SALAM_TAG, "salam-ppa"],
        tests=[TestFunction(check, name=name)],
    )


suite(
    "salam-ppa-analytical",
    BASIC,
    analytical(BASIC_EXPECTED, 2, equivalence=True),
    runs=2,
)
suite(
    "salam-ppa-zero-latency",
    BASIC,
    analytical(
        BASIC_EXPECTED,
        1,
        [
            f"--param={HW}.cycle_counts.add=0",
            f"--param={HW}.cycle_counts.mul=0",
        ],
        equivalence=True,
    ),
)
suite(
    "salam-ppa-inventory-loops",
    LOOP,
    analytical(LOOP_EXPECTED, 2, equivalence=True),
    inputs=(2,),
    runs=2,
)
suite(
    "salam-ppa-reachable-callee-widths",
    CALLEE,
    analytical(CALLEE_EXPECTED, 2, equivalence=True),
    inputs=(2,),
    runs=2,
)
suite(
    "salam-ppa-unused-positive-limit",
    BASIC,
    analytical(
        dict(BASIC_EXPECTED, area=74, leakage=14e-6),
        1,
        [f"--param={HW}.functional_units.float_adder.limit=2"],
    ),
)
suite(
    "salam-ppa-reset-windows",
    BASIC,
    analytical(
        BASIC_EXPECTED, 2, [f"--param={HW}.cycle_counts.mul=20"], reset=True
    ),
    runs=2,
    host_options={"STATS_RESET_AFTER_START": 1, "STATS_DUMPS_AT_END": 2},
)


def missing_coverage(params):
    run_variant(params, "strict", model_args(missing=True), rejected=True)
    outdir, text = run_variant(
        params, "incomplete", model_args(strict=False, missing=True)
    )
    output_check(read_text(os.path.join(outdir, "system.terminal")), 15, 1)
    entries = reports(text)
    if (
        len(entries) != 1
        or entries[0]["coverage area/dynamic/leakage"]
        != "1/0/1; approximate=0"
    ):
        raise AssertionError(
            "missing operation energy was not explicitly incomplete"
        )
    # The explicit operation binding still validates the physical unit;
    # unavailable event energy must not erase known area or leakage.
    check_reports(
        text,
        dict(
            BASIC_EXPECTED,
            dynamic=45e-12,
            coverage="1/0/1",
            missing_events=1,
        ),
        1,
    )
    if "op:integer_multiplier:mul:i32,i32->i32:energy" not in text:
        raise AssertionError("missing operation was not identified")


suite("salam-ppa-missing-strict", BASIC, missing_coverage)


def unbound_physical_operation(params):
    # The default integer_multiplier maps sdiv and mul to the same unit.
    # Both have i32,i32->i32 types, but this fixture characterizes only mul.
    # Thus the two SSA slots remain known; multiplier area/leakage do not.
    identity = "op:integer_multiplier:sdiv:i32,i32->i32:physical_binding"
    run_variant(params, "strict-physical-binding", model_args(), rejected=True)
    expected = dict(
        output=4,
        slots={"i32": 2},
        reads="2/64",
        writes="2/64",
        area=4,
        dynamic=20e-12,
        leakage=1e-6,
        coverage="0/0/0",
        missing_events=1,
        missing=[identity],
    )
    analytical(expected, 1, equivalence=True)(params)


suite(
    "salam-ppa-physical-operation-binding",
    DIVISION,
    unbound_physical_operation,
    inputs=(12,),
)


def incompatible_widths(params):
    run_variant(params, "strict", model_args(), rejected=True)
    outdir, text = run_variant(params, "incomplete", model_args(strict=False))
    output_check(read_text(os.path.join(outdir, "system.terminal")), 4, 1)
    entries = reports(text)
    if (
        len(entries) != 1
        or entries[0]["coverage area/dynamic/leakage"]
        != "0/0/0; approximate=0"
    ):
        raise AssertionError(
            "mixed physical FU widths were silently extrapolated"
        )
    if "fu:integer_adder:signature:i16,i16->i16" not in text:
        raise AssertionError(
            "incompatible physical signature was not identified"
        )


suite(
    "salam-ppa-mixed-fu-width-incomplete",
    MIXED_FU,
    incompatible_widths,
    inputs=(2,),
)


def unbound_arithmetic(params):
    extra = [f"--param={HW}.inst_config.add.functional_unit=0"]
    outdir, _ = run_variant(
        params, "strict-unbound", model_args() + extra, rejected=True
    )
    if "fu:unbound:add:i32,i32->i32" not in read_text(
        os.path.join(outdir, "simerr.txt")
    ):
        raise AssertionError("strict rejection did not identify unbound add")
    expected = dict(
        BASIC_EXPECTED,
        area=38,
        leakage=6e-6,
        dynamic=51e-12,
        coverage="0/0/0",
        missing_events=1,
        timing="unavailable",
        missing=["fu:unbound:add:i32,i32->i32"],
    )
    analytical(expected, 1, extra, equivalence=True)(params)


suite("salam-ppa-unbound-arithmetic", BASIC, unbound_arithmetic)


def functional_unit_timing(params):
    cases = (
        # name, declared latency/II/pipeline, actual cycles/II, failure
        ("serial-boundary", 2e-9, 2e-9, False, 2, 0, None),
        ("short-latency", 2e-9, 0, False, 1, 0, "minimum_latency"),
        ("short-serial-II", 0, 3e-9, False, 2, 0, "minimum_issue_interval"),
        ("wrong-organization", 0, 0, False, 2, 1, "pipeline_organization"),
        ("pipeline-boundary", 2e-9, 1e-9, True, 2, 1, None),
    )
    for name, latency, interval, pipeline, cycles, actual_ii, failure in cases:
        values = coefficients()
        index = values["keys"].index("fu:integer_adder")
        values["minimum_latency_s"][index] = latency
        values["minimum_issue_interval_s"][index] = interval
        values["pipelined"][index] = pipeline
        extra = [
            f"--param={MODEL}.{field}={values[field]!r}"
            for field in (
                "minimum_latency_s",
                "minimum_issue_interval_s",
                "pipelined",
            )
        ]
        extra += [
            "--param=system.harness_clstr.compute_clk_domain.clock=['1GHz']",
            f"--param={HW}.cycle_counts.add={cycles}",
            f"--param={HW}.functional_units.integer_adder.initiation_interval={actual_ii}",
        ]
        if failure:
            run_variant(
                params, name + "-strict", model_args() + extra, rejected=True
            )
        outdir, text = run_variant(
            params, name, model_args(strict=not failure) + extra
        )
        output_check(read_text(os.path.join(outdir, "system.terminal")), 15, 1)
        expected = dict(BASIC_EXPECTED)
        if failure:
            expected.update(coverage="0/0/0", timing="incompatible")
            if failure not in text:
                raise AssertionError("FU timing failure lost its reason")
        check_reports(text, expected, 1)
        disabled, _ = run_variant(
            params, name + "-disabled", model_args(False, strict=False) + extra
        )
        equal_observations(outdir, disabled)
    values = coefficients()["timing_available"]
    values[0] = False
    outdir, text = run_variant(
        params,
        "unknown-timing",
        model_args() + [f"--param={MODEL}.timing_available={values!r}"],
    )
    check_reports(text, dict(BASIC_EXPECTED, timing="unavailable"), 1)
    if "fu:integer_adder:timing_constraints_unavailable" not in text:
        raise AssertionError("unknown FU timing was silently replaced by zero")
    # An unused positive-limit unit still has a configured physical II.
    values = coefficients()
    index = values["keys"].index("fu:float_adder")
    values["pipelined"][index] = True
    values["minimum_issue_interval_s"][index] = 2e-9
    extra = [
        f"--param={MODEL}.{field}={values[field]!r}"
        for field in ("pipelined", "minimum_issue_interval_s")
    ]
    extra += [
        "--param=system.harness_clstr.compute_clk_domain.clock=['1GHz']",
        f"--param={HW}.functional_units.float_adder.limit=2",
        f"--param={HW}.functional_units.float_adder.initiation_interval=1",
    ]
    run_variant(params, "idle-II-strict", model_args() + extra, rejected=True)
    outdir, text = run_variant(
        params, "idle-II-incomplete", model_args(strict=False) + extra
    )
    check_reports(
        text,
        dict(
            BASIC_EXPECTED,
            area=74,
            leakage=14e-6,
            coverage="0/0/0",
            timing="incompatible",
        ),
        1,
    )
    if "fu:float_adder:timing:minimum_issue_interval" not in text:
        raise AssertionError("idle physical FU II mismatch was omitted")


suite("salam-ppa-fu-timing-compatibility", BASIC, functional_unit_timing)
