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

# Power reporting must not change accelerator results or timing.

import json
import math
import os
import re
import subprocess

import salam_testlib as st
from testlib import (
    TempdirFixture,
    TestFunction,
    TestSuite,
)

COMM = "system.harness_clstr.kernel"
MODEL = COMM + ".hw_interface.salam_power_model"


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
        ("register:i32", "ssa32", "", 2, 0.5e-6, None, 3e-12, 7e-12),
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


def model_args(enabled):
    args = [
        f"--param={MODEL}.{field}={value!r}"
        for field, value in coefficients().items()
    ]
    args += [
        f"--param={MODEL}.enabled={enabled}",
        f"--param={MODEL}.strict=False",
        f"--param={MODEL}.report_details=False",
    ]
    for memory in ("in", "out", "mem"):
        args.append(
            f"--param=getattr(system.harness_clstr, {memory!r})"
            f".ppa.enabled={enabled}"
        )
    return args


def read_text(path):
    with open(path) as stream:
        return stream.read()


def run_variant(params, enabled):
    command, env, outdir = st._fs_command(
        params,
        "salam-workload",
        180,
        1000000000,
        model_args(enabled),
        [],
        subdir="enabled" if enabled else "disabled",
    )
    result = subprocess.run(command, env=env)
    if result.returncode or not re.search(
        st.M5_EXIT_REGEX, read_text(os.path.join(outdir, "simout.txt")), re.M
    ):
        raise AssertionError(f"power-reporting run did not finish: {outdir}")
    configuration = json.loads(read_text(os.path.join(outdir, "config.json")))
    model = configuration
    for name in MODEL.split("."):
        model = model[name]
    if model["enabled"] != enabled:
        raise AssertionError("power model enable setting was not applied")
    output = read_text(os.path.join(outdir, "system.terminal"))
    observed = re.findall(r"^RUN\[(\d+)\] OUT\[0\]=(-?\d+) ", output, re.M)
    if observed != [("0", "15"), ("1", "15")]:
        raise AssertionError(f"unexpected accelerator results: {observed}")
    return outdir


def power_reporting_preserves_execution(params):
    enabled, disabled = (run_variant(params, state) for state in (True, False))
    first, second = st.read_stats(enabled), st.read_stats(disabled)
    if not first or len(first) != len(second):
        raise AssertionError("power reporting changed statistics dumps")
    for left, right in zip(first, second):
        if left["simTicks"] <= 0 or left[COMM + ".llvm_interface.cycles"] <= 0:
            raise AssertionError("accelerator execution was not measured")
        for key in set(left) | set(right):
            if key.startswith("host") or any(
                part in key
                for part in (".power_state.", ".salam_power_model.", ".ppa.")
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
                    f"power reporting changed {key}: {a} != {b}"
                )


gem5 = st.SalamGem5Fixture()
tempdir = TempdirFixture()
TestSuite(
    name="salam-ppa-timing-equivalence",
    fixtures=[
        gem5,
        tempdir,
        st.SalamHarnessFixture(
            "salam-workload",
            gem5,
            tempdir,
            "ppa_basic.ll.in",
            [2, 3],
            1,
            runs=2,
        ),
    ],
    tags=[st.SALAM_TAG, "salam-ppa"],
    tests=[
        TestFunction(
            power_reporting_preserves_execution,
            name="salam-ppa-timing-equivalence",
        )
    ],
)
