# Copyright (c) 2026 - The Unviersity of Wisconsin
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

import math
import re
from pathlib import Path
from statistics import mean

from testlib import (
    Gem5Fixture,
    TempdirFixture,
    TestFunction,
    TestSuite,
    config,
    constants,
)
from testlib.helper import log_call

POWER_STATS = "board.processor.cores.core.power_model.pm0."


def run_example(
    params, mode, *args, debug_options=("--debug-flags=PowerLog",)
):
    gem5 = params.fixtures[constants.gem5_binary_fixture_name].path
    out = Path(params.fixtures[constants.tempdir_fixture_name].path) / mode
    example = (
        Path(config.base_dir)
        / "configs/example/gem5_library/arm-power-modeling-example.py"
    )
    log_call(
        params.log,
        [gem5, "-d", out, "-re", *debug_options, example, *args],
        time=params.time,
    )
    return out


def read_power_stats(out):
    stats = (out / constants.gem5_simulation_stats).read_text()
    return {
        name: float(value)
        for name, value in re.findall(
            rf"^{re.escape(POWER_STATS)}(\S+)\s+(\S+)", stats, re.MULTILINE
        )
    }


def check_sampling(params):
    for mode, interval in (("unsampled", "0"), ("sampled", "1000")):
        args = ("--pwr-interval", interval)
        if mode == "sampled":
            out = run_example(
                params,
                mode,
                *args,
                "--power-log=power.log",
                debug_options=(
                    "--debug-flags=PwrIntervalEvent",
                    "--debug-file=debug.log",
                ),
            )
        else:
            out = run_example(params, mode, *args)

        stdout = (out / constants.gem5_simulation_stdout).read_text()
        assert "Hello world!" in stdout
        report = stdout
        if mode == "sampled":
            report = (out / "power.log").read_text()
            debug = (out / "debug.log").read_text()
            assert "Starting sampling of" in debug
            assert "Starting sampling of" not in report
            assert "Runtime Dynamic =" not in debug

        samples = re.findall(
            r"Runtime Dynamic = (\S+) W\n  Total Static = (\S+) W", report
        )
        dynamic = [float(dyn) for dyn, _ in samples]
        static = [float(st) for _, st in samples]
        assert dynamic and all(value >= 0 for value in dynamic)
        assert max(dynamic) > 0 and all(value > 0 for value in static)

        stats = read_power_stats(out)
        if mode == "unsampled":
            assert len(samples) == 1
            assert "powerDist::samples" not in stats
            continue

        assert len(samples) > 1
        for name, values in (
            ("power", [d + s for d, s in zip(dynamic, static)]),
            ("dynamicPower", dynamic),
            ("staticPower", static),
        ):
            assert stats[name + "Dist::samples"] == len(values)
            for suffix, expected in (
                ("Dist::mean", mean(values)),
                ("Min", min(values)),
                ("Max", max(values)),
            ):
                actual = stats[name + suffix]
                assert math.isclose(
                    actual, expected, abs_tol=5e-7
                ), f"{name}{suffix}: {actual} != {expected}"


for isa in (constants.all_compiled_tag, constants.arm_tag):
    for host in constants.supported_hosts:
        for variant in (constants.opt_tag, constants.debug_tag):
            TestSuite(
                name=f"test-arm-power-modeling-{isa}-{host}-{variant}",
                fixtures=(Gem5Fixture(isa, variant),),
                tests=(
                    TestFunction(check_sampling, fixtures=(TempdirFixture(),)),
                ),
                tags=(isa, host, variant, constants.quick_tag),
            )
