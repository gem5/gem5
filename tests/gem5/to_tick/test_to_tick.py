# Copyright (c) 2022 The Regents of the University of California
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
from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

# This test sets the tick to max tick via the `simulator.run` function. This is
# set to 100. Therefore, at the end of the execution the expected current tick
# should be 100, with the max tick still 100. The number of expected ticks to
# max is therefore 0.
gem5_verify_config(
    name="test-to-max-tick-at-execution-100",
    verifiers=[
        verifier.MatchStdoutNoPerf(
            joinpath(getcwd(), "ref", "tick-to-max-at-execution-100.txt")
        )
    ],
    fixtures=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "to_tick",
        "configs",
        "tick-to-max.py",
    ),
    config_args=[
        "--resource-directory",
        resource_path,
        "--set-ticks-at-execution",
        "100",
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# This test sets the max tick  via the `simulator.run` function at tick 100.
# The `m5.setMaxTick` function is then called after, passing the value 200 .
# This means at the end of execution the current tick is 100, and the max tick
# is 200. The number of expected ticks to max is therefore 100.
gem5_verify_config(
    name="test-to-max-tick-at-execution-and-after-100-200",
    verifiers=[
        verifier.MatchStdoutNoPerf(
            joinpath(
                getcwd(),
                "ref",
                "tick-to-max-at-execution-and-after-100-200.txt",
            )
        )
    ],
    fixtures=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "to_tick",
        "configs",
        "tick-to-max.py",
    ),
    config_args=[
        "--resource-directory",
        resource_path,
        "--set-ticks-at-execution",
        "100",
        "--set-ticks-after",
        "200",
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# This test sets the max tick to 250 via the `m5.setMaxTick` prior to running
# `simulator.run`. This means at the end of execution the current tick is 250
# and the max tick is 250. The expected number of ticks to max is therefore 0.
gem5_verify_config(
    name="test-to-max-tick-before-execution-250",
    verifiers=[
        verifier.MatchStdoutNoPerf(
            joinpath(getcwd(), "ref", "tick-to-max-before-execution-250.txt")
        )
    ],
    fixtures=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "to_tick",
        "configs",
        "tick-to-max.py",
    ),
    config_args=[
        "--resource-directory",
        resource_path,
        "--set-ticks-before",
        "250",
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Tests the scheduling of a tick exit event at tick 100.
gem5_verify_config(
    name="test-to-tick-exit-100",
    verifiers=[
        verifier.MatchStdoutNoPerf(
            joinpath(getcwd(), "ref", "tick-exit-100.txt")
        )
    ],
    fixtures=(),
    config=joinpath(
        config.base_dir, "tests", "gem5", "to_tick", "configs", "tick-exit.py"
    ),
    config_args=["--resource-directory", resource_path, "--tick-exits", "100"],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Tests the scheduling of a tick exit event at tick 10, 20, 30, and 40.
gem5_verify_config(
    name="test-to-tick-exit-10-20-30-40",
    verifiers=[
        verifier.MatchStdoutNoPerf(
            joinpath(getcwd(), "ref", "tick-exit-10-20-30-40.txt")
        )
    ],
    fixtures=(),
    config=joinpath(
        config.base_dir, "tests", "gem5", "to_tick", "configs", "tick-exit.py"
    ),
    config_args=[
        "--resource-directory",
        resource_path,
        "--tick-exits",
        "10",
        "20",
        "30",
        "40",
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)
