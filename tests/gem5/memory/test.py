# Copyright (c) 2018 The Regents of the University of California.
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
"""
Test file for simple memory test
TODO: Add stats checking
"""
from testlib import *

gem5_verify_config(
    name="simple_mem_default",
    verifiers=(),  # No need for verfiers this will return non-zero on fail
    config=joinpath(getcwd(), "simple-run.py"),
    config_args=[],
    valid_isas=(constants.null_tag,),
    length=constants.long_tag,
)

simple_mem_params = [
    ("inf-bandwidth", {"bandwidth": "0GB/s"}),
    ("low-latency", {"latency": "1ns"}),
    ("high-latency", {"latency": "1us"}),
    ("low-bandwidth", {"bandwidth": "1MB/s"}),
    ("high-var", {"latency_var": "100ns"}),
]


for name, params in simple_mem_params:
    args = ["--" + key + "=" + val for key, val in params.items()]

    gem5_verify_config(
        name="simple_mem_" + name,
        verifiers=(),  # No need for verfiers this will return non-zero on fail
        config=joinpath(getcwd(), "simple-run.py"),
        config_args=args,
        valid_isas=(constants.null_tag,),
        length=constants.long_tag,
    )  # This tests for validity as well as performance

gem5_verify_config(
    name="memtest",
    verifiers=(),  # No need for verfiers this will return non-zero on fail
    config=joinpath(getcwd(), "memtest-run.py"),
    config_args=[],
    valid_isas=(constants.null_tag,),
    length=constants.long_tag,
)

null_tests = [
    ("garnet_synth_traffic", None, ["--sim-cycles", "5000000"]),
    ("memcheck", None, ["--maxtick", "2000000000", "--prefetchers"]),
    (
        "ruby_mem_test-garnet",
        "ruby_mem_test",
        [
            "--abs-max-tick",
            "20000000",
            "--network=garnet",
        ],
    ),
    (
        "ruby_mem_test-simple",
        "ruby_mem_test",
        [
            "--abs-max-tick",
            "20000000",
            "--functional",
            "10",
            "--network=simple",
        ],
    ),
    (
        "ruby_mem_test-simple-extra",
        "ruby_mem_test",
        [
            "--abs-max-tick",
            "20000000",
            "--functional",
            "10",
            "--network=simple",
            "--simple-physical-channels",
        ],
    ),
    (
        "ruby_mem_test-simple-extra-multicore",
        "ruby_mem_test",
        [
            "--abs-max-tick",
            "20000000",
            "--functional",
            "10",
            "--network=simple",
            "--simple-physical-channels",
            "--num-cpus=4",
        ],
    ),
    ("ruby_random_test", None, ["--maxloads", "5000"]),
    ("ruby_direct_test", None, ["--requests", "50000"]),
]

for test_name, basename_noext, args in null_tests:
    if basename_noext == None:
        basename_noext = test_name
    gem5_verify_config(
        name=test_name,
        fixtures=(),
        verifiers=(),
        config=joinpath(
            config.base_dir, "configs", "example", basename_noext + ".py"
        ),
        config_args=args,
        valid_isas=(constants.null_tag,),
        valid_hosts=constants.supported_hosts,
        length=constants.long_tag,
    )
