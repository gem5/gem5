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

import os
import re

import salam_testlib as st
import yaml
from testlib import (
    TempdirFixture,
    TestFunction,
    TestSuite,
)

INPUTS = (0x13579BDF, -1234567)
RUNS = 2


class StreamHarnessFixture(st.SalamHarnessFixture):
    def _write_inputs(self):
        super()._write_inputs()
        path = os.path.join(self.accpath, "config.yml")
        with open(path) as stream:
            document = yaml.safe_load(stream)
        accelerator = next(
            item["Accelerator"]
            for item in document["acc_cluster"]
            if "Accelerator" in item
        )
        accelerator.append(
            {
                "Var": [
                    {
                        "Name": "PIPE",
                        "Type": "Stream",
                        "StreamSize": 4,
                        "BufferSize": 8,
                        "InCon": "Kernel",
                        "OutCon": "Kernel",
                    }
                ]
            }
        )
        with open(path, "w") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)


def check_roundtrip(params):
    outdir, simout = st.read_simout(params)
    if not re.search(st.M5_EXIT_REGEX, simout, re.MULTILINE):
        raise AssertionError("stream kernel did not reach m5_exit")
    with open(os.path.join(outdir, "system.terminal")) as stream:
        observed = [
            tuple(map(int, match))
            for match in re.findall(
                r"^RUN\[(\d+)\] OUT\[(\d+)\]=(-?\d+) ",
                stream.read(),
                re.MULTILINE,
            )
        ]
    expected = [
        (run, index, value)
        for run in range(RUNS)
        for index, value in enumerate(INPUTS)
    ]
    if observed != expected:
        raise AssertionError(f"stream payloads {observed} != {expected}")
    stats = st.read_stats(outdir)[-1]
    # Two 32-bit words cross the stream in each direction per invocation.
    for direction in ("read", "write"):
        key = f"system.harness_clstr.pipe.stream.{direction}Bytes"
        if stats[key] != 16:
            raise AssertionError(f"{key}: {stats[key]} != 16")


gem5 = st.SalamGem5Fixture()
tempdir = TempdirFixture()
workload = StreamHarnessFixture(
    "salam-workload",
    gem5,
    tempdir,
    "stream_roundtrip.c",
    INPUTS,
    n_out=2,
    runs=RUNS,
)
TestSuite(
    name="salam-kernel-stream-roundtrip",
    fixtures=[gem5, tempdir, workload],
    tags=[st.SALAM_TAG, "salam-correctness", "salam-stream-correctness"],
    tests=[
        TestFunction(
            st.run_fs("salam-workload", 180, 1000000000),
            name="salam-kernel-stream-roundtrip",
        ),
        TestFunction(check_roundtrip, name="salam-stream-payload-and-bytes"),
    ],
)
