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


# Unaligned and partial-word traffic must preserve storage contents.

import json
import os
import re
import subprocess

import salam_testlib as st
from testlib import (
    TempdirFixture,
    TestFunction,
    TestSuite,
    constants,
)


def byte_enable_traffic(params):
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    parameters = os.path.join(outdir, "storage_parameters.json")
    with open(parameters, "w") as stream:
        json.dump(
            dict(
                kind="spm",
                requestors=1,
                reset=False,
                bandwidth="128MiB/s",
                model=dict(enabled=False),
            ),
            stream,
        )
    script = os.path.join(
        os.path.dirname(__file__), "configs", "storage_traffic.py"
    )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script, parameters],
        env=gem5.tools.env(),
        timeout=180,
    )
    with open(os.path.join(outdir, "simout.txt")) as stream:
        output = stream.read()
    if result.returncode or not re.search(
        r"^STORAGE_CHECK_PASSED \d+$", output, re.M
    ):
        raise AssertionError("partial-word storage payload check did not pass")
    with open(os.path.join(outdir, "config.json")) as stream:
        configuration = json.load(stream)
    prefix = configuration["system"]["generators"][0]["path"] + "."
    stats = st.read_stats(outdir)[-1]
    # The config issues three byte writes, two unaligned four-byte writes,
    # and four four-byte reads; its payload check verifies the changed bytes.
    for suffix, expected in (
        ("totalReads", 4),
        ("bytesRead", 16),
        ("totalWrites", 5),
        ("bytesWritten", 11),
    ):
        if stats[prefix + suffix] != expected:
            raise AssertionError(
                f"incomplete storage traffic: {prefix + suffix}"
            )


TestSuite(
    name="salam-storage-byte-enable",
    fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[
        TestFunction(byte_enable_traffic, name="salam-storage-byte-enable")
    ],
)
