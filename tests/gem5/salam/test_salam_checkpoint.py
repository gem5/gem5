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


"""SALAM checkpoint state, active traffic, and continued operation."""

import json
import os
import subprocess

import salam_testlib as st
from testlib import *

CHECKPOINT_CONFIG = os.path.join(
    os.path.dirname(__file__), "configs", "checkpoint.py"
)


def _run(params, case, phase, checkpoint):
    gem5 = params.fixtures["salam-gem5"]
    base = params.fixtures[constants.tempdir_fixture_name].path
    outdir = os.path.join(base, case + "-" + phase)
    command = [
        gem5.path,
        "-d",
        outdir,
        "-re",
        CHECKPOINT_CONFIG,
        case,
        phase,
        "--checkpoint",
        checkpoint,
    ]
    if case in ("idle", "active"):
        workload = params.fixtures["salam-workload"]
        command += ["--accpath", workload.accpath]
    result = subprocess.run(command, env=gem5.tools.env(), timeout=180)
    output = ""
    for filename in ("simout.txt", "simerr.txt"):
        path = os.path.join(outdir, filename)
        if os.path.isfile(path):
            with open(path) as stream:
                output += stream.read()
    assert result.returncode == 0, output
    assert "SALAM CHECKPOINT ROUNDTRIP " + case + " " + phase in output, output
    if phase == "save":
        assert "CHECKPOINT PRECONDITION " + case in output, output
        assert os.path.isfile(os.path.join(checkpoint, "m5.cpt")), output
    with open(os.path.join(outdir, "observation.json")) as stream:
        return json.load(stream)


def _roundtrip(params, case):
    base = params.fixtures[constants.tempdir_fixture_name].path
    checkpoint = os.path.join(base, "checkpoint-" + case)
    # The producer also continues after saving. Comparing its checked result
    # against a fresh process catches restore omissions and rescheduling bugs.
    original = _run(params, case, "save", checkpoint)
    restored = _run(params, case, "restore", checkpoint)
    assert restored == original, (case, original, restored)
    return restored


def accelerator_checkpoint(params):
    # Ordinary checkpoint support stays available in a SALAM-enabled build.
    _roundtrip(params, "control")
    for case in ("idle", "active"):
        result = _roundtrip(params, case)
        assert result["arguments"] == [19, 7]
        assert result["first"] == [26, 20]
        assert result["new_arguments"] == [35, 12]
        assert result["second"] == [47, 47]


def noncoherent_checkpoint(params):
    _roundtrip(params, "noncoherent")


def stream_dma_checkpoint(params):
    _roundtrip(params, "stream-dma")


def storage_checkpoint(params):
    _roundtrip(params, "storage")


for name, function in (
    ("accelerator", accelerator_checkpoint),
    ("noncoherent-dma", noncoherent_checkpoint),
    ("stream-dma", stream_dma_checkpoint),
    ("storage", storage_checkpoint),
):
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    fixtures = [gem5, tempdir]
    if name == "accelerator":
        fixtures.append(
            st.SalamHarnessFixture(
                "salam-workload",
                gem5,
                tempdir,
                "checkpoint.c",
                [],
                2,
                hw_config={"instructions": {"add": {"runtime_cycles": 50}}},
                ir_patterns=(r"define.*@top\(i32.*i32", r"\badd i32\b"),
                clocks={"Compute": "1GHz", "Fabric": "1GHz"},
            )
        )
    TestSuite(
        name="salam-checkpoint-" + name,
        fixtures=fixtures,
        tags=[st.SALAM_TAG],
        tests=[TestFunction(function, name="salam-checkpoint-" + name)],
    )
