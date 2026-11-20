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


# Actual device transport tests, without compiling an accelerator workload.
import os
import subprocess

import salam_testlib as st
from testlib import *


def device_case(case, diagnostic=None):
    def run(params):
        gem5 = params.fixtures["salam-gem5"]
        outdir = params.fixtures[constants.tempdir_fixture_name].path
        result = subprocess.run(
            [
                gem5.path,
                "-d",
                outdir,
                "-re",
                os.path.join(
                    os.path.dirname(__file__),
                    (
                        "dma_check.py"
                        if case.startswith(("dma-", "comm-registers-"))
                        else "device_check.py"
                    ),
                ),
                case,
            ],
            env=gem5.tools.env(),
            timeout=60,
        )
        if diagnostic is not None:
            assert result.returncode != 0, f"{case} succeeded: {outdir}"
            with open(os.path.join(outdir, "simerr.txt")) as stream:
                assert diagnostic in stream.read()
            return
        assert result.returncode == 0, f"{case} failed: {outdir}"
        with open(os.path.join(outdir, "simout.txt")) as stream:
            output = stream.read()
            assert f"DEVICE CHECK PASSED {case}" in output

    return run


# Keep one configured representative for each protocol or regression.
for case in (
    "register-completion-0-0-0-1",
    "register-completion-10-30-1-0",
    "register-batch-1-0",
    "register-write-blocked-0-1",
    "register-overlap",
    "register-blocked",
    "register-boundaries",
    "spm-boundaries",
    "spm-blocked",
    "stream-empty",
    "stream-full",
    "dma-registers-nc",
    "dma-registers-stream",
    "comm-registers-config-big",
    "comm-registers-premap",
    "dma-fifo-timing",
    "dma-fifo-bypass-small-backdoor",
    "dma-tail",
    "dma-atomic",
    "dma-read-ready",
    "dma-write-ready",
):
    name = "salam-device-" + case
    TestSuite(
        name=name,
        fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
        tags=[st.SALAM_TAG, "salam-correctness", "salam-device-correctness"],
        tests=[TestFunction(device_case(case), name=name)],
    )


# Invalid layout, argument extent, and both DMA register implementations.
for case, diagnostic in (
    ("comm-registers-zero-flags", "CommInterface flags size must be nonzero"),
    (
        "comm-registers-argument-cross",
        "CommInterface argument access exceeds register range",
    ),
    ("dma-registers-nc-cross-read", "DMA PIO access exceeds register range"),
    (
        "dma-registers-stream-cross-write",
        "Stream DMA PIO access crosses register boundary",
    ),
):
    name = "salam-device-" + case
    TestSuite(
        name=name,
        fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
        tags=[st.SALAM_TAG, "salam-correctness", "salam-device-correctness"],
        tests=[TestFunction(device_case(case, diagnostic), name=name)],
    )
