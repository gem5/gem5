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
import json
import os
import re
import subprocess

import salam_testlib as st
from testlib import *


def device_case(case):
    def run(params):
        gem5 = params.fixtures["salam-gem5"]
        outdir = params.fixtures[constants.tempdir_fixture_name].path
        result = subprocess.run(
            [
                gem5.path,
                "-d",
                outdir,
                "-re",
                "--debug-flags=SALAMScratchpad,DMA",
                os.path.join(
                    os.path.dirname(__file__),
                    (
                        "dma_check.py"
                        if case.startswith("dma-")
                        else "device_check.py"
                    ),
                ),
                case,
            ],
            env=gem5.tools.env(),
            timeout=60,
        )
        assert result.returncode == 0, f"{case} failed: {outdir}"
        with open(os.path.join(outdir, "simout.txt")) as stream:
            output = stream.read()
            assert f"DEVICE CHECK PASSED {case}" in output
        if case == "spm-order":
            ready = {
                int(addr, 16): int(tick)
                for addr, tick in re.findall(
                    r"response (0x[0-9a-f]+|0) ready at ([0-9]+)", output
                )
            }
            with open(os.path.join(outdir, "device-results.json")) as stream:
                responses = json.load(stream)["responses"][0]
            assert len(ready) == len(responses) == 8
            for tick, addr, size, data in responses:
                assert tick == ready[addr], (addr, tick, ready[addr])

    return run


for case in (
    *(
        f"register-completion-{access}-{write}-{writer}-{reader}"
        for access, write in ((10, 0), (10, 5), (10, 10), (10, 30), (0, 0))
        for writer in (0, 1)
        for reader in (0, 1)
    ),
    *(
        f"register-{kind}-{writer}-{reader}"
        for kind in ("batch", "write-blocked")
        for writer in (0, 1)
        for reader in (0, 1)
    ),
    "register-functional",
    "register-overlap",
    "register-blocked",
    "register-boundaries",
    "spm-boundaries",
    "spm-order",
    "spm-blocked",
    "stream-empty",
    "stream-full",
    "dma-fifo-bypass",
    "dma-fifo-bypass-backdoor",
    "dma-fifo-bypass-small",
    "dma-fifo-bypass-small-backdoor",
    "dma-fifo-timing",
    "dma-two",
    "dma-one",
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
