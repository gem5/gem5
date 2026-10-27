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
                "--debug-flags=SALAMScratchpad,DMA"
                + (
                    ",DeviceMMR"
                    if case.startswith(("dma-registers-", "comm-registers-"))
                    else ""
                ),
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
    "dma-registers-nc",
    "dma-registers-stream",
    "comm-registers-little",
    "comm-registers-big",
    "comm-registers-config-little",
    "comm-registers-config-big",
    "comm-registers-premap",
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


for case, diagnostic in (
    *(
        (f"comm-registers-{suffix}", diagnostic)
        for suffix, diagnostic in (
            ("zero-flags", "CommInterface flags size must be nonzero"),
            ("zero-map", "CommInterface flags and config exceed PIO size"),
            ("large-flags", "CommInterface flags and config exceed PIO size"),
            ("large-config", "CommInterface flags and config exceed PIO size"),
            (
                "overflow-config",
                "CommInterface flags and config exceed PIO size",
            ),
            ("size-read", "Unsupported CommInterface register read size 3"),
            (
                "argument-zero-size",
                "Unsupported CommInterface argument size 0",
            ),
            ("argument-size", "Unsupported CommInterface argument size 3"),
        )
    ),
    *(
        (
            f"comm-registers-{extent}-{operation}",
            f"CommInterface PIO access {diagnostic} register range",
        )
        for extent, diagnostic in (
            ("before", "precedes"),
            ("after", "exceeds"),
            ("cross", "exceeds"),
        )
        for operation in ("read", "write")
    ),
    *(
        (
            f"comm-registers-argument-{extent}",
            "CommInterface argument access exceeds register range",
        )
        for extent in ("cross", "after", "offset", "empty")
    ),
    ("dma-registers-nc-cross-read", "DMA PIO access exceeds register range"),
    ("dma-registers-nc-cross-write", "DMA PIO access exceeds register range"),
    (
        "dma-registers-nc-small-map",
        "DMA PIO size must cover 21 register bytes",
    ),
    (
        "dma-registers-stream-cross-read",
        "Stream DMA PIO access crosses register boundary",
    ),
    (
        "dma-registers-stream-cross-write",
        "Stream DMA PIO access crosses register boundary",
    ),
    (
        "dma-registers-stream-small-map",
        "stream DMA PIO size must cover 32 register bytes",
    ),
    (
        "dma-registers-stream-wide-read",
        "Unsupported stream DMA register read size 8",
    ),
):
    name = "salam-device-" + case
    TestSuite(
        name=name,
        fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
        tags=[st.SALAM_TAG, "salam-correctness", "salam-device-correctness"],
        tests=[TestFunction(device_case(case, diagnostic), name=name)],
    )
