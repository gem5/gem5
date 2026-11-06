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


# Trace-enabled short writes must log only the current packet's bytes.

import os
import re

import salam_testlib as st
from testlib import *


def debug_short_writes(params):
    st.run_fs(
        "salam-workload",
        180,
        1000000000,
        fs_args=[
            "--param",
            "system.harness_clstr.kernel.enable_debug_msgs=True",
        ],
        gem5_args=["--debug-flags=CommInterface"],
    )(params)
    outdir, out = st.read_simout(params)
    if not re.search(st.M5_EXIT_REGEX, out, re.M):
        raise AssertionError("traced writes did not reach m5_exit")
    with open(os.path.join(outdir, "system.terminal")) as f:
        uart = f.read()
    for index, value in enumerate([0x56783412, 0x12345678] * 3):
        if not re.search(rf"^RUN\[0\] OUT\[{index}\]={value} ", uart, re.M):
            raise AssertionError(
                f"traced write payload mismatch: OUT[{index}]"
            )
    widths = {kind: set() for kind in ("local", "spm", "reg")}
    lines = out.splitlines()
    expected = {
        1: {"12", "34"},
        2: {"78 56"},
        4: {"78 56 34 12", "12 34 78 56"},
    }
    for index, line in enumerate(lines):
        match = re.search(
            r"Trying to write to addr: .*?, (\d+) bytes through port: "
            r".*\.(local|spm|reg)\[",
            line,
        )
        if match is None:
            continue
        size, kind = int(match.group(1)), match.group(2)
        dump = re.search(r": 00000000  ((?:[0-9a-f]{2} )+)", lines[index + 1])
        if dump is None:
            raise AssertionError("traced packet has no bounded byte dump")
        data = dump.group(1).strip()
        if len(data.split()) != size or data not in expected[size]:
            raise AssertionError(f"incorrect {size}-byte dump: {data!r}")
        widths[kind].add(size)
    if any(sizes != {1, 2, 4} for sizes in widths.values()):
        raise AssertionError(f"short writes missed a port: {widths}")


_gem5 = st.SalamGem5Fixture()
_temp = TempdirFixture()
TestSuite(
    name="salam-debug-short-writes",
    fixtures=[
        _gem5,
        _temp,
        st.SalamHarnessFixture(
            "salam-workload",
            _gem5,
            _temp,
            "debug_short_writes.c",
            [],
            6,
            cache_size=16384,
            ir_patterns=[
                r"store volatile i8",
                r"store volatile i16",
                r"store volatile i32",
            ],
        ),
    ],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[TestFunction(debug_short_writes, name="salam-debug-short-writes")],
)
