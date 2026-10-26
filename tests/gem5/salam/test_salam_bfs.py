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

# Checked BFS regression for SALAM.
#
# Run explicitly; these suites carry only the "salam" tag:
#   ./main.py run gem5/salam --include-tags salam
#
# BFS is built from a fresh copy of the tracked sources with CHECK
# enabled, and the host program's own checker decides the result.

import os
import re
import subprocess

import salam_testlib as st
from testlib import *

BFS_WALL = 300
BFS_TICKS = 4000000000


def bfs_suite(name, edits, verifiers):
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    workload = st.SalamWorkloadFixture(
        "salam-workload", gem5, tempdir, "bfs", ["bfs", "common"], edits
    )
    tests = [
        TestFunction(
            st.run_fs("salam-workload", BFS_WALL, BFS_TICKS), name=name
        )
    ]
    tests += [
        v.instantiate_test(f"{name}-{label}") for label, v in verifiers.items()
    ]
    return TestSuite(
        name=name,
        fixtures=[gem5, tempdir, workload],
        tags=[st.SALAM_TAG],
        tests=tests,
    )


def wrong_expected_count(src):
    st.replace_once(
        src + "/bfs/sw/data.c",
        "chk[N_LEVELS] = {1, 26,",
        "chk[N_LEVELS] = {2, 26,",
    )


bfs_suite(
    "salam-bfs-check",
    [st.enable_check("bfs")],
    {
        "passed": verifier.MatchFileRegex(
            re.compile(r"^Check Passed$"), ["system.terminal"]
        ),
        "no-mismatch": st.NoMatchFileRegex(
            re.compile(r"^(Mismatch|Check Failed)"), ["system.terminal"]
        ),
        "m5-exit": verifier.MatchFileRegex(
            re.compile(st.M5_EXIT_REGEX), ["simout.txt"]
        ),
    },
)


def set_clocks(clocks):
    # Give the BFS cluster a Clocks item.
    def edit(src):
        import yaml

        path = os.path.join(src, "bfs", "config.yml")
        with open(path) as f:
            document = yaml.safe_load(f)
        document["acc_cluster"].append({"Clocks": dict(clocks)})
        with open(path, "w") as f:
            f.write("---\n")
            yaml.safe_dump(document, f, sort_keys=False)

    return edit


# The checked results do not depend on the cluster clocks.
bfs_suite(
    "salam-bfs-check-clocks",
    [
        st.enable_check("bfs"),
        set_clocks({"Compute": "50MHz", "Fabric": "500MHz"}),
    ],
    {
        "passed": verifier.MatchFileRegex(
            re.compile(r"^Check Passed$"), ["system.terminal"]
        ),
        "no-mismatch": st.NoMatchFileRegex(
            re.compile(r"^(Mismatch|Check Failed)"), ["system.terminal"]
        ),
        "m5-exit": verifier.MatchFileRegex(
            re.compile(st.M5_EXIT_REGEX), ["simout.txt"]
        ),
    },
)

bfs_suite(
    "salam-bfs-negative-control",
    [st.enable_check("bfs"), wrong_expected_count],
    {
        "failed": verifier.MatchFileRegex(
            re.compile(r"^Check Failed$"), ["system.terminal"]
        ),
        "mismatch": verifier.MatchFileRegex(
            re.compile(r"^Mismatch: 1 found, 2 expected"), ["system.terminal"]
        ),
        "not-passed": st.NoMatchFileRegex(
            re.compile(r"^Check Passed$"), ["system.terminal"]
        ),
        "m5-exit": verifier.MatchFileRegex(
            re.compile(st.M5_EXIT_REGEX), ["simout.txt"]
        ),
    },
)


def clean_restores_sources(params):
    wl = params.fixtures["salam-workload"]
    src = os.path.join(wl.root, "src")
    subprocess.run(
        [wl.gem5.tools.make, "-C", os.path.join(src, "bfs"), "clean"],
        env=wl.build_env,
        check=True,
        capture_output=True,
    )
    left = set()
    for dirpath, _dirnames, filenames in os.walk(src):
        for name in filenames:
            path = os.path.relpath(os.path.join(dirpath, name), src)
            left.add(os.path.join(st.BENCH_DIR, path))
    extra = sorted(left - set(wl.manifest["inputs"]))
    if extra:
        raise AssertionError(f"make clean left build products: {extra}")


_clean_gem5 = st.SalamGem5Fixture()
_clean_tempdir = TempdirFixture()
TestSuite(
    name="salam-bfs-clean",
    fixtures=[
        _clean_gem5,
        _clean_tempdir,
        st.SalamWorkloadFixture(
            "salam-workload",
            _clean_gem5,
            _clean_tempdir,
            "bfs",
            ["bfs", "common"],
            [st.enable_check("bfs")],
        ),
    ],
    tags=[st.SALAM_TAG],
    tests=[TestFunction(clean_restores_sources, name="salam-bfs-clean")],
)
