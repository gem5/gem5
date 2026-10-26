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

# Checks the default hardware profile against the selected LLVM.

import os
import re
import subprocess

import salam_testlib as st
import yaml
from testlib import *

# inst_list.yml entries whose opcode_num is not an LLVM opcode.
SENTINELS = {"fmuladd": 99}
# inst_list.yml names that differ from the Instruction.def names.
ALIASES = {"gep": "getelementptr"}


def check_opcodes(params):
    tools = st.toolchain()
    include = st._output([tools.llvm_config, "--includedir"]).strip()
    with open(os.path.join(include, "llvm", "IR", "Instruction.def")) as f:
        opcodes = {
            name.lower(): int(number)
            for number, name in re.findall(
                r"HANDLE_\w*INST\s*\(\s*(\d+)\s*,\s*(\w+)\s*,", f.read()
            )
        }
    profile = os.path.join(config.base_dir, st.DEFAULT_PROFILE)
    with open(profile) as f:
        rel = yaml.safe_load(f)["instructions"]
    with open(os.path.join(os.path.dirname(profile), rel)) as f:
        instructions = yaml.safe_load(f)["instructions"]
    wrong = []
    for name, entry in instructions.items():
        if name in SENTINELS:
            expected = SENTINELS[name]
        else:
            llvm = ALIASES.get(name, name.removesuffix("_inst"))
            expected = opcodes.get(llvm)
        if entry["opcode_num"] != expected:
            wrong.append((name, entry["opcode_num"], expected))
    if wrong:
        raise AssertionError(
            f"opcode_num differs from LLVM {tools.llvm_version}: {wrong}"
        )


TestSuite(
    name="salam-inst-opcodes",
    fixtures=[],
    tags=[st.SALAM_TAG],
    tests=[TestFunction(check_opcodes, name="salam-inst-opcodes")],
)


def cycle_counts_rejects_memory_latency(params):
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    script = os.path.join(outdir, "cycle_counts_load.py")
    with open(script, "w") as f:
        f.write(
            "import m5\n"
            "from m5.objects import CycleCounts, Root\n"
            "root = Root(full_system=False)\n"
            "root.cycle_counts = CycleCounts(load=1)\n"
            "m5.instantiate()\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script], env=gem5.tools.env()
    )
    with open(os.path.join(outdir, "simerr.txt")) as f:
        err = f.read()
    if result.returncode == 0 or "load and store must be 0" not in err:
        raise AssertionError(
            f"CycleCounts(load=1) was not rejected (rc {result.returncode})"
        )


_gem5 = st.SalamGem5Fixture()
TestSuite(
    name="salam-cycle-counts-memory-latency",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG],
    tests=[
        TestFunction(
            cycle_counts_rejects_memory_latency,
            name="salam-cycle-counts-memory-latency",
        )
    ],
)


def from_profile_makes_new_objects(params):
    # Two accelerators' units and instructions must be distinct objects,
    # one for every parameter the generated container declares.
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    script = os.path.join(outdir, "from_profile.py")
    with open(script, "w") as f:
        f.write(
            "from m5.objects import FunctionalUnits, InstConfig\n"
            "from m5.SimObject import SimObject\n"
            "for cls in (FunctionalUnits, InstConfig):\n"
            "    a, b = cls.from_profile(), cls.from_profile()\n"
            "    names = {\n"
            "        name\n"
            "        for name, param in cls._params.items()\n"
            "        if isinstance(param.ptype, type)\n"
            "        and issubclass(param.ptype, SimObject)\n"
            "    }\n"
            "    assert names, cls\n"
            "    for obj in (a, b):\n"
            "        assert set(obj._children.keys()) == names, cls\n"
            "    for name in names:\n"
            "        assert a._children[name] is not b._children[name], name\n"
            "    print(cls.__name__, len(names), 'distinct')\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script], env=gem5.tools.env()
    )
    with open(os.path.join(outdir, "simout.txt")) as f:
        out = f.read()
    if result.returncode != 0 or out.count(" distinct") != 2:
        raise AssertionError(
            f"from_profile() check failed (rc {result.returncode}); "
            f"see {outdir}"
        )


TestSuite(
    name="salam-from-profile",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG],
    tests=[
        TestFunction(from_profile_makes_new_objects, name="salam-from-profile")
    ],
)
