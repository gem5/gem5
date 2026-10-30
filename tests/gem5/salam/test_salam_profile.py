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

import json
import os
import re
import shutil
import subprocess

import salam_testlib as st
import yaml
from testlib import *

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


def cycle_counts_checks_control_latency(params):
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    for instruction in ("br", "ret", "switch_inst", "call"):
        for cycles in (0, 7):
            run_dir = os.path.join(outdir, f"{instruction}-{cycles}")
            os.makedirs(run_dir)
            script = os.path.join(run_dir, "cycle_counts.py")
            with open(script, "w") as f:
                f.write(
                    "import m5\n"
                    "from m5.objects import CycleCounts, Root\n"
                    "root = Root(full_system=False)\n"
                    "root.cycle_counts = CycleCounts(**"
                    f"{{{instruction!r}: {cycles}}})\n"
                    "m5.instantiate()\n"
                    "print('CycleCounts instantiated')\n"
                )
            result = subprocess.run(
                [gem5.path, "-d", run_dir, "-re", script],
                env=gem5.tools.env(),
            )
            with open(os.path.join(run_dir, "simerr.txt")) as f:
                err = f.read()
            if cycles and instruction != "call":
                if (
                    result.returncode == 0
                    or "terminator latency is not modeled" not in err
                    or instruction not in err
                ):
                    raise AssertionError(
                        f"CycleCounts({instruction}={cycles}) was not "
                        f"rejected with its diagnostic; see {run_dir}"
                    )
            else:
                with open(os.path.join(run_dir, "simout.txt")) as f:
                    out = f.read()
                if (
                    result.returncode != 0
                    or "CycleCounts instantiated" not in out
                ):
                    raise AssertionError(
                        f"CycleCounts({instruction}={cycles}) failed; "
                        f"see {run_dir}"
                    )


TestSuite(
    name="salam-cycle-counts-control-latency",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[
        TestFunction(
            cycle_counts_checks_control_latency,
            name="salam-cycle-counts-control-latency",
        )
    ],
)


def from_profile_makes_new_objects(params):
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    script = os.path.join(
        os.path.dirname(__file__), "check_salam_ownership.py"
    )
    expected = st._expected_profile(gem5.tools, gem5.profile)
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script, json.dumps(expected)],
        env=gem5.tools.env(),
        timeout=60,
    )
    with open(os.path.join(outdir, "simout.txt")) as stream:
        output = stream.read()
    if result.returncode or "SALAM_PROFILE_OWNERSHIP_PASSED" not in output:
        raise AssertionError(
            f"profile ownership check failed (rc {result.returncode}); "
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


def dma_config_constructs_devices(params):
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    records = []
    for dma_type, prefix in (("NonCoherent", "nc"), ("Stream", "stream")):
        for suffix, sizes in (
            ("default", {}),
            ("explicit", {"BufferSize": 512, "MaxReqSize": 64}),
            ("buffer", {"BufferSize": 256}),
            ("request", {"MaxReqSize": 32}),
        ):
            record = {
                "Name": f"{prefix}_{suffix}",
                "Type": dma_type,
                "PIOMaster": "LocalBus",
                **sizes,
            }
            if suffix == "explicit":
                record.update(
                    {"InterruptNum": 68}
                    if dma_type == "NonCoherent"
                    else {"ReadInt": 0, "WriteInt": 255}
                )
            records.append(record)
    with open(os.path.join(outdir, "config.yml"), "w") as f:
        yaml.safe_dump(
            {"acc_cluster": [{"Name": "clstr"}, {"DMA": records}]}, f
        )
    script = os.path.join(outdir, "dma_config.py")
    with open(script, "w") as f:
        f.write(
            "from m5.objects import (IOXBar, NoncoherentDma, Root, "
            "StreamDma, System, SystemXBar, VExpress_GEM5_V1)\n"
            "from gem5.salam.cluster import build_clusters\n"
            "root = Root(full_system=False)\n"
            "root.system = system = System(cache_line_size=128)\n"
            "system.iobus = IOXBar()\n"
            "system.membus = SystemXBar()\n"
            "system.realview = VExpress_GEM5_V1()\n"
            f"build_clusters(system, {outdir!r})\n"
            f"records = {records!r}\n"
            "for record in records:\n"
            "    dma = getattr(system.clstr, record['Name'])\n"
            "    stream = record['Type'] == 'Stream'\n"
            "    assert isinstance(dma, StreamDma if stream "
            "else NoncoherentDma)\n"
            "    capacity = record.get('BufferSize', 1024)\n"
            "    request = record.get('MaxReqSize', 128)\n"
            "    assert int(dma.max_req_size.unproxy(dma)) == request\n"
            "    assert int(dma.max_pending) == 8\n"
            "    if stream:\n"
            "        assert int(dma.read_buffer_size) == capacity\n"
            "        assert int(dma.write_buffer_size) == capacity\n"
            "        assert int(dma.stream_size) == 8\n"
            "        assert int(dma.rd_int) == record.get('ReadInt', 210)\n"
            "        assert int(dma.wr_int) == record.get('WriteInt', 211)\n"
            "    else:\n"
            "        assert int(dma.buffer_size) == capacity\n"
            "        assert int(dma.int_num) == "
            "record.get('InterruptNum', 200)\n"
            "    print(record['Name'], capacity, request, 'checked')\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script], env=gem5.tools.env()
    )
    with open(os.path.join(outdir, "simout.txt")) as f:
        out = f.read()
    if result.returncode != 0 or out.count(" checked") != len(records):
        raise AssertionError(
            f"constructed DMA configuration failed; see {outdir}"
        )


TestSuite(
    name="salam-dma-config",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[
        TestFunction(dma_config_constructs_devices, name="salam-dma-config")
    ],
)


def cache_config_constructs_devices(params):
    # Exercise accepted workload spellings through real MemorySize params.
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    cases = (
        (32768, 32768),
        ("32768", 32768),
        ("32Ki", 32768),
        ("32k", 32768),
        ("0x8000", 32768),
        ("1Mi", 1048576),
        ("+32Ki", 32768),
        ("0o40Ki", 32768),
        ("0b100000Ki", 32768),
    )
    cluster = [{"Name": "clstr"}]
    for index, (value, _) in enumerate(cases):
        cluster.append(
            {
                "Accelerator": [
                    {
                        "Name": f"kernel_{index}",
                        "IrPath": "hw/kernel.ll",
                        "Profile": "kernel",
                        "PIOSize": 5,
                        "PIOMaster": "LocalBus",
                    },
                    {
                        "Var": [
                            {
                                "Name": f"cache_{index}",
                                "Type": "Cache",
                                "Size": value,
                            }
                        ]
                    },
                ]
            }
        )
    with open(os.path.join(outdir, "config.yml"), "w") as f:
        yaml.safe_dump(
            {"acc_cluster": cluster, "hw_config": {"kernel": None}}, f
        )
    script = os.path.join(outdir, "cache_config.py")
    with open(script, "w") as f:
        f.write(
            "from m5.objects import (IOXBar, Root, System, "
            "SystemXBar, VExpress_GEM5_V1)\n"
            "from gem5.salam.cluster import AcceleratorCache, "
            "build_clusters\n"
            "root = Root(full_system=False)\n"
            "root.system = system = System()\n"
            "system.iobus = IOXBar()\n"
            "system.membus = SystemXBar()\n"
            "system.realview = VExpress_GEM5_V1()\n"
            f"build_clusters(system, {outdir!r})\n"
            f"cases = {cases!r}\n"
            "for index, (value, expected) in enumerate(cases):\n"
            "    cache = getattr(system.clstr, f'cache_{index}')\n"
            "    assert isinstance(cache, AcceleratorCache), "
            "(value, type(cache))\n"
            "    assert int(cache.size) == expected, "
            "(value, cache.size, expected)\n"
            "    print(repr(value), expected, 'checked')\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script], env=gem5.tools.env()
    )
    with open(os.path.join(outdir, "simout.txt")) as f:
        out = f.read()
    if result.returncode != 0 or out.count(" checked") != len(cases):
        raise AssertionError(
            f"constructed cache configuration failed; see {outdir}"
        )


TestSuite(
    name="salam-cache-config",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[
        TestFunction(
            cache_config_constructs_devices, name="salam-cache-config"
        )
    ],
)


def connection_config_constructs_devices(params):
    # Exercise actual embedded SimObject parameter and port construction.
    # This does not instantiate or simulate the configured accelerators.
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    items = [
        {"Name": "clstr"},
        {"Clocks": {"Compute": "100MHz", "Fabric": "10ns"}},
        {
            "DMA": [
                {"Name": name, "Type": kind, "PIOMaster": "local_bus"}
                for name, kind in (
                    ("ndma", "NonCoherent"),
                    ("same", "Stream"),
                    ("split", "Stream"),
                )
            ]
        },
    ]
    for name, connections in (
        ("A", {"StreamIn": "SAME", "StreamOut": "same"}),
        ("B", {"StreamIn": "split"}),
        ("C", {"StreamOut": "SPLIT"}),
    ):
        fields = {
            "Name": name,
            "IrPath": "unused.ll",
            "Profile": "k",
            "PIOSize": 5,
            "PIOMaster": "LocalBus",
            "LocalSlaves": "LOCAL_BUS",
            **connections,
        }
        if name == "B":
            fields["Var"] = [
                {
                    "Name": "Flow",
                    "Type": "Stream",
                    "StreamSize": 8,
                    "InCon": "B",
                    "OutCon": "C",
                }
            ]
        items.append({"Accelerator": [fields]})
    with open(os.path.join(outdir, "config.yml"), "w") as stream:
        yaml.safe_dump(
            {"acc_cluster": items, "hw_config": {"k": None}}, stream
        )
    script = os.path.join(outdir, "connection_config.py")
    with open(script, "w") as stream:
        stream.write(
            "from m5.objects import (IOXBar, Root, System, SystemXBar, "
            "VExpress_GEM5_V1)\n"
            "from gem5.salam.cluster import build_clusters\n"
            "root = Root(full_system=False)\n"
            "root.system = system = System(cache_line_size=128)\n"
            "system.iobus = IOXBar()\n"
            "system.membus = SystemXBar()\n"
            "system.realview = VExpress_GEM5_V1()\n"
            f"build_clusters(system, {outdir!r})\n"
            "c = system.clstr\n"
            "for obj in (c.ndma, c.same, c.split, c.a, c.b, c.c):\n"
            "    assert obj.pio.peer.simobj is c.local_bus\n"
            "for obj in (c.a, c.b, c.c):\n"
            "    assert obj.local[0].peer.simobj is c.local_bus\n"
            "assert c.same.stream_in.peer.simobj is c.a\n"
            "assert c.same.stream_out.peer.simobj is c.a\n"
            "assert len(c.a.stream) == 2\n"
            "assert c.split.stream_in.peer.simobj is c.b\n"
            "assert c.split.stream_out.peer.simobj is c.c\n"
            "assert c.flow.stream_in.peer.simobj is c.b\n"
            "assert c.flow.stream_out.peer.simobj is c.c\n"
            "for clock in (c.compute_clk_domain, c.fabric_clk_domain):\n"
            "    assert abs(clock.clock[0].value - 1e-8) < 1e-20\n"
            "print('SALAM connection configuration checked')\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", script],
        env=gem5.tools.env(),
        timeout=60,
    )
    with open(os.path.join(outdir, "simout.txt")) as stream:
        output = stream.read()
    marker = "SALAM connection configuration checked"
    if result.returncode != 0 or output.splitlines().count(marker) != 1:
        raise AssertionError(
            f"constructed connection configuration failed; see {outdir}"
        )


TestSuite(
    name="salam-connection-config",
    fixtures=[_gem5, TempdirFixture()],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[
        TestFunction(
            connection_config_constructs_devices,
            name="salam-connection-config",
        )
    ],
)


# These profiles replace the selected binary. Run each suite separately,
# then rebuild the default profile before running ordinary SALAM suites.
def ownership_profile(singleton):
    def edit(directory):
        config_path = os.path.join(directory, "salam-hw-config.yml")
        with open(config_path) as stream:
            selected = yaml.safe_load(stream)
        instruction_path = os.path.join(directory, selected["instructions"])
        with open(instruction_path) as stream:
            instructions = yaml.safe_load(stream)
        if singleton:
            units = os.path.join(directory, selected["functional_units"])
            for alias in selected["functional_unit_order"]:
                if alias != "integer_adder":
                    shutil.rmtree(os.path.join(units, alias))
            selected["functional_unit_order"] = ["integer_adder"]
            unit_path = os.path.join(
                units, "integer_adder", "integer_adder.yml"
            )
            with open(unit_path) as stream:
                unit = yaml.safe_load(stream)
            unit["functional_unit"]["parameters"]["instructions"] = ["add"]
            with open(unit_path, "w") as stream:
                yaml.safe_dump(unit, stream, sort_keys=False)
            instructions["instructions"] = {
                "add": instructions["instructions"]["add"]
            }
        else:
            selected["functional_unit_order"].reverse()
            instructions["instructions"] = dict(
                reversed(list(instructions["instructions"].items()))
            )
        with open(config_path, "w") as stream:
            yaml.safe_dump(selected, stream, sort_keys=False)
        with open(instruction_path, "w") as stream:
            yaml.safe_dump(instructions, stream, sort_keys=False)

    return edit


for profile_name, singleton in (("singleton", True), ("reordered", False)):
    name = "salam-profile-" + profile_name
    selected = os.path.join(config.build_dir, name, "salam-hw-config.yml")
    TestSuite(
        name=name,
        fixtures=[
            st.SalamProfileFixture(selected, ownership_profile(singleton)),
            st.SalamGem5Fixture(selected),
            TempdirFixture(),
        ],
        tags=["profile-ownership", "profile-" + profile_name],
        tests=[TestFunction(from_profile_makes_new_objects, name=name)],
    )
