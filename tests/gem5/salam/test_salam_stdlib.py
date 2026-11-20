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


# Checks of the gem5 standard library SALAM board.

import configparser
import json
import os
import re
import subprocess

import salam_testlib as st
from testlib import *

BOARD_CHECK = os.path.join(os.path.dirname(__file__), "board_check.py")
PARITY_STDLIB = "tests/gem5/salam/legacy_parity.py"


def _board_check(params, *options):
    # Runs board_check.py on the harness workload; returns its result and
    # output directory.
    gem5 = params.fixtures["salam-gem5"]
    harness = params.fixtures["salam-workload"]
    outdir = os.path.join(
        params.fixtures[constants.tempdir_fixture_name].path,
        "-".join(o.lstrip("-") for o in options) or "board",
    )
    command = [
        gem5.path,
        "-d",
        outdir,
        "-re",
        BOARD_CHECK,
        "--binary",
        harness.elf,
        "--accpath",
        harness.accpath,
        *options,
    ]
    result = subprocess.run(command, env=gem5.tools.env(), timeout=180)
    return result, outdir


def _read(outdir, name):
    with open(os.path.join(outdir, name)) as f:
        return f.read()


def board_instantiates(params):
    # A bare-metal workload entered at the ELF's entry point: no device
    # tree, no bootloader, and UART 0 ends the run at end of transmission.
    # The cluster's PIO bus hangs off the I/O bus, its coherency bus (and
    # through it the DMA) off the hierarchy's memory bus, and the
    # accelerator interrupts the platform's GIC.
    result, outdir = _board_check(params)
    if result.returncode != 0:
        raise AssertionError(f"board check failed; see {outdir}")
    out = _read(outdir, "simout.txt")
    for line in ("attached harness_clstr", "instantiated"):
        if line not in out.splitlines():
            raise AssertionError(f"no {line!r} in {outdir}/simout.txt")
    ini = configparser.ConfigParser(interpolation=None, strict=False)
    ini.optionxform = str
    ini.read(os.path.join(outdir, "config.ini"))
    harness = params.fixtures["salam-workload"]
    clstr = "board.harness_clstr"
    expected = [
        ("board.workload", "type", "ArmFsWorkload"),
        ("board.workload", "object_file", harness.elf),
        ("board.workload", "dtb_addr", "0"),
        ("board.workload", "dtb_filename", ""),
        ("board.workload", "boot_loader", ""),
        ("board.realview.uart0", "end_on_eot", "true"),
        (clstr + ".local_bus", "cpu_side_ports", "board.iobus.mem_side_ports"),
        (
            clstr + ".coherency_bus",
            "mem_side_ports",
            "board.cache_hierarchy.membus.cpu_side_ports",
        ),
        (clstr + ".dma", "dma", clstr + ".coherency_bus.cpu_side_ports"),
        (clstr + ".kernel", "gic", "board.realview.gic"),
        (clstr + ".kernel", "int_num", "68"),
    ]
    for section, key, value in expected:
        found = ini.get(section, key, fallback=None)
        # A connected port names its peer with an index.
        if found is None or found.split("[")[0] != value:
            raise AssertionError(f"[{section}] {key}={found!r}, not {value}")
    if os.path.exists(os.path.join(outdir, "device.dtb")):
        raise AssertionError("a device tree was generated")


def board_rejects_ruby(params):
    # The board connects only classic hierarchies with a memory bus.
    result, outdir = _board_check(params, "--ruby")
    err = _read(outdir, "simerr.txt")
    if result.returncode == 0 or (
        "SALAMArmBoard needs a classic cache hierarchy with a membus, "
        "not PrivateL1CacheHierarchy" not in err
    ):
        raise AssertionError(f"Ruby hierarchy not rejected; see {outdir}")


def board_needs_program_first(params):
    # Accelerators attach to the platform the program's setup creates.
    result, outdir = _board_check(params, "--attach-first")
    err = _read(outdir, "simerr.txt")
    if result.returncode == 0 or (
        "set_bare_metal_binary must be called before attach_accelerators"
        not in err
    ):
        raise AssertionError(f"early attach not rejected; see {outdir}")


_gem5 = st.SalamGem5Fixture()
_tempdir = TempdirFixture()
TestSuite(
    name="salam-stdlib-board",
    fixtures=[
        _gem5,
        _tempdir,
        st.SalamHarnessFixture(
            "salam-workload", _gem5, _tempdir, "sanity.c", [2, 3], 1
        ),
    ],
    tags=[st.SALAM_TAG],
    tests=[
        TestFunction(board_instantiates, name="salam-stdlib-board"),
        TestFunction(board_rejects_ruby, name="salam-stdlib-board-ruby"),
        TestFunction(
            board_needs_program_first, name="salam-stdlib-board-order"
        ),
    ],
)


def schema_parameters(params):
    import yaml

    harness = params.fixtures["salam-workload"]
    path = os.path.join(harness.accpath, "config.yml")
    with open(path) as stream:
        document = yaml.safe_load(stream)
    items = document["acc_cluster"]
    for item in items:
        if "DMA" in item:
            for dma in item["DMA"]:
                dma.pop("BufferSize", None)
                dma.pop("MaxReqSize", None)
        if "Accelerator" not in item:
            continue
        acc = item["Accelerator"]
        acc[0]["IrPath"] = harness.ir
        for entry in acc:
            for variable in entry.get("Var", []):
                if variable["Type"] == "SPM":
                    variable.update(
                        ReadyMode="false",
                        ResetOnRead="0",
                        ReadOnInvalid="yes",
                        WriteOnValid="n",
                    )
        acc.append(
            {
                "Var": [
                    {
                        "Name": "Flow",
                        "Type": "Stream",
                        "StreamSize": 8,
                        "InCon": "Kernel",
                        "OutCon": "Other",
                    },
                    {
                        "Name": "Cache",
                        "Type": "Cache",
                        "Size": "32Ki",
                    },
                ]
            }
        )
        acc[0]["StreamIn"] = "sDMA"
        acc[0]["StreamOut"] = "SDMA"
    items.extend(
        [
            {
                "DMA": [
                    {
                        "Name": "sdma",
                        "Type": "Stream",
                        "PIOMaster": "localbus",
                        "BufferSize": 512,
                        "MaxReqSize": 32,
                    }
                ]
            },
            {
                "Accelerator": [
                    {
                        "Name": "Other",
                        "IrPath": harness.ir,
                        "Profile": "kernel",
                        "PIOSize": 5,
                        "PIOMaster": "LocalBus",
                    }
                ]
            },
        ]
    )
    with open(path, "w") as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
    result, outdir = _board_check(params)
    if result.returncode != 0:
        raise AssertionError(f"schema construction failed; see {outdir}")
    ini = configparser.ConfigParser(interpolation=None, strict=False)
    ini.optionxform = str
    ini.read(os.path.join(outdir, "config.ini"))
    cluster = "board.harness_clstr."
    expected = {
        ("flow", "buffer_size"): "256",
        ("flow", "stream_size"): "8",
        ("Cache", "size"): "32768",
        ("mem", "ready_mode"): "false",
        ("mem", "reset_on_scratchpad_read"): "false",
        ("mem", "read_on_invalid"): "true",
        ("mem", "write_on_valid"): "false",
        ("dma", "buffer_size"): "1024",
        ("dma", "max_req_size"): "64",
        ("sdma", "read_buffer_size"): "512",
        ("sdma", "write_buffer_size"): "512",
        ("sdma", "max_req_size"): "32",
        ("sdma", "stream_size"): "8",
        ("kernel.llvm_interface", "in_file"): harness.ir,
    }
    for (child, field), value in expected.items():
        actual = ini.get(cluster + child, field, fallback=None)
        if actual != value:
            raise AssertionError(
                f"{child}.{field}={actual!r}, expected {value!r}"
            )
    for child, field, endpoint in (
        ("flow", "stream_in", "kernel.stream"),
        ("flow", "stream_out", "other.stream"),
        ("sdma", "stream_in", "kernel.stream"),
        ("sdma", "stream_out", "kernel.stream"),
    ):
        actual = ini.get(cluster + child, field)
        if not actual.startswith(cluster + endpoint + "["):
            raise AssertionError(f"{child}.{field} connected to {actual}")


_schema_gem5 = st.SalamGem5Fixture()
_schema_temp = TempdirFixture()
TestSuite(
    name="salam-stdlib-schema-parameters",
    fixtures=[
        _schema_gem5,
        _schema_temp,
        st.SalamHarnessFixture(
            "salam-workload", _schema_gem5, _schema_temp, "sanity.c", [2, 3], 1
        ),
    ],
    tags=[st.SALAM_TAG, "salam-correctness"],
    tests=[TestFunction(schema_parameters, name="schema-parameters")],
)


BFS_WALL = 300
BFS_TICKS = 4000000000


def source_tree_import_detected(params):
    # The wrapper the stdlib runs use reports a module loaded from util/.
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    script = os.path.join(
        os.path.dirname(__file__), "configs", "import_util.py"
    )
    configurator = os.path.join(
        config.base_dir, "util", "SALAM-tools", "SALAM-Configurator"
    )
    result = subprocess.run(
        [
            gem5.path,
            "-d",
            outdir,
            "-re",
            st.NO_SOURCE_TREE,
            script,
            configurator,
        ],
        env=gem5.tools.env(),
    )
    err = _read(outdir, "simerr.txt")
    if result.returncode == 0 or (
        "modules loaded from the source tree: "
        + os.path.join(configurator, "workload_config.py")
        not in err
    ):
        raise AssertionError(f"util/ import not reported; see {outdir}")


TestSuite(
    name="salam-stdlib-no-source-tree",
    fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
    tags=[st.SALAM_TAG],
    tests=[
        TestFunction(
            source_tree_import_detected, name="salam-stdlib-no-source-tree"
        )
    ],
)


# Map object names, not model parameters. Wrapper SubSystems only organize
# Python components; their children are compared under the mapped paths.
PARITY_PATHS = {
    "board.processor.cores.core": "system.cpu",
    "board.processor.clk_domain": "system.cpu_clk_domain",
    "board.processor.voltage_domain": "system.cpu_voltage_domain",
    "board.cache_hierarchy.l1icaches": "system.cpu.icache",
    "board.cache_hierarchy.l1dcaches": "system.cpu.dcache",
    "board.cache_hierarchy.l2cache": "system.l2",
    "board.cache_hierarchy.l2bus": "system.tol2bus",
    "board.cache_hierarchy.membus": "system.membus",
    "board.cache_hierarchy.iocache": "system.iocache",
    "board.memory.mem_ctrl": "system.mem_ctrls",
    "board.iobridge": "system.bridge",
    "board.pci_devices": "system.pci_ide",
    "processor.cores.core": "cpu",
    "board": "system",
}
PARITY_WRAPPERS = {
    "board.processor",
    "board.processor.cores",
    "board.memory",
    "board.cache_hierarchy",
}


def parity_name(value):
    for src, dst in sorted(PARITY_PATHS.items(), key=lambda p: -len(p[0])):
        value = re.sub(
            r"(?<![A-Za-z0-9])" + re.escape(src) + r"(?=[.\[:\s]|$)",
            dst,
            value,
        )
    return value


def parity_config(outdir, stdlib):
    ini = configparser.ConfigParser(interpolation=None, strict=False)
    ini.optionxform = str
    ini.read(os.path.join(outdir, "config.ini"))
    if not ini.sections():
        raise AssertionError(f"no instantiated configuration in {outdir}")
    values = {}
    for section in ini.sections():
        if stdlib and section in PARITY_WRAPPERS:
            if ini[section]["type"] != "SubSystem":
                raise AssertionError(f"{section} is no longer a wrapper")
            continue
        name = parity_name(section) if stdlib else section
        for key, value in ini[section].items():
            if key == "children":
                # Parent/child ownership changes with component wrappers;
                # every modeled child and every port peer is checked below.
                continue
            if stdlib:
                value = parity_name(value)
            # Xbar vector indexes encode append order, not a priority.
            # Compare their connected peer sets; retain every endpoint.
            value = re.sub(r"((?:cpu|mem)_side_ports)\[\d+\]", r"\1", value)
            if key in ("cpu_side_ports", "mem_side_ports"):
                value = " ".join(sorted(value.split()))
            values[name + "." + key] = value
    return values


def parity_difference(left, right):
    return {
        key: (left.get(key), right.get(key))
        for key in sorted(left.keys() | right.keys())
        if left.get(key) != right.get(key)
        and not (
            left.get(key) != left.get(key) and right.get(key) != right.get(key)
        )
    }


def parity_stats(outdir, stdlib):
    ini = configparser.ConfigParser(interpolation=None, strict=False)
    ini.optionxform = str
    ini.read(os.path.join(outdir, "config.ini"))
    labels = {}
    for section in ini.sections():
        if ini.get(section, "type", fallback="") not in (
            "CoherentXBar",
            "NoncoherentXBar",
        ):
            continue
        name = parity_name(section) if stdlib else section
        for side, layers in (
            ("cpu", ("respLayer",)),
            ("mem", ("reqLayer", "snoopLayer")),
        ):
            peers = ini.get(section, side + "_side_ports", fallback="").split()
            if side == "mem" and "default" in ini[section]:
                peers.append(ini[section]["default"])
            for index, peer in enumerate(peers):
                if stdlib:
                    peer = parity_name(peer)
                peer = re.sub(r"(_side_ports)\[\d+\]", r"\1", peer)
                # Xbar stat labels use allocation indexes. Identify each
                # lane by its connected peer, preserving its own counters.
                for layer in layers:
                    labels[f"{name}.{layer}{index}."] = (
                        f"{name}.{layer}[{peer}]."
                    )
                labels[f"{name}.{side}_side_port[{index}]"] = (
                    f"{name}.{side}_side_port[{peer}]"
                )
    dumps = []
    for original in st.read_stats(outdir):
        values = {}
        for key, value in original.items():
            if key.startswith("host"):
                continue
            name = parity_name(key) if stdlib else key
            for src, dst in sorted(labels.items(), key=lambda p: -len(p[0])):
                name = name.replace(src, dst)
            if name in values:
                raise AssertionError(f"statistic name collision: {name}")
            values[name] = value
        dumps.append(values)
    return dumps


def legacy_stdlib_parity(params):
    import json

    tempdir = params.fixtures[constants.tempdir_fixture_name].path
    workload = params.fixtures["salam-workload"]
    gem5 = params.fixtures["salam-gem5"]
    # One fixture builds one workload. Both paths use these exact files and
    # the same executable, including the same embedded hardware profile.
    artifacts = [gem5.path, workload.elf]
    for directory, _, files in os.walk(workload.accpath):
        artifacts += [os.path.join(directory, f) for f in sorted(files)]
    before = {p: st.sha256_file(p) for p in artifacts}
    st.run_fs(
        "salam-workload",
        BFS_WALL,
        BFS_TICKS,
        subdir="legacy",
    )(params)
    st.run_stdlib(
        "salam-workload",
        BFS_WALL,
        BFS_TICKS,
        subdir="stdlib",
        config_script=PARITY_STDLIB,
    )(params)
    after = {p: st.sha256_file(p) for p in artifacts}
    if before != after:
        raise AssertionError("binary or workload changed between launchers")
    with open(os.path.join(tempdir, "parity-artifacts.json"), "w") as f:
        json.dump(before, f, indent=2, sort_keys=True)

    legacy = os.path.join(tempdir, "legacy")
    stdlib = os.path.join(tempdir, "stdlib")
    different = parity_difference(
        parity_config(legacy, False), parity_config(stdlib, True)
    )
    if different:
        raise AssertionError(f"effective configuration differs: {different}")

    outputs = []
    for directory, terminal in (
        (legacy, "system.terminal"),
        (stdlib, "board.terminal"),
    ):
        out = _read(directory, "simout.txt")
        uart = _read(directory, terminal)
        if "Check Passed" not in uart or re.search(
            r"^(Mismatch|Check Failed)", uart, re.M
        ):
            raise AssertionError(f"checked BFS failed in {directory}")
        exit_line = re.search(st.M5_EXIT_REGEX, out, re.M)
        if exit_line is None:
            raise AssertionError(f"missing expected exit in {directory}")
        outputs.append((exit_line.group(), uart))
    if outputs[0] != outputs[1]:
        raise AssertionError("BFS output or total simulated timing differs")
    if "No modules were loaded from configs/ or util/." not in _read(
        stdlib, "simout.txt"
    ):
        raise AssertionError("missing source independence assertion")

    left = parity_stats(legacy, False)
    right = parity_stats(stdlib, True)
    if not left or len(left) != len(right):
        raise AssertionError("statistics dump counts differ or are zero")
    for a, b in zip(left, right):
        # Only host measurements are excluded. Simulated time, power-state
        # residency, CPU/memory counters and all SALAM statistics are checked.
        different = parity_difference(a, b)
        if different:
            raise AssertionError(
                f"architectural statistics differ: {different}"
            )
    with open(os.path.join(tempdir, "parity-summary.json"), "w") as f:
        json.dump(
            {
                "configuration_parameters": len(parity_config(legacy, False)),
                "statistics_per_dump": [len(d) for d in left],
                "exit": outputs[0][0],
                "comparison": "exact, excluding host measurements",
            },
            f,
            indent=2,
        )


_parity_gem5 = st.SalamGem5Fixture()
_parity_temp = TempdirFixture()
TestSuite(
    name="salam-stdlib-parity",
    fixtures=[
        _parity_gem5,
        _parity_temp,
        st.SalamWorkloadFixture(
            "salam-workload",
            _parity_gem5,
            _parity_temp,
            "bfs",
            ["bfs", "common"],
            [st.enable_check("bfs")],
        ),
    ],
    tags=[st.SALAM_TAG],
    tests=[TestFunction(legacy_stdlib_parity, name="salam-stdlib-parity")],
)


def device_payload(params):
    result, outdir = _board_check(params, "--device-kind", "0")
    if result.returncode != 0:
        raise AssertionError(f"device run failed; see {outdir}")
    uart = _read(outdir, "board.terminal")
    if "DEVICE_CHECK_PASSED" not in uart or "DEVICE_CHECK_FAILED" in uart:
        raise AssertionError(f"payload mismatch; see {outdir}")
    if not re.search(st.M5_EXIT_REGEX, _read(outdir, "simout.txt"), re.M):
        raise AssertionError("device run did not reach m5_exit")
    dumps = st.read_stats(outdir)
    if len(dumps) != 2:
        raise AssertionError("unexpected dump/reset boundaries")
    prefix = "board.harness_clstr.dma."
    expected = {
        "transfers": 1,
        "bytes": 128,
        "beats": 1,
        "writePackets": 1,
        "memoryPackets.readPackets": 2,
        "memoryPackets.writePackets": 2,
        "memoryPackets.readResponses": 2,
        "memoryPackets.writeResponses": 2,
        "memoryPackets.readBytes": 128,
        "memoryPackets.writeBytes": 128,
    }
    for counter, value in expected.items():
        actual = sum(dump[prefix + counter] for dump in dumps)
        if actual != value:
            raise AssertionError(f"{counter}: {actual} != {value}")
    attempts = sum(d[prefix + "memoryPackets.readAttempts"] for d in dumps)
    if attempts <= expected["memoryPackets.readPackets"]:
        raise AssertionError("read request retry was not exercised")
    first, final = dumps
    port = prefix + "memoryPackets."
    if not (
        first[port + "readPackets"] > 0
        and first[port + "readResponses"] == 0
        and final[port + "readResponses"] > final[port + "readPackets"]
    ):
        raise AssertionError("reset did not occur with DMA reads pending")


_device_gem5 = st.SalamGem5Fixture()
_device_temp = TempdirFixture()
TestSuite(
    name="salam-device-0-reset-1",
    fixtures=[
        _device_gem5,
        _device_temp,
        st.SalamHarnessFixture(
            "salam-workload",
            _device_gem5,
            _device_temp,
            "sanity.c",
            [],
            0,
            host_options={
                "DEVICE_TEST_KIND": 0,
                "DEVICE_TRANSFER_BYTES": 128,
                "DEVICE_RESET": 1,
            },
        ),
    ],
    tags=[st.SALAM_TAG, "salam-devices"],
    tests=[TestFunction(device_payload, name="salam-device-0-reset-1")],
)


# Read every supported width through the buffer, DMA stream port, and
# DMA PIO data aperture. Payload checks run on the simulated CPU.
def stream_widths(kind):
    def test(params):
        result, outdir = _board_check(params, "--device-kind", str(kind))
        if result.returncode != 0:
            raise AssertionError(f"stream width run failed; see {outdir}")
        uart = _read(outdir, "board.terminal")
        if "DEVICE_CHECK_PASSED" not in uart or "DEVICE_CHECK_FAILED" in uart:
            raise AssertionError(f"stream width payload mismatch: {outdir}")
        if not re.search(st.M5_EXIT_REGEX, _read(outdir, "simout.txt"), re.M):
            raise AssertionError("stream width run did not reach m5_exit")
        dumps = st.read_stats(outdir)
        if len(dumps) != 1:
            raise AssertionError("missing stream width statistics")
        prefix = (
            "board.test_stream_buffer"
            if kind == 1
            else "board.test_stream_dma"
        )
        blocks = 1 if kind == 1 else 8
        for counter, expected in {
            "readPackets": 5 * blocks,
            "writePackets": 4 * blocks,
            "readBytes": 16 * blocks,
            "writeBytes": 16 * blocks,
        }.items():
            actual = dumps[0][prefix + ".stream." + counter]
            if actual != expected:
                raise AssertionError(f"{counter}: {actual} != {expected}")
        attempts = dumps[0][prefix + ".stream.readAttempts"]
        if (kind == 3 and attempts != 0) or (
            kind != 3 and attempts < 5 * blocks
        ):
            raise AssertionError("read attempts counted on the wrong port")

    return test


# Buffer, DMA stream port, and DMA PIO aperture have distinct read paths.
for _kind in (1, 2, 3):
    _gem5 = st.SalamGem5Fixture()
    _temp = TempdirFixture()
    _name = f"salam-stream-widths-{_kind}"
    TestSuite(
        name=_name,
        fixtures=[
            _gem5,
            _temp,
            st.SalamHarnessFixture(
                "salam-workload",
                _gem5,
                _temp,
                "sanity.c",
                [],
                0,
                host_options={
                    "DEVICE_TEST_KIND": _kind,
                    "DEVICE_READ_WIDTHS": 1,
                },
            ),
        ],
        tags=[st.SALAM_TAG, "salam-stream-widths"],
        tests=[TestFunction(stream_widths(_kind), name=_name)],
    )


# Keep routing, partial chunks, bypass refills, and independent stream FIFOs.
# Each tuple is name, kind, route, length, read/write capacity, max request,
# rejection expected, and bypass mode.
_FIFO_CASES = (
    (
        "noncoherent-memory-to-cluster-partial-tail",
        0,
        1,
        160,
        64,
        64,
        64,
        False,
        False,
    ),
    (
        "noncoherent-memory-to-cluster-reject",
        0,
        1,
        128,
        64,
        64,
        128,
        True,
        False,
    ),
    (
        "noncoherent-cluster-to-memory-short",
        0,
        2,
        128,
        128,
        128,
        256,
        False,
        False,
    ),
    (
        "noncoherent-memory-to-cluster-bypass-partial-refill",
        0,
        1,
        128,
        64,
        64,
        48,
        False,
        True,
    ),
    (
        "noncoherent-cluster-to-memory-bypass-subline-fifo",
        0,
        2,
        72,
        32,
        32,
        24,
        False,
        True,
    ),
    ("stream-partial-tail", 2, 0, 160, 64, 64, 64, False, False),
    ("stream-read-reject", 2, 0, 128, 64, 128, 128, True, False),
    ("stream-write-reject", 2, 0, 128, 128, 64, 128, True, False),
    ("stream-bypass", 2, 0, 128, 64, 64, 128, False, True),
)


def dma_fifo_capacity(case):
    _, kind, route, length, rd_size, wr_size, maximum, reject, bypass = case

    def test(params):
        options = ["--dma-max-req-size", str(maximum)]
        if kind == 0:
            options += [
                "--dma-buffer-size",
                str(2 * rd_size),
                "--dma-local-memory",
            ]
        else:
            options += [
                "--dma-read-buffer-size",
                str(rd_size),
                "--dma-write-buffer-size",
                str(wr_size),
            ]
        if bypass:
            options.append("--dma-bypass")
        result, outdir = _board_check(
            params, "--device-kind", str(kind), *options
        )
        device = (
            "board.harness_clstr.dma" if kind == 0 else "board.test_stream_dma"
        )
        out = _read(outdir, "simout.txt")
        if reject:
            err = _read(outdir, "simerr.txt")
            required = (
                device,
                f"transfer length {length}",
                f"{min(length, maximum)}-byte chunk",
                "FIFO capacity",
                f"{min(rd_size, wr_size)} bytes",
            )
            if result.returncode == 0 or any(x not in err for x in required):
                raise AssertionError(f"missing launch diagnostic: {outdir}")
            return
        if result.returncode != 0:
            raise AssertionError(f"DMA run failed; see {outdir}")
        uart = _read(outdir, "board.terminal")
        if (
            "DEVICE_CHECK_PASSED" not in uart
            or "DEVICE_CHECK_FAILED" in uart
            or not re.search(st.M5_EXIT_REGEX, out, re.M)
        ):
            raise AssertionError(f"DMA payload/progress failed: {outdir}")
        ini = configparser.ConfigParser()
        ini.read(os.path.join(outdir, "config.ini"))
        expected_mode = "atomic_noncaching" if bypass else "timing"
        if ini.get("board", "mem_mode") != expected_mode:
            raise AssertionError("DMA fixture selected the wrong FIFO mode")
        dumps = st.read_stats(outdir)
        if len(dumps) != 1:
            raise AssertionError("missing DMA statistics")
        stats = dumps[0]
        groups = ["memoryPackets"]
        if kind == 0:
            groups.append("clusterPackets")
        total_attempts = total_packets = 0
        for direction in ("read", "write"):
            for group in groups:
                prefix = device + "." + group + "." + direction
                packets = stats[prefix + "Packets"]
                attempts = stats[prefix + "Attempts"]
                if packets != stats[prefix + "Responses"]:
                    raise AssertionError("unmatched accepted DMA packets")
                if attempts < packets:
                    raise AssertionError(
                        "fewer attempts than accepted packets"
                    )
                active = kind != 0 or (group == "clusterPackets") == (
                    (route == 1) == (direction == "write")
                )
                expected_bytes = length if active else 0
                if stats[prefix + "Bytes"] != expected_bytes:
                    raise AssertionError(f"wrong selected FIFO: {prefix}")
                total_attempts += attempts
                total_packets += packets
        if bypass and total_attempts != total_packets:
            raise AssertionError("atomic transfer retried a request")
        if not bypass and maximum == 256 and total_attempts == total_packets:
            raise AssertionError("short-transfer retry was not exercised")
        if kind == 0:
            expected = {
                "transfers": 1,
                "bytes": length,
                "beats": (length + maximum - 1) // maximum,
                "writePackets": (length + maximum - 1) // maximum,
            }
        else:
            expected = {
                "transfers.readTransfers": 1,
                "transfers.writeTransfers": 1,
                "transfers.readFrames": 1,
                "transfers.writeFrames": 1,
                "transfers.readBytes": length,
                "transfers.writeBytes": length,
                "stream.readBytes": length,
                "stream.writeBytes": length,
                "stream.readPackets": length // 8,
                "stream.writePackets": length // 4,
            }
        for counter, value in expected.items():
            if stats[device + "." + counter] != value:
                raise AssertionError(f"incorrect {device}.{counter}")

    return test


for _case in _FIFO_CASES:
    _suffix, _kind, _route, _length, *_ = _case
    _gem5 = st.SalamGem5Fixture()
    _temp = TempdirFixture()
    _name = "salam-dma-fifo-" + _suffix
    TestSuite(
        name=_name,
        fixtures=[
            _gem5,
            _temp,
            st.SalamHarnessFixture(
                "salam-workload",
                _gem5,
                _temp,
                "sanity.c",
                [],
                0,
                host_options={
                    "DEVICE_TEST_KIND": _kind,
                    "DEVICE_TRANSFER_BYTES": _length,
                    "DEVICE_DMA_ROUTE": _route,
                    "DEVICE_INCREMENTAL_STREAM": 1,
                },
            ),
        ],
        tags=[st.SALAM_TAG, "salam-correctness", "salam-devices"],
        tests=[TestFunction(dma_fifo_capacity(_case), name=_name)],
    )
