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
    if "--device-kind" in options:
        command.insert(
            4,
            "--debug-flags=DMA,NoncoherentDma,StreamDma,StreamBuffer,PseudoInst",
        )
    if "--queue-timing" in options:
        command.insert(4, "--debug-flags=SALAMIssue,SALAMLaunch")
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


BFS_WALL = 300
BFS_TICKS = 4000000000


def stdlib_bfs_suite(name, edits, verifiers):
    # BFS through configs/example/gem5_library/salam, from a copy built
    # in the test.
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    workload = st.SalamWorkloadFixture(
        "salam-workload", gem5, tempdir, "bfs", ["bfs", "common"], edits
    )
    verifiers = dict(verifiers)
    verifiers["m5-exit"] = verifier.MatchFileRegex(
        re.compile(st.M5_EXIT_REGEX), ["simout.txt"]
    )
    verifiers["no-source-tree"] = verifier.MatchFileRegex(
        re.compile(r"^No modules were loaded from configs/ or util/\.$"),
        ["simout.txt"],
    )
    tests = [
        TestFunction(
            st.run_stdlib("salam-workload", BFS_WALL, BFS_TICKS), name=name
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


stdlib_bfs_suite(
    "salam-stdlib-bfs-check",
    [st.enable_check("bfs")],
    {
        "passed": verifier.MatchFileRegex(
            re.compile(r"^Check Passed$"), ["board.terminal"]
        ),
        "no-mismatch": st.NoMatchFileRegex(
            re.compile(r"^(Mismatch|Check Failed)"), ["board.terminal"]
        ),
    },
)


def source_tree_import_detected(params):
    # The wrapper the stdlib runs use reports a module loaded from util/.
    gem5 = params.fixtures["salam-gem5"]
    outdir = params.fixtures[constants.tempdir_fixture_name].path
    script = os.path.join(outdir, "imports_util.py")
    configurator = os.path.join(
        config.base_dir, "util", "SALAM-tools", "SALAM-Configurator"
    )
    with open(script, "w") as f:
        f.write(
            "import sys\n"
            f"sys.path.insert(0, {configurator!r})\n"
            "import workload_config\n"
        )
    result = subprocess.run(
        [gem5.path, "-d", outdir, "-re", st.NO_SOURCE_TREE, script],
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


def legacy_stdlib_parity(params, expected_output=("Check Passed",)):
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
    st.run_fs("salam-workload", BFS_WALL, BFS_TICKS, subdir="legacy")(params)
    st.run_stdlib("salam-workload", BFS_WALL, BFS_TICKS, subdir="stdlib")(
        params
    )
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
        if any(line not in uart for line in expected_output) or re.search(
            r"^(Mismatch|Check Failed)", uart, re.M
        ):
            raise AssertionError(f"checked BFS failed in {directory}")
        exit_line = re.search(st.M5_EXIT_REGEX, out, re.M)
        if exit_line is None:
            raise AssertionError(f"missing expected exit in {directory}")
        invocations = re.findall(
            r"^\s+(?:Runtime|Compute passes|Invocation duration):.*$",
            out,
            re.M,
        )
        if not invocations:
            raise AssertionError("no accelerator invocation timings")
        outputs.append((exit_line.group(), uart, invocations))
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
                "invocation_timings": outputs[0][2],
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


_reset_gem5 = st.SalamGem5Fixture()
_reset_temp = TempdirFixture()
TestSuite(
    name="salam-stdlib-parity-reset",
    fixtures=[
        _reset_gem5,
        _reset_temp,
        st.SalamHarnessFixture(
            "salam-workload",
            _reset_gem5,
            _reset_temp,
            "sanity.c",
            [2, 3],
            1,
            runs=2,
            host_options={
                "STATS_RESET_AFTER_START": 1,
                "STATS_DUMPS_AT_END": 2,
            },
        ),
    ],
    tags=[st.SALAM_TAG],
    tests=[
        TestFunction(
            lambda params: legacy_stdlib_parity(
                params, ("RUN[0] OUT[0]=5", "RUN[1] OUT[0]=5")
            ),
            name="salam-stdlib-parity-reset",
        )
    ],
)


def device_payload(kind, reset):
    def test(params):
        result, outdir = _board_check(params, "--device-kind", str(kind))
        if result.returncode != 0:
            raise AssertionError(f"device run failed; see {outdir}")
        out = _read(outdir, "simout.txt")
        uart = _read(outdir, "board.terminal")
        if "DEVICE_CHECK_PASSED" not in uart or "DEVICE_CHECK_FAILED" in uart:
            raise AssertionError(f"payload mismatch; see {outdir}")
        if not re.search(st.M5_EXIT_REGEX, out, re.M):
            raise AssertionError("device run did not reach m5_exit")
        stats = st.read_stats(outdir)
        if not stats:
            raise AssertionError("device run produced no statistics")
        check_device_counters(outdir, kind, reset)

    return test


for _kind in range(3):
    for _reset in (False, True):
        _device_gem5 = st.SalamGem5Fixture()
        _device_temp = TempdirFixture()
        _options = {"DEVICE_TEST_KIND": _kind}
        if _reset:
            _options["DEVICE_RESET"] = 1
        _name = f"salam-device-{_kind}-reset-{int(_reset)}"
        TestSuite(
            name=_name,
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
                    host_options=_options,
                ),
            ],
            tags=[st.SALAM_TAG, "salam-devices"],
            tests=[TestFunction(device_payload(_kind, _reset), name=_name)],
        )


def check_device_counters(outdir, kind, reset):
    dumps = st.read_stats(outdir)
    if len(dumps) != (2 if reset else 1):
        raise AssertionError("unexpected dump/reset boundaries")
    final = dumps[-1]
    out = _read(outdir, "simout.txt")
    start = dumps[0]["finalTick"] if reset else 0
    end = final["finalTick"]

    def total(name):
        if any(name not in d for d in dumps):
            raise AssertionError(f"missing required counter {name}")
        return sum(d[name] for d in dumps)

    def expect(prefix, values):
        for key, expected in values.items():
            actual = total(prefix + key)
            if actual != expected:
                raise AssertionError(f"{prefix}{key}: {actual} != {expected}")

    def average(name, area, current=0):
        # AverageStorage includes the current tick. Text output uses six
        # fractional digits; this checks that representation, not a timing
        # tolerance or a value learned from the simulation's counters.
        expected = float(f"{(area + current) / (end - start + 1):.6f}")
        if final[name] != expected:
            raise AssertionError(f"{name}: {final[name]} != {expected}")

    if kind != 1:
        device = (
            "board.harness_clstr.dma" if kind == 0 else "board.test_stream_dma"
        )
        port = device + ".memoryPackets."
        # The fixed transfer spans two aligned 64-byte cache lines. CPU
        # initialization/verification traffic does not use the DMA port.
        expect(
            port,
            {
                "readPackets": 2,
                "writePackets": 2,
                "readResponses": 2,
                "writeResponses": 2,
                "readBytes": 128,
                "writeBytes": 128,
            },
        )
        if total(port + "readAttempts") <= 2:
            raise AssertionError("read request retry was not exercised")
        if total(port + "writeAttempts") < 2:
            raise AssertionError("missing write attempts")
        if reset and not (
            dumps[0][port + "readPackets"] > 0
            and dumps[0][port + "readResponses"] == 0
            and final[port + "readResponses"] > final[port + "readPackets"]
        ):
            raise AssertionError("reset did not occur with DMA reads pending")

        if kind == 0:
            expect(
                device + ".",
                {
                    "transfers": 1,
                    "bytes": 128,
                    "beats": 1,
                    "writePackets": 1,
                },
            )
            begins = re.findall(
                r"^(\d+): " + re.escape(device) + r": SRC:", out, re.M
            )
            finishes = re.findall(
                r"^(\d+): " + re.escape(device) + r": Transfer completed",
                out,
                re.M,
            )
            if len(begins) != 1 or len(finishes) != 1:
                raise AssertionError("missing DMA transfer boundaries")
            average(
                device + ".active",
                max(0, int(finishes[0]) - max(start, int(begins[0]))),
            )
        else:
            expect(
                device + ".transfers.",
                {
                    "readTransfers": 1,
                    "writeTransfers": 1,
                    "readFrames": 1,
                    "writeFrames": 1,
                    "readBytes": 128,
                    "writeBytes": 128,
                },
            )
            for direction, completed in (
                ("read", "read"),
                ("write", "written"),
            ):
                begins = re.findall(
                    r"^(\d+): "
                    + re.escape(device)
                    + f": Init frame {direction} from",
                    out,
                    re.M,
                )
                # Write messages say "to"; the model's message uses "to"
                # for its destination and "from" for its read source.
                if direction == "write":
                    begins = re.findall(
                        r"^(\d+): "
                        + re.escape(device)
                        + r": Init frame write to",
                        out,
                        re.M,
                    )
                finishes = re.findall(
                    r"^(\d+): "
                    + re.escape(device)
                    + f": Frame 1 of 1 {completed}",
                    out,
                    re.M,
                )
                if len(begins) != 1 or len(finishes) != 1:
                    raise AssertionError("missing stream DMA frame boundaries")
                average(
                    device + f".transfers.{direction}Active",
                    max(0, int(finishes[0]) - max(start, int(begins[0]))),
                )

    if kind in (1, 2):
        device = (
            "board.test_stream_buffer"
            if kind == 1
            else "board.test_stream_dma"
        )
        prefix = device + ".stream."
        # Explicit 4-byte stores and 8-byte paired LDM loads in main.c.
        beats = 2 if kind == 1 else 16
        expect(
            prefix,
            {
                "readPackets": beats,
                "writePackets": 2 * beats,
                "readBytes": 8 * beats,
                "writeBytes": 8 * beats,
            },
        )
        if (
            total(prefix + "readAttempts") + total(prefix + "writeAttempts")
            <= 3 * beats
        ):
            raise AssertionError("stream port retry was not exercised")
        if kind == 1:
            events = [
                (int(t), int(size) * (1 if op == "write" else -1))
                for t, op, size in re.findall(
                    r"^(\d+): board.test_stream_buffer: A (read|write) "
                    r"request of size (\d+)",
                    out,
                    re.M,
                )
            ]
            if [(change) for _, change in events] != [4, 4, 4, 4, -8, -8]:
                raise AssertionError("missing accepted stream packet trace")
            if reset and not (
                dumps[0][prefix + "writePackets"] == 1
                and events[0][0] < start < events[0][0] + 100000000
            ):
                raise AssertionError(
                    "reset missed the pending stream response"
                )
            level = cursor = area = 0
            for tick, change in events:
                area += level * max(0, min(tick, end) - max(cursor, start))
                level += change
                cursor = tick
            area += level * max(0, end - max(cursor, start))
            average(device + ".bufferOccupancy", area, level)


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


# The response events must update occupancy before the next compute edge.
# Derive the integral from issue/completion traces, not from sampled stats.
def queue_occupancy(params):
    def run(*options):
        result, outdir = _board_check(params, "--queue-timing", *options)
        if result.returncode != 0:
            raise AssertionError(f"queue timing run failed: {outdir}")
        out = _read(outdir, "simout.txt")
        exit_line = re.search(st.M5_EXIT_REGEX, out, re.M)
        if exit_line is None or "RUN[0] OUT[0]=1" not in _read(
            outdir, "board.terminal"
        ):
            raise AssertionError("queue timing workload did not pass")
        events = [
            (int(t), op, kind)
            for t, op, kind in re.findall(
                r"^(\d+): .*llvm_interface: (issue|complete) (load|store)\b",
                out,
                re.M,
            )
        ]
        return outdir, out, exit_line.group(), events

    before_dir, before, exit_line, events = run()
    first, period = map(
        int,
        re.search(
            r"first compute pass at (\d+) \(period (\d+)\)", before
        ).groups(),
    )
    boundaries = []
    for kind in ("load", "store"):
        issue = [t for t, op, k in events if op == "issue" and k == kind]
        complete = [t for t, op, k in events if op == "complete" and k == kind]
        if len(issue) != 1 or len(complete) != 1 or issue[0] >= complete[0]:
            raise AssertionError("expected one asynchronous load and store")
        next_edge = first + ((complete[0] - first) // period + 1) * period
        if not (complete[0] - first) % period or complete[0] + 1 >= next_edge:
            raise AssertionError("completion was not between compute edges")
        boundaries.append(complete[0] + 1)
    boundaries.sort()
    outdir, out, new_exit, new_events = run(
        "--stats-at", ",".join(map(str, boundaries))
    )
    if new_exit != exit_line or new_events != events:
        raise AssertionError("statistics operations changed execution")
    dumps = st.read_stats(outdir)
    if len(dumps) != 3:
        raise AssertionError("expected two boundary dumps and a final dump")
    with open(os.path.join(outdir, "queue-timing.json"), "w") as f:
        json.dump(
            {
                "events": events,
                "boundaries": boundaries,
                "exit": exit_line,
                "period": period,
            },
            f,
            indent=2,
        )
    start = 0
    for dump in dumps:
        end = int(dump["finalTick"])
        for kind, counter in (("load", "readOps"), ("store", "writeOps")):
            issue = next(
                t for t, op, k in events if op == "issue" and k == kind
            )
            complete = next(
                t for t, op, k in events if op == "complete" and k == kind
            )
            area = max(0, min(end, complete) - max(start, issue))
            current = int(issue <= end < complete)
            expected = float(f"{(area + current) / (end - start + 1):.6f}")
            actual = dump[
                "board.harness_clstr.kernel.llvm_interface." + counter
            ]
            if actual != expected:
                raise AssertionError(
                    f"{counter} in [{start}, {end}]: {actual} != {expected}; "
                    f"residence [{issue}, {complete}]"
                )
        start = end


_queue_gem5 = st.SalamGem5Fixture()
_queue_temp = TempdirFixture()
TestSuite(
    name="salam-queue-occupancy",
    fixtures=[
        _queue_gem5,
        _queue_temp,
        st.SalamHarnessFixture(
            "salam-workload",
            _queue_gem5,
            _queue_temp,
            "spm_load.c",
            [0],
            1,
        ),
    ],
    tags=[st.SALAM_TAG, "salam-queue-occupancy"],
    tests=[TestFunction(queue_occupancy, name="salam-queue-occupancy")],
)
