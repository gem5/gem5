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


# Actual memory-service accounting with synthetic, dimensional coefficients.

import json
import math
import os
import re
import subprocess

import salam_testlib as st
from testlib import *

MODEL = "system.memory.ppa."
PHASE = 1000000
CONFIG_DIR = os.path.join(os.path.dirname(__file__), "configs")


def model(kind, mask="byte_enable", enabled=True, strict=True):
    # SRAM read/write coefficients are per four-byte word; flip-flop
    # coefficients are per transferred bit. Area/leakage describe one
    # complete 4096-byte physical instance in either case.
    return dict(
        enabled=enabled,
        strict=strict,
        report_details=True,
        keys=["memory"],
        ids=["controlled-" + kind],
        signatures=[""],
        timing_available=[False],
        minimum_latency_s=[0],
        minimum_issue_interval_s=[0],
        pipelined=[False],
        sources=[json.dumps({"synthetic": True, "test": "ppa-storage"})],
        area_um2=[100],
        leakage_w=[2e-6],
        event_j=[0],
        read_j=[2e-12 if kind == "spm" else 0.01e-12],
        write_j=[3e-12 if kind == "spm" else 0.02e-12],
        area_status=["characterized"],
        leakage_status=["characterized"],
        event_status=["unavailable"],
        read_status=["characterized"],
        write_status=["characterized"],
        capacity_bytes=4096,
        word_bytes=4,
        banks=1,
        read_ports=0,
        write_ports=0,
        rw_ports=1,
        mask_model=mask,
        minimum_access_interval_s=0,
        storage_physical="sram" if kind == "spm" else "flip_flop",
        storage_scope="whole_device",
        memory_timing_available=True,
    )


def run_config(
    params,
    name,
    script,
    values,
    rejected=False,
    reason="memory:partial_write_physical_model",
):
    gem5 = params.fixtures["salam-gem5"]
    outdir = os.path.join(
        params.fixtures[constants.tempdir_fixture_name].path, name
    )
    os.makedirs(outdir)
    config_path = os.path.join(CONFIG_DIR, script)
    parameters = os.path.join(outdir, "storage_parameters.json")
    with open(parameters, "w") as stream:
        json.dump(values, stream)
    result = subprocess.run(
        [
            gem5.path,
            "-d",
            outdir,
            "-re",
            "--debug-flags=MemoryAccess,NoncoherentXBar,SALAMScratchpad,DMA,NoncoherentDma",
            config_path,
            parameters,
        ],
        env=gem5.tools.env(),
        timeout=180,
    )
    if rejected:
        if result.returncode == 0:
            raise AssertionError("strict unsupported storage was accepted")
        error = read(outdir, "simerr.txt")
        if reason not in error:
            raise AssertionError(f"unexpected strict failure: {error}")
    elif result.returncode:
        raise AssertionError(f"storage run failed: {outdir}")
    return outdir


def read(outdir, filename):
    with open(os.path.join(outdir, filename)) as stream:
        return stream.read()


def near(actual, expected):
    if not math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-24):
        raise AssertionError(f"{actual} != independent reference {expected}")


def energy(dump, reads, writes, dynamic, prefix=MODEL, elapsed=None):
    for suffix, expected in (
        ("knownAreaUm2", 100),
        ("readWords", reads),
        ("writeWords", writes),
        ("areaComplete", 1),
        ("dynamicComplete", 1),
        ("leakageComplete", 1),
        ("approximate", 0),
        ("missingEventCount", 0),
        ("memoryWordCountsAvailable", 1),
    ):
        if dump[prefix + suffix] != expected:
            raise AssertionError(f"{prefix + suffix} != {expected}")
    seconds = dump[prefix + "elapsedSeconds"]
    if elapsed is not None:
        near(seconds, elapsed)
    near(dump[prefix + "knownDynamicEnergyJ"], dynamic)
    near(dump[prefix + "knownLeakageEnergyJ"], 2e-6 * seconds)
    near(dump[prefix + "knownTotalEnergyJ"], dynamic + 2e-6 * seconds)
    if seconds:
        near(
            dump[prefix + "knownAveragePowerW"],
            (dynamic + 2e-6 * seconds) / seconds,
        )
    elif dump[prefix + "powerAvailable"] != 0:
        raise AssertionError("zero-duration power must be unavailable")


def same_behavior(enabled, disabled):
    first, second = st.read_stats(enabled), st.read_stats(disabled)
    if len(first) != len(second):
        raise AssertionError("PPA changed dump count")
    for a, b in zip(first, second):
        for key in set(a) | set(b):
            if key.startswith("host") or ".ppa." in key:
                continue
            x, y = a.get(key), b.get(key)
            if x != y and not (
                isinstance(x, float)
                and isinstance(y, float)
                and math.isnan(x)
                and math.isnan(y)
            ):
                raise AssertionError(f"PPA changed {key}: {x} != {y}")
    regex = r"^PAYLOAD .+$|^STORAGE_CHECK_PASSED .+$|^Exiting @ tick .+$"
    if re.findall(regex, read(enabled, "simout.txt"), re.M) != re.findall(
        regex, read(disabled, "simout.txt"), re.M
    ):
        raise AssertionError("PPA changed payload or completion tick")


def traffic(
    kind,
    mask="byte_enable",
    requestors=1,
    reset=False,
    fast=False,
    equivalence=False,
):
    def check(params):
        values = dict(
            kind=kind,
            requestors=requestors,
            reset=reset,
            bandwidth="1TiB/s" if fast else "128MiB/s",
            model=model(kind, mask),
        )
        outdir = run_config(params, "enabled", "storage_traffic.py", values)
        output = read(outdir, "simout.txt")
        if "STORAGE_CHECK_PASSED 3000000" not in output:
            raise AssertionError("missing functional/traffic completion")
        dumps = st.read_stats(outdir)
        if len(dumps) < 5:
            raise AssertionError("required dumps did not run")
        energy(dumps[0], 0, 0, 0, elapsed=0)
        rmw = mask == "read_modify_write"
        first_dynamic = (
            (15e-12 if rmw else 9e-12) if kind == "spm" else 0.48e-12
        )
        energy(
            dumps[1],
            requestors * (3 if rmw else 0),
            requestors * 3,
            requestors * first_dynamic,
            elapsed=1e-6,
        )
        final_reads = 4 + (4 if reset else 7) * rmw
        final_writes = 4 if reset else 7
        if kind == "spm":
            final_dynamic = final_reads * 2e-12 + final_writes * 3e-12
        else:
            final_dynamic = 128 * 0.01e-12 + (64 if reset else 88) * 0.02e-12
        for dump in dumps[2:]:
            energy(
                dump,
                requestors * final_reads,
                requestors * final_writes,
                requestors * final_dynamic,
                elapsed=2e-6 if reset else 3e-6,
            )
        # Response counters establish that every generated request completed.
        # Reset occurs only after the first three writes have completed.
        config = json.loads(read(outdir, "config.json"))
        generators = config["system"]["generators"]
        if len(generators) != requestors:
            raise AssertionError("unexpected traffic generator inventory")
        for generator in generators:
            prefix = generator["path"] + "."
            final = dumps[2]
            for suffix, expected in (
                ("totalReads", 4),
                ("bytesRead", 16),
                ("totalWrites", 2 if reset else 5),
                ("bytesWritten", 8 if reset else 11),
            ):
                if final[prefix + suffix] != expected:
                    raise AssertionError(
                        f"incomplete traffic {prefix + suffix}"
                    )
        if kind == "spm" and not fast:
            if "busy: request rejected" not in output:
                raise AssertionError("SPM request retry was not exercised")
        if fast and not re.search(r"recvTimingResp:.* BUSY", output):
            raise AssertionError("response retry was not exercised")
        if equivalence or reset:
            values["model"]["enabled"] = False
            disabled = run_config(
                params, "disabled", "storage_traffic.py", values
            )
            same_behavior(outdir, disabled)

    return check


def register_functional_ports(params):
    values = dict(model=model("register"))
    values["model"]["rw_ports"] = 2
    outdir = run_config(
        params, "enabled", "storage_register_functional.py", values
    )
    dumps = st.read_stats(outdir)
    if len(dumps) < 4:
        raise AssertionError("missing register functional snapshots")
    # Functional accesses through either port, pending delta updates, and
    # queued-response updates add no transfers. Response retry adds none.
    for dump, reads, writes in zip(dumps[:4], (0, 4, 8, 10), (0, 2, 2, 2)):
        energy(
            dump,
            reads,
            writes,
            reads * 32 * 0.01e-12 + writes * 32 * 0.02e-12,
        )
    for suffix in ("numReads", "numWrites", "bytesRead", "bytesWritten"):
        for key, value in dumps[0].items():
            if key.startswith("system.memory." + suffix) and value:
                raise AssertionError("functional access changed " + key)
    if "STORAGE_CHECK_PASSED 31001" not in read(outdir, "simout.txt"):
        raise AssertionError("register functional or timing contract failed")
    values["model"]["enabled"] = False
    disabled = run_config(
        params, "disabled", "storage_register_functional.py", values
    )
    same_behavior(outdir, disabled)


def unsupported_mask(params):
    values = dict(
        kind="spm",
        requestors=1,
        reset=False,
        bandwidth="128MiB/s",
        model=model("spm", "unsupported"),
    )
    run_config(params, "strict", "storage_traffic.py", values, rejected=True)
    values["model"]["strict"] = False
    values["reset_after_traffic"] = True
    outdir = run_config(params, "incomplete", "storage_traffic.py", values)
    output = read(outdir, "simout.txt")
    if "STORAGE_CHECK_PASSED" not in output:
        raise AssertionError("unsupported mask changed execution")
    dumps = st.read_stats(outdir)
    final = dumps[2]
    near(final[MODEL + "knownDynamicEnergyJ"], 8e-12)
    if (
        final[MODEL + "missingEventCount"] != 5
        or final[MODEL + "memoryWordCountsAvailable"] != 0
        or final[MODEL + "readWords"] != 4
        or final[MODEL + "writeWords"] != 0
    ):
        raise AssertionError(
            "unsupported serviced writes lack missing-access counts"
        )
    empty = dumps[-1]
    energy(empty, 0, 0, 0, elapsed=0)
    last = output.split("   PPA model: system.memory.ppa")[-1]
    if "memory:partial_write_physical_model" in last:
        raise AssertionError("old missing-access reason leaked across reset")
    if (
        final[MODEL + "dynamicComplete"] != 0
        or final[MODEL + "knownAreaUm2"] != 100
        or "memory:partial_write_physical_model" not in output
    ):
        raise AssertionError("incomplete memory coverage lost its identity")


def dma(reset):
    def check(params):
        workload = params.fixtures["salam-workload"]
        values = dict(
            binary=workload.elf, accpath=workload.accpath, model=model("spm")
        )
        outdir = run_config(params, "dma-enabled", "storage_dma.py", values)
        output = read(outdir, "simout.txt")
        terminal = read(outdir, "board.terminal")
        if (
            "DEVICE_CHECK_PASSED" not in terminal
            or "DEVICE_CHECK_FAILED" in terminal
            or not re.search(st.M5_EXIT_REGEX, output, re.M)
        ):
            raise AssertionError("CPU/DMA payload checker did not pass")
        dumps = st.read_stats(outdir)
        if len(dumps) != (2 if reset else 1):
            raise AssertionError("DMA reset/dump boundary missing")
        prefix = "board.test_memory.ppa."
        # 32 CPU source stores and 32 CPU destination loads, plus two
        # aligned 64-byte DMA reads and writes: 64 words each direction.
        for suffix, expected in (("readWords", 64), ("writeWords", 64)):
            if sum(d[prefix + suffix] for d in dumps) != expected:
                raise AssertionError("CPU and DMA physical traffic mismatch")
        near(sum(d[prefix + "knownDynamicEnergyJ"] for d in dumps), 320e-12)
        for dump in dumps:
            reads, writes = (
                dump[prefix + "readWords"],
                dump[prefix + "writeWords"],
            )
            energy(
                dump,
                reads,
                writes,
                reads * 2e-12 + writes * 3e-12,
                prefix=prefix,
            )
        dma_prefix = "board.harness_clstr.dma.memoryPackets."
        if sum(d[dma_prefix + "readAttempts"] for d in dumps) <= 2:
            raise AssertionError("DMA request retry did not execute")
        if reset and not (
            dumps[0][dma_prefix + "readPackets"] > 0
            and dumps[0][dma_prefix + "readResponses"] == 0
        ):
            raise AssertionError("reset did not occur with DMA reads pending")
        config = json.loads(read(outdir, "config.json"))["board"]
        start, end = map(int, config["test_memory"]["range"].split(":"))
        system_prefix = config["path"] + "."
        cpu = config["processor"]["cores"][0]["core"]["path"]
        cpu = cpu.removeprefix(system_prefix) + ".data"
        dma = config["harness_clstr"]["dma"]["path"]
        dma = dma.removeprefix(system_prefix)
        # AbstractMemory's free tracePacket helper uses the global debug
        # name. Select this physical memory by its configured address range
        # and exact requestors, excluding functional initialization elsewhere.
        services = []
        for direction, requestor, size, address in re.findall(
            r"^\s*\d+: global: (Read|Write) from (\S+) of size (\d+) "
            r"on address (0x[0-9a-fA-F]+|0)\b",
            output,
            re.M,
        ):
            size, address = int(size), int(address, 16)
            if start <= address and address + size <= end:
                services.append((direction, requestor, size, address))
        expected_services = (
            [("Write", cpu, 4, start + 4 * index) for index in range(32)]
            + [
                ("Read", cpu, 4, start + 256 + 4 * index)
                for index in range(32)
            ]
            + [("Read", dma, 64, start + 64 * index) for index in range(2)]
            + [
                ("Write", dma, 64, start + 256 + 64 * index)
                for index in range(2)
            ]
        )
        if sorted(services) != sorted(expected_services):
            raise AssertionError("CPU/DMA memory service sequence differs")
        if reset:
            values["model"]["enabled"] = False
            disabled = run_config(
                params, "dma-disabled", "storage_dma.py", values
            )
            same_behavior(outdir, disabled)
            if terminal != read(disabled, "board.terminal"):
                raise AssertionError("memory PPA changed DMA payload")

    return check


def timing_boundaries(params):
    values = dict(
        kind="spm",
        requestors=1,
        reset=False,
        bandwidth="128MiB/s",
        access_cycles=0,
        model=model("spm"),
    )
    values["model"]["minimum_access_latency_s"] = 1e-9
    run_config(
        params,
        "latency-strict",
        "storage_traffic.py",
        values,
        rejected=True,
        reason="memory:configured_access_latency",
    )
    values["model"]["strict"] = False
    outdir = run_config(
        params, "latency-incomplete", "storage_traffic.py", values
    )
    final = st.read_stats(outdir)[-1]
    if final[MODEL + "dynamicComplete"] != 0:
        raise AssertionError("incompatible access latency was hidden")
    if "memory:configured_access_latency" not in read(outdir, "simout.txt"):
        raise AssertionError("incompatible access latency lacks an identifier")
    near(final[MODEL + "knownDynamicEnergyJ"], 29e-12)
    # 500000000 B/s admits one byte every 2 ns. A partial-word RMW needs
    # two 1 ns physical services, exactly satisfying this boundary.
    values = dict(
        kind="spm",
        requestors=1,
        reset=False,
        bandwidth="500000000B/s",
        model=model("spm", "read_modify_write"),
    )
    values["model"]["minimum_access_interval_s"] = 1e-9
    outdir = run_config(params, "rmw-boundary", "storage_traffic.py", values)
    energy(st.read_stats(outdir)[-1], 11, 7, 43e-12, elapsed=3e-6)
    # A faster configured service interval cannot absorb both accesses.
    values["bandwidth"] = "640000000B/s"
    run_config(
        params,
        "rmw-too-fast",
        "storage_traffic.py",
        values,
        rejected=True,
        reason="memory:configured_service_rate",
    )
    values = dict(
        kind="register",
        requestors=1,
        reset=False,
        bandwidth="128MiB/s",
        model=model("register"),
    )
    values["model"]["minimum_access_latency_s"] = 1e-9
    run_config(
        params,
        "register-latency",
        "storage_traffic.py",
        values,
        rejected=True,
        reason="memory:immediate_RegisterBank_read_latency",
    )
    values["model"]["minimum_access_latency_s"] = 0
    values["model"]["mask_model"] = "read_modify_write"
    run_config(
        params,
        "register-rmw",
        "storage_traffic.py",
        values,
        rejected=True,
        reason="flip-flop read-modify-write is unsupported",
    )


def cell_only_scope(params):
    values = dict(
        kind="register",
        requestors=1,
        reset=False,
        bandwidth="128MiB/s",
        model=model("register"),
    )
    values["model"]["storage_scope"] = "storage_cells"
    reason = "selection_decoder_port_logic"
    run_config(
        params,
        "cell-only-strict",
        "storage_traffic.py",
        values,
        rejected=True,
        reason=reason,
    )
    values["model"]["strict"] = False
    outdir = run_config(
        params, "cell-only-incomplete", "storage_traffic.py", values
    )
    final = st.read_stats(outdir)[-1]
    if "STORAGE_CHECK_PASSED" not in read(outdir, "simout.txt"):
        raise AssertionError("cell-only reporting changed execution")
    for name in ("areaComplete", "dynamicComplete", "leakageComplete"):
        if final[MODEL + name] != 0:
            raise AssertionError(
                "cell-only storage claimed whole-device coverage"
            )
    near(final[MODEL + "knownAreaUm2"], 100)
    near(final[MODEL + "knownDynamicEnergyJ"], 3.04e-12)
    near(final[MODEL + "knownLeakageEnergyJ"], 6e-12)
    if reason not in read(outdir, "simout.txt"):
        raise AssertionError("cell-only omitted logic is not identified")


def unknown_word_width(params):
    values = dict(
        kind="spm",
        requestors=1,
        reset=False,
        reset_after_traffic=True,
        bandwidth="128MiB/s",
        model=dict(enabled=True, strict=False, report_details=True),
    )
    outdir = run_config(params, "unknown-width", "storage_traffic.py", values)
    output = read(outdir, "simout.txt")
    if "STORAGE_CHECK_PASSED" not in output:
        raise AssertionError("unknown characterization changed execution")
    dumps = st.read_stats(outdir)
    for dump, missing in (
        (dumps[0], 0),
        (dumps[1], 3),
        (dumps[2], 9),
        (dumps[-1], 0),
    ):
        if (
            dump[MODEL + "missingEventCount"] != missing
            or dump[MODEL + "readWords"] != 0
            or dump[MODEL + "writeWords"] != 0
            or dump[MODEL + "memoryWordCountsAvailable"] != 0
        ):
            raise AssertionError(
                "unknown-width accesses invented physical words"
            )
        near(dump[MODEL + "knownDynamicEnergyJ"], 0)
    for direction in ("read", "write"):
        if "memory:unknown_word_width:" + direction not in output:
            raise AssertionError("unknown serviced access lost its identity")
    last = output.split("   PPA model: system.memory.ppa")[-1]
    if "memory:unknown_word_width:" in last:
        raise AssertionError(
            "old unknown-access identity leaked into empty window"
        )


for name, test in (
    ("salam-ppa-spm-byte-enable", traffic("spm", equivalence=True)),
    ("salam-ppa-spm-rmw", traffic("spm", "read_modify_write")),
    ("salam-ppa-spm-shared-reset", traffic("spm", requestors=2, reset=True)),
    ("salam-ppa-spm-response-retry", traffic("spm", fast=True)),
    (
        "salam-ppa-register-shared-reset",
        traffic("register", requestors=2, reset=True),
    ),
    ("salam-ppa-register-functional-ports", register_functional_ports),
    ("salam-ppa-storage-unsupported", unsupported_mask),
    ("salam-ppa-storage-timing-boundaries", timing_boundaries),
    ("salam-ppa-storage-cell-only", cell_only_scope),
    ("salam-ppa-storage-unknown-width", unknown_word_width),
):
    TestSuite(
        name=name,
        fixtures=[st.SalamGem5Fixture(), TempdirFixture()],
        tags=[st.SALAM_TAG, "salam-ppa", "salam-ppa-storage"],
        tests=[TestFunction(test, name=name)],
    )

for reset in (False, True):
    gem5 = st.SalamGem5Fixture()
    tempdir = TempdirFixture()
    options = {"DEVICE_TEST_KIND": 0}
    if reset:
        options["DEVICE_RESET"] = 1
    name = f"salam-ppa-storage-dma-reset-{int(reset)}"
    TestSuite(
        name=name,
        fixtures=[
            gem5,
            tempdir,
            st.SalamHarnessFixture(
                "salam-workload",
                gem5,
                tempdir,
                "sanity.c",
                [],
                0,
                host_options=options,
            ),
        ],
        tags=[st.SALAM_TAG, "salam-ppa", "salam-ppa-storage"],
        tests=[TestFunction(dma(reset), name=name)],
    )
