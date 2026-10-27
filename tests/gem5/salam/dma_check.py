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


# DMA packet concurrency and readiness, using the production FIFO engines.
import argparse
import json
import os
import struct

import m5
from m5.objects import (
    AddrRange,
    ArmSystem,
    CommInterface,
    GicV2,
    NoncoherentDma,
    NoncoherentXBar,
    RealView,
    Root,
    SALAMDeviceTester,
    SimpleMemory,
    SrcClockDomain,
    StreamDma,
    StubWorkload,
    System,
    VoltageDomain,
)


def check_read_fifo(case):
    # Exercise the shared dev/DmaReadFifo directly, without a SALAM DMA
    # controller. The same payload is consumed in non-dividing chunks.
    bypass = "bypass" in case
    capacity = 16 if "small" in case else 64
    chunk = 12 if capacity == 16 else 48
    length = 2 * chunk + 2
    system = System()
    system.clk_domain = SrcClockDomain(
        clock="1GHz", voltage_domain=VoltageDomain()
    )
    system.mem_mode = "atomic_noncaching" if bypass else "timing"
    system.mem_ranges = [AddrRange(0, size=4096)]
    system.bus = NoncoherentXBar(
        width=8,
        frontend_latency=0,
        forward_latency=0,
        response_latency=0,
        header_latency=0,
    )
    system.memory = SimpleMemory(
        range=system.mem_ranges[0],
        latency="10ns",
        bandwidth="1000GiB/s",
        enable_backdoor="backdoor" in case,
    )
    system.memory.port = system.bus.mem_side_ports
    system.system_port = system.bus.cpu_side_ports
    system.tester = SALAMDeviceTester()
    system.tester.port = system.bus.cpu_side_ports
    system.tester.fifo_port = system.bus.cpu_side_ports
    root = Root(full_system=False, system=system)
    m5.instantiate()
    payload = bytes((17 * index + 3) % 256 for index in range(length))
    system.getCCObject().physProxy.write(0, payload)
    tester = system.tester
    states = []
    # A second transfer verifies that ending a block and draining all data
    # leaves the port usable, including any retained memory backdoor.
    for _ in range(2):
        tester.startReadFifo(0, length, capacity, chunk)
        state = tester.readFifoState()
        assert state[0] == (capacity if bypass else 0), state
        assert state[5] == int(bypass), state
        observed = bytearray()
        attempts = 0
        while len(observed) < length and attempts < 100:
            size = min(chunk, length - len(observed))
            data = tester.readFifoBytes(size)
            attempts += 1
            if not data:
                assert not bypass, "Synchronous FIFO refill stopped progress"
                event = m5.simulate(20000)
                assert event.getCause() == "simulate() limit reached"
                continue
            assert len(data) == size
            observed.extend(data)
            state = tester.readFifoState()
            states.append(state)
            assert state[0] <= capacity, state
        assert bytes(observed) == payload
        state = tester.readFifoState()
        assert state[:4] == [0, 1, 0, 1], state
        assert state[4] > 0 and state[5] == 1, state
        assert not tester.readFifoBytes(1)
        assert tester.readFifoState() == state
    result_path = os.path.join(m5.options.outdir, "device-results.json")
    with open(result_path, "w") as f:
        json.dump({"case": case, "states": states}, f, indent=2)
    print("DEVICE CHECK PASSED", case)


def check_registers(case):
    # Component-only register/argument checks; checked BFS covers launches.
    comm = case.startswith("comm-registers-")
    noncoherent = "-nc" in case
    register_bytes = 21 if noncoherent or comm else 32
    pio_size = register_bytes if noncoherent or comm else 40
    flags_size = 2 if "-config-" in case else 1
    config_size = 3 if "-config-" in case else 0
    byte_order = "big" if case.endswith("-big") else "little"
    premap = case.endswith("-premap")
    premapped = [0x8877665544332211, 0xFEDCBA9876543210]
    invalid_layout = {
        "comm-registers-zero-map": (0, 1, 0),
        "comm-registers-zero-flags": (register_bytes, 0, 0),
        "comm-registers-large-flags": (register_bytes, register_bytes + 1, 0),
        "comm-registers-large-config": (register_bytes, 1, register_bytes),
        "comm-registers-overflow-config": (register_bytes, 1, (1 << 64) - 1),
    }
    if case in invalid_layout:
        pio_size, flags_size, config_size = invalid_layout[case]
    elif case.endswith("small-map"):
        pio_size = register_bytes - 1
    elif premap:
        # Premapped slots need no argument bytes in the register image.
        pio_size = flags_size
    elif case.endswith("argument-empty"):
        config_size = pio_size - flags_size
    argument_start = flags_size + config_size
    argument_bytes = pio_size - argument_start
    system = ArmSystem(highest_el_is_64=False)
    system.workload = StubWorkload(byte_order=byte_order)
    system.clk_domain = SrcClockDomain(
        clock="1GHz", voltage_domain=VoltageDomain()
    )
    system.mem_mode = "timing"
    system.mem_ranges = [AddrRange(0, size=4096)]
    system.realview = RealView()
    system.realview.gic = GicV2(dist_addr=0x10000, cpu_addr=0x20000)
    system.bus = NoncoherentXBar(
        width=8,
        frontend_latency=0,
        forward_latency=0,
        response_latency=0,
        header_latency=0,
    )
    system.realview.gic.pio = system.bus.mem_side_ports
    system.memory = SimpleMemory(range=system.mem_ranges[0])
    system.memory.port = system.bus.mem_side_ports
    device_type = (
        CommInterface if comm else NoncoherentDma if noncoherent else StreamDma
    )
    system.device = device_type(
        pio_addr=0x30000,
        pio_size=pio_size,
        gic=system.realview.gic,
    )
    if comm:
        system.device.flags_size = flags_size
        system.device.config_size = config_size
        system.device.premap_data = premap
        system.device.data_bases = premapped
        system.device.enable_debug_msgs = True
    else:
        system.device.dma = system.bus.cpu_side_ports
        if noncoherent:
            system.local_memory = SimpleMemory(
                range=AddrRange(0x4000, size=4096)
            )
            system.device.cluster_dma = system.local_memory.port
        else:
            system.device.stream_addr = 0x40000
            system.device.status_addr = 0x40008
    system.system_port = system.bus.cpu_side_ports
    system.tester = SALAMDeviceTester()
    # Direct attachment lets boundary probes reach the device itself,
    # independently of any interconnect range check.
    system.tester.port = system.device.pio
    root = Root(full_system=False, system=system)
    m5.instantiate()
    tester = system.tester
    base = 0x30000
    if case in invalid_layout or case.endswith("small-map"):
        raise AssertionError("An invalid register layout was accepted")
    invalid_access = {
        "cross-read": (True, base + register_bytes - 1, 2),
        "cross-write": (False, base + register_bytes - 1, 2),
        "before-read": (True, base - 1, 1),
        "before-write": (False, base - 1, 1),
        "after-read": (True, base + register_bytes, 1),
        "after-write": (False, base + register_bytes, 1),
        "wide-read": (True, base + 4, 8),
        "size-read": (True, base + 4, 3),
    }
    for suffix, (read, addr, size) in invalid_access.items():
        if case.endswith(suffix):
            if read:
                tester.functionalRead(addr, size)
            else:
                tester.functionalWrite(addr, size, 0xBBAA)
            raise AssertionError("An invalid register access was accepted")
    invalid_argument = {
        "argument-cross": (argument_bytes - 1, 2),
        "argument-after": (argument_bytes, 1),
        "argument-offset": ((1 << 32) - 1, 1),
        "argument-empty": (0, 1),
        "argument-zero-size": (0, 0),
        "argument-size": (0, 3),
    }
    for suffix, (offset, size) in invalid_argument.items():
        if case.endswith(suffix):
            tester.readCommArgument(system.device.getCCObject(), offset, size)
            raise AssertionError("An invalid argument access was accepted")
    if premap:
        observations = []
        # Preserve the existing offset/8 lookup, independent of requested
        # width and of the memory-mapped argument extent.
        for offset in range(16):
            for size in (0, 1, 2, 3, 4, 8, 16):
                value = tester.readCommArgument(
                    system.device.getCCObject(), offset, size
                )
                assert value == premapped[offset // 8], (offset, size, value)
                observations.append([offset, size, value])
        assert m5.curTick() == 0
        result_path = os.path.join(m5.options.outdir, "device-results.json")
        with open(result_path, "w") as f:
            json.dump(
                {
                    "case": case,
                    "scope": "component-only",
                    "argument_reads": observations,
                },
                f,
                indent=2,
            )
        print("DEVICE CHECK PASSED", case)
        return

    # The reference is the packed byte image, independently of the host's
    # alignment and integer representation. Keep the run bit clear.
    expected = bytearray([0] + list(range(0x41, 0x40 + register_bytes)))
    offset = 0
    while offset < register_bytes:
        size = min(8, register_bytes - offset)
        tester.functionalWrite(
            base + offset,
            size,
            int.from_bytes(expected[offset : offset + size], "little"),
        )
        offset += size
    widths = (1, 2, 4, 8) if noncoherent or comm else (1, 2, 4)
    observations = []
    argument_observations = []

    def verify():
        for size in widths:
            for offset in range(register_bytes - size + 1):
                value = tester.functionalRead(base + offset, size)
                wanted = int.from_bytes(
                    expected[offset : offset + size], "little"
                )
                assert value == wanted, (offset, size, value, wanted)
                observations.append([offset, size, value])
            if comm:
                for offset in range(argument_bytes - size + 1):
                    value = tester.readCommArgument(
                        system.device.getCCObject(), offset, size
                    )
                    start = argument_start + offset
                    wanted = int.from_bytes(
                        expected[start : start + size], byte_order
                    )
                    assert value == wanted, (offset, size, value, wanted)
                    argument_observations.append([offset, size, value])

    verify()
    tester.functionalWrite(base + 4, 2, 0xBBAA)
    expected[4:6] = bytes([0xAA, 0xBB])
    verify()
    tester.functionalWrite(base + register_bytes - 1, 1, 0xC5)
    expected[-1] = 0xC5
    verify()
    masked_data = bytes(range(0xE1, 0xE9))
    byte_enable = [True, False] * 4
    tester.functionalWriteMasked(
        base + 2, 8, int.from_bytes(masked_data, "little"), byte_enable
    )
    for index, enabled in enumerate(byte_enable):
        if enabled:
            expected[2 + index] = masked_data[index]
    verify()
    assert m5.curTick() == 0
    result_path = os.path.join(m5.options.outdir, "device-results.json")
    with open(result_path, "w") as f:
        json.dump(
            {
                "case": case,
                "scope": "component-only",
                "reads": observations,
                "argument_reads": argument_observations,
            },
            f,
            indent=2,
        )
    print("DEVICE CHECK PASSED", case)


parser = argparse.ArgumentParser()
parser.add_argument("case")
args = parser.parse_args()
if args.case.startswith(("dma-registers-", "comm-registers-")):
    check_registers(args.case)
    raise SystemExit(0)
if args.case.startswith("dma-fifo-"):
    check_read_fifo(args.case)
    raise SystemExit(0)
ready = args.case in ("dma-read-ready", "dma-write-ready")
atomic = args.case == "dma-atomic"
slots = 1 if args.case == "dma-one" or ready else 2
length = 70 if args.case == "dma-tail" else 128
system = ArmSystem(highest_el_is_64=False)
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=VoltageDomain()
)
system.mem_mode = "atomic_noncaching" if atomic else "timing"
system.mem_ranges = [AddrRange(0, size=4096)]
system.realview = RealView()
system.realview.gic = GicV2(dist_addr=0x10000, cpu_addr=0x20000)
system.bus = NoncoherentXBar(
    width=8,
    frontend_latency=0,
    forward_latency=0,
    response_latency=0,
    header_latency=0,
)
system.realview.gic.pio = system.bus.mem_side_ports
system.memory = SimpleMemory(
    range=system.mem_ranges[0],
    latency="10ns" if ready else "100ns",
    bandwidth="1000GiB/s",
)
system.memory.port = system.bus.mem_side_ports
system.device = StreamDma(
    pio_addr=0x30000,
    pio_size=160,
    stream_addr=0x40000,
    status_addr=0x40008,
    read_buffer_size=16 if ready else 128,
    write_buffer_size=8 if ready else 128,
    max_req_size=4 if ready else 64,
    max_pending=slots,
    gic=system.realview.gic,
    bandwidth="1000GiB/s",
    pio_delay="1ns",
)
system.device.pio = system.bus.mem_side_ports
system.device.dma = system.bus.cpu_side_ports
system.system_port = system.bus.cpu_side_ports
system.reader = SALAMDeviceTester()
system.writer = SALAMDeviceTester()
system.reader.port = system.device.stream_out
system.writer.port = system.device.stream_in
root = Root(full_system=False, system=system)
m5.instantiate()
proxy = system.getCCObject().physProxy


def until(tick):
    event = m5.simulate(tick - m5.curTick())
    assert event.getCause() == "simulate() limit reached"
    assert m5.curTick() == tick


until(1)
registers = bytearray(32)
# Keep interrupt status asserted; frequency zero requests no GIC delivery.
# The test needs no CPU contexts or interrupt-controller initialization.
registers[0] = 0x30
if args.case == "dma-read-ready":
    payload = bytes(range(1, 33))
    proxy.write(0, payload)
    registers[0] |= 1
    registers[20:24] = struct.pack("<I", 32)
    registers[24] = registers[25] = 1
    assert not system.reader.send(True, 0x40000, 8, 0)
    proxy.write(0x30000, registers)
    until(15000)
    assert not system.reader.getRequestRetries()  # only first four bytes
    until(60000)
    assert len(system.reader.getRequestRetries()) == 1
    assert system.reader.getResponses()[0][3] == int.from_bytes(
        payload[:8], "little"
    )
    # The 32-byte frame exceeds the FIFO: readiness must precede onIdle.
    assert proxy.read(0x30000, 1)[0] & 4
elif args.case == "dma-write-ready":
    proxy.write(0x30020, bytes(range(1, 9)))
    registers[0] |= 2
    registers[26:30] = struct.pack("<I", 16)
    registers[30] = registers[31] = 1
    assert not system.writer.send(
        False, 0x40000, 8, int.from_bytes(bytes(range(9, 17)), "little")
    )
    proxy.write(0x30000, registers)
    until(5000)
    assert not system.writer.getRequestRetries()  # only four bytes free
    until(100000)
    assert len(system.writer.getRequestRetries()) == 1
    assert len(system.writer.getResponses()) == 1
    assert proxy.read(0, 16) == bytes(range(1, 17))
else:
    payload = bytes(range(length))
    proxy.write(0x30020, payload)
    registers[0] |= 2
    registers[26:30] = struct.pack("<I", length)
    registers[30] = registers[31] = 1
    proxy.write(0x30000, registers)
    until(50000)
    m5.stats.dump()
    if not atomic:
        with open(os.path.join(m5.options.outdir, "stats.txt")) as stream:
            stats = {
                fields[0]: float(fields[1])
                for line in stream
                if len(fields := line.split()) >= 2
                and fields[0].startswith("system.device.memoryPackets.")
            }
        assert stats["system.device.memoryPackets.writePackets"] == slots
        assert stats["system.device.memoryPackets.writeResponses"] == 0
    until(400000)
    assert proxy.read(0, length) == payload
    assert proxy.read(length, 8) == bytes(8)

with open(os.path.join(m5.options.outdir, "device-results.json"), "w") as f:
    json.dump(
        {
            "case": args.case,
            "read_retries": system.reader.getRequestRetries(),
            "write_retries": system.writer.getRequestRetries(),
            "read_responses": system.reader.getResponses(),
            "write_responses": system.writer.getResponses(),
        },
        f,
        indent=2,
    )
print("DEVICE CHECK PASSED", args.case)
