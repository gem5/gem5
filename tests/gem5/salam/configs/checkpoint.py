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


"""Save and restore real SALAM traffic in separate gem5 processes."""

import argparse
import json
from pathlib import Path

import m5
from m5.objects import (
    AddrRange,
    ArmSystem,
    ArmTimingSimpleCPU,
    GicV2,
    NoncoherentDma,
    NoncoherentXBar,
    RealView,
    RegisterBank,
    Root,
    SALAMDeviceTester,
    ScratchpadMemory,
    SimpleMemory,
    SrcClockDomain,
    StreamBuffer,
    StreamDma,
    StubWorkload,
    SystemXBar,
    VoltageDomain,
)

MEMORY = 0x100000
DESTINATION = 0x200000
PIO = 0x30000
STREAM = 0x40000
STATUS = STREAM + 8
SPM = 0x4000
SENTINEL = 0xEE
ARGUMENTS = (19, 7)
NEW_ARGUMENTS = (35, 12)
COPY_SIZE = 193
FRAME_SIZE = 32
FRAMES = 3


def payload(size, multiplier=17):
    return bytes((multiplier * i + 3) % 128 for i in range(size))


def step(seconds=1e-9):
    event = m5.simulate(max(1, m5.ticks.fromSeconds(seconds)))
    assert event.getCause() == "simulate() limit reached", event.getCause()


def wait_for(condition, seconds=1e-9, attempts=20000):
    for _ in range(attempts):
        if condition():
            return
        step(seconds)
    raise AssertionError("SALAM checkpoint continuation did not complete")


def component(cluster=False):
    system = ArmSystem(highest_el_is_64=False)
    system.workload = StubWorkload()
    system.clk_domain = SrcClockDomain(
        clock="1GHz", voltage_domain=VoltageDomain()
    )
    system.mem_mode = "timing"
    system.mem_ranges = [AddrRange(MEMORY, size=65536)]
    bus = NoncoherentXBar(
        width=8,
        frontend_latency=0,
        forward_latency=0,
        response_latency=0,
        header_latency=1,
    )
    if cluster:
        system.membus = SystemXBar()
        system.iobus = bus
        system.membus.mem_side_ports = system.iobus.cpu_side_ports
        memory_bus = system.membus
        io_bus = system.iobus
    else:
        system.bus = bus
        memory_bus = system.bus
        io_bus = system.bus
    system.memory = SimpleMemory(range=system.mem_ranges[0], latency="10ns")
    system.memory.port = memory_bus.mem_side_ports
    system.local_memory = SimpleMemory(
        range=AddrRange(DESTINATION, size=65536), latency="10ns"
    )
    system.local_memory.port = memory_bus.mem_side_ports
    system.system_port = memory_bus.cpu_side_ports
    system.realview = RealView()
    system.realview.gic = GicV2(dist_addr=0x10000, cpu_addr=0x20000)
    system.realview.gic.pio = io_bus.mem_side_ports
    return system


def accelerator(accpath):
    from gem5.salam.cluster import build_clusters

    system = component(cluster=True)
    build_clusters(system, accpath)
    system.harness_clstr.kernel.int_num = -1
    system.tester = SALAMDeviceTester()
    system.tester.port = system.membus.cpu_side_ports
    return system


def dma(kind):
    system = component()
    if kind == "noncoherent":
        # GicV2 requires a registered, non-halted context even while its
        # distributor is disabled. Keep a real CPU suspended throughout.
        system.cpu = ArmTimingSimpleCPU()
        system.cpu.createInterruptController()
        system.cpu.createThreads()
        system.cpu.connectBus(system.bus)
        system.device = NoncoherentDma(
            pio_addr=PIO,
            pio_size=21,
            gic=system.realview.gic,
            int_num=95,
            buffer_size=64,
            max_req_size=16,
            max_pending=1,
            # Leave a checkpoint boundary between memory responses and the
            # next service pass, so the roundtrip retains unfinished work.
            process_cycles=100,
        )
        system.device.cluster_dma = system.bus.cpu_side_ports
    else:
        system.device = StreamDma(
            pio_addr=PIO,
            pio_size=160,
            stream_addr=STREAM,
            status_addr=STATUS,
            gic=system.realview.gic,
            rd_int=0,
            wr_int=0,
            read_buffer_size=32,
            write_buffer_size=64,
            max_req_size=16,
            max_pending=1,
        )
        system.read_status = SALAMDeviceTester()
        system.read_status.port = system.device.status_out
        system.write_status = SALAMDeviceTester()
        system.write_status.port = system.device.status_in
    system.device.dma = system.bus.cpu_side_ports
    system.tester = SALAMDeviceTester()
    system.tester.port = system.device.pio
    return system


def storage():
    system = component()
    system.registers = RegisterBank(
        range=AddrRange(0, size=4096), access_cycles=1, write_cycles=30
    )
    system.scratchpad = ScratchpadMemory(
        range=AddrRange(SPM, size=4096),
        access_cycles=1,
        access_cycles_var=5,
        ready_mode=True,
        bandwidth="1MiB/s",
    )
    system.stream = StreamBuffer(
        stream_address=STREAM,
        stream_size=8,
        status_address=STATUS,
        buffer_size=32,
        bandwidth="1MiB/s",
    )
    connections = {
        "reg_writer": system.registers.reg_port,
        "reg_reader": system.registers.load_port,
        "spm_writer": system.scratchpad.port,
        "spm_reader": system.scratchpad.spm_ports,
        "stream_writer": system.stream.stream_in,
        "stream_reader": system.stream.stream_out,
        "stream_status": system.stream.status_out,
    }
    for name, port in connections.items():
        setattr(system, name, SALAMDeviceTester())
        getattr(system, name).port = port
    return system


def checkpoint(args):
    print("CHECKPOINT PRECONDITION " + args.case, flush=True)
    m5.checkpoint(args.checkpoint)
    assert Path(args.checkpoint, "m5.cpt").is_file()


def args_write(system, values):
    address = int(system.harness_clstr.kernel.pio_addr)
    for i, value in enumerate(values):
        system.tester.functionalWrite(address + 1 + 4 * i, 4, value)


def args_read(system):
    comm = system.harness_clstr.kernel.getCCObject()
    return [system.tester.readCommArgument(comm, 4 * i, 4) for i in range(2)]


def accelerator_result(system):
    address = int(system.harness_clstr.out.range.start)
    data = system.getCCObject().physProxy.read(address, 8)
    return [int.from_bytes(data[i : i + 4], "little") for i in (0, 4)]


def run_accelerator(system):
    tester = system.tester
    address = int(system.harness_clstr.kernel.pio_addr)
    tester.functionalWrite(address, 1, 1)
    wait_for(lambda: tester.functionalRead(address, 1) & 4)
    assert not tester.functionalRead(address, 1) & 3
    return accelerator_result(system)


def accelerator_case(args, system):
    address = int(system.harness_clstr.kernel.pio_addr)
    tester = system.tester
    if args.phase == "save":
        args_write(system, ARGUMENTS)
        assert args_read(system) == list(ARGUMENTS)
        if args.case == "active":
            tester.functionalWrite(address, 1, 1)
            step()
            # The configured 50-cycle add cannot finish in one 1 GHz cycle.
            assert tester.functionalRead(address, 1) & 2
            assert not tester.functionalRead(address, 1) & 4
        else:
            m5.drain()
            assert tester.functionalRead(address, 1) == 0
        checkpoint(args)
    assert args_read(system) == list(ARGUMENTS)
    initial_flags = tester.functionalRead(address, 1)
    if args.case == "active":
        assert initial_flags & 4 and not initial_flags & 3
        first = accelerator_result(system)
    else:
        assert initial_flags == 0
        first = run_accelerator(system)
    assert first == [sum(ARGUMENTS), ARGUMENTS[0] ^ ARGUMENTS[1]]
    args_write(system, NEW_ARGUMENTS)
    second = run_accelerator(system)
    assert second == [sum(NEW_ARGUMENTS), NEW_ARGUMENTS[0] ^ NEW_ARGUMENTS[1]]
    return {
        "initial_flags": initial_flags,
        "arguments": list(ARGUMENTS),
        "first": first,
        "new_arguments": args_read(system),
        "second": second,
    }


def copy_program(tester, source, destination, size):
    for offset, width, value in (
        (1, 8, source),
        (9, 8, destination),
        (17, 4, size),
    ):
        tester.functionalWrite(PIO + offset, width, value)


def copied_prefix(proxy, expected):
    actual = proxy.read(DESTINATION, len(expected))
    count = 0
    for actual_byte, expected_byte in zip(actual, expected):
        if actual_byte != expected_byte:
            break
        count += 1
    assert actual[count:] == bytes([SENTINEL]) * (len(expected) - count)
    return count


def noncoherent_case(args, system):
    proxy = system.getCCObject().physProxy
    tester = system.tester
    first = payload(COPY_SIZE)
    second = payload(COPY_SIZE, 23)
    if args.phase == "save":
        proxy.write(MEMORY, first)
        proxy.write(MEMORY + 256, second)
        proxy.write(DESTINATION, bytes([SENTINEL]) * 512)
        copy_program(tester, MEMORY, DESTINATION, COPY_SIZE)
        m5.drain()
        assert tester.functionalRead(PIO + 1, 8) == MEMORY
        assert tester.functionalRead(PIO + 9, 8) == DESTINATION
        tester.functionalWrite(PIO, 1, 1)
        wait_for(lambda: copied_prefix(proxy, first) > 0)
        assert tester.functionalRead(PIO, 1) & 2
        assert copied_prefix(proxy, first) < COPY_SIZE
        checkpoint(args)
    assert tester.functionalRead(PIO, 1) & 2
    assert 0 < copied_prefix(proxy, first) < COPY_SIZE
    assert tester.functionalRead(PIO + 1, 8) == MEMORY
    assert tester.functionalRead(PIO + 9, 8) == DESTINATION
    assert tester.functionalRead(PIO + 17, 4) == COPY_SIZE
    wait_for(lambda: tester.functionalRead(PIO, 1) & 4)
    assert not tester.functionalRead(PIO, 1) & 2
    assert proxy.read(DESTINATION, COPY_SIZE) == first
    copy_program(tester, MEMORY + 256, DESTINATION + 256, COPY_SIZE)
    tester.functionalWrite(PIO, 1, 1)
    wait_for(lambda: tester.functionalRead(PIO, 1) & 4)
    assert proxy.read(DESTINATION + 256, COPY_SIZE) == second
    gap = proxy.read(DESTINATION + COPY_SIZE, 256 - COPY_SIZE)
    tail = proxy.read(DESTINATION + 256 + COPY_SIZE, 256 - COPY_SIZE)
    assert gap == tail == bytes([SENTINEL]) * (256 - COPY_SIZE)
    assert system.cpu.totalInsts() == 0
    return {"first": first.hex(), "second": second.hex(), "gap": gap.hex()}


def stream_program(tester, source, destination, frames):
    values = (
        (1, 1, 1),
        (2, 1, 1),
        (4, 4, source),
        (8, 4, 0),
        (12, 4, destination),
        (16, 4, 0),
        (20, 4, FRAME_SIZE),
        (24, 1, frames),
        (25, 1, frames),
        (26, 4, FRAME_SIZE),
        (30, 1, frames),
        (31, 1, frames),
    )
    for offset, width, value in values:
        tester.functionalWrite(PIO + offset, width, value)


def stream_put(tester, data):
    assert len(data) % 8 == 0
    for offset in range(0, len(data), 8):
        tester.functionalWrite(
            PIO + 32, 8, int.from_bytes(data[offset : offset + 8], "little")
        )


def stream_continue(
    system, expected_read, expected_write, prefix=b"", written=0
):
    tester = system.tester
    result = bytearray(prefix)
    for _ in range(20000):
        available = system.read_status.functionalRead(STATUS, 4)
        while available >= 8:
            result += tester.functionalRead(PIO + 32, 8).to_bytes(8, "little")
            available -= 8
        buffered = system.write_status.functionalRead(STATUS, 4)
        while written < len(expected_write) and buffered <= 56:
            stream_put(tester, expected_write[written : written + 8])
            written += 8
            buffered += 8
        flags = tester.functionalRead(PIO, 1)
        if not flags & 0x0C and len(result) == len(expected_read):
            break
        step()
    else:
        raise AssertionError("Stream DMA did not resume its finite frames")
    assert bytes(result) == expected_read
    return bytes(result), flags


def stream_dma_case(args, system):
    proxy = system.getCCObject().physProxy
    tester = system.tester
    size = FRAME_SIZE * FRAMES
    first = payload(size)
    written = payload(size, 23)
    if args.phase == "save":
        proxy.write(MEMORY, first)
        proxy.write(MEMORY + 256, payload(FRAME_SIZE, 29))
        proxy.write(DESTINATION, bytes([SENTINEL]) * 512)
        stream_program(tester, MEMORY, DESTINATION, FRAMES)
        stream_put(tester, written[:48])
        tester.functionalWrite(PIO, 1, 3)
        wait_for(
            lambda: system.read_status.functionalRead(STATUS, 4) >= 8
            and proxy.read(DESTINATION, 1) == written[:1]
        )
        assert tester.functionalRead(PIO, 1) & 0x0C == 0x0C
        prefix = tester.functionalRead(PIO + 32, 8).to_bytes(8, "little")
        assert prefix == first[:8]
        assert proxy.read(DESTINATION, size) != written
        checkpoint(args)
    assert tester.functionalRead(PIO, 1) & 0x0C == 0x0C
    assert 0 < system.read_status.functionalRead(STATUS, 4) <= 32
    assert tester.functionalRead(PIO + 4, 4) == MEMORY
    assert tester.functionalRead(PIO + 12, 4) == DESTINATION
    received, flags = stream_continue(system, first, written, first[:8], 48)
    assert flags == 0x30
    assert proxy.read(DESTINATION, size) == written
    next_read = payload(FRAME_SIZE, 29)
    next_write = payload(FRAME_SIZE, 31)
    stream_program(tester, MEMORY + 256, DESTINATION + 256, 1)
    stream_put(tester, next_write)
    tester.functionalWrite(PIO, 1, 3)
    received_again, flags = stream_continue(
        system, next_read, next_write, written=FRAME_SIZE
    )
    assert flags == 0x30
    assert proxy.read(DESTINATION + 256, FRAME_SIZE) == next_write
    gap = proxy.read(DESTINATION + size, 256 - size)
    tail = proxy.read(DESTINATION + 256 + FRAME_SIZE, 256 - FRAME_SIZE)
    assert gap == bytes([SENTINEL]) * (256 - size)
    assert tail == bytes([SENTINEL]) * (256 - FRAME_SIZE)
    return {
        "received": received.hex(),
        "written": written.hex(),
        "received_again": received_again.hex(),
        "written_again": next_write.hex(),
        "flags": flags,
    }


def ready(system, offset):
    return system.spm_reader.scratchpadReady(
        system.scratchpad.getCCObject(), SPM + offset, 8, True
    )


def timing_access(tester, read, address, value=0):
    previous = len(tester.getResponses())
    issued = m5.curTick()
    accepted = tester.send(read, address, 8, value)
    wait_for(lambda: len(tester.getResponses()) > previous)
    response = tester.getResponses()[-1]
    return accepted, response[3], response[0] - issued


def storage_case(args, system):
    initial = 0x8877665544332211
    published = 0x0807060504030201
    later = 0x1817161514131211
    chunks = [payload(8, multiplier) for multiplier in (17, 23, 29)]
    if args.phase == "save":
        system.reg_writer.functionalWrite(0, 8, initial)
        timing_access(system.spm_writer, False, SPM, initial)
        step(20e-6)
        assert ready(system, 0)
        assert timing_access(system.spm_reader, True, SPM)[1] == initial
        step(20e-6)
        assert not ready(system, 0)
        timing_access(system.spm_writer, False, SPM + 8, published)
        assert ready(system, 8)
        system.stream_writer.functionalWrite(
            STREAM, 8, int.from_bytes(chunks[0], "little")
        )
        assert timing_access(
            system.stream_writer,
            False,
            STREAM,
            int.from_bytes(chunks[1], "little"),
        )[0]
        consumed = system.stream_reader.functionalRead(STREAM, 4)
        assert consumed.to_bytes(4, "little") == chunks[0][:4]
        assert system.stream_status.functionalRead(STATUS, 4) == 12
        # Atomic writes leave a pending publication event without queuing
        # a response. Drain must wait for that event on its own.
        assert system.reg_writer.atomicWrite(0, 8, published) > 0
        assert system.reg_writer.send(True, 0, 8, 0)
        assert system.reg_writer.getResponses()[-1][3] == initial
        checkpoint(args)
    assert not ready(system, 0) and ready(system, 8)
    assert not ready(system, 16)
    assert timing_access(system.reg_writer, True, 0)[1] == published
    assert system.stream_status.functionalRead(STATUS, 4) == 12
    # Both ports were busy past their response and checkpoint drain. New
    # requests must retain their retry obligation until bandwidth releases.
    previous_spm = len(system.spm_writer.getResponses())
    previous_stream = len(system.stream_writer.getResponses())
    issued = m5.curTick()
    assert not system.spm_writer.send(False, SPM + 16, 8, later)
    assert not system.stream_writer.send(
        False, STREAM, 8, int.from_bytes(chunks[2], "little")
    )
    wait_for(
        lambda: len(system.spm_writer.getResponses()) > previous_spm
        and len(system.stream_writer.getResponses()) > previous_stream,
        seconds=1e-7,
    )
    retry_delays = [
        system.spm_writer.getResponses()[-1][0] - issued,
        system.stream_writer.getResponses()[-1][0] - issued,
    ]
    assert all(delay > 0 for delay in retry_delays)
    assert ready(system, 8) and ready(system, 16)
    values = []
    latencies = []
    for offset, expected in ((8, published), (16, later)):
        accepted, value, delay = timing_access(
            system.spm_reader, True, SPM + offset
        )
        assert accepted and value == expected
        assert (
            m5.ticks.fromSeconds(1e-9) <= delay <= m5.ticks.fromSeconds(7e-9)
        )
        values.append(value)
        latencies.append(delay)
        assert not ready(system, offset)
        # This reader also has a bandwidth interval; settle it before reuse.
        step(20e-6)
    assert system.stream_status.functionalRead(STATUS, 4) == 20
    stream_bytes = bytearray()
    for size in (8, 8, 4):
        stream_bytes += system.stream_reader.functionalRead(
            STREAM, size
        ).to_bytes(size, "little")
    assert bytes(stream_bytes) == chunks[0][4:] + chunks[1] + chunks[2]
    assert system.stream_status.functionalRead(STATUS, 4) == 0
    assert timing_access(system.reg_writer, False, 8, later)[0]
    assert timing_access(system.reg_writer, True, 8)[1] == later
    return {
        "register": published,
        "later_register": later,
        "scratchpad": values,
        "latencies": latencies,
        "retry_delays": retry_delays,
        "stream": stream_bytes.hex(),
    }


parser = argparse.ArgumentParser()
parser.add_argument(
    "case",
    choices=(
        "idle",
        "active",
        "noncoherent",
        "stream-dma",
        "storage",
        "control",
    ),
)
parser.add_argument("phase", choices=("save", "restore"))
parser.add_argument("--checkpoint", required=True)
parser.add_argument("--accpath")
args = parser.parse_args()
if args.case in ("idle", "active"):
    system = accelerator(args.accpath)
elif args.case in ("noncoherent", "stream-dma"):
    system = dma(args.case)
elif args.case == "storage":
    system = storage()
else:
    system = component()
root = Root(full_system=args.case == "noncoherent", system=system)
m5.instantiate(args.checkpoint if args.phase == "restore" else None)
if args.case == "noncoherent" and args.phase == "save":
    system.tester.suspendThread()
if args.case in ("idle", "active"):
    observation = accelerator_case(args, system)
elif args.case == "noncoherent":
    observation = noncoherent_case(args, system)
elif args.case == "stream-dma":
    observation = stream_dma_case(args, system)
elif args.case == "storage":
    observation = storage_case(args, system)
else:
    proxy = system.getCCObject().physProxy
    data = payload(37)
    if args.phase == "save":
        proxy.write(MEMORY, data)
        checkpoint(args)
    assert proxy.read(MEMORY, len(data)) == data
    proxy.write(MEMORY + 64, data[::-1])
    assert proxy.read(MEMORY + 64, len(data)) == data[::-1]
    observation = {"payload": data.hex(), "new_payload": data[::-1].hex()}
observation["finish_tick"] = m5.curTick()
with open(Path(m5.options.outdir, "observation.json"), "w") as stream:
    json.dump(observation, stream, indent=2)
print("SALAM CHECKPOINT ROUNDTRIP " + args.case + " " + args.phase, flush=True)
