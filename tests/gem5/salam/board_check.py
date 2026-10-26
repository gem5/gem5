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


# Builds a SALAMArmBoard around a workload and writes its configuration
# without simulating. With --ruby, it tries a Ruby cache hierarchy, which
# the board does not connect; with --attach-first, it attaches the
# accelerators before setting the program.

import argparse

import m5

from gem5.components.cachehierarchies.chi.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.salam.board import SALAMArmBoard

parser = argparse.ArgumentParser()
parser.add_argument("--binary")
parser.add_argument("--accpath")
parser.add_argument("--ruby", action="store_true")
parser.add_argument("--attach-first", action="store_true")
parser.add_argument("--device-kind", type=int)
parser.add_argument("--dma-buffer-size", type=int)
parser.add_argument("--dma-read-buffer-size", type=int)
parser.add_argument("--dma-write-buffer-size", type=int)
parser.add_argument("--dma-max-req-size", type=int)
parser.add_argument("--dma-local-memory", action="store_true")
parser.add_argument("--dma-bypass", action="store_true")
parser.add_argument("--queue-timing", action="store_true")
parser.add_argument("--stats-at", default="")
args = parser.parse_args()

if args.ruby:
    hierarchy = PrivateL1CacheHierarchy(size="16KiB", assoc=4)
else:
    hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
    )

board = SALAMArmBoard(
    clk_freq="1GHz",
    processor=SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC if args.dma_bypass else CPUTypes.TIMING,
        isa=ISA.ARM,
        num_cores=1,
        clk_freq="1GHz",
    ),
    # The program spans from 0 to above 2 GiB, and loading it needs at
    # least that much memory.
    memory=SingleChannelDDR4_2400(size="4GiB"),
    cache_hierarchy=hierarchy,
)
if args.attach_first:
    board.attach_accelerators(args.accpath)
board.set_bare_metal_binary(args.binary)
(cluster,) = board.attach_accelerators(args.accpath)
print("attached", cluster.name)


if args.device_kind is not None:
    from m5.objects import (
        AddrRange,
        SimpleMemory,
        StreamBuffer,
        StreamDma,
    )

    board.test_memory = SimpleMemory(
        range=AddrRange(0x2FF10000, size=4096),
        latency="10us",
        bandwidth="8MiB/s",
        enable_backdoor=not args.dma_bypass,
    )
    board.test_memory.port = board.iobus.mem_side_ports
    if args.dma_local_memory:
        board.test_local_memory = SimpleMemory(
            range=AddrRange(0x2FF11000, size=4096),
            latency="10us",
            bandwidth="8MiB/s",
            enable_backdoor=not args.dma_bypass,
        )
        board.test_local_memory.port = (
            board.harness_clstr.local_bus.mem_side_ports
        )
    if args.device_kind == 0:
        board.harness_clstr.coherency_bus.header_latency = 10
    if args.device_kind == 1:
        board.test_stream_buffer = StreamBuffer(
            stream_address=0x2FF20000,
            stream_size=8,
            status_address=0x2FF20008,
            buffer_size=32,
            stream_cycles=100000,
            bandwidth="32KiB/s",
        )
        board.test_stream_buffer.stream_in = board.iobus.mem_side_ports
    if args.device_kind in (2, 3):
        hierarchy.membus.header_latency = 10
        board.test_stream_dma = StreamDma(
            pio_addr=0x2FF30000,
            pio_size=40 if args.device_kind == 3 else 32,
            stream_addr=0x2FF30040 if args.device_kind == 3 else 0x2FF30020,
            status_addr=0x2FF30048 if args.device_kind == 3 else 0x2FF30028,
            stream_size=8,
            read_buffer_size=128,
            write_buffer_size=128,
            max_req_size=64,
            max_pending=2,
            pio_delay="1ns",
            bandwidth="1MiB/s",
            gic=board.realview.gic,
        )
        board.test_stream_dma.pio = board.iobus.mem_side_ports
        board.test_stream_dma.stream_in = board.iobus.mem_side_ports
        board.test_stream_dma.status_out = board.iobus.mem_side_ports
        board.test_stream_dma.dma = hierarchy.membus.cpu_side_ports

if args.device_kind in (0, 2):
    dma = (
        board.harness_clstr.dma
        if args.device_kind == 0
        else board.test_stream_dma
    )
    for field in (
        "buffer_size",
        "read_buffer_size",
        "write_buffer_size",
        "max_req_size",
    ):
        value = getattr(args, "dma_" + field)
        if value is not None:
            setattr(dma, field, value)

if args.queue_timing:
    board.harness_clstr.out.access_cycles = 3
    board.harness_clstr.out.write_cycles = 3

root = board._pre_instantiate()
if args.dma_bypass:
    from gem5.components.boards.mem_mode import MemMode

    board.set_mem_mode(MemMode.ATOMIC_NONCACHING)
m5.instantiate()
print("instantiated")

if args.device_kind is not None or args.queue_timing:
    for tick in filter(None, args.stats_at.split(",")):
        tick = int(tick)
        event = m5.simulate(tick - m5.curTick())
        assert m5.curTick() == tick
        assert event.getCause() == "simulate() limit reached"
        m5.stats.dump()
        m5.stats.reset()
    event = m5.simulate(4000000000 - m5.curTick())
    print(f"Exiting @ tick {m5.curTick()} because {event.getCause()}")
