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

"""Check SALAM profile parameters and constructed workload devices."""

import argparse
import json

import m5
from m5.objects import (
    CycleCounts,
    IOXBar,
    NoncoherentDma,
    Root,
    StreamDma,
    System,
    SystemXBar,
    VExpress_GEM5_V1,
)

from gem5.salam.cluster import (
    AcceleratorCache,
    build_clusters,
)


def build_system(root, workload, cache_line_size=None):
    parameters = {}
    if cache_line_size is not None:
        parameters["cache_line_size"] = cache_line_size
    root.system = system = System(**parameters)
    system.iobus = IOXBar()
    system.membus = SystemXBar()
    system.realview = VExpress_GEM5_V1()
    build_clusters(system, workload)
    return system


def check_dma(system, records):
    for record in records:
        dma = getattr(system.clstr, record["Name"])
        stream = record["Type"] == "Stream"
        assert isinstance(dma, StreamDma if stream else NoncoherentDma)
        capacity = record.get("BufferSize", 1024)
        request = record.get("MaxReqSize", 128)
        assert int(dma.max_req_size.unproxy(dma)) == request
        assert int(dma.max_pending) == 8
        if stream:
            assert int(dma.read_buffer_size) == capacity
            assert int(dma.write_buffer_size) == capacity
            assert int(dma.stream_size) == 8
            assert int(dma.rd_int) == record.get("ReadInt", 210)
            assert int(dma.wr_int) == record.get("WriteInt", 211)
        else:
            assert int(dma.buffer_size) == capacity
            assert int(dma.int_num) == record.get("InterruptNum", 200)
        print(record["Name"], capacity, request, "checked")


def check_cache(system, cases):
    for index, (value, expected) in enumerate(cases):
        cache = getattr(system.clstr, f"cache_{index}")
        assert isinstance(cache, AcceleratorCache), (value, type(cache))
        assert int(cache.size) == expected, (value, cache.size, expected)
        print(repr(value), expected, "checked")


def check_connections(system):
    c = system.clstr
    for obj in (c.ndma, c.same, c.split, c.a, c.b, c.c):
        assert obj.pio.peer.simobj is c.local_bus
    for obj in (c.a, c.b, c.c):
        assert obj.local[0].peer.simobj is c.local_bus
    assert c.same.stream_in.peer.simobj is c.a
    assert c.same.stream_out.peer.simobj is c.a
    assert len(c.a.stream) == 2
    assert c.split.stream_in.peer.simobj is c.b
    assert c.split.stream_out.peer.simobj is c.c
    assert c.flow.stream_in.peer.simobj is c.b
    assert c.flow.stream_out.peer.simobj is c.c
    for clock in (c.compute_clk_domain, c.fabric_clk_domain):
        assert abs(clock.clock[0].value - 1e-8) < 1e-20
    print("SALAM connection configuration checked")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        required=True,
        choices=(
            "memory-latency",
            "control-latency",
            "dma",
            "cache",
            "connections",
        ),
    )
    parser.add_argument(
        "--instruction", choices=("br", "ret", "switch_inst", "call")
    )
    parser.add_argument("--cycles", type=int)
    parser.add_argument("--workload")
    parser.add_argument("--expected")
    args = parser.parse_args()
    if args.case == "control-latency":
        if args.instruction is None or args.cycles is None:
            parser.error("control-latency requires --instruction and --cycles")
    elif args.case in ("dma", "cache", "connections"):
        if args.workload is None:
            parser.error(f"{args.case} requires --workload")
        if args.case != "connections" and args.expected is None:
            parser.error(f"{args.case} requires --expected")

    root = Root(full_system=False)
    if args.case == "memory-latency":
        root.cycle_counts = CycleCounts(load=1)
        m5.instantiate()
    elif args.case == "control-latency":
        root.cycle_counts = CycleCounts(**{args.instruction: args.cycles})
        m5.instantiate()
        print("CycleCounts instantiated")
    else:
        system = build_system(
            root, args.workload, None if args.case == "cache" else 128
        )
        if args.case == "connections":
            check_connections(system)
        else:
            with open(args.expected) as stream:
                expected = json.load(stream)
            if args.case == "dma":
                check_dma(system, expected)
            else:
                check_cache(system, expected)


if __name__ == "__m5_main__":
    main()
