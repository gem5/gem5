#!/usr/bin/env python3

# Copyright (c) 2026 Rajesh Gangam
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

# Decode a gem5 E-Trace protobuf file and print packets in human-readable
# format.  Follows the same pattern as util/decode_inst_dep_trace.py.

import sys

import protolib

try:
    import etrace_pb2
except ImportError:
    print("Did not find proto definition, attempting to generate")
    from subprocess import call

    error = call(
        [
            "protoc",
            "--python_out=util",
            "--proto_path=src/proto",
            "src/proto/etrace.proto",
        ]
    )
    if not error:
        import etrace_pb2

        print("Generated proto definitions for E-Trace")
    else:
        print("Failed to import proto definitions")
        exit(-1)

def to_signed64(val):
    if val >= (1 << 63):
        return val - (1 << 64)
    return val


FORMAT_NAMES = {
    etrace_pb2.ETracePacket.BRANCH_MAP: "BRANCH_MAP",
    etrace_pb2.ETracePacket.BRANCH_MAP_ADDR: "BRANCH_MAP_ADDR",
    etrace_pb2.ETracePacket.ADDR_ONLY: "ADDR_ONLY",
    etrace_pb2.ETracePacket.SYNC: "SYNC",
}

SUBFORMAT_NAMES = {
    etrace_pb2.ETracePacket.START: "START",
    etrace_pb2.ETracePacket.TRAP: "TRAP",
    etrace_pb2.ETracePacket.CONTEXT: "CONTEXT",
    etrace_pb2.ETracePacket.SUPPORT: "SUPPORT",
}

PRIV_NAMES = {0: "U", 1: "S", 3: "M"}


def format_branch_map(bmap, count):
    bits = []
    for i in range(count):
        bits.append("N" if (bmap >> i) & 1 else "T")
    return "".join(bits)


def main():
    if len(sys.argv) < 2:
        print("Usage:", sys.argv[0], "<protobuf input> [--stats]")
        exit(-1)

    show_stats = "--stats" in sys.argv

    proto_in = protolib.openFileRd(sys.argv[1])

    magic_number = proto_in.read(4).decode()
    if magic_number != "gem5":
        print("Unrecognized file")
        exit(-1)

    header = etrace_pb2.ETraceHeader()
    protolib.decodeMessage(proto_in, header)

    print(f"Object: {header.obj_id}")
    print(f"Tick freq: {header.tick_freq}")
    print(f"Arch width: {header.arch_width}")
    print()

    num_packets = 0
    num_sync = 0
    num_trap = 0
    num_branch_map = 0
    total_branches = 0
    reconstructed_addr = 0

    packet = etrace_pb2.ETracePacket()
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1
        fmt = FORMAT_NAMES.get(packet.format, str(packet.format))

        if not show_stats:
            line = f"[{num_packets:6d}] tick={packet.tick:12d} fmt={fmt:<16s}"

            if packet.format == etrace_pb2.ETracePacket.SYNC:
                subfmt = SUBFORMAT_NAMES.get(
                    packet.subformat, str(packet.subformat)
                )
                priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                reconstructed_addr = packet.address
                line += f" sub={subfmt:<8s} addr=0x{packet.address:016x}"
                line += f" priv={priv}"
                if packet.subformat == etrace_pb2.ETracePacket.TRAP:
                    num_trap += 1
                    line += f" cause={packet.cause}"
                    line += f" tval=0x{packet.tval:x}"
                    line += f" int={packet.interrupt}"
                else:
                    num_sync += 1
                    if packet.branch_count > 0:
                        bm = format_branch_map(
                            packet.branch_map, packet.branch_count
                        )
                        line += f" branch={bm}"

            elif packet.format == etrace_pb2.ETracePacket.BRANCH_MAP_ADDR:
                num_branch_map += 1
                delta = to_signed64(packet.address)
                reconstructed_addr += delta
                reconstructed_addr &= (1 << 64) - 1
                bm = format_branch_map(packet.branch_map, packet.branch_count)
                total_branches += packet.branch_count
                line += (
                    f" addr=0x{reconstructed_addr:016x}"
                    f" (delta={delta:+d})"
                    f" branches={bm}"
                )

            elif packet.format == etrace_pb2.ETracePacket.BRANCH_MAP:
                num_branch_map += 1
                bm = format_branch_map(packet.branch_map, packet.branch_count)
                total_branches += packet.branch_count
                line += f" branches={bm}"

            elif packet.format == etrace_pb2.ETracePacket.ADDR_ONLY:
                delta = to_signed64(packet.address)
                reconstructed_addr += delta
                reconstructed_addr &= (1 << 64) - 1
                line += f" addr=0x{reconstructed_addr:016x}"

            print(line)
        else:
            if packet.format == etrace_pb2.ETracePacket.SYNC:
                if packet.subformat == etrace_pb2.ETracePacket.TRAP:
                    num_trap += 1
                else:
                    num_sync += 1
            else:
                num_branch_map += 1
                total_branches += packet.branch_count

    print()
    print("--- Statistics ---")
    print(f"Total packets:      {num_packets}")
    print(f"Sync packets:       {num_sync}")
    print(f"Trap packets:       {num_trap}")
    print(f"Branch map packets: {num_branch_map}")
    print(f"Total branches:     {total_branches}")

    proto_in.close()


if __name__ == "__main__":
    main()
