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

DATA_FORMAT_NAMES = {
    etrace_pb2.ETraceDataPacket.LOAD_ADDR_DATA: "LOAD_ADDR_DATA",
    etrace_pb2.ETraceDataPacket.STORE_ADDR_DATA: "STORE_ADDR_DATA",
    etrace_pb2.ETraceDataPacket.LOAD_ADDR_ONLY: "LOAD_ADDR_ONLY",
    etrace_pb2.ETraceDataPacket.STORE_ADDR_ONLY: "STORE_ADDR_ONLY",
    etrace_pb2.ETraceDataPacket.LOAD_DATA_ONLY: "LOAD_DATA_ONLY",
    etrace_pb2.ETraceDataPacket.STORE_DATA_ONLY: "STORE_DATA_ONLY",
    etrace_pb2.ETraceDataPacket.ATOMIC: "ATOMIC",
}

ATOMIC_SUBTYPE_NAMES = {
    etrace_pb2.ETraceDataPacket.SWAP: "SWAP",
    etrace_pb2.ETraceDataPacket.ADD: "ADD",
    etrace_pb2.ETraceDataPacket.AND: "AND",
    etrace_pb2.ETraceDataPacket.OR: "OR",
    etrace_pb2.ETraceDataPacket.XOR: "XOR",
    etrace_pb2.ETraceDataPacket.MAX: "MAX",
    etrace_pb2.ETraceDataPacket.MIN: "MIN",
    etrace_pb2.ETraceDataPacket.MAXU: "MAXU",
    etrace_pb2.ETraceDataPacket.MINU: "MINU",
    etrace_pb2.ETraceDataPacket.LR: "LR",
    etrace_pb2.ETraceDataPacket.SC: "SC",
}


def format_branch_map(bmap, count):
    bits = []
    for i in range(count):
        bits.append("N" if (bmap >> i) & 1 else "T")
    return "".join(bits)


def format_ioptions(ioptions):
    flags = []
    if ioptions & (1 << 0):
        flags.append("impl_ret")
    if ioptions & (1 << 1):
        flags.append("bpred")
    if ioptions & (1 << 2):
        flags.append("jtc")
    if ioptions & (1 << 3):
        flags.append("sijump")
    if ioptions & (1 << 4):
        flags.append("impl_exc")
    return "|".join(flags) if flags else "none"


def format_data_bytes(data):
    return " ".join(f"{b:02x}" for b in data)


def decode_instruction_trace(filename, show_stats):
    proto_in = protolib.openFileRd(filename)

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
    num_addr_only = 0
    num_support = 0
    num_context = 0
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

                if packet.subformat == etrace_pb2.ETracePacket.TRAP:
                    num_trap += 1
                    reconstructed_addr = packet.address
                    priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                    line += f" sub={subfmt:<8s}"
                    if packet.thaddr:
                        line += f" addr=0x{packet.address:016x}"
                    else:
                        line += " addr=<inferable>"
                    line += f" priv={priv}"
                    line += f" cause={packet.cause}"
                    line += f" tval=0x{packet.tval:x}"
                    line += f" int={packet.interrupt}"

                elif packet.subformat == etrace_pb2.ETracePacket.SUPPORT:
                    num_support += 1
                    line += f" sub={subfmt:<8s}"
                    line += f" ienable={packet.ienable}"
                    line += f" qual={packet.qual_status}"
                    line += f" iopts={format_ioptions(packet.ioptions)}"
                    if packet.denable:
                        line += f" denable={packet.denable}"
                        line += f" dopts={packet.doptions}"

                elif packet.subformat == etrace_pb2.ETracePacket.CONTEXT:
                    num_context += 1
                    line += f" sub={subfmt:<8s}"
                    line += f" context=0x{packet.context:x}"

                else:
                    num_sync += 1
                    priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                    reconstructed_addr = packet.address
                    line += f" sub={subfmt:<8s} addr=0x{packet.address:016x}"
                    line += f" priv={priv}"
                    if packet.branch_count > 0:
                        bm = format_branch_map(
                            packet.branch_map, packet.branch_count
                        )
                        line += f" branch={bm}"

            elif packet.format == etrace_pb2.ETracePacket.BRANCH_MAP_ADDR:
                num_branch_map += 1
                if packet.HasField("saddress"):
                    delta = packet.saddress
                else:
                    delta = packet.address
                reconstructed_addr += delta
                reconstructed_addr &= (1 << 64) - 1
                bm = format_branch_map(
                    packet.branch_map, packet.branch_count
                )
                total_branches += packet.branch_count
                line += (
                    f" addr=0x{reconstructed_addr:016x}"
                    f" (delta={delta:+d})"
                    f" branches={bm}"
                )
                if packet.HasField("jtc_index"):
                    line += f" jtc={packet.jtc_index}"
                if packet.branch_pred_count > 0:
                    line += f" bpred={packet.branch_pred_count}"

            elif packet.format == etrace_pb2.ETracePacket.BRANCH_MAP:
                num_branch_map += 1
                bm = format_branch_map(
                    packet.branch_map, packet.branch_count
                )
                total_branches += packet.branch_count
                line += f" branches={bm}"
                if packet.branch_pred_count > 0:
                    line += f" bpred={packet.branch_pred_count}"

            elif packet.format == etrace_pb2.ETracePacket.ADDR_ONLY:
                num_addr_only += 1
                if packet.HasField("saddress"):
                    delta = packet.saddress
                else:
                    delta = packet.address
                reconstructed_addr += delta
                reconstructed_addr &= (1 << 64) - 1
                line += f" addr=0x{reconstructed_addr:016x}"
                if packet.notify:
                    line += " NOTIFY"
                if packet.updiscon:
                    line += " UPDISCON"
                if packet.irreport:
                    line += f" IRREPORT(depth={packet.irdepth})"

            print(line)
        else:
            if packet.format == etrace_pb2.ETracePacket.SYNC:
                if packet.subformat == etrace_pb2.ETracePacket.TRAP:
                    num_trap += 1
                elif packet.subformat == etrace_pb2.ETracePacket.SUPPORT:
                    num_support += 1
                elif packet.subformat == etrace_pb2.ETracePacket.CONTEXT:
                    num_context += 1
                else:
                    num_sync += 1
            elif packet.format == etrace_pb2.ETracePacket.ADDR_ONLY:
                num_addr_only += 1
            else:
                num_branch_map += 1
                total_branches += packet.branch_count

    print()
    print("--- Statistics ---")
    print(f"Total packets:      {num_packets}")
    print(f"Sync packets:       {num_sync}")
    print(f"Trap packets:       {num_trap}")
    print(f"Support packets:    {num_support}")
    print(f"Context packets:    {num_context}")
    print(f"Branch map packets: {num_branch_map}")
    print(f"Addr-only packets:  {num_addr_only}")
    print(f"Total branches:     {total_branches}")

    proto_in.close()


def decode_data_trace(filename, show_stats):
    proto_in = protolib.openFileRd(filename)

    magic_number = proto_in.read(4).decode()
    if magic_number != "gem5":
        print("Unrecognized file")
        exit(-1)

    header = etrace_pb2.ETraceHeader()
    protolib.decodeMessage(proto_in, header)

    print(f"Data Trace — Object: {header.obj_id}")
    print()

    num_packets = 0
    num_loads = 0
    num_stores = 0
    num_atomics = 0
    reconstructed_addr = 0

    packet = etrace_pb2.ETraceDataPacket()
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1
        fmt = DATA_FORMAT_NAMES.get(packet.format, str(packet.format))

        is_load = packet.format in (
            etrace_pb2.ETraceDataPacket.LOAD_ADDR_DATA,
            etrace_pb2.ETraceDataPacket.LOAD_ADDR_ONLY,
            etrace_pb2.ETraceDataPacket.LOAD_DATA_ONLY,
        )
        is_store = packet.format in (
            etrace_pb2.ETraceDataPacket.STORE_ADDR_DATA,
            etrace_pb2.ETraceDataPacket.STORE_ADDR_ONLY,
            etrace_pb2.ETraceDataPacket.STORE_DATA_ONLY,
        )
        is_atomic = (
            packet.format == etrace_pb2.ETraceDataPacket.ATOMIC
        )

        if is_load:
            num_loads += 1
        elif is_store:
            num_stores += 1
        elif is_atomic:
            num_atomics += 1

        if not show_stats:
            line = f"[{num_packets:6d}] tick={packet.tick:12d} fmt={fmt:<18s}"
            line += f" size={packet.size}"

            has_addr = packet.format in (
                etrace_pb2.ETraceDataPacket.LOAD_ADDR_DATA,
                etrace_pb2.ETraceDataPacket.STORE_ADDR_DATA,
                etrace_pb2.ETraceDataPacket.LOAD_ADDR_ONLY,
                etrace_pb2.ETraceDataPacket.STORE_ADDR_ONLY,
                etrace_pb2.ETraceDataPacket.ATOMIC,
            )
            if has_addr:
                reconstructed_addr += packet.address
                reconstructed_addr &= (1 << 64) - 1
                line += f" addr=0x{reconstructed_addr:016x}"

            if packet.data:
                line += f" data=[{format_data_bytes(packet.data)}]"

            if is_atomic:
                sub = ATOMIC_SUBTYPE_NAMES.get(
                    packet.atomic_subtype, str(packet.atomic_subtype)
                )
                line += f" op={sub}"
                if packet.operand:
                    line += (
                        f" operand=[{format_data_bytes(packet.operand)}]"
                    )

            print(line)

    print()
    print("--- Data Trace Statistics ---")
    print(f"Total packets: {num_packets}")
    print(f"Loads:         {num_loads}")
    print(f"Stores:        {num_stores}")
    print(f"Atomics:       {num_atomics}")

    proto_in.close()


def main():
    if len(sys.argv) < 2:
        print(
            "Usage:",
            sys.argv[0],
            "<protobuf input> [--stats] [--data-trace]",
        )
        exit(-1)

    show_stats = "--stats" in sys.argv
    data_trace = "--data-trace" in sys.argv
    filename = sys.argv[1]

    if data_trace:
        decode_data_trace(filename, show_stats)
    else:
        decode_instruction_trace(filename, show_stats)


if __name__ == "__main__":
    main()
