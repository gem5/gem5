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

# Decode a gem5 E-Trace v2.0 protobuf file and print packets in
# human-readable form. Understands the spec-conformant packet layout:
# discovery parameters carried in the header, Format 0/1/2/3 with
# spec numeric codes, and the XOR-chain disambiguation bits.

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
    etrace_pb2.ETracePacket.FORMAT_0: "F0",
    etrace_pb2.ETracePacket.FORMAT_1: "F1",
    etrace_pb2.ETracePacket.FORMAT_2: "F2",
    etrace_pb2.ETracePacket.FORMAT_3: "F3",
}

SUBFORMAT_NAMES = {
    etrace_pb2.ETracePacket.START: "START",
    etrace_pb2.ETracePacket.TRAP: "TRAP",
    etrace_pb2.ETracePacket.CONTEXT: "CONTEXT",
    etrace_pb2.ETracePacket.SUPPORT: "SUPPORT",
}

F0_SUBFORMAT_NAMES = {
    etrace_pb2.ETracePacket.F0_BRANCH_COUNT: "F0.PBC",
    etrace_pb2.ETracePacket.F0_JTC: "F0.JTC",
}

QUAL_STATUS_NAMES = {
    etrace_pb2.ETracePacket.QS_NO_CHANGE: "no_change",
    etrace_pb2.ETracePacket.QS_ENDED_REP: "ended_rep",
    etrace_pb2.ETracePacket.QS_TRACE_LOST: "trace_lost",
    etrace_pb2.ETracePacket.QS_ENDED_NTR: "ended_ntr",
}

PRIV_NAMES = {0: "U", 1: "S", 3: "M"}

DATA_FORMAT_NAMES = {
    etrace_pb2.ETraceDataPacket.LOAD_ALIGNED: "LOAD",
    etrace_pb2.ETraceDataPacket.LOAD_UNALIGNED: "LOAD_UA",
    etrace_pb2.ETraceDataPacket.STORE_ALIGNED: "STORE",
    etrace_pb2.ETraceDataPacket.STORE_UNALIGNED: "STORE_UA",
    etrace_pb2.ETraceDataPacket.LOAD_DATA_RESP: "LOAD_RESP",
    etrace_pb2.ETraceDataPacket.CSR: "CSR",
    etrace_pb2.ETraceDataPacket.ATOMIC: "ATOMIC",
}

ATOMIC_SUBTYPE_NAMES = {
    etrace_pb2.ETraceDataPacket.AMOSWAP: "SWAP",
    etrace_pb2.ETraceDataPacket.AMOADD: "ADD",
    etrace_pb2.ETraceDataPacket.AMOAND: "AND",
    etrace_pb2.ETraceDataPacket.AMOOR: "OR",
    etrace_pb2.ETraceDataPacket.AMOXOR: "XOR",
    etrace_pb2.ETraceDataPacket.AMOMAX: "MAX",
    etrace_pb2.ETraceDataPacket.AMOMIN: "MIN",
    etrace_pb2.ETraceDataPacket.AMO_RESERVED: "RESERVED",
}


def format_branch_map(bmap, count):
    """Render a branch map as a taken/not-taken string.

    Per spec, bit 0 is the oldest branch; 0 = taken, 1 = not-taken.
    """
    bits = []
    for i in range(count):
        bits.append("N" if (bmap >> i) & 1 else "T")
    return "".join(bits)


def format_ioptions(ioptions):
    """Decode ioptions bits — layout is fixed by the encoder (see header)."""
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
    if ioptions & (1 << 5):
        flags.append("full_addr")
    return "|".join(flags) if flags else "none"


def format_doptions(doptions):
    flags = []
    if doptions & (1 << 0):
        flags.append("no_data")
    if doptions & (1 << 1):
        flags.append("no_addr")
    if doptions & (1 << 2):
        flags.append("full_data")
    return "|".join(flags) if flags else "none"


def format_data_bytes(data):
    return " ".join(f"{b:02x}" for b in data)


def branches_field_to_count(branches_field):
    """The on-wire `branches` field is the EXACT count of valid
    branch_map bits (spec: "branches=12" means branch_map is 15 bits
    long with the 12 LSBs valid) -- not a tapered width code. 0 means
    the no-address 31-bit form.
    """
    if branches_field == 0:
        return 31, True  # no-address form
    return branches_field, False


def decode_instruction_trace(filename, show_stats, verify_updiscon=False):
    proto_in = protolib.openFileRd(filename)

    magic_number = proto_in.read(4).decode()
    if magic_number != "gem5":
        print("Unrecognized file")
        exit(-1)

    header = etrace_pb2.ETraceHeader()
    protolib.decodeMessage(proto_in, header)

    print(f"Object: {header.obj_id}")
    print(f"Version: {header.ver}")
    print(f"Tick freq: {header.tick_freq}")
    print(f"Arch width: {header.arch_width}")
    print("Discovery parameters:")
    print(f"  iaddress_width_p     = {header.iaddress_width_p}")
    print(f"  iaddress_lsb_p       = {header.iaddress_lsb_p}")
    print(f"  privilege_width_p    = {header.privilege_width_p}")
    print(f"  ecause_width_p       = {header.ecause_width_p}")
    print(f"  context_width_p      = {header.context_width_p}")
    print(f"  time_width_p         = {header.time_width_p}")
    print(f"  f0s_width_p          = {header.f0s_width_p}")
    print(f"  cache_size_p         = {header.cache_size_p}")
    print(f"  call_counter_size_p  = {header.call_counter_size_p}")
    print(f"  bpred_size_p         = {header.bpred_size_p}")
    print(
        f"  ioptions_bits (adv)  = 0x{header.ioptions_bits:x} "
        f"({format_ioptions(header.ioptions_bits)})"
    )
    print(
        f"  doptions_bits (adv)  = 0x{header.doptions_bits:x} "
        f"({format_doptions(header.doptions_bits)})"
    )
    print()

    num_packets = 0
    num_sync_start = 0
    num_trap = 0
    num_branch_map = 0
    num_addr_only = 0
    num_support = 0
    num_context = 0
    num_f0_pbc = 0
    num_f0_jtc = 0
    total_branches = 0
    reconstructed_addr = 0
    prev_addr_msb = 0  # for XOR-chain reconstruction
    jtc_mirror = {}  # decoder mirror of the encoder's JTC

    packet = etrace_pb2.ETracePacket()
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1
        fmt = FORMAT_NAMES.get(packet.format, str(packet.format))

        if show_stats:
            # Count-only pass — advance state where the wire semantics
            # require it, but skip formatting.
            _classify_for_stats(
                packet,
                counters := {
                    "sync": num_sync_start,
                    "trap": num_trap,
                    "branch_map": num_branch_map,
                    "addr_only": num_addr_only,
                    "support": num_support,
                    "context": num_context,
                    "f0_pbc": num_f0_pbc,
                    "f0_jtc": num_f0_jtc,
                    "branches": total_branches,
                },
            )
            num_sync_start = counters["sync"]
            num_trap = counters["trap"]
            num_branch_map = counters["branch_map"]
            num_addr_only = counters["addr_only"]
            num_support = counters["support"]
            num_context = counters["context"]
            num_f0_pbc = counters["f0_pbc"]
            num_f0_jtc = counters["f0_jtc"]
            total_branches = counters["branches"]
            continue

        line = f"[{num_packets:6d}] tick={packet.tick:12d} fmt={fmt:<4s}"

        # Shift factor for wire address recovery (spec iaddress_lsb_p).
        lsb = header.iaddress_lsb_p

        if packet.format == etrace_pb2.ETracePacket.FORMAT_3:
            subfmt = SUBFORMAT_NAMES.get(
                packet.subformat, str(packet.subformat)
            )
            line += f" sub={subfmt:<8s}"

            if packet.subformat == etrace_pb2.ETracePacket.TRAP:
                num_trap += 1
                reconstructed_addr = packet.address << lsb
                priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                if packet.HasField("address"):
                    line += f" addr=0x{reconstructed_addr:016x}"
                else:
                    line += " addr=<inferable>"
                line += f" priv={priv} branch={int(packet.branch)}"
                line += f" cause={packet.cause}"
                if not packet.interrupt:
                    line += f" tval=0x{packet.tval:x}"
                line += f" int={packet.interrupt}"
                if packet.HasField("time"):
                    line += f" time={packet.time}"
                # Reset JTC / bpred mirror on any sync.
                jtc_mirror.clear()

            elif packet.subformat == etrace_pb2.ETracePacket.SUPPORT:
                num_support += 1
                qs = QUAL_STATUS_NAMES.get(
                    packet.qual_status, str(packet.qual_status)
                )
                line += f" ienable={int(packet.ienable)}"
                line += f" qual={qs}"
                line += f" iopts={format_ioptions(packet.ioptions)}"
                line += f" denable={int(packet.denable)}"
                line += f" dloss={int(packet.dloss)}"
                if packet.doptions:
                    line += f" dopts={format_doptions(packet.doptions)}"

            elif packet.subformat == etrace_pb2.ETracePacket.CONTEXT:
                num_context += 1
                priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                line += f" priv={priv}"
                line += f" context=0x{packet.context:x}"
                if packet.HasField("time"):
                    line += f" time={packet.time}"

            else:  # START
                num_sync_start += 1
                priv = PRIV_NAMES.get(packet.priv, str(packet.priv))
                reconstructed_addr = packet.address << lsb
                line += (
                    f" addr=0x{reconstructed_addr:016x} priv={priv}"
                    f" branch={int(packet.branch)}"
                )
                if packet.HasField("time"):
                    line += f" time={packet.time}"
                if packet.HasField("context"):
                    line += f" context=0x{packet.context:x}"
                jtc_mirror.clear()

            prev_addr_msb = (reconstructed_addr >> 63) & 1

        elif packet.format == etrace_pb2.ETracePacket.FORMAT_1:
            num_branch_map += 1
            branches_field = packet.branches
            count, no_addr_form = branches_field_to_count(branches_field)
            if packet.HasField("saddress"):
                # Delta is in wire units (shifted-out iaddress_lsb_p);
                # multiply back to byte units before accumulating.
                delta = packet.saddress << lsb
                reconstructed_addr = (reconstructed_addr + delta) & (
                    (1 << 64) - 1
                )
                # Mirror the encoder's JTC: whenever we see an
                # address-carrying Format 1 or Format 2 packet, its
                # target could be JTC-cached later.
                idx = (
                    (reconstructed_addr >> 1)
                    & ((1 << header.cache_size_p) - 1)
                    if header.cache_size_p > 0
                    else None
                )
                if idx is not None:
                    jtc_mirror[idx] = reconstructed_addr
                bm = format_branch_map(packet.branch_map, count)
                total_branches += count
                line += (
                    f" addr=0x{reconstructed_addr:016x}"
                    f" (delta={delta:+d})"
                    f" branches={bm}"
                )
                if verify_updiscon:
                    addr_msb = (reconstructed_addr >> 63) & 1
                    expected_notify = addr_msb
                    if packet.notify != expected_notify:
                        line += " [!notify-chain]"
                    prev_addr_msb = addr_msb
                if packet.irreport != packet.updiscon:
                    line += f" IRREPORT(depth={packet.irdepth})"
                if packet.updiscon != packet.notify:
                    line += " UPDISCON"
            else:
                # No-address Format 1 (31-bit map).
                bm = format_branch_map(packet.branch_map, 31)
                total_branches += 31
                line += f" branches={bm} [31-bit]"

            if packet.branch_count > 0:
                line += f" bpred+={packet.branch_count}"

        elif packet.format == etrace_pb2.ETracePacket.FORMAT_2:
            num_addr_only += 1
            delta = packet.saddress << lsb
            reconstructed_addr = (reconstructed_addr + delta) & ((1 << 64) - 1)
            if header.cache_size_p > 0:
                idx = (reconstructed_addr >> 1) & (
                    (1 << header.cache_size_p) - 1
                )
                jtc_mirror[idx] = reconstructed_addr
            line += f" addr=0x{reconstructed_addr:016x} (delta={delta:+d})"
            if packet.irreport != packet.updiscon:
                line += f" IRREPORT(depth={packet.irdepth})"
            if packet.updiscon != packet.notify:
                line += " UPDISCON"

        elif packet.format == etrace_pb2.ETracePacket.FORMAT_0:
            sub = F0_SUBFORMAT_NAMES.get(
                packet.f0_subformat, str(packet.f0_subformat)
            )
            line += f" sub={sub:<7s}"
            if packet.f0_subformat == etrace_pb2.ETracePacket.F0_BRANCH_COUNT:
                num_f0_pbc += 1
                # Value on wire is (pbc - 31).
                pbc = packet.branch_count + 31
                line += f" pbc={pbc}"
                if packet.branch_fmt & 0x2:
                    if packet.HasField("saddress"):
                        delta = packet.saddress
                        reconstructed_addr = (reconstructed_addr + delta) & (
                            (1 << 64) - 1
                        )
                        line += (
                            f" addr=0x{reconstructed_addr:016x}"
                            f" (delta={delta:+d})"
                        )
                    # branch_fmt 10 (0x2): address points to a branch
                    # predicted correctly (or a non-branch instruction).
                    # branch_fmt 11 (0x3): address points to a branch
                    # that failed prediction. Only the latter is a
                    # misprediction.
                    line += (
                        " (mispred)"
                        if packet.branch_fmt == 0x3
                        else " (predicted)"
                    )
            elif packet.f0_subformat == etrace_pb2.ETracePacket.F0_JTC:
                num_f0_jtc += 1
                idx = packet.jtc_index
                line += f" idx={idx}"
                # Look up in mirror; if unseen, note it.
                if idx in jtc_mirror:
                    tgt = jtc_mirror[idx]
                    reconstructed_addr = tgt
                    line += f" -> 0x{tgt:016x}"
                else:
                    line += " [uncached-in-mirror]"
                if packet.branches > 0:
                    count, _ = branches_field_to_count(packet.branches)
                    bm = format_branch_map(packet.branch_map, count)
                    total_branches += count
                    line += f" branches={bm}"
                if packet.irreport:
                    line += f" IRREPORT(depth={packet.irdepth})"

        print(line)

    print()
    print("--- Statistics ---")
    print(f"Total packets:      {num_packets}")
    print(f"Sync (start):       {num_sync_start}")
    print(f"Trap packets:       {num_trap}")
    print(f"Support packets:    {num_support}")
    print(f"Context packets:    {num_context}")
    print(f"Branch map packets: {num_branch_map}")
    print(f"Addr-only packets:  {num_addr_only}")
    print(f"F0 PBC packets:     {num_f0_pbc}")
    print(f"F0 JTC packets:     {num_f0_jtc}")
    print(f"Total branches:     {total_branches}")

    proto_in.close()


def _classify_for_stats(packet, counters):
    """Increment stat counters based on packet type."""
    if packet.format == etrace_pb2.ETracePacket.FORMAT_3:
        if packet.subformat == etrace_pb2.ETracePacket.TRAP:
            counters["trap"] += 1
        elif packet.subformat == etrace_pb2.ETracePacket.SUPPORT:
            counters["support"] += 1
        elif packet.subformat == etrace_pb2.ETracePacket.CONTEXT:
            counters["context"] += 1
        else:
            counters["sync"] += 1
    elif packet.format == etrace_pb2.ETracePacket.FORMAT_1:
        counters["branch_map"] += 1
        if packet.HasField("saddress"):
            count, _ = branches_field_to_count(packet.branches)
            counters["branches"] += count
        else:
            counters["branches"] += 31
    elif packet.format == etrace_pb2.ETracePacket.FORMAT_2:
        counters["addr_only"] += 1
    elif packet.format == etrace_pb2.ETracePacket.FORMAT_0:
        if packet.f0_subformat == etrace_pb2.ETracePacket.F0_BRANCH_COUNT:
            counters["f0_pbc"] += 1
        else:
            counters["f0_jtc"] += 1


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
    num_csr = 0
    # Per-size (log2 bytes) baseline for delta reconstruction.
    baseline_by_size = {}

    packet = etrace_pb2.ETraceDataPacket()
    while protolib.decodeMessage(proto_in, packet):
        num_packets += 1
        fmt = DATA_FORMAT_NAMES.get(packet.format, str(packet.format))
        log_size = packet.size
        bytes_ = 1 << log_size

        is_load = packet.format in (
            etrace_pb2.ETraceDataPacket.LOAD_ALIGNED,
            etrace_pb2.ETraceDataPacket.LOAD_UNALIGNED,
            etrace_pb2.ETraceDataPacket.LOAD_DATA_RESP,
        )
        is_store = packet.format in (
            etrace_pb2.ETraceDataPacket.STORE_ALIGNED,
            etrace_pb2.ETraceDataPacket.STORE_UNALIGNED,
        )
        is_atomic = packet.format == etrace_pb2.ETraceDataPacket.ATOMIC
        is_csr = packet.format == etrace_pb2.ETraceDataPacket.CSR

        if is_load:
            num_loads += 1
        elif is_store:
            num_stores += 1
        elif is_atomic:
            num_atomics += 1
        elif is_csr:
            num_csr += 1

        if not show_stats:
            line = (
                f"[{num_packets:6d}] tick={packet.tick:12d} fmt={fmt:<9s}"
                f" size={bytes_}B(log{log_size})"
                f" diff={packet.diff}"
            )

            if packet.HasField("address"):
                # `address` is sint64 on the wire; mask to 64-bit
                # unsigned so kernel/negative addresses render right.
                # diff bits: 00 full addr, else delta.
                if packet.diff == 0:
                    addr = packet.address & ((1 << 64) - 1)
                else:
                    addr = (
                        baseline_by_size.get(log_size, 0) + packet.address
                    ) & ((1 << 64) - 1)
                baseline_by_size[log_size] = addr
                line += f" addr=0x{addr:016x}"

            if packet.data:
                line += f" data=[{format_data_bytes(packet.data)}]"

            if is_atomic:
                sub = ATOMIC_SUBTYPE_NAMES.get(
                    packet.atomic_subtype, str(packet.atomic_subtype)
                )
                line += f" op={sub}"
                if packet.operand:
                    line += f" operand=[{format_data_bytes(packet.operand)}]"
            if packet.is_lr:
                line += " [LR]"
            if packet.is_sc:
                line += " [SC]"

            print(line)

    print()
    print("--- Data Trace Statistics ---")
    print(f"Total packets: {num_packets}")
    print(f"Loads:         {num_loads}")
    print(f"Stores:        {num_stores}")
    print(f"Atomics:       {num_atomics}")
    print(f"CSR accesses:  {num_csr}")

    proto_in.close()


def main():
    if len(sys.argv) < 2:
        print(
            "Usage:",
            sys.argv[0],
            "<protobuf input> [--stats] [--data-trace]",
            "[--verify-updiscon]",
        )
        exit(-1)

    show_stats = "--stats" in sys.argv
    data_trace = "--data-trace" in sys.argv
    verify_updiscon = "--verify-updiscon" in sys.argv
    filename = sys.argv[1]

    if data_trace:
        decode_data_trace(filename, show_stats)
    else:
        decode_instruction_trace(
            filename, show_stats, verify_updiscon=verify_updiscon
        )


if __name__ == "__main__":
    main()
