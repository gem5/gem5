#!/usr/bin/env python3

# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# (University of Wisconsin-Madison)
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

import argparse
import os

import config_parser


def parse_cur_args():
    argparser = argparse.ArgumentParser(description="SALAM System Builder")
    argparser.add_argument(
        "--bench-path",
        help="Path to the benchmark directory relative to ACC_BENCH_PATH "
        "(e.g. bfs → $ACC_BENCH_PATH/bfs)",
        required=True,
    )
    argparser.add_argument(
        "--sys-name",
        help="Accepted for compatibility with run_system.sh; this option "
        "no longer selects a generated Python configuration file.",
        required=True,
        default=None,
    )
    argparser.add_argument(
        "--config-name",
        help="Name of the configuration file in the root of the bench."
        " Defaults to config.yml",
        required=False,
        default="config.yml",
    )
    argparser.add_argument(
        "--m5-path", help="Path to M5 Directory", required=False, default=None
    )
    argparser.add_argument(
        "--acc-bench-path",
        help="Path to ACC_BENCH_PATH root",
        required=False,
        default=None,
    )
    return argparser.parse_args()


def is_generated_begin(line):
    return line.strip() in (
        "//BEGIN GENERATED CODE",
        "// BEGIN GENERATED CODE",
    )


def is_generated_end(line):
    return line.strip() in ("//END GENERATED CODE", "// END GENERATED CODE")


# Register offsets are part of the accelerator memory map. These records
# become #define lines.
_NONCOHERENT_REGS = (
    ("Flags", 0),
    ("RdAddr", 1),
    ("WrAddr", 9),
    ("CopyLen", 17),
)
_STREAM_REGS = (
    ("Flags", 0),
    ("RdAddr", 4),
    ("WrAddr", 12),
    ("RdFrameSize", 20),
    ("NumRdFrames", 24),
    ("RdFrameBufSize", 25),
    ("WrFrameSize", 26),
    ("NumWrFrames", 30),
    ("WrFrameBufSize", 31),
    ("Stream", 32),
)


def _define(name, address):
    return "#define " + name + " " + hex(address) + "\n"


def _generate_dma(dma):
    lines = []
    if dma.dmaType == "NonCoherent":
        lines.append("// NonCoherentDMA\n")
        layout = _NONCOHERENT_REGS
    elif dma.dmaType == "Stream":
        lines.append("// StreamDMA\n")
        layout = _STREAM_REGS
    else:
        return lines
    prefix = dma.name.upper() + "_"
    for suffix, offset in layout:
        lines.append(_define(prefix + suffix, dma.address + offset))
    if dma.dmaType == "Stream":
        lines.append(_define(prefix + "Status", dma.statusAddress))
    return lines


def _generate_accelerator(acc):
    lines = ["// Accelerator: " + acc.name.upper() + "\n"]
    lines.append(_define(acc.name.upper(), acc.address))
    for var in acc.variables:
        if "Cache" in var.type:
            continue
        lines.append(_define(var.name, var.address))
        if "Stream" in var.type:
            lines.append(_define(var.name + "_Status", var.statusAddress))
    return lines


# Generate the header region for one cluster.
def generate_header_region(cluster):
    lines = [
        "// BEGIN GENERATED CODE\n",
        "// Cluster: " + cluster.name.upper() + "\n",
    ]
    for dma in cluster.dmas:
        lines.extend(_generate_dma(dma))
    for acc in cluster.accs:
        lines.extend(_generate_accelerator(acc))
    lines.append("// END GENERATED CODE\n")
    return lines


def write_generated_header(path, lines):
    with open(path, "w") as header_file:
        header_file.write("".join(lines))


def load_og_header(clusters, working_dir: str):
    # Read in existing header
    header_list = []
    for i in clusters:
        begin = None
        end = None
        try:
            f = open(working_dir + i.name + "_hw_defines.h")
            oldHeader = f.readlines()
            for line_num in range(0, len(oldHeader)):
                if is_generated_begin(oldHeader[line_num]):
                    begin = line_num
                elif is_generated_end(oldHeader[line_num]):
                    end = line_num
            if begin is not None and end is not None and begin <= end:
                del oldHeader[begin : end + 1]
            header_list.append(oldHeader)
        except Exception:
            print("No Header Found")
            emptyList = []
            header_list.append(emptyList)
    return header_list


def gen_header(header_list, clusters, working_dir: str):
    # Write out headers. Pair each preserved header with its cluster so
    # multi-cluster configs do not append every cluster into every file.
    if len(header_list) != len(clusters):
        raise ValueError(
            "Header and cluster counts must match when generating headers."
        )
    for preserved, cluster in zip(header_list, clusters):
        # Preserved lines keep their relative order, including text that
        # originally followed the old generated region. The new region is
        # appended, matching the previous loader.
        region = list(preserved) + generate_header_region(cluster)
        write_generated_header(
            working_dir + cluster.name + "_hw_defines.h", region
        )


def main():

    args = parse_cur_args()
    # This requires M5_PATH to point to your gem5-SALAM directory
    M5_Path = os.getenv("M5_PATH")
    acc_bench_path = os.getenv("ACC_BENCH_PATH")

    if M5_Path is None:
        print("Looking for Path Argument from Command Line")
        if args.m5_path is None:
            raise Exception("Path argument required when M5_PATH not set")
        M5_Path = args.m5_path
        if M5_Path is None:
            raise Exception("M5_PATH Not Found")

    if acc_bench_path is None:
        print("Looking for ACC_BENCH_PATH argument from Command Line")
        if args.acc_bench_path is None:
            raise Exception(
                "ACC_BENCH_PATH not set and --acc-bench-path not provided"
            )
        acc_bench_path = args.acc_bench_path
        if acc_bench_path is None:
            raise Exception("ACC_BENCH_PATH Not Found")

    # --sys-name is retained for run_system.sh compatibility.
    working_dir = acc_bench_path + "/" + args.bench_path + "/"
    max_address = 0x2FFFFFFF  # 0x13FFFFFF
    clusters = config_parser.load_clusters(working_dir, args.config_name)
    # Parse original header for custom code
    header_list = load_og_header(clusters=clusters, working_dir=working_dir)
    # Make the header files with custom code
    gen_header(
        header_list=header_list, clusters=clusters, working_dir=working_dir
    )
    # Warn if the size is greater than allowed
    if clusters[-1].top_address > max_address:
        print("WARNING: Address range is greater than defined for gem5")


if __name__ == "__main__":
    main()
