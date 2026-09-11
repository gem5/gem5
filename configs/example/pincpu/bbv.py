# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
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
# Copyright (c) 2026 The Board of Trustees of the Leland Stanford
# Junior University
# All rights reserved.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
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

# Run a SE workload under PinCPU, collecting a basic block vector (BBV)
# profile.

import argparse
import collections
import json
import os
import sys
import types

from m5.util import addToPath

addToPath("../../")

from common import (
    CacheConfig,
    CpuConfig,
    MemConfig,
    ObjectList,
)
from common.Caches import *
from common.FileSystemConfig import config_filesystem

from util import (
    make_parser,
    make_process,
)

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.params import NULL
from m5.util import (
    fatal,
    warn,
)

from gem5.isas import ISA

parser = make_parser()
parser.add_argument(
    "--bbv",
    required=True,
    type=os.path.abspath,
    help="Path to basic block trace file",
)
parser.add_argument(
    "--bbvinfo",
    type=os.path.abspath,
    help="Path to basic block extra info file",
)
parser.add_argument(
    "--warmup",
    required=True,
    type=int,
    help="Warmup period, in number of instructions",
)
parser.add_argument(
    "--interval",
    required=True,
    type=int,
    help="Interval size, in number of instructions",
)
parser.add_argument(
    "--tolerance",
    type=int,
    default=2,
    help="Interval tolerance factor",
)
args = parser.parse_args()
process = make_process(args)

CPUClass = ObjectList.cpu_list.get("X86PinCPU")
assert int(CPUClass.numThreads) == 1
assert not args.smt
assert args.num_cpus == 1

# PinCPU is single threaded, and the asserts above require a single CPU.
np = 1
mp0_path = process.executable
system = System(
    cpu=[CPUClass(cpu_id=i) for i in range(np)],
    mem_mode=CPUClass.memory_mode(),
    mem_ranges=[AddrRange(args.mem_size)],
    cache_line_size=args.cacheline_size,
)
system.shared_backstore = f"physmem"
system.auto_unlink_shared_backstore = True
# PinCPU passes the backstore fd to the Pin subprocess; an anonymous
# segment keeps it out of /dev/shm and releases it when both exit.
system.anonymous_shared_backstore = True
cpu = system.cpu[0]

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(
    clock=args.cpu_clock, voltage_domain=system.cpu_voltage_domain
)

# If elastic tracing is enabled, then configure the cpu and attach the elastic
# trace probe
if args.elastic_trace_en:
    CpuConfig.config_etrace(CPUClass, system.cpu, args)


# Set pin params.
cpu = system.cpu[0]
cpu.pinToolArgs = "-bbhist 1"
cpu.countInsts = True

# for cpu in system.cpu:
#     cpu.usePerf = True

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
cpu.clk_domain = system.cpu_clk_domain

system.m5ops_base = max(0xFFFF0000, Addr(args.mem_size).getValue())

process.maxStackSize = args.max_stack_size

cpu.workload = process
cpu.createThreads()

system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports
CacheConfig.config_cache(args, system)
MemConfig.config_mem(args, system)
config_filesystem(system, args)

system.workload = SEWorkload.init_compatible(mp0_path)

root = Root(full_system=False, system=system)
m5.instantiate()

# Launch the pin subprocess, so we can
# set syscall breakpoints.
m5.simulate(0)
exit_sysnos = [
    60,  # exit
    231,  # exit_group
]
for exit_sysno in exit_sysnos:
    cpu.executePinCommand(f"sysbreak {exit_sysno}")

clean_exit_cause = "exiting with last active thread context"
break_exit_cause = "pin-breakpoint"


def run_until(cmd: str):
    cpu.executePinCommand(cmd)
    exit_cause = m5.simulate()
    if exit_cause == clean_exit_cause:
        return True
    if exit_cause != break_exit_cause:
        print(
            f"bbv: expected exit cause '{break_exit_cause}', "
            f"got '{exit_cause}'",
            file=sys.stderr,
        )
        exit(1)
    return False


class Exit(BaseException):
    pass


def run_for_n(counter: str, n: int):
    # TODO: Should standardize command to 'count <name>'
    count = int(cpu.executePinCommand(f"{counter}count"))
    cpu.executePinCommand(f"breakpoint {counter} {count + n}")
    exit_cause = m5.simulate().getCause()
    if exit_cause == clean_exit_cause:
        raise Exit()
    if exit_cause != break_exit_cause:
        print(
            f"bbv: expected exit cause '{break_exit_cause}', "
            f"got '{exit_cause}'",
            file=sys.stderr,
        )
        exit(1)


def run_for_n_insts(n: int):
    run_for_n("inst", n)
    return int(cpu.executePinCommand("instcount"))


def parse_bbhist(s: str) -> list:
    lines = s.split("\n")
    assert len(lines[-1]) == 0
    lines = lines[:-1]
    result = list()

    total_insts = 0
    for line in lines:
        count, block = line.split()
        count = int(count)
        block = block.split(",")
        result.append((block, count))
        total_insts += count * len(block)
    return result, total_insts


block_to_id_dict = dict()


def block_to_id(block: str) -> int:
    block = str(block)
    if block not in block_to_id_dict:
        block_to_id_dict[block] = len(block_to_id_dict) + 1
    return block_to_id_dict[block]


def dump_bbhist(s: str, f, good: bool):
    # Check if this interval is of a sane length.
    f.write("T")
    l, total_insts = parse_bbhist(s)
    if total_insts <= args.interval * args.tolerance:
        for block, count in l:
            assert count > 0
            id = block_to_id(block)
            weight = count * len(block)
            f.write(f" :{id}:{weight}")
    else:
        f.write(" :1:1")
    f.write("\n")


# Instruction counts at each interval boundary.
warmups = [0]
intervals = []
bbhists = []

f_bbv = open(args.bbv, "w")

try:
    # Prime the loop by running for <warmup> instructions
    # and then discarding and resetting the bbhist.
    intervals.append(run_for_n_insts(args.warmup))

    cpu.executePinCommand("bbhist reset")

    while True:
        # Run for <interval> - <warmup> instructions.
        warmups.append(run_for_n_insts(args.interval - args.warmup))

        # Run to the end of the current interval (<warmup> instructions).
        intervals.append(run_for_n_insts(args.warmup))

        # Dump and reset the bbhist.
        bbhist = cpu.executePinCommand("bbhist dump")
        idx = len(bbhists)
        warmup = warmups[idx]
        interval_begin = intervals[idx]
        interval_end = intervals[idx + 1]
        good = (
            interval_end - warmup
            <= (args.warmup + args.interval) * args.tolerance
        )
        dump_bbhist(bbhist, f_bbv, good)
        bbhists.append(None)
        cpu.executePinCommand("bbhist reset")
        print(f"bbv: dumped interval {len(bbhists)}!", file=sys.stderr)

except Exit:
    pass

# The workload exited cleanly.
# Now, process the data into proper files.
#   - bbv.txt: The basic block vector file.
#   - bbv.info.txt: Metadata about the vector file, organized into triples:
#     warmup-begin warmup-end/interval-begin interval-end

assert len(warmups) >= len(intervals) and len(warmups) >= len(bbhists)
assert (
    len(warmups) - len(intervals) <= 1 and len(intervals) - len(bbhists) <= 1
)

# Generate bbv.info.txt.
if args.bbvinfo:
    with open(args.bbvinfo, "w") as f:
        for i in range(len(bbhists)):
            warmup = warmups[i]
            interval_begin = intervals[i]
            interval_end = intervals[i + 1]
            print(warmup, interval_begin, interval_end, file=f)
