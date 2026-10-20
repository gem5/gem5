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

# Run the SE workload using PinCPU, count the number of times each basic
# block executes, and dump it to file at the end of the execution.

import argparse
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
    "--bbhist",
    required=True,
    type=os.path.abspath,
    help="Path to bbhist output file",
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

# Launch the pin subprocess,
# so we can set syscall breakpoints.
m5.simulate(0)
exit_sysnos = [
    60,  # exit
    231,  # exit_group
]
for exit_sysno in exit_sysnos:
    cpu.executePinCommand(f"sysbreak {exit_sysno}")
exit_cause = m5.simulate().getCause()
if exit_cause != "pin-breakpoint":
    print(
        f"bbhist: simulation stopped with unexpected reason "
        f"'{exit_cause}' (expected 'pin-breakpoint')",
        file=sys.stderr,
    )
    exit(1)
bbhist = cpu.executePinCommand("bbhist dump")

with open(args.bbhist, "w") as f:
    f.write(bbhist)

exit_cause = m5.simulate().getCause()
if exit_cause != "exiting with last active thread context":
    print(f"bbhist: unexpected exit reason:", exit_cause, file=sys.stderr)
    exit(1)
print("Exited:", exit_cause, file=sys.stderr)
