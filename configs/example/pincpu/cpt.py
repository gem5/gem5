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

# Checkpoint a SE workload using PinCPU.

import argparse
import os
import sys

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


def instruction_counts(s):
    """Parse a comma separated list of positive instruction counts."""
    try:
        counts = [int(part) for part in s.split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"expected a comma separated list of integers, got '{s}'"
        )
    if any(count <= 0 for count in counts):
        raise argparse.ArgumentTypeError("instruction counts must be positive")
    return counts


parser.add_argument(
    "--checkpoint-at",
    metavar="INSTS",
    type=instruction_counts,
    required=True,
    help="Comma separated instruction counts at which to take a checkpoint, "
    "e.g. --checkpoint-at 1000000,5000000",
)
args = parser.parse_args()
process = make_process(args)

CPUClass = ObjectList.cpu_list.get("X86PinCPU")
assert int(CPUClass.numThreads) == 1
assert not args.smt
assert args.num_cpus == 1

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


# Set pin params. countInsts enables the instcount plugin, which is what
# registers the "inst" counter that `breakpoint inst` resolves through.
cpu.countInsts = True

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

# gem5 runs SimObject::startup() on the first call to m5.simulate(), and that
# is where PinCPU launches the Pin subprocess. Nothing can be asked of Pin
# before then, so get it running with a zero length simulation.
m5.simulate(0)

m5.options.outdir = os.path.abspath(m5.options.outdir)

# The simulation only moves forward, so visit the requested points in order.
# Duplicates would ask us to stop twice in the same place.
checkpoint_insts = sorted(set(args.checkpoint_at))


WORKLOAD_DONE = "exiting with last active thread context"


def run_until(target):
    """Run until the guest has retired `target` instructions.

    Returns the instruction count actually reached, which may overshoot
    slightly: Pin stops at the end of the basic block containing the
    breakpoint, not at the exact instruction. Returns None if the workload
    finished first.
    """
    cpu.executePinCommand(f"breakpoint inst {target}")
    exit_cause = m5.simulate().getCause()
    if exit_cause == WORKLOAD_DONE:
        return None
    if exit_cause != "pin-breakpoint":
        print(f"cpt: unexpected exit cause: {exit_cause}", file=sys.stderr)
        exit(1)
    return int(cpu.executePinCommand("instcount"))


for target in checkpoint_insts:
    inst = run_until(target)
    if inst is None:
        # A shorter workload than the requested points; the remaining ones are
        # unreachable too, since the list is sorted.
        print(
            f"cpt: workload finished before instruction {target}, "
            f"skipping this and any later checkpoints",
            file=sys.stderr,
        )
        break

    path = os.path.join(m5.options.outdir, f"cpt.{inst}")
    m5.checkpoint(path)
    m5.stats.dump()
    print(f"cpt: checkpoint at instruction {inst} -> {path}", file=sys.stderr)
else:
    # Every checkpoint was taken, so the workload still has work left.
    exit_cause = m5.simulate().getCause()
    if exit_cause != WORKLOAD_DONE:
        print(f"cpt: unexpected exit cause: {exit_cause}", file=sys.stderr)
        exit(1)
