# Copyright (c) 2026 The Board of Trustees of the Leland Stanford
# Junior University
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

"""Run a static SE workload under X86PinCPU.

Usage: run.py <binary> [--checkpoint-at INSTS]
"""

import argparse
import os
import sys

import m5
from m5.objects import (
    AddrRange,
    DDR3_1600_8x8,
    MemCtrl,
    Process,
    Root,
    SEWorkload,
    SrcClockDomain,
    System,
    SystemXBar,
    VoltageDomain,
)
from m5.objects.X86PinCPU import X86PinCPU

parser = argparse.ArgumentParser()
parser.add_argument("binary", help="Static binary to run")
parser.add_argument(
    "--checkpoint-at",
    type=int,
    default=None,
    help="Take a checkpoint after this many instructions, then continue",
)
args = parser.parse_args()

process = Process(pid=100)
process.executable = args.binary
process.cmd = [args.binary]
process.cwd = os.getcwd()
process.addrSpaceShift = -0x100000000000

system = System(
    cpu=[X86PinCPU(cpu_id=0)],
    mem_mode=X86PinCPU.memory_mode(),
    mem_ranges=[AddrRange("512MiB")],
    cache_line_size=64,
)

# PinCPU requires a backing store shared with the Pin subprocess.
# Make it anonymous so it auto-deletes on exit and does not
# pollute the shared memory namespace.
system.shared_backstore = "physmem"
system.auto_unlink_shared_backstore = True
system.anonymous_shared_backstore = True

system.voltage_domain = VoltageDomain()
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=system.voltage_domain
)
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports
system.workload = SEWorkload.init_compatible(process.executable)

cpu = system.cpu[0]
cpu.clk_domain = system.clk_domain

if args.checkpoint_at is not None:
    cpu.countInsts = True
cpu.workload = process
cpu.createThreads()
cpu.createInterruptController()

# PinCPU does not use these, but gem5 still requires them to be connected.
cpu.icache_port = system.membus.cpu_side_ports
cpu.dcache_port = system.membus.cpu_side_ports
cpu.interrupts[0].pio = system.membus.mem_side_ports
cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
cpu.interrupts[0].int_responder = system.membus.mem_side_ports

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8(range=system.mem_ranges[0])
system.mem_ctrl.port = system.membus.mem_side_ports

root = Root(full_system=False, system=system)
m5.instantiate()

WORKLOAD_DONE = "exiting with last active thread context"

if args.checkpoint_at is not None:
    # Need to initialize the Pin subprocess before
    # we can issue commands, like 'breakpoint inst'.
    m5.simulate(0)

    cpu.executePinCommand(f"breakpoint inst {args.checkpoint_at}")
    exit_cause = m5.simulate().getCause()
    if exit_cause != "pin-breakpoint":
        print(f"unexpected exit cause: {exit_cause}", file=sys.stderr)
        sys.exit(1)

    inst = int(cpu.executePinCommand("instcount"))
    m5.checkpoint(os.path.join(m5.options.outdir, "cpt"))
    print(f"pincpu: checkpointed at instruction {inst}", file=sys.stderr)

exit_cause = m5.simulate().getCause()
if exit_cause != WORKLOAD_DONE:
    print(f"unexpected exit cause: {exit_cause}", file=sys.stderr)
    sys.exit(1)
