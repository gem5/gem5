# Copyright (c) 2018 The Regents of the University of California
# All Rights Reserved.
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

import m5
from m5.objects import *


class L1Cache(Cache):
    """Simple L1 Cache with default values"""

    assoc = 8
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 16
    tgts_per_mshr = 20

    def connectBus(self, bus):
        """Connect this cache to a memory-side bus"""
        self.mem_side = bus.cpu_side_ports

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU-side port
        This must be defined in a subclass"""
        raise NotImplementedError


class L1ICache(L1Cache):
    """Simple L1 instruction cache with default values"""

    # Set the default size
    size = "32KiB"

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port


class L1DCache(L1Cache):
    """Simple L1 data cache with default values"""

    # Set the default size
    size = "32KiB"

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port


class L2Cache(Cache):
    """Simple L2 Cache with default values"""

    # Default parameters
    size = "512KiB"
    assoc = 16
    tag_latency = 10
    data_latency = 10
    response_latency = 1
    mshrs = 20
    tgts_per_mshr = 12

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


class MySimpleMemory(SimpleMemory):
    latency = "1ns"


valid_cpu = {
    "X86AtomicSimpleCPU": X86AtomicSimpleCPU,
    "X86TimingSimpleCPU": X86TimingSimpleCPU,
    "X86DerivO3CPU": X86O3CPU,
    "ArmAtomicSimpleCPU": ArmAtomicSimpleCPU,
    "ArmTimingSimpleCPU": ArmTimingSimpleCPU,
    "ArmMinorCPU": ArmMinorCPU,
    "ArmDerivO3CPU": ArmO3CPU,
    "RiscvAtomicSimpleCPU": RiscvAtomicSimpleCPU,
    "RiscvTimingSimpleCPU": RiscvTimingSimpleCPU,
    "RiscvMinorCPU": RiscvMinorCPU,
    "RiscvDerivO3CPU": RiscvO3CPU,
}

valid_mem = {"SimpleMemory": MySimpleMemory, "DDR3_1600_8x8": DDR3_1600_8x8}

parser = argparse.ArgumentParser()
parser.add_argument("binary", type=str)
parser.add_argument("--cpu")
parser.add_argument("--mem", choices=valid_mem.keys(), default="SimpleMemory")

args = parser.parse_args()

system = System()

system.workload = SEWorkload.init_compatible(args.binary)

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

if args.cpu not in (
    "X86AtomicSimpleCPU",
    "ArmAtomicSimpleCPU",
    "RiscvAtomicSimpleCPU",
):
    system.mem_mode = "timing"

system.mem_ranges = [AddrRange("512MiB")]

system.cpu = valid_cpu[args.cpu]()

if args.cpu in (
    "X86AtomicSimpleCPU",
    "ArmAtomicSimpleCPU",
    "RiscvAtomicSimpleCPU",
):
    system.membus = SystemXBar()
    system.cpu.icache_port = system.membus.cpu_side_ports
    system.cpu.dcache_port = system.membus.cpu_side_ports
else:
    system.cpu.l1d = L1DCache()
    system.cpu.l1i = L1ICache()
    system.l1_to_l2 = L2XBar()
    system.l2cache = L2Cache()
    system.membus = SystemXBar()
    system.cpu.l1d.connectCPU(system.cpu)
    system.cpu.l1d.connectBus(system.l1_to_l2)
    system.cpu.l1i.connectCPU(system.cpu)
    system.cpu.l1i.connectBus(system.l1_to_l2)
    system.l2cache.connectCPUSideBus(system.l1_to_l2)
    system.l2cache.connectMemSideBus(system.membus)

system.cpu.createInterruptController()
if args.cpu in ("X86AtomicSimpleCPU", "X86TimingSimpleCPU", "X86DerivO3CPU"):
    system.cpu.interrupts[0].pio = system.membus.mem_side_ports
    system.cpu.interrupts[0].int_master = system.membus.cpu_side_ports
    system.cpu.interrupts[0].int_slave = system.membus.mem_side_ports

system.mem_ctrl = valid_mem[args.mem]()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports
system.system_port = system.membus.cpu_side_ports

process = Process()
process.cmd = [args.binary]
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system=False, system=system)
m5.instantiate()

exit_event = m5.simulate()

if exit_event.getCause() != "exiting with last active thread context":
    exit(1)
