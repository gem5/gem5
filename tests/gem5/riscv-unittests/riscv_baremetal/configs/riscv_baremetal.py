# Copyright (c) 2026 The Regents of the University of California
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

import argparse
import sys

import m5
from m5.objects import (
    AddrRange,
    Bridge,
    DDR3_1600_8x8,
    HiFive,
    IOXBar,
    MemCtrl,
    PMAChecker,
    RiscvBareMetal,
    RiscvRTC,
    RiscvSemihosting,
    RiscvSystem,
    Root,
    SrcClockDomain,
    SystemXBar,
    VoltageDomain,
)
from m5.objects.RiscvCPU import RiscvAtomicSimpleCPU
from m5.util.convert import toMemorySize

parser = argparse.ArgumentParser(
    description="Run a RISC-V bare-metal unit-test binary."
)
parser.add_argument("binary", help="The bare-metal RISC-V binary to run.")
parser.add_argument(
    "--mem-size",
    default="128MiB",
    help="Amount of memory to attach to the test system.",
)

args = parser.parse_args()

memory_start = 0x7FFFF000
reset_vector_size = 0x1000

system = RiscvSystem()
system.mem_mode = "atomic"
system.mem_ranges = [
    AddrRange(
        start=memory_start,
        size=toMemorySize(args.mem_size) + reset_vector_size,
    )
]

system.workload = RiscvBareMetal(
    bootloader=args.binary,
    semihosting=RiscvSemihosting(cmd_line=args.binary),
)

system.iobus = IOXBar()
system.membus = SystemXBar()
system.system_port = system.membus.cpu_side_ports

system.platform = HiFive()
system.platform.rtc = RiscvRTC(frequency="100MHz")
system.platform.clint.int_pin = system.platform.rtc.int_pin
system.platform.pci_host.internal_connect()
system.platform.pci_host.connect_upper_bus(system.iobus, True)
system.platform.attachOnChipIO(system.membus)
system.platform.attachOffChipIO(system.iobus)
system.platform.attachPlic()
system.platform.setNumCores(1)

system.bridge = Bridge(delay="50ns")
system.bridge.mem_side_port = system.iobus.cpu_side_ports
system.bridge.cpu_side_port = system.membus.mem_side_ports
system.bridge.ranges = system.platform._off_chip_ranges()

system.iobridge = Bridge(delay="50ns", ranges=system.mem_ranges)
system.iobridge.cpu_side_port = system.iobus.mem_side_ports
system.iobridge.mem_side_port = system.membus.cpu_side_ports

system.cache_line_size = 64
system.voltage_domain = VoltageDomain(voltage="1V")
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=system.voltage_domain
)
system.cpu_voltage_domain = VoltageDomain()
system.cpu_clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=system.cpu_voltage_domain
)

system.cpu = RiscvAtomicSimpleCPU(clk_domain=system.cpu_clk_domain, cpu_id=0)
system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
system.cpu.createInterruptController()
system.cpu.createThreads()

uncacheable_range = [
    *system.platform._on_chip_ranges(),
    *system.platform._off_chip_ranges(),
]
system.cpu.mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)

system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8(range=system.mem_ranges[0])
system.mem_ctrl.port = system.membus.mem_side_ports

root = Root(full_system=True, system=system)

m5.instantiate()

exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}.")
sys.exit(exit_event.getCode())
