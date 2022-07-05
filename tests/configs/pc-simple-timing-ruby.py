# Copyright (c) 2012 Mark D. Hill and David A. Wood
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

import m5, os, argparse, sys
from m5.objects import *

m5.util.addToPath("../configs/")
from common.Benchmarks import SysConfig
from common import FSConfig, SysPaths
from ruby import Ruby
from common import Options

# Add the ruby specific and protocol specific options
parser = argparse.ArgumentParser()
Options.addCommonOptions(parser)
Ruby.define_options(parser)
args = parser.parse_args()

# Set the default cache size and associativity to be very small to encourage
# races between requests and writebacks.
args.l1d_size = "32kB"
args.l1i_size = "32kB"
args.l2_size = "4MB"
args.l1d_assoc = 2
args.l1i_assoc = 2
args.l2_assoc = 2
args.num_cpus = 2

# the system
mdesc = SysConfig(disks=["linux-x86.img"])
system = FSConfig.makeLinuxX86System(
    "timing", args.num_cpus, mdesc=mdesc, Ruby=True
)
system.kernel = SysPaths.binary("x86_64-vmlinux-2.6.22.9")
# Dummy voltage domain for all our clock domains
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

system.kernel = FSConfig.binary("x86_64-vmlinux-2.6.22.9.smp")
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=system.voltage_domain
)
system.cpu_clk_domain = SrcClockDomain(
    clock="2GHz", voltage_domain=system.voltage_domain
)
system.cpu = [
    TimingSimpleCPU(cpu_id=i, clk_domain=system.cpu_clk_domain)
    for i in range(args.num_cpus)
]

Ruby.create_system(args, True, system, system.iobus, system._dma_ports)

# Create a seperate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(
    clock=args.ruby_clock, voltage_domain=system.voltage_domain
)

# Connect the ruby io port to the PIO bus,
# assuming that there is just one such port.
system.iobus.mem_side_ports = system.ruby._io_port.in_ports

for (i, cpu) in enumerate(system.cpu):
    # create the interrupt controller
    cpu.createInterruptController()
    # Tie the cpu ports to the correct ruby system ports
    system.ruby._cpu_ports[i].connectCpuPorts(cpu)

root = Root(full_system=True, system=system)
m5.ticks.setGlobalFrequency("1THz")
