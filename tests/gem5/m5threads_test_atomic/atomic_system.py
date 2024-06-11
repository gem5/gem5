# Copyright (c) 2020 The Regents of the University of California
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
import sys

from caches import *

import m5
from m5.objects import *

parser = argparse.ArgumentParser(description="m5threads atomic tester")
parser.add_argument("--cpu-type", default="DerivO3CPU")
parser.add_argument("--num-cores", default="8")
parser.add_argument("--cmd")

args = parser.parse_args()

root = Root(full_system=False)
root.system = System()

root.system.workload = SEWorkload.init_compatible(args.cmd)

root.system.clk_domain = SrcClockDomain()
root.system.clk_domain.clock = "3GHz"
root.system.clk_domain.voltage_domain = VoltageDomain()
root.system.mem_mode = "timing"
root.system.mem_ranges = [AddrRange("512MB")]

if args.cpu_type == "DerivO3CPU":
    root.system.cpu = [
        SparcO3CPU(cpu_id=i) for i in range(int(args.num_cores))
    ]
elif args.cpu_type == "TimingSimpleCPU":
    root.system.cpu = [
        SparcTimingSimpleCPU(cpu_id=i) for i in range(int(args.num_cores))
    ]
else:
    print("ERROR: CPU Type '" + args.cpu_type + "' not supported")
    sys.exit(1)

root.system.membus = SystemXBar()
root.system.membus.badaddr_responder = BadAddr()
root.system.membus.default = root.system.membus.badaddr_responder.pio

root.system.system_port = root.system.membus.cpu_side_ports

process = Process(executable=args.cmd, cmd=[args.cmd, str(args.num_cores)])

for cpu in root.system.cpu:
    cpu.workload = process
    cpu.createThreads()
    cpu.createInterruptController()

    # Create a memory bus, a coherent crossbar, in this case
    cpu.l2bus = L2XBar()

    # Create an L1 instruction and data cache
    cpu.icache = L1ICache()
    cpu.dcache = L1DCache()

    # Connect the instruction and data caches to the CPU
    cpu.icache.connectCPU(cpu)
    cpu.dcache.connectCPU(cpu)

    # Hook the CPU ports up to the l2bus
    cpu.icache.connectBus(cpu.l2bus)
    cpu.dcache.connectBus(cpu.l2bus)

    # Create an L2 cache and connect it to the l2bus
    cpu.l2cache = L2Cache()
    cpu.l2cache.connectCPUSideBus(cpu.l2bus)

    # Connect the L2 cache to the L3 bus
    cpu.l2cache.connectMemSideBus(root.system.membus)

root.system.mem_ctrl = DDR3_1600_8x8()
root.system.mem_ctrl.range = root.system.mem_ranges[0]
root.system.mem_ctrl.port = root.system.membus.mem_side_ports

m5.instantiate()
exit_event = m5.simulate()
