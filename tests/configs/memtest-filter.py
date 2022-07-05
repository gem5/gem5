# Copyright (c) 2006-2007 The Regents of The University of Michigan
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

import m5
from m5.objects import *

m5.util.addToPath("../configs/")
from common.Caches import *

# MAX CORES IS 8 with the fals sharing method
nb_cores = 8
cpus = [MemTest() for i in range(nb_cores)]

# system simulated
system = System(
    cpu=cpus,
    physmem=SimpleMemory(),
    membus=SystemXBar(width=16, snoop_filter=SnoopFilter()),
)
# Dummy voltage domain for all our clock domains
system.voltage_domain = VoltageDomain()
system.clk_domain = SrcClockDomain(
    clock="1GHz", voltage_domain=system.voltage_domain
)

# Create a seperate clock domain for components that should run at
# CPUs frequency
system.cpu_clk_domain = SrcClockDomain(
    clock="2GHz", voltage_domain=system.voltage_domain
)

system.toL2Bus = L2XBar(
    clk_domain=system.cpu_clk_domain, snoop_filter=SnoopFilter()
)
system.l2c = L2Cache(clk_domain=system.cpu_clk_domain, size="64kB", assoc=8)
system.l2c.cpu_side = system.toL2Bus.mem_side_ports

# connect l2c to membus
system.l2c.mem_side = system.membus.cpu_side_ports

# add L1 caches
for cpu in cpus:
    # All cpus are associated with cpu_clk_domain
    cpu.clk_domain = system.cpu_clk_domain
    cpu.l1c = L1Cache(size="32kB", assoc=4)
    cpu.l1c.cpu_side = cpu.port
    cpu.l1c.mem_side = system.toL2Bus.cpu_side_ports

system.system_port = system.membus.cpu_side_ports

# connect memory to membus
system.physmem.port = system.membus.mem_side_ports


# -----------------------
# run simulation
# -----------------------

root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"
