# Copyright (c) 2016, University of Kaiserslautern
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
# OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Matthias Jung

import m5
import optparse

from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../../../configs/common/')

from Caches import *

# This configuration shows a simple setup of a Elastic Trace Player (eTraceCPU)
# and an external TLM port for SystemC co-simulation.
#
# We assume a DRAM size of 512MB and L1 cache sizes of 32KB and an L2 cache
# size of 1MB.
#
# Base System Architecture:
#
#                  +-----------+       ^
# +-------------+  | eTraceCPU |       |
# | System Port |  +-----+-----+       |
# +------+------+  | $D1 | $I1 |       |
#        |         +--+--+--+--+       |
#        |            |     |          | gem5 World (see this file)
#        |         +--v-----v--+       |
#        |         | toL2Bus   |       |
#        |         +-----+-----+       |
#        |               |             |
#        |         +-----v-----+       |
#        |         |    L2     |       |
#        |         +-----+-----+       |
#        |               |             |
# +------v---------------v-----+       |
# |           Membus           |       v
# +----------------+-----------+       External Port (see sc_port.*)
#                  |                   ^
#              +---v---+               | TLM World
#              |  TLM  |               | (see sc_target.*)
#              +-------+               v
#
#
# Create a system with a Crossbar and an Elastic Trace Player as CPU:

# Setup System:
system = System(cpu=TraceCPU(cpu_id=0),
                mem_mode='timing',
                mem_ranges = [AddrRange('1024MB')],
                cache_line_size = 64)

# Create a top-level voltage domain:
system.voltage_domain = VoltageDomain()

# Create a source clock for the system. This is used as the clock period for
# xbar and memory:
system.clk_domain = SrcClockDomain(clock =  '1GHz',
        voltage_domain = system.voltage_domain)

# Create a CPU voltage domain:
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs. In case of Trace CPUs this clock
# is actually used only by the caches connected to the CPU:
system.cpu_clk_domain = SrcClockDomain(clock = '1GHz',
        voltage_domain = system.cpu_voltage_domain)

# Setup CPU and its L1 caches:
system.cpu.createInterruptController()
system.cpu.icache = L1_ICache(size="32kB")
system.cpu.dcache = L1_DCache(size="32kB")
system.cpu.icache.cpu_side = system.cpu.icache_port
system.cpu.dcache.cpu_side = system.cpu.dcache_port

# Assign input trace files to the eTraceCPU:
system.cpu.instTraceFile="system.cpu.traceListener.inst.gz"
system.cpu.dataTraceFile="system.cpu.traceListener.data.gz"

# Setting up L1 BUS:
system.tol2bus = L2XBar()
system.l2cache = L2Cache(size="1MB")
system.physmem = SimpleMemory() # This must be instantiated, even if not needed

# Create a external TLM port:
system.tlm = ExternalSlave()
system.tlm.addr_ranges = [AddrRange('4096MB')]
system.tlm.port_type = "tlm_slave"
system.tlm.port_data = "transactor1"

# Connect everything:
system.membus = SystemXBar()
system.system_port = system.membus.slave
system.cpu.icache.mem_side = system.tol2bus.slave
system.cpu.dcache.mem_side = system.tol2bus.slave
system.tol2bus.master = system.l2cache.cpu_side
system.l2cache.mem_side = system.membus.slave
system.membus.master = system.tlm.port

# Start the simulation:
root = Root(full_system = False, system = system)
root.system.mem_mode = 'timing'
m5.instantiate()
m5.simulate() # Simulation time specified later on commandline
