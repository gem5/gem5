#
# Copyright (c) 2016, Dresden University of Technology (TU Dresden)
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
# Authors: Christian Menard
#

import m5
from m5.objects import *

import os

# Base System Architecture:
#         +-----+           ^
#         | TLM |           | TLM World
#         +--+--+           | (see main.cc)
#            |              v
# +----------v-----------+  External Port (see sc_master_port.*)
# |        Membus        |  ^
# +----------+-----------+  |
#            |              | gem5 World
#        +---v----+         |
#        | Memory |         |
#        +--------+         v
#

# Create a system with a Crossbar and a simple Memory:
system = System()
system.membus = IOXBar(width = 16)
system.physmem = SimpleMemory(range = AddrRange('512MB'))
system.clk_domain = SrcClockDomain(clock = '1.5GHz',
    voltage_domain = VoltageDomain(voltage = '1V'))

# Create a external TLM port:
system.tlm = ExternalMaster()
system.tlm.port_type = "tlm_master"
system.tlm.port_data = "transactor"

# Route the connections:
system.system_port = system.membus.slave
system.physmem.port = system.membus.master
system.tlm.port = system.membus.slave
system.mem_mode = 'timing'

# Start the simulation:
root = Root(full_system = False, system = system)
m5.instantiate()
m5.simulate()
