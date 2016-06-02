# Copyright (c) 2012 ARM Limited
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
#
# Authors: Andreas Hansson

import m5
from m5.objects import *

# both traffic generator and communication monitor are only available
# if we have protobuf support, so potentially skip this test
require_sim_object("TrafficGen")
require_sim_object("CommMonitor")

# even if this is only a traffic generator, call it cpu to make sure
# the scripts are happy
cpu = TrafficGen(
    config_file=srcpath("tests/quick/se/70.tgen/tgen-dram-ctrl.cfg"))

# system simulated
system = System(cpu = cpu, physmem = DDR3_1600_x64(),
                membus = IOXBar(width = 16),
                clk_domain = SrcClockDomain(clock = '1GHz',
                                            voltage_domain =
                                            VoltageDomain()))

# add a communication monitor
system.monitor = CommMonitor()

# connect the traffic generator to the bus via a communication monitor
system.cpu.port = system.monitor.slave
system.monitor.master = system.membus.slave

# connect the system port even if it is not used in this example
system.system_port = system.membus.slave

# connect memory to the membus
system.physmem.port = system.membus.master

# -----------------------
# run simulation
# -----------------------

root = Root(full_system = False, system = system)
root.system.mem_mode = 'timing'
