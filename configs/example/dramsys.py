# Copyright (c) 2022 Fraunhofer IESE
# All rights reserved
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

traffic_gen = PyTrafficGen()
system = System()
vd = VoltageDomain(voltage="1V")

system.mem_mode = "timing"

system.cpu = traffic_gen

dramsys = DRAMSys(
    configuration="ext/dramsys/DRAMSys/configs/ddr4-example.json",
    resource_directory="ext/dramsys/DRAMSys/configs",
)

system.target = dramsys
system.transactor = Gem5ToTlmBridge32()
system.clk_domain = SrcClockDomain(clock="1.5GHz", voltage_domain=vd)

# Connect everything:
system.transactor.gem5 = system.cpu.port
system.transactor.tlm = system.target.tlm

kernel = SystemC_Kernel(system=system)
root = Root(full_system=False, systemc_kernel=kernel)

m5.instantiate()
idle = traffic_gen.createIdle(100000)
linear = traffic_gen.createLinear(10000000, 0, 16777216, 64, 500, 1500, 65, 0)
random = traffic_gen.createRandom(10000000, 0, 16777216, 64, 500, 1500, 65, 0)
traffic_gen.start([linear, idle, random])

cause = m5.simulate(20000000).getCause()
print(cause)
