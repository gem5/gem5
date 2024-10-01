# Copyright (c) 2022, Fraunhofer IESE
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
import os

import m5
from m5.objects import *

# Create a config to be used by the traffic generator
cfg_file_name = "memcheck.cfg"
cfg_file_path = os.path.dirname(__file__) + "/" + cfg_file_name
cfg_file = open(cfg_file_path, "w")

# Three states, with random, linear and idle behaviours. The random
# and linear states access memory in the range [0 : 16 Mbyte] with 8
# byte and 64 byte accesses respectively.
cfg_file.write("STATE 0 10000000 RANDOM 65 0 16777216 8 50000 150000 0\n")
cfg_file.write("STATE 1 10000000 LINEAR 65 0 16777216 64 50000 150000 0\n")
cfg_file.write("STATE 2 10000000 IDLE\n")
cfg_file.write("INIT 0\n")
cfg_file.write("TRANSITION 0 1 0.5\n")
cfg_file.write("TRANSITION 0 2 0.5\n")
cfg_file.write("TRANSITION 1 0 0.5\n")
cfg_file.write("TRANSITION 1 2 0.5\n")
cfg_file.write("TRANSITION 2 0 0.5\n")
cfg_file.write("TRANSITION 2 1 0.5\n")
cfg_file.close()

system = System()
vd = VoltageDomain(voltage="1V")

system.mem_mode = "timing"

system.cpu = TrafficGen(config_file=cfg_file_path)
system.target = TLM_Target()
system.physmem = (
    SimpleMemory()
)  # This must be instanciated, even if not needed
# system.mem.addr_ranges = [AddrRange('512MiB')]
system.transactor = Gem5ToTlmBridge32()
system.clk_domain = SrcClockDomain(clock="1.5GHz", voltage_domain=vd)

# Connect everything:
system.transactor.gem5 = system.cpu.port
system.transactor.tlm = system.target.tlm

kernel = SystemC_Kernel(system=system)
root = Root(full_system=False, systemc_kernel=kernel)

m5.instantiate(None)

cause = m5.simulate(m5.MaxTick).getCause()
print(cause)
