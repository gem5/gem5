# Copyright (c) 2017, University of Kaiserslautern
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
# Author: Éder F. Zulian

import argparse
import sys

import m5
from m5.objects import *
from m5.util import *

addToPath("../")

from common import (
    HMC,
    MemConfig,
    ObjectList,
)

pd = "Simple 'hello world' example using HMC as main memory"
parser = argparse.ArgumentParser(description=pd)
parser.add_argument(
    "--cpu-type",
    type=str,
    default="X86TimingSimpleCPU",
    choices=ObjectList.CPUList().get_names(),
    help="CPU model to use",
)
HMC.add_options(parser)
options = parser.parse_args()
# create the system we are going to simulate
system = System()
# use timing mode for the interaction between requestor-responder ports
system.mem_mode = "timing"
# set the clock frequency of the system
clk = "1GHz"
vd = VoltageDomain(voltage="1V")
system.clk_domain = SrcClockDomain(clock=clk, voltage_domain=vd)
# create a CPU
system.cpu = ObjectList().get(options.cpu_type)()
# config memory system
MemConfig.config_mem(options, system)
# hook the CPU ports up to the membus
system.cpu.icache_port = system.membus.cpu_side_ports
system.cpu.dcache_port = system.membus.cpu_side_ports
# create the interrupt controller for the CPU and connect to the membus
system.cpu.createInterruptController()
# connect special port in the system to the membus. This port is a
# functional-only port to allow the system to read and write memory.
system.system_port = system.membus.cpu_side_ports
# run 'hello' and use the compiled ISA to find the binary


binary = (
    "tests/test-progs/hello/bin/"
    + ObjectList.CPUList().get_isa(options.cpu_type).name.lower()
    + "/linux/hello"
)

# create a process for a simple "Hello World" application
process = Process()
# cmd is a list which begins with the executable (like argv)
process.cmd = [binary]
# set the system workload
system.workload = SEWorkload.init_compatible(binary)
# set the cpu workload
system.cpu.workload = process
# create thread contexts
system.cpu.createThreads()
# set up the root SimObject
root = Root(full_system=False, system=system)
m5.instantiate()
m5.simulate()
