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
from m5.defines import buildEnv
from m5.util import addToPath
import os, optparse, sys

m5.util.addToPath('../configs/')

from common import Options
from ruby import Ruby

parser = optparse.OptionParser()
Options.addCommonOptions(parser)

# Add the ruby specific and protocol specific options
Ruby.define_options(parser)

(options, args) = parser.parse_args()

#
# Set the default cache size and associativity to be very small to encourage
# races between requests and writebacks.
#
options.l1d_size="256B"
options.l1i_size="256B"
options.l2_size="512B"
options.l3_size="1kB"
options.l1d_assoc=2
options.l1i_assoc=2
options.l2_assoc=2
options.l3_assoc=2

nb_cores = 4
cpus = [ TimingSimpleCPU(cpu_id=i) for i in range(nb_cores) ]

# overwrite the num_cpus to equal nb_cores
options.num_cpus = nb_cores

# system simulated
system = System(cpu = cpus, clk_domain = SrcClockDomain(clock = '1GHz'))

# Create a seperate clock domain for components that should run at
# CPUs frequency
system.cpu.clk_domain = SrcClockDomain(clock = '2GHz')

Ruby.create_system(options, False, system)

# Create a separate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock)

assert(options.num_cpus == len(system.ruby._cpu_ports))

for (i, cpu) in enumerate(system.cpu):
    # create the interrupt controller
    cpu.createInterruptController()

    #
    # Tie the cpu ports to the ruby cpu ports
    #
    cpu.connectAllPorts(system.ruby._cpu_ports[i])

# -----------------------
# run simulation
# -----------------------

root = Root( full_system=False, system = system )
root.system.mem_mode = 'timing'
