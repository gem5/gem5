# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2010 Advanced Micro Devices, Inc.
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
#
# Authors: Ron Dreslinski

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, optparse, sys

# Get paths we might need
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)
addToPath(config_root+'/configs/common')
addToPath(config_root+'/configs/ruby')
addToPath(config_root+'/configs/topologies')

import Ruby
import Options

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
options.ports=32

#MAX CORES IS 8 with the fals sharing method
nb_cores = 8

# ruby does not support atomic, functional, or uncacheable accesses
cpus = [ MemTest(atomic=False, percent_functional=50,
                 percent_uncacheable=0, suppress_func_warnings=True) \
         for i in xrange(nb_cores) ]

# overwrite options.num_cpus with the nb_cores value
options.num_cpus = nb_cores
 
# system simulated
system = System(cpu = cpus,
                funcmem = SimpleMemory(in_addr_map = False),
                physmem = SimpleMemory(null = True),
                funcbus = NoncoherentBus())
# Dummy voltage domain for all our clock domains
system.voltage_domain = VoltageDomain()
system.clk_domain = SrcClockDomain(clock = '1GHz',
                                   voltage_domain = system.voltage_domain)

# Create a seperate clock domain for components that should run at
# CPUs frequency
system.cpu_clk_domain = SrcClockDomain(clock = '2GHz',
                                       voltage_domain = system.voltage_domain)

# All cpus are associated with cpu_clk_domain
for cpu in cpus:
    cpu.clk_domain = system.cpu_clk_domain

system.mem_ranges = AddrRange('256MB')

Ruby.create_system(options, system)

# Create a separate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                        voltage_domain = system.voltage_domain)

assert(len(cpus) == len(system.ruby._cpu_ports))

for (i, ruby_port) in enumerate(system.ruby._cpu_ports):
     #
     # Tie the cpu test and functional ports to the ruby cpu ports and
     # physmem, respectively
     #
     cpus[i].test = ruby_port.slave
     cpus[i].functional = system.funcbus.slave
     
     #
     # Since the memtester is incredibly bursty, increase the deadlock
     # threshold to 1 million cycles
     #
     ruby_port.deadlock_threshold = 1000000

# connect reference memory to funcbus
system.funcmem.port = system.funcbus.master

# -----------------------
# run simulation
# -----------------------

root = Root(full_system = False, system = system)
root.system.mem_mode = 'timing'

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency('1ns')
