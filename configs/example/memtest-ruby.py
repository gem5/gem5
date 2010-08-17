# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
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
#          Brad Beckmann

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, optparse, sys
addToPath('../common')
addToPath('../ruby')

import Ruby

if buildEnv['FULL_SYSTEM']:
    panic("This script requires system-emulation mode (*_SE).")

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = optparse.OptionParser()

parser.add_option("-a", "--atomic", action="store_true",
                  help="Use atomic (non-timing) mode")
parser.add_option("-b", "--blocking", action="store_true",
                  help="Use blocking caches")
parser.add_option("-l", "--maxloads", metavar="N", default=0,
                  help="Stop after N loads")
parser.add_option("-f", "--functional", type="int", default=0,
                  metavar="PCT",
                  help="Target percentage of functional accesses "
                  "[default: %default]")
parser.add_option("-u", "--uncacheable", type="int", default=0,
                  metavar="PCT",
                  help="Target percentage of uncacheable accesses "
                  "[default: %default]")

parser.add_option("--progress", type="int", default=1000,
                  metavar="NLOADS",
                  help="Progress message interval "
                  "[default: %default]")

#
# Set the default cache size and associativity to be very small to encourage
# races between requests and writebacks.
#
parser.add_option("--l1d_size", type="string", default="256B")
parser.add_option("--l1i_size", type="string", default="256B")
parser.add_option("--l2_size", type="string", default="512B")
parser.add_option("--l1d_assoc", type="int", default=2)
parser.add_option("--l1i_assoc", type="int", default=2)
parser.add_option("--l2_assoc", type="int", default=2)

execfile(os.path.join(config_root, "common", "Options.py"))

(options, args) = parser.parse_args()

if args:
     print "Error: script doesn't take any positional arguments"
     sys.exit(1)

block_size = 64

if options.num_cpus > block_size:
     print "Error: Number of testers %d limited to %d because of false sharing" \
           % (options.num_cpus, block_size)
     sys.exit(1)

cpus = [ MemTest(atomic=options.atomic, max_loads=options.maxloads, \
                 percent_functional=options.functional, \
                 percent_uncacheable=options.uncacheable, \
                 progress_interval=options.progress) \
         for i in xrange(options.num_cpus) ]

system = System(cpu = cpus,
                funcmem = PhysicalMemory(),
                physmem = PhysicalMemory())

system.ruby = Ruby.create_system(options, system.physmem)

assert(len(cpus) == len(system.ruby.cpu_ruby_ports))

for (i, cpu) in enumerate(cpus):
    #
    # Tie the memtester ports to the correct system ports
    #
    cpu.test = system.ruby.cpu_ruby_ports[i].port
    cpu.functional = system.funcmem.port

# -----------------------
# run simulation
# -----------------------

root = Root( system = system )
if options.atomic:
    root.system.mem_mode = 'atomic'
else:
    root.system.mem_mode = 'timing'

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency('1ns')

# instantiate configuration
m5.instantiate()

# simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
