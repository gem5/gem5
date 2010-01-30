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

parser = optparse.OptionParser()

parser.add_option("-a", "--atomic", action="store_true",
                  help="Use atomic (non-timing) mode")
parser.add_option("-b", "--blocking", action="store_true",
                  help="Use blocking caches")
parser.add_option("-l", "--maxloads", metavar="N", default=0,
                  help="Stop after N loads")
parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")

parser.add_option("-t", "--testers", type="int", metavar="N", default=1,
                  help="number of testers/cores")

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

(options, args) = parser.parse_args()

if args:
     print "Error: script doesn't take any positional arguments"
     sys.exit(1)

block_size = 64

if options.testers > block_size:
     print "Error: Number of testers %d limited to %d because of false sharing" \
           % (options.testers, block_size)
     sys.exit(1)

cpus = [ MemTest(atomic=options.atomic, max_loads=options.maxloads, \
                 percent_functional=options.functional, \
                 percent_uncacheable=options.uncacheable, \
                 progress_interval=options.progress) \
         for i in xrange(options.testers) ]

system = System(cpu = cpus,
                funcmem = PhysicalMemory(),
                physmem = PhysicalMemory())

class L1Cache(RubyCache):
    assoc = 2
    latency = 3
    size = 32768

class L2Cache(RubyCache):
    assoc = 16
    latency = 15
    size = 1048576

# It would be nice to lump all the network nodes into a single list,
# but for consistency with the old scripts I'm segregating them by
# type.  I'm not sure if this is really necessary or not.
 
# net_nodes = []
l1_cntrl_nodes = []
dir_cntrl_nodes = []

for cpu in cpus:
    l1_cntrl = L1Cache_Controller()
    cpu_seq = RubySequencer(controller = l1_cntrl,
                            icache = L1Cache(controller = l1_cntrl),
                            dcache = L1Cache(controller = l1_cntrl))
    cpu.controller = l1_cntrl
    cpu.sequencer = cpu_seq
    cpu.test = cpu_seq.port
    cpu_seq.funcmem_port = system.physmem.port
    cpu.functional = system.funcmem.port
    
    dir_cntrl = Directory_Controller(version = i,
                                     directory = RubyDirectoryMemory(),
                                     memory_control = RubyMemoryControl())

    # net_nodes += [l1_cntrl, dir_cntrl]
    l1_cntrl_nodes.append(l1_cntrl)
    dir_cntrl_nodes.append(dir_cntrl)

network = SimpleNetwork(topology = makeCrossbar(l1_cntrl_nodes + \
                                                dir_cntrl_nodes))

mem_size_mb = sum([int(dir_cntrl.directory.size_mb) \
                   for dir_cntrl in dir_cntrl_nodes])

system.ruby = RubySystem(network = network,
                         profiler = RubyProfiler(),
                         tracer = RubyTracer(),
                         debug = RubyDebug(),
                         mem_size_mb = mem_size_mb)


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
m5.instantiate(root)

# simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
