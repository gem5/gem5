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
#
# Authors: Ron Dreslinski

import m5
from m5.objects import *
import os, optparse, sys
m5.AddToPath('../common')

parser = optparse.OptionParser()

parser.add_option("-c", "--cache-levels", type="int", default=2,
                  metavar="LEVELS",
                  help="Number of cache levels [default: %default]")
parser.add_option("-a", "--atomic", action="store_true",
                  help="Use atomic (non-timing) mode")
parser.add_option("-b", "--blocking", action="store_true",
                  help="Use blocking caches")
parser.add_option("-l", "--maxloads", default="1G", metavar="N",
                  help="Stop after N loads [default: %default]")
parser.add_option("-m", "--maxtick", type="int", default=m5.MaxTick,
                  metavar="T",
                  help="Stop after T ticks")
parser.add_option("-n", "--numtesters", type="int", default=8,
                  metavar="N",
                  help="Number of tester pseudo-CPUs [default: %default]")

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

# Should generalize this someday... would be cool to have a loop that
# just iterates, adding a level of caching each time.
#if options.cache_levels != 2 and options.cache_levels != 0:
#     print "Error: number of cache levels must be 0 or 2"
#     sys.exit(1)

if options.blocking:
     num_l1_mshrs = 1
     num_l2_mshrs = 1
else:
     num_l1_mshrs = 12
     num_l2_mshrs = 92

block_size = 64

# --------------------
# Base L1 Cache
# ====================

class L1(BaseCache):
    latency = '1ns'
    block_size = block_size
    mshrs = num_l1_mshrs
    tgts_per_mshr = 8

# ----------------------
# Base L2 Cache
# ----------------------

class L2(BaseCache):
    block_size = block_size
    latency = '10ns'
    mshrs = num_l2_mshrs
    tgts_per_mshr = 16
    write_buffers = 8

if options.numtesters > block_size:
     print "Error: Number of testers limited to %s because of false sharing" \
           % (block_size)
     sys.exit(1)

cpus = [ MemTest(atomic=options.atomic, max_loads=options.maxloads,
                 percent_functional=options.functional,
                 percent_uncacheable=options.uncacheable,
                 progress_interval=options.progress)
         for i in xrange(options.numtesters) ]

# system simulated
system = System(cpu = cpus, funcmem = PhysicalMemory(),
                physmem = PhysicalMemory(latency = "100ns"),
                membus = Bus(clock="500MHz", width=16))

# l2cache & bus
if options.cache_levels == 2:
    system.toL2Bus = Bus(clock="500MHz", width=16)
    system.l2c = L2(size='64kB', assoc=8)
    system.l2c.cpu_side = system.toL2Bus.port

    # connect l2c to membus
    system.l2c.mem_side = system.membus.port

# add L1 caches
for cpu in cpus:
    if options.cache_levels == 2:
         cpu.l1c = L1(size = '32kB', assoc = 4)
         cpu.test = cpu.l1c.cpu_side
         cpu.l1c.mem_side = system.toL2Bus.port
    elif options.cache_levels == 1:
         cpu.l1c = L1(size = '32kB', assoc = 4)
         cpu.test = cpu.l1c.cpu_side
         cpu.l1c.mem_side = system.membus.port
    else:
         cpu.test = system.membus.port
    system.funcmem.port = cpu.functional

# connect memory to membus
system.physmem.port = system.membus.port


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
