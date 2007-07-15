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

parser.add_option("-t", "--treespec", type="string",
                  help="Colon-separated multilevel tree specification")


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

if not options.treespec:
     # convert simple cache_levels option to treespec
     treespec = [options.numtesters, 1]
     numtesters = options.numtesters
else:
     try:
          treespec = [int(x) for x in options.treespec.split(':')]
          numtesters = reduce(lambda x,y: x*y, treespec)
     except:
          print "Error parsing treespec option"
          sys.exit(1)

if numtesters > block_size:
     print "Error: Number of testers limited to %s because of false sharing" \
           % (block_size)
     sys.exit(1)

if len(treespec) < 1:
     print "Error parsing treespec"
     sys.exit(1)

# define prototype L1 cache
proto_l1 = BaseCache(size = '32kB', assoc = 4, block_size = block_size,
                     latency = '1ns', tgts_per_mshr = 8)

if options.blocking:
     proto_l1.mshrs = 1
else:
     proto_l1.mshrs = 8

# build a list of prototypes, one for each cache level (L1 is at end,
# followed by the tester pseudo-cpu objects)
prototypes = [ proto_l1,
               MemTest(atomic=options.atomic, max_loads=options.maxloads,
                       percent_functional=options.functional,
                       percent_uncacheable=options.uncacheable,
                       progress_interval=options.progress) ]

while len(prototypes) < len(treespec):
     # clone previous level and update params
     prev = prototypes[0]
     next = prev()
     next.size = prev.size * 4
     next.latency = prev.latency * 10
     next.assoc = prev.assoc * 2
     prototypes.insert(0, next)

# system simulated
system = System(funcmem = PhysicalMemory(),
                physmem = PhysicalMemory(latency = "100ns"))

def make_level(spec, prototypes, attach_obj, attach_port):
     fanout = spec[0]
     parent = attach_obj # use attach obj as config parent too
     if fanout > 1:
          new_bus = Bus(clock="500MHz", width=16)
          new_bus.port = getattr(attach_obj, attach_port)
          parent.cpu_side_bus = new_bus
          attach_obj = new_bus
          attach_port = "port"
     objs = [prototypes[0]() for i in xrange(fanout)]
     if len(spec) > 1:
          # we just built caches, more levels to go
          parent.cache = objs
          for cache in objs:
               cache.mem_side = getattr(attach_obj, attach_port)
               make_level(spec[1:], prototypes[1:], cache, "cpu_side")
     else:
          # we just built the MemTest objects
          parent.cpu = objs
          for t in objs:
               t.test = getattr(attach_obj, attach_port)
               t.functional = system.funcmem.port

make_level(treespec, prototypes, system.physmem, "port")

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
