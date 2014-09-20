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

import optparse
import sys

import m5
from m5.objects import *

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

#
# The "tree" specification is a colon-separated list of one or more
# integers.  The first integer is the number of caches/testers
# connected directly to main memory.  The last integer in the list is
# the number of testers associated with the uppermost level of memory
# (L1 cache, if there are caches, or main memory if no caches).  Thus
# if there is only one integer, there are no caches, and the integer
# specifies the number of testers connected directly to main memory.
# The other integers (if any) specify the number of caches at each
# level of the hierarchy between.
#
# Examples:
#
#  "2:1"    Two caches connected to memory with a single tester behind each
#           (single-level hierarchy, two testers total)
#
#  "2:2:1"  Two-level hierarchy, 2 L1s behind each of 2 L2s, 4 testers total
#
parser.add_option("-t", "--treespec", type="string", default="8:1",
                  help="Colon-separated multilevel tree specification, "
                  "see script comments for details "
                  "[default: %default]")

parser.add_option("--force-bus", action="store_true",
                  help="Use bus between levels even with single cache")

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
parser.add_option("--sys-clock", action="store", type="string",
                  default='1GHz',
                  help = """Top-level clock for blocks running at system
                  speed""")

(options, args) = parser.parse_args()

if args:
     print "Error: script doesn't take any positional arguments"
     sys.exit(1)

block_size = 64

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
proto_l1 = BaseCache(size = '32kB', assoc = 4,
                     hit_latency = 1, response_latency = 1,
                     tgts_per_mshr = 8)

if options.blocking:
     proto_l1.mshrs = 1
else:
     proto_l1.mshrs = 4

# build a list of prototypes, one for each level of treespec, starting
# at the end (last entry is tester objects)
prototypes = [ MemTest(atomic=options.atomic, max_loads=options.maxloads,
                       percent_functional=options.functional,
                       percent_uncacheable=options.uncacheable,
                       progress_interval=options.progress) ]

# next comes L1 cache, if any
if len(treespec) > 1:
     prototypes.insert(0, proto_l1)

# now add additional cache levels (if any) by scaling L1 params
for scale in treespec[:-2]:
     # clone previous level and update params
     prev = prototypes[0]
     next = prev()
     next.size = prev.size * scale
     next.hit_latency = prev.hit_latency * 10
     next.response_latency = prev.response_latency * 10
     next.assoc = prev.assoc * scale
     next.mshrs = prev.mshrs * scale
     prototypes.insert(0, next)

# system simulated
system = System(funcmem = SimpleMemory(in_addr_map = False),
                funcbus = NoncoherentXBar(),
                physmem = SimpleMemory(latency = "100ns"),
                cache_line_size = block_size)


system.voltage_domain = VoltageDomain(voltage = '1V')

system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                        voltage_domain = system.voltage_domain)

def make_level(spec, prototypes, attach_obj, attach_port):
     fanout = spec[0]
     parent = attach_obj # use attach obj as config parent too
     if len(spec) > 1 and (fanout > 1 or options.force_bus):
          port = getattr(attach_obj, attach_port)
          new_bus = CoherentXBar(width=16)
          if (port.role == 'MASTER'):
               new_bus.slave = port
               attach_port = "master"
          else:
               new_bus.master = port
               attach_port = "slave"
          parent.cpu_side_bus = new_bus
          attach_obj = new_bus
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
               t.functional = system.funcbus.slave

make_level(treespec, prototypes, system.physmem, "port")

# connect reference memory to funcbus
system.funcbus.master = system.funcmem.port

# -----------------------
# run simulation
# -----------------------

root = Root( full_system = False, system = system )
if options.atomic:
    root.system.mem_mode = 'atomic'
else:
    root.system.mem_mode = 'timing'

# The system port is never used in the tester so merely connect it
# to avoid problems
root.system.system_port = root.system.funcbus.slave

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency('1ns')

# instantiate configuration
m5.instantiate()

# simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
