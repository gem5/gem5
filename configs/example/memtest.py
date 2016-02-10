# Copyright (c) 2015 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
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
#          Andreas Hansson

import optparse
import random
import sys

import m5
from m5.objects import *

# This example script stress tests the memory system by creating false
# sharing in a tree topology. At the bottom of the tree is a shared
# memory, and then at each level a number of testers are attached,
# along with a number of caches that them selves fan out to subtrees
# of testers and caches. Thus, it is possible to create a system with
# arbitrarily deep cache hierarchies, sharing or no sharing of caches,
# and testers not only at the L1s, but also at the L2s, L3s etc.

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

# The tree specification consists of two colon-separated lists of one
# or more integers, one for the caches, and one for the testers. The
# first integer is the number of caches/testers closest to main
# memory. Each cache then fans out to a subtree. The last integer in
# the list is the number of caches/testers associated with the
# uppermost level of memory. The other integers (if any) specify the
# number of caches/testers connected at each level of the crossbar
# hierarchy. The tester string should have one element more than the
# cache string as there should always be testers attached to the
# uppermost caches.

parser.add_option("-c", "--caches", type="string", default="2:2:1",
                  help="Colon-separated cache hierarchy specification, "
                  "see script comments for details "
                  "[default: %default]")
parser.add_option("-t", "--testers", type="string", default="1:1:0:2",
                  help="Colon-separated tester hierarchy specification, "
                  "see script comments for details "
                  "[default: %default]")
parser.add_option("-f", "--functional", type="int", default=10,
                  metavar="PCT",
                  help="Target percentage of functional accesses "
                  "[default: %default]")
parser.add_option("-u", "--uncacheable", type="int", default=10,
                  metavar="PCT",
                  help="Target percentage of uncacheable accesses "
                  "[default: %default]")
parser.add_option("-r", "--random", action="store_true",
                  help="Generate a random tree topology")
parser.add_option("--progress", type="int", default=100000,
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

# Get the total number of testers
def numtesters(cachespec, testerspec):
     # Determine the tester multiplier for each level as the
     # elements are per subsystem and it fans out
     multiplier = [1]
     for c in cachespec:
          multiplier.append(multiplier[-1] * c)

     total = 0
     for t, m in zip(testerspec, multiplier):
          total += t * m

     return total

block_size = 64

# Start by parsing the command line options and do some basic sanity
# checking
if options.random:
     # Generate a tree with a valid number of testers
     while True:
          tree_depth = random.randint(1, 4)
          cachespec = [random.randint(1, 3) for i in range(tree_depth)]
          testerspec = [random.randint(1, 3) for i in range(tree_depth + 1)]
          if numtesters(cachespec, testerspec) < block_size:
               break

     print "Generated random tree -c", ':'.join(map(str, cachespec)), \
         "-t", ':'.join(map(str, testerspec))
else:
     try:
          cachespec = [int(x) for x in options.caches.split(':')]
          testerspec = [int(x) for x in options.testers.split(':')]
     except:
          print "Error: Unable to parse caches or testers option"
          sys.exit(1)

     if len(cachespec) < 1:
          print "Error: Must have at least one level of caches"
          sys.exit(1)

     if len(cachespec) != len(testerspec) - 1:
          print "Error: Testers must have one element more than caches"
          sys.exit(1)

     if testerspec[-1] == 0:
          print "Error: Must have testers at the uppermost level"
          sys.exit(1)

     for t in testerspec:
          if t < 0:
               print "Error: Cannot have a negative number of testers"
               sys.exit(1)

     for c in cachespec:
          if c < 1:
               print "Error: Must have 1 or more caches at each level"
               sys.exit(1)

     if numtesters(cachespec, testerspec) > block_size:
          print "Error: Limited to %s testers because of false sharing" \
              % (block_size)
          sys.exit(1)

# Define a prototype L1 cache that we scale for all successive levels
proto_l1 = Cache(size = '32kB', assoc = 4,
                 hit_latency = 1, response_latency = 1,
                 tgts_per_mshr = 8, clusivity = 'mostly_incl',
                 writeback_clean = True)

if options.blocking:
     proto_l1.mshrs = 1
else:
     proto_l1.mshrs = 4

cache_proto = [proto_l1]

# Now add additional cache levels (if any) by scaling L1 params, the
# first element is Ln, and the last element L1
for scale in cachespec[:-1]:
     # Clone previous level and update params
     prev = cache_proto[0]
     next = prev()
     next.size = prev.size * scale
     next.hit_latency = prev.hit_latency * 10
     next.response_latency = prev.response_latency * 10
     next.assoc = prev.assoc * scale
     next.mshrs = prev.mshrs * scale

     # Swap the inclusivity/exclusivity at each level. L2 is mostly
     # exclusive with respect to L1, L3 mostly inclusive, L4 mostly
     # exclusive etc.
     next.writeback_clean = not prev.writeback_clean
     if (prev.clusivity.value == 'mostly_incl'):
          next.clusivity = 'mostly_excl'
     else:
          next.clusivity = 'mostly_incl'

     cache_proto.insert(0, next)

# Make a prototype for the tester to be used throughout
proto_tester = MemTest(max_loads = options.maxloads,
                       percent_functional = options.functional,
                       percent_uncacheable = options.uncacheable,
                       progress_interval = options.progress)

# Set up the system along with a simple memory and reference memory
system = System(physmem = SimpleMemory(),
                cache_line_size = block_size)

system.voltage_domain = VoltageDomain(voltage = '1V')

system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                        voltage_domain = system.voltage_domain)

# For each level, track the next subsys index to use
next_subsys_index = [0] * (len(cachespec) + 1)

# Recursive function to create a sub-tree of the cache and tester
# hierarchy
def make_cache_level(ncaches, prototypes, level, next_cache):
     global next_subsys_index, proto_l1, testerspec, proto_tester

     index = next_subsys_index[level]
     next_subsys_index[level] += 1

     # Create a subsystem to contain the crossbar and caches, and
     # any testers
     subsys = SubSystem()
     setattr(system, 'l%dsubsys%d' % (level, index), subsys)

     # The levels are indexing backwards through the list
     ntesters = testerspec[len(cachespec) - level]

     # Scale the progress threshold as testers higher up in the tree
     # (smaller level) get a smaller portion of the overall bandwidth,
     # and also make the interval of packet injection longer for the
     # testers closer to the memory (larger level) to prevent them
     # hogging all the bandwidth
     limit = (len(cachespec) - level + 1) * 100000000
     testers = [proto_tester(interval = 10 * (level * level + 1),
                             progress_check = limit) \
                     for i in xrange(ntesters)]
     if ntesters:
          subsys.tester = testers

     if level != 0:
          # Create a crossbar and add it to the subsystem, note that
          # we do this even with a single element on this level
          xbar = L2XBar()
          subsys.xbar = xbar
          if next_cache:
               xbar.master = next_cache.cpu_side

          # Create and connect the caches, both the ones fanning out
          # to create the tree, and the ones used to connect testers
          # on this level
          tree_caches = [prototypes[0]() for i in xrange(ncaches[0])]
          tester_caches = [proto_l1() for i in xrange(ntesters)]

          subsys.cache = tester_caches + tree_caches
          for cache in tree_caches:
               cache.mem_side = xbar.slave
               make_cache_level(ncaches[1:], prototypes[1:], level - 1, cache)
          for tester, cache in zip(testers, tester_caches):
               tester.port = cache.cpu_side
               cache.mem_side = xbar.slave
     else:
          if not next_cache:
               print "Error: No next-level cache at top level"
               sys.exit(1)

          if ntesters > 1:
               # Create a crossbar and add it to the subsystem
               xbar = L2XBar()
               subsys.xbar = xbar
               xbar.master = next_cache.cpu_side
               for tester in testers:
                    tester.port = xbar.slave
          else:
               # Single tester
               testers[0].port = next_cache.cpu_side

# Top level call to create the cache hierarchy, bottom up
make_cache_level(cachespec, cache_proto, len(cachespec), None)

# Connect the lowest level crossbar to the memory
last_subsys = getattr(system, 'l%dsubsys0' % len(cachespec))
last_subsys.xbar.master = system.physmem.port
last_subsys.xbar.point_of_coherency = True

root = Root(full_system = False, system = system)
if options.atomic:
    root.system.mem_mode = 'atomic'
else:
    root.system.mem_mode = 'timing'

# The system port is never used in the tester so merely connect it
# to avoid problems
root.system.system_port = last_subsys.xbar.slave

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(options.maxtick)

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()
