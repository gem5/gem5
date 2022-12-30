# Copyright (c) 2015-2016 ARM Limited
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

import argparse
import random
import sys

import m5
from m5.objects import *

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)


parser.add_argument(
    "-a", "--atomic", action="store_true", help="Use atomic (non-timing) mode"
)
parser.add_argument(
    "-b", "--blocking", action="store_true", help="Use blocking caches"
)
parser.add_argument(
    "-m",
    "--maxtick",
    type=int,
    default=m5.MaxTick,
    metavar="T",
    help="Stop after T ticks",
)
parser.add_argument(
    "-p", "--prefetchers", action="store_true", help="Use prefetchers"
)
parser.add_argument(
    "-s", "--stridepref", action="store_true", help="Use strided prefetchers"
)

# This example script has a lot in common with the memtest.py in that
# it is designed to stress tests the memory system. However, this
# script uses oblivious traffic generators to create the stimuli, and
# couples them with memcheckers to verify that the data read matches
# the allowed outcomes. Just like memtest.py, the traffic generators
# and checkers are placed in a tree topology. At the bottom of the
# tree is a shared memory, and then at each level a number of
# generators and checkers are attached, along with a number of caches
# that them selves fan out to subtrees of generators and caches. Thus,
# it is possible to create a system with arbitrarily deep cache
# hierarchies, sharing or no sharing of caches, and generators not
# only at the L1s, but also at the L2s, L3s etc.
#
# The tree specification consists of two colon-separated lists of one
# or more integers, one for the caches, and one for the
# testers/generators. The first integer is the number of
# caches/testers closest to main memory. Each cache then fans out to a
# subtree. The last integer in the list is the number of
# caches/testers associated with the uppermost level of memory. The
# other integers (if any) specify the number of caches/testers
# connected at each level of the crossbar hierarchy. The tester string
# should have one element more than the cache string as there should
# always be testers attached to the uppermost caches.
#
# Since this script tests actual sharing, there is also a possibility
# to stress prefetching and the interaction between prefetchers and
# caches. The traffic generators switch between random address streams
# and linear address streams to ensure that the prefetchers will
# trigger. By default prefetchers are off.

parser.add_argument(
    "-c",
    "--caches",
    type=str,
    default="3:2",
    help="Colon-separated cache hierarchy specification, "
    "see script comments for details ",
)
parser.add_argument(
    "-t",
    "--testers",
    type=str,
    default="1:0:2",
    help="Colon-separated tester hierarchy specification, "
    "see script comments for details ",
)
parser.add_argument(
    "-r",
    "--random",
    action="store_true",
    help="Generate a random tree topology",
)
parser.add_argument(
    "--sys-clock",
    action="store",
    type=str,
    default="1GHz",
    help="""Top-level clock for blocks running at system
                  speed""",
)

args = parser.parse_args()

# Start by parsing the command line args and do some basic sanity
# checking
if args.random:
    # Generate a tree with a valid number of testers
    tree_depth = random.randint(1, 4)
    cachespec = [random.randint(1, 3) for i in range(tree_depth)]
    testerspec = [random.randint(1, 3) for i in range(tree_depth + 1)]
    print(
        "Generated random tree -c",
        ":".join(map(str, cachespec)),
        "-t",
        ":".join(map(str, testerspec)),
    )
else:
    try:
        cachespec = [int(x) for x in args.caches.split(":")]
        testerspec = [int(x) for x in args.testers.split(":")]
    except:
        print("Error: Unable to parse caches or testers option")
        sys.exit(1)

    if len(cachespec) < 1:
        print("Error: Must have at least one level of caches")
        sys.exit(1)

    if len(cachespec) != len(testerspec) - 1:
        print("Error: Testers must have one element more than caches")
        sys.exit(1)

    if testerspec[-1] == 0:
        print("Error: Must have testers at the uppermost level")
        sys.exit(1)

    for t in testerspec:
        if t < 0:
            print("Error: Cannot have a negative number of testers")
            sys.exit(1)

    for c in cachespec:
        if c < 1:
            print("Error: Must have 1 or more caches at each level")
            sys.exit(1)

# Determine the tester multiplier for each level as the string
# elements are per subsystem and it fans out
multiplier = [1]
for c in cachespec:
    if c < 1:
        print("Error: Must have at least one cache per level")
    multiplier.append(multiplier[-1] * c)

numtesters = 0
for t, m in zip(testerspec, multiplier):
    numtesters += t * m

# Define a prototype L1 cache that we scale for all successive levels
proto_l1 = Cache(
    size="32kB",
    assoc=4,
    tag_latency=1,
    data_latency=1,
    response_latency=1,
    tgts_per_mshr=8,
)

if args.blocking:
    proto_l1.mshrs = 1
else:
    proto_l1.mshrs = 4

if args.prefetchers:
    proto_l1.prefetcher = TaggedPrefetcher()
elif args.stridepref:
    proto_l1.prefetcher = StridePrefetcher()

cache_proto = [proto_l1]

# Now add additional cache levels (if any) by scaling L1 params, the
# first element is Ln, and the last element L1
for scale in cachespec[:-1]:
    # Clone previous level and update params
    prev = cache_proto[0]
    next = prev()
    next.size = prev.size * scale
    next.tag_latency = prev.tag_latency * 10
    next.data_latency = prev.data_latency * 10
    next.response_latency = prev.response_latency * 10
    next.assoc = prev.assoc * scale
    next.mshrs = prev.mshrs * scale
    cache_proto.insert(0, next)

# Create a config to be used by all the traffic generators
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

# Make a prototype for the tester to be used throughout
proto_tester = TrafficGen(config_file=cfg_file_path)

# Set up the system along with a DRAM controller
system = System(physmem=MemCtrl(dram=DDR3_1600_8x8()))

system.voltage_domain = VoltageDomain(voltage="1V")

system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

system.memchecker = MemChecker()

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
    setattr(system, "l%dsubsys%d" % (level, index), subsys)

    # The levels are indexing backwards through the list
    ntesters = testerspec[len(cachespec) - level]

    testers = [proto_tester() for i in range(ntesters)]
    checkers = [
        MemCheckerMonitor(memchecker=system.memchecker)
        for i in range(ntesters)
    ]
    if ntesters:
        subsys.tester = testers
        subsys.checkers = checkers

    if level != 0:
        # Create a crossbar and add it to the subsystem, note that
        # we do this even with a single element on this level
        xbar = L2XBar(width=32)
        subsys.xbar = xbar
        if next_cache:
            xbar.mem_side_ports = next_cache.cpu_side

        # Create and connect the caches, both the ones fanning out
        # to create the tree, and the ones used to connect testers
        # on this level
        tree_caches = [prototypes[0]() for i in range(ncaches[0])]
        tester_caches = [proto_l1() for i in range(ntesters)]

        subsys.cache = tester_caches + tree_caches
        for cache in tree_caches:
            cache.mem_side = xbar.cpu_side_ports
            make_cache_level(ncaches[1:], prototypes[1:], level - 1, cache)
        for tester, checker, cache in zip(testers, checkers, tester_caches):
            tester.port = checker.cpu_side_port
            checker.mem_side_port = cache.cpu_side
            cache.mem_side = xbar.cpu_side_ports
    else:
        if not next_cache:
            print("Error: No next-level cache at top level")
            sys.exit(1)

        if ntesters > 1:
            # Create a crossbar and add it to the subsystem
            xbar = L2XBar(width=32)
            subsys.xbar = xbar
            xbar.mem_side_ports = next_cache.cpu_side
            for tester, checker in zip(testers, checkers):
                tester.port = checker.cpu_side_port
                checker.mem_side_port = xbar.cpu_side_ports
        else:
            # Single tester
            testers[0].port = checkers[0].cpu_side_port
            checkers[0].mem_side_port = next_cache.cpu_side


# Top level call to create the cache hierarchy, bottom up
make_cache_level(cachespec, cache_proto, len(cachespec), None)

# Connect the lowest level crossbar to the memory
last_subsys = getattr(system, "l%dsubsys0" % len(cachespec))
last_subsys.xbar.mem_side_ports = system.physmem.port
last_subsys.xbar.point_of_coherency = True

root = Root(full_system=False, system=system)
if args.atomic:
    root.system.mem_mode = "atomic"
else:
    root.system.mem_mode = "timing"

# The system port is never used in the tester so merely connect it
# to avoid problems
root.system.system_port = last_subsys.xbar.cpu_side_ports

# Instantiate configuration
m5.instantiate()

# Simulate until program terminates
exit_event = m5.simulate(args.maxtick)

print("Exiting @ tick", m5.curTick(), "because", exit_event.getCause())
