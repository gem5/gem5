# Copyright (c) 2024 ARM Limited
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

# This script showcases the functionality of cache partitioning policies,
# containg a simple system comprised of a memory requestor (TrafficGen),
# a cache enforcing policies for requests and a SimpleMemory backing store.
#
# Using the Way policy, the cache should show the following statistics in the
# provided configuration:
#
# | Allocated Ways | 1 | 2   | 3   | 4   | 5   | 6   | 7   | 8    |
# |----------------|---|-----|-----|-----|-----|-----|-----|------|
# | Cache Hits     | 0 | 256 | 384 | 512 | 640 | 768 | 896 | 1024 |
#
# Using the MaxCapacity policy, expected results are the following:
#
# | Allocation % | 10 | 20  | 30  | 40  | 50  | 60  | 70  | 80  | 90  | 100  |
# |--------------|----|-----|-----|-----|-----|-----|-----|-----|-----|------|
# | Cache Hits   | 0  | 152 | 307 | 409 | 512 | 614 | 716 | 819 | 921 | 1024 |

import argparse

import m5
from m5.objects import *


def capacityAllocation(capacity_str):
    """
    Verify that Max Capacity partitioning policy has been provided with a suitable
    configuration
    """
    capacity = float(capacity_str)

    if capacity > 1 or capacity < 0:
        raise argparse.ArgumentTypeError(
            "Max Capacity Policy needs allocation in range [0, 1]"
        )

    return capacity


def wayAllocation(way_str):
    """
    Verify that Way partitioning policy has been provided with a suitable
    configuration
    """
    way_alloc = int(way_str)

    if way_alloc < 0:
        raise argparse.ArgumentTypeError(
            "Way Policy needs positive number of ways"
        )

    return way_alloc


def generatePartPolicy(args):
    """
    Generate Partitioning Policy object based on provided arguments
    """
    assert args.policy in [
        "way",
        "max_capacity",
    ], "Only support generating way and max_capacity policies"

    if args.policy == "way":
        allocated_ways = [way for way in range(0, args.way_allocation)]
        allocation = WayPolicyAllocation(partition_id=0, ways=allocated_ways)

        return WayPartitioningPolicy(allocations=[allocation])

    if args.policy == "max_capacity":
        return MaxCapacityPartitioningPolicy(
            partition_ids=[0], capacities=[args.capacity_allocation]
        )


def configSystem():
    """
    Configure base system and memory
    """

    system = System(membus=IOXBar(width=128))
    system.clk_domain = SrcClockDomain(
        clock="10THz",
        voltage_domain=VoltageDomain(),
    )

    # Memory configuration
    system.mem_ctrl = SimpleMemory(bandwidth="1GiB/s", latency="10ns")

    # add memory
    system.mem_ctrl.range = AddrRange("64KiB")
    system.mem_ctrl.port = system.membus.mem_side_ports
    return system


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)

parser.add_argument(
    "--policy",
    default="way",
    choices=["way", "max_capacity"],
    help="This option defines which Cache Partitioning Policy to use for "
    "the system cache",
)

parser.add_argument(
    "--capacity-allocation",
    type=capacityAllocation,
    default=0.5,
    help="The amount of the cache to partition to the default PartitionID "
    "when using Max Capacity Cache Partitioning Policy in [0,1] range",
)

parser.add_argument(
    "--way-allocation",
    type=wayAllocation,
    default=4,
    help="The number of ways in the cache to partition to the default "
    "PartitionID when using Way Cache Partitioning Policy",
)

args = parser.parse_args()

m5.ticks.setGlobalFrequency("10THz")
system = configSystem()

# create a cache to sit between the memory and traffic gen to enforce
# partitioning policies
part_manager = PartitionManager(
    partitioning_policies=[generatePartPolicy(args)]
)
system.cache = NoncoherentCache(
    size="64KiB",
    assoc=8,
    partitioning_manager=part_manager,
    tag_latency=0,
    data_latency=0,
    response_latency=0,
    mshrs=1,
    tgts_per_mshr=8,
    write_buffers=1,
    replacement_policy=MRURP(),
)
system.cache.mem_side = system.membus.cpu_side_ports

# instantiate traffic gen and connect to crossbar
system.tgen = PyTrafficGen()
system.tgen.port = system.cache.cpu_side

# finalise config and run simulation
root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"
m5.instantiate()

# configure traffic generator to do 2x 64KiB sequential reads from address 0
# to 65536; one to warm up the cache one to test cache partitioning
linear_tgen = system.tgen.createLinear(
    1000000000, 0, 65536, 64, 1, 1, 100, 65536
)
exit_tgen = system.tgen.createExit(1)
system.tgen.start([linear_tgen, linear_tgen, exit_tgen])

# handle exit reporting
exit_event = m5.simulate(2000000000)
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
