# Copyright (c) 2023 The University of Edinburgh
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
#

"""
Fetch directed instruction prefetch (FDP) example

This gem5 configuation script creates a simple simulation setup
with a single O3 CPU model and decoupled front-end. Is serves as a
starting point for the FDP implementation.
As workload a simple "Hello World!" program is used.

FDP is tested with the X86, Arm, RiscV isa which can be specified by
with the `--isa` flag.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt \
    configs/example/gem5_library/fdp-hello.py \
    --isa <isa> \
    [--disable-fdp]
```
"""

import argparse

from m5.objects import (
    AssociativeBTB,
    LTAGE,
    TaggedPrefetcher,
    FetchDirectedPrefetcher,
    L2XBar,
)

from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.resources.resource import obtain_resource
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.classic.caches.l1icache import L1ICache
from gem5.components.cachehierarchies.classic.caches.mmu_cache import MMUCache
from gem5.components.cachehierarchies.classic.caches.l1dcache import L1DCache
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.base_cpu_processor import BaseCPUProcessor
from gem5.components.processors.simple_core import SimpleCore
from gem5.simulate.simulator import Simulator


isa_choices = {
    "X86": ISA.X86,
    "Arm": ISA.ARM,
    "RiscV": ISA.RISCV,
}

workloads = {
    "hello": {
        "Arm": "arm-hello64-static",
        "X86": "x86-hello64-static",
        "RiscV": "riscv-hello",
    },
}


parser = argparse.ArgumentParser(
    description="An example configuration script to run FDP."
)

# The only positional argument accepted is the benchmark name in this script.

parser.add_argument(
    "--isa",
    type=str,
    default="X86",
    help="The ISA to simulate.",
    choices=isa_choices.keys(),
)

parser.add_argument(
    "--workload",
    type=str,
    default="hello",
    help="The workload to simulate.",
    choices=workloads.keys(),
)

parser.add_argument(
    "--disable-fdp",
    action="store_true",
    help="Disable FDP to get evaluate baseline",
)

args = parser.parse_args()


# This check ensures the gem5 binary is compiled to the correct ISA target.
# If not, an exception will be thrown.
requires(isa_required=isa_choices[args.isa])

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MB")


## FDP needs the AssociativeBTB.
class BTB(AssociativeBTB):
    numEntries = "8kB"
    assoc = 4


class BPLTage(LTAGE):
    instShiftAmt = 0
    BTB = BTB()
    requiresBTBHit = True


# We need a custom cache hierarchy to incorporate the FDP prefetcher.
class CacheHierarchy(PrivateL1CacheHierarchy):
    def __init__(self, icache, dcache):
        super().__init__(l1i_size="", l1d_size="")
        self.icache = icache
        self.dcache = dcache

    def incorporate_cache(self, board: AbstractBoard) -> None:

        # Set up the system port for functional access from the simulator.
        board.connect_system_port(self.membus.cpu_side_ports)

        for _, port in board.get_memory().get_mem_ports():
            self.membus.mem_side_ports = port

        cpu = board.get_processor().get_cores()[0]

        # Create and connect the instruction cache
        cpu.connect_icache(self.icache.cpu_side)
        self.icache.mem_side = self.membus.cpu_side_ports

        # Also the data cache
        cpu.connect_dcache(self.dcache.cpu_side)
        self.dcache.mem_side = self.membus.cpu_side_ports

        # Finally the cache for the MMU page walks
        self.mmucache = MMUCache(size="8KiB")
        self.mmucache.mem_side = self.membus.cpu_side_ports
        self.mmubus = L2XBar(width=64)
        self.mmubus.mem_side_ports = self.mmucache.cpu_side
        cpu.connect_walker_ports(
            self.mmubus.cpu_side_ports, self.mmubus.cpu_side_ports
        )

        if board.has_coherent_io():
            self._setup_io_cache(board)

        ## Connect the interrupt ports
        if board.get_processor().get_isa() == ISA.X86:
            cpu.connect_interrupt(
                self.membus.mem_side_ports, self.membus.cpu_side_ports
            )
        else:
            cpu.connect_interrupt()


# 1. Decoupled front-end ------------------------------------------------
# First setup the decoupled front-end. Its implemented in the O3 core.
# Create the processor with one core
processor = SimpleProcessor(
    cpu_type=CPUTypes.O3, isa=isa_choices[args.isa], num_cores=1
)
cpu = processor.cores[0].core

# We need to configure the decoupled front-end with some specific parameters.
# First the fetch buffer and fetch target size. We want double the size of
# the fetch buffer to be able to run ahead of fetch
cpu.fetchBufferSize = 16
cpu.fetchTargetWidth = 32

# The decoupled front-end leverages the BTB to find branches in the fetch
# stream. Starting from the end of the last fetch target it will search
# all addresses until a hit. However, for fixed size instruction architectures
# like ARM only every n-th address must be checked.
if args.isa == "Arm":
    cpu.minInstSize = (
        4  # Note Arm has a 2 byte thumb mode but we ignore it here
    )
elif args.isa == "RiscV":
    cpu.minInstSize = 4  # RiscV has a 2 byte compressed mode.
else:  # Variable length ISA (x86) must search every byte
    cpu.minInstSize = 1


# Use the AssociativeBTB as its the only one that supports
# the decoupled front-end at the moment.
cpu.branchPred = BPLTage()

# Finally the `decoupledFrontEnd` parameter enables the decoupled front-end.
# Disable it to get the baseline.
if args.disable_fdp:
    cpu.decoupledFrontEnd = False
else:
    cpu.decoupledFrontEnd = True


print(
    "Running {} on {}, FDP {}".format(
        args.workload, args.isa, "disabled" if args.disable_fdp else "enabled"
    )
)


# 2. Instruction prefetcher ---------------------------------------------
# The decoupled front-end is only the first part.
# Now we also need the instruction prefetcher which listens to the
# insertions into the fetch target queue (FTQ) to issue prefetches.

# Create the icache and the prefetcher
icache = L1ICache(size="32kB")

if args.disable_fdp:
    # If FDP is disabled we use the TaggedPrefetcher (Next-Line Prefetcher)
    icache.prefetcher = TaggedPrefetcher()
else:
    ## Setup the FDP prefetcher
    icache.prefetcher = FetchDirectedPrefetcher(
        use_virtual_addresses=True,
        # The FDP prefetcher needs to know to which CPU to listent to.
        cpu=cpu,
    )

# Register the MMU to allow address translation
icache.prefetcher.registerMMU(processor.cores[0].core.mmu)

# Incorporate the icache into the cache hierarchy.
cache_hierarchy = CacheHierarchy(icache, L1DCache(size="32kB"))

# The gem5 library simble board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a simple "Hello World!"
# program compiled to the ARM ISA. The `Resource` class will automatically
# download the binary from the gem5 Resources cloud bucket if it's not already
# present.
board.set_se_binary_workload(
    # The `Resource` class reads the `resources.json` file from the gem5
    # resources repository:
    # https://gem5.googlesource.com/public/gem5-resource.
    # Any resource specified in this file will be automatically retrieved.
    # At the time of writing, this file is a WIP and does not contain all
    # resources. Jira ticket: https://gem5.atlassian.net/browse/GEM5-1096
    obtain_resource(workloads[args.workload][args.isa])
)


# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
