# Copyright (c) 2023 The University of Edinburgh
# Copyright (c) 2025 Technical University of Munich
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

This gem5 configuation script creates a simple simulation setup with a single
O3 CPU model and decoupled front-end. Is serves as a starting point for the
FDP implementation. As workload a simple "Hello World!" program is used.

FDP is tested with the X86, Arm, RISC-V isa which can be specified
using the --isa flag.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt \
    configs/example/arm/fdp_neoverse_v2.py \
    --isa <isa> \
    [--disable-fdp]
```
"""

import argparse

import m5
from m5.util import addToPath

m5.util.addToPath("../..")

from common.cores.arm import neoverse_v2

from m5.objects import (
    TAGE_SC_L_64KB,
    BranchPredictor,
    FetchDirectedPrefetcher,
    L2XBar,
    MultiPrefetcher,
    SimpleBTB,
    TaggedPrefetcher,
)

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.caches.mmu_cache import MMUCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.base_cpu_core import BaseCPUCore
from gem5.components.processors.base_cpu_processor import BaseCPUProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    BinaryResource,
    obtain_resource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

workloads = {
    "hello": "arm-hello64-static",
}


parser = argparse.ArgumentParser(
    description="An example configuration script to run FDP."
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
    help="Disable FDP to evaluate baseline performance.",
)

args = parser.parse_args()


requires(isa_required=ISA.ARM)

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MiB")


# 1. Instruction prefetcher ---------------------------------------------
# The decoupled front-end is only the first part.
# Now we also need the instruction prefetcher which listens to the
# insertions into the fetch target queue (FTQ) to issue prefetches.


class CacheHierarchy(PrivateL1PrivateL2CacheHierarchy):
    def __init__(self):
        super().__init__("", "", "")

    def incorporate_cache(self, board: AbstractBoard) -> None:
        board.connect_system_port(self.membus.cpu_side_ports)

        for _, port in board.get_memory().get_mem_ports():
            self.membus.mem_side_ports = port

        self.l1icaches = [
            neoverse_v2.L1I()
            for i in range(board.get_processor().get_num_cores())
        ]

        # Add the prefetchers to the L1I caches and register the MMU.
        for i in range(board.get_processor().get_num_cores()):
            cpu = board.get_processor().cores[i].core

            self.l1icaches[i].prefetcher = MultiPrefetcher()
            if not args.disable_fdp:
                pf = FetchDirectedPrefetcher(
                    use_virtual_addresses=True, cpu=cpu
                )
                # Optionally register the cache to prefetch into to enable
                # cache snooping
                pf.registerCache(self.l1icaches[i])
                self.l1icaches[i].prefetcher.prefetchers.append(pf)

            self.l1icaches[i].prefetcher.prefetchers.append(
                TaggedPrefetcher(use_virtual_addresses=True)
            )

            for pf in self.l1icaches[i].prefetcher.prefetchers:
                pf.registerMMU(cpu.mmu)

        self.l1dcaches = [
            neoverse_v2.L1D()
            for i in range(board.get_processor().get_num_cores())
        ]
        self.l2buses = [
            L2XBar() for i in range(board.get_processor().get_num_cores())
        ]
        self.l2caches = [
            neoverse_v2.L2()
            for i in range(board.get_processor().get_num_cores())
        ]
        self.mmucaches = [
            MMUCache(size="8KiB")
            for _ in range(board.get_processor().get_num_cores())
        ]

        self.mmubuses = [
            L2XBar(width=64)
            for i in range(board.get_processor().get_num_cores())
        ]

        if board.has_coherent_io():
            self._setup_io_cache(board)

        for i, cpu in enumerate(board.get_processor().get_cores()):

            cpu.connect_icache(self.l1icaches[i].cpu_side)
            self.l1icaches[i].mem_side = self.l2buses[i].cpu_side_ports

            cpu.connect_dcache(self.l1dcaches[i].cpu_side)
            self.l1dcaches[i].mem_side = self.l2buses[i].cpu_side_ports

            self.mmucaches[i].mem_side = self.l2buses[i].cpu_side_ports

            self.mmubuses[i].mem_side_ports = self.mmucaches[i].cpu_side
            self.l2buses[i].mem_side_ports = self.l2caches[i].cpu_side

            self.membus.cpu_side_ports = self.l2caches[i].mem_side

            cpu.connect_walker_ports(
                self.mmubuses[i].cpu_side_ports,
                self.mmubuses[i].cpu_side_ports,
            )

            cpu.connect_interrupt()


cache_hierarchy = CacheHierarchy()


# 2. Decoupled front-end ------------------------------------------------
# Next setup the decoupled front-end. Its implemented in the O3 core.
# Create the processor with one core

processor = BaseCPUProcessor(
    cores=[BaseCPUCore(neoverse_v2.NeoverseV2(), isa=ISA.ARM)]
)

for core in processor.cores:
    cpu = core.core

    # The `decoupledFrontEnd` parameter enables the decoupled front-end.
    # Disable it to get the baseline.
    if args.disable_fdp:
        cpu.decoupledFrontEnd = False
    else:
        cpu.decoupledFrontEnd = True


print(
    f"Running {args.workload} on NeoverseV2 "
    f"FDP {'disabled' if args.disable_fdp else 'enabled'}"
)


# The gem5 library simple board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a simple "Hello World!"
# program compiled to the specified ISA. The `Resource` class will automatically
# download the binary from the gem5 Resources cloud bucket if it's not already
# present.
board.set_se_binary_workload(obtain_resource(workloads[args.workload]))

# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

print("Simulation done.")
