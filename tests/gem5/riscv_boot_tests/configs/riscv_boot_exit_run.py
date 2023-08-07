# Copyright (c) 2021 The Regents of the University of California
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

"""
This example runs a simple linux boot on the RiscvBoard.

Characteristics
---------------

* Runs exclusively on the RISC-V ISA with the classic caches
"""

from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.resources.resource import Resource
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.simulate.simulator import Simulator
from gem5.resources.workload import Workload

import argparse
import importlib

parser = argparse.ArgumentParser(
    description="A script to run the RISCV boot exit tests."
)

parser.add_argument(
    "-n", "--num-cpus", type=int, required=True, help="The number of CPUs."
)

parser.add_argument(
    "-c",
    "--cpu",
    type=str,
    choices=("kvm", "atomic", "timing", "o3", "minor"),
    required=True,
    help="The CPU type.",
)

parser.add_argument(
    "-m",
    "--mem-system",
    type=str,
    choices=("classic", "mesi_two_level"),
    required=True,
    help="The memory system.",
)

parser.add_argument(
    "-d",
    "--dram-class",
    type=str,
    required=False,
    default="DualChannelDDR3_1600",
    help="The python class for the memory interface to use",
)

parser.add_argument(
    "-t",
    "--tick-exit",
    type=int,
    required=False,
    help="The tick to exit the simulation.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

# Run a check to ensure the right version of gem5 is being used.
requires(isa_required=ISA.RISCV)

if args.mem_system == "classic":
    from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )

    # Setup the cache hierarchy.
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
    )
elif args.mem_system == "mesi_two_level":
    from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
        MESITwoLevelCacheHierarchy,
    )

    # Setup the cache hierarchy.
    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size="16kB",
        l1d_assoc=8,
        l1i_size="16kB",
        l1i_assoc=8,
        l2_size="256kB",
        l2_assoc=16,
        num_l2_banks=1,
    )

# Setup the system memory.
python_module = "gem5.components.memory"
memory_class = getattr(importlib.import_module(python_module), args.dram_class)
memory = memory_class(size="4GiB")

# Setup a processor.
if args.cpu == "kvm":
    cpu_type = CPUTypes.KVM
elif args.cpu == "atomic":
    cpu_type = CPUTypes.ATOMIC
elif args.cpu == "timing":
    cpu_type = CPUTypes.TIMING
elif args.cpu == "o3":
    cpu_type = CPUTypes.O3
elif args.cpu == "minor":
    cpu_type = CPUTypes.MINOR
else:
    raise NotImplementedError(
        f"CPU type '{args.cpu}' is not supported in the boot tests."
    )

processor = SimpleProcessor(
    cpu_type=cpu_type, isa=ISA.RISCV, num_cores=args.num_cpus
)

# Setup the board.
board = RiscvBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload.
workload = Workload(
    "riscv-ubuntu-20.04-boot", resource_directory=args.resource_directory
)
board.set_workload(workload)


simulator = Simulator(board=board)

if args.tick_exit:
    simulator.run(max_ticks=args.tick_exit)
else:
    simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
