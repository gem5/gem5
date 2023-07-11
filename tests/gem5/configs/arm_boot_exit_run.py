# Copyright (c) 2022 The Regents of the University of California
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
This example runs a simple linux boot on the ArmBoard.

Characteristics
---------------

* Runs exclusively on the ARM ISA with the classic caches
"""

from gem5.isas import ISA
from m5.objects import ArmDefaultRelease
from gem5.utils.requires import requires
from gem5.resources.resource import Resource
from gem5.simulate.simulator import Simulator
from m5.objects import VExpress_GEM5_Foundation
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.arm_board import ArmBoard
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import (
    get_cpu_types_str_set,
    get_cpu_type_from_str,
    CPUTypes,
)

import argparse
import importlib

parser = argparse.ArgumentParser(
    description="A script to run the ARM boot exit tests."
)

parser.add_argument(
    "-n",
    "--num-cpus",
    type=int,
    required=True,
    help="The number of CPUs.",
)

parser.add_argument(
    "-c",
    "--cpu",
    type=str,
    choices=get_cpu_types_str_set(),
    required=True,
    help="The CPU type.",
)

parser.add_argument(
    "-m",
    "--mem-system",
    type=str,
    choices=("no_cache", "classic", "chi", "mesi_two_level", "mi_example"),
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
requires(isa_required=ISA.ARM)

if args.mem_system == "no_cache":
    from gem5.components.cachehierarchies.classic.no_cache import NoCache

    cache_hierarchy = NoCache()

elif args.mem_system == "classic":
    from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )

    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
    )

elif args.mem_system == "chi":
    requires(coherence_protocol_required=CoherenceProtocol.CHI)
    from gem5.components.cachehierarchies.chi.private_l1_cache_hierarchy import (
        PrivateL1CacheHierarchy,
    )

    cache_hierarchy = PrivateL1CacheHierarchy(
        size="16kB",
        assoc=4,
    )

elif args.mem_system == "mesi_two_level":
    requires(coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL)
    from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
        MESITwoLevelCacheHierarchy,
    )

    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size="32kB",
        l1d_assoc=8,
        l1i_size="32kB",
        l1i_assoc=8,
        l2_size="256kB",
        l2_assoc=16,
        num_l2_banks=2,
    )

elif args.mem_system == "mi_example":
    requires(coherence_protocol_required=CoherenceProtocol.MI_EXAMPLE)
    from gem5.components.cachehierarchies.ruby.mi_example_cache_hierarchy import (
        MIExampleCacheHierarchy,
    )

    cache_hierarchy = MIExampleCacheHierarchy(size="32kB", assoc=4)
else:
    raise NotImplementedError(
        f"Memory type '{args.mem_system}' is not supported in the boot tests."
    )

# Setup the system memory.
python_module = "gem5.components.memory"
memory_class = getattr(importlib.import_module(python_module), args.dram_class)
memory = memory_class(size="4GiB")

# Setup a processor.

cpu_type = get_cpu_type_from_str(args.cpu)

processor = SimpleProcessor(
    cpu_type=cpu_type, num_cores=args.num_cpus, isa=ISA.ARM
)


# The ArmBoard requires a `release` to be specified.

release = ArmDefaultRelease()

# The platform sets up the memory ranges of all the on-chip and off-chip
# devices present on the ARM system.

platform = VExpress_GEM5_Foundation()

# Setup the board.
board = ArmBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    release=release,
    platform=platform,
)

# Set the Full System workload.
board.set_kernel_disk_workload(
    kernel=Resource(
        "arm64-linux-kernel-5.4.49",
        resource_directory=args.resource_directory,
    ),
    bootloader=Resource(
        "arm64-bootloader-foundation",
        resource_directory=args.resource_directory,
    ),
    disk_image=Resource(
        "arm64-ubuntu-20.04-img",
        resource_directory=args.resource_directory,
    ),
)

simulator = Simulator(board=board)

if args.tick_exit:
    simulator.run(max_ticks=args.tick_exit)
else:
    simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(),
        simulator.get_last_exit_event_cause(),
    )
)
