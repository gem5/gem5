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
This script will run a simple boot exit test.
"""

import m5

from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.components.boards.x86_board import X86Board
from gem5.components.processors.cpu_types import (
    get_cpu_types_str_set,
    get_cpu_type_from_str,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.simulate.simulator import Simulator
from gem5.resources.workload import Workload

import argparse
import importlib


parser = argparse.ArgumentParser(
    description="A script to run the gem5 boot test. This test boots the "
    "linux kernel."
)
parser.add_argument(
    "-m",
    "--mem-system",
    type=str,
    choices=("classic", "mi_example", "mesi_two_level"),
    required=True,
    help="The memory system.",
)
parser.add_argument(
    "-n",
    "--num-cpus",
    type=int,
    choices=(1, 2, 4, 8),
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
    "-d",
    "--dram-class",
    type=str,
    required=False,
    default="DualChannelDDR3_1600",
    help="The python class for the memory interface to use",
)
parser.add_argument(
    "-b",
    "--boot-type",
    type=str,
    choices=("systemd", "init"),
    required=True,
    help="The boot type.",
)

parser.add_argument(
    "-t",
    "--tick-exit",
    type=int,
    required=False,
    help="The tick to exit the simulation. Note: using this may make the "
    "selected boot-type selection pointless.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

coherence_protocol_required = None
if args.mem_system == "mi_example":
    coherence_protocol_required = "MI_example"
elif args.mem_system == "mesi_two_level":
    coherence_protocol_required = "MESI_Two_Level"

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=coherence_protocol_required,
    kvm_required=(args.cpu == "kvm"),
)

cache_hierarchy = None
if args.mem_system == "mi_example":
    from gem5.components.cachehierarchies.ruby.mi_example_cache_hierarchy import (
        MIExampleCacheHierarchy,
    )

    cache_hierarchy = MIExampleCacheHierarchy(size="32kB", assoc=8)
elif args.mem_system == "mesi_two_level":
    from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
        MESITwoLevelCacheHierarchy,
    )

    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1d_size="16kB",
        l1d_assoc=8,
        l1i_size="16kB",
        l1i_assoc=8,
        l2_size="256kB",
        l2_assoc=16,
        num_l2_banks=1,
    )
elif args.mem_system == "classic":
    from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
        PrivateL1CacheHierarchy,
    )

    cache_hierarchy = PrivateL1CacheHierarchy(l1d_size="16kB", l1i_size="16kB")
else:
    raise NotImplementedError(
        f"Memory system '{args.mem_system}' is not supported in the boot tests."
    )

assert cache_hierarchy != None

# Setup the system memory.
# Warning: This must be kept at 3GB for now. X86Motherboard does not support
# anything else right now!
python_module = "gem5.components.memory"
memory_class = getattr(importlib.import_module(python_module), args.dram_class)
memory = memory_class(size="3GiB")

# Setup a Processor.
processor = SimpleProcessor(
    cpu_type=get_cpu_type_from_str(args.cpu),
    isa=ISA.X86,
    num_cores=args.num_cpus,
)

# Setup the motherboard.
motherboard = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

kernal_args = motherboard.get_default_kernel_args()
if args.boot_type == "init":
    kernal_args.append("init=/root/exit.sh")

# Set the workload.
workload = Workload(
    "x86-ubuntu-18.04-boot", resource_directory=args.resource_directory
)
workload.set_parameter("kernel_args", kernal_args)
motherboard.set_workload(workload)

# Begin running of the simulation. This will exit once the Linux system boot
# is complete.
print("Running with ISA: " + processor.get_isa().name)
print("Running with protocol: " + type(cache_hierarchy).__name__)
print()

print("Beginning simulation!")
simulator = Simulator(board=motherboard)

if args.tick_exit:
    simulator.run(max_ticks=args.tick_exit)
else:
    simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
