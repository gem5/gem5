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
from m5.objects import Root

import sys
import os

# This is a lame hack to get the imports working correctly.
# TODO: This needs fixed.
sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.pardir,
        os.pardir,
        os.pardir,
    )
)

from components_library.runtime import (
    get_runtime_coherence_protocol,
    get_runtime_isa,
)
from components_library.boards.x86_board import X86Board
from components_library.memory.single_channel import SingleChannelDDR3_1600
from components_library.processors.simple_processor import SimpleProcessor
from components_library.processors.cpu_types import CPUTypes
from components_library.isas import ISA
from components_library.coherence_protocol import CoherenceProtocol
from components_library.resources.resource import Resource

import os
import argparse

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
    choices=("kvm", "atomic", "timing", "o3"),
    required=True,
    help="The CPU type.",
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

parser.add_argument(
    "-o",
    "--override-download",
    action="store_true",
    help="Override a local resource if the hashes do not match.",
)

args = parser.parse_args()

# We check the gem5 binary is correct at runtime runtime.
if get_runtime_isa() != ISA.X86:
    raise EnvironmentError("The boot-exit tests must be run with the X86 ISA.")

if (
    args.mem_system == "mi_example"
    and get_runtime_coherence_protocol() != CoherenceProtocol.MI_EXAMPLE
):
    raise EnvironmentError("gem5 binary not compiled to MI_EXAMPLE.")

if (
    args.mem_system == "mesi_two_level"
    and get_runtime_coherence_protocol() != CoherenceProtocol.MESI_TWO_LEVEL
):
    raise EnvironmentError("gem5 binary not compiled to MESI_Two_Level.")

cache_hierarchy = None
if args.mem_system == "mi_example":
    from components_library.cachehierarchies.ruby.\
        mi_example_cache_hierarchy import (
        MIExampleCacheHierarchy,
    )

    cache_hierarchy = MIExampleCacheHierarchy(size="32kB", assoc=8)
elif args.mem_system == "mesi_two_level":
    from components_library.cachehierarchies.ruby.\
        mesi_two_level_cache_hierarchy import (
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
    from components_library.cachehierarchies.classic.\
        private_l1_cache_hierarchy import (
        PrivateL1CacheHierarchy,
    )

    cache_hierarchy = PrivateL1CacheHierarchy(l1d_size="16kB", l1i_size="16kB")
else:
    raise NotImplementedError(
        "Memory system '{}' is not supported in the boot tests.".format(
            args.mem_system
        )
    )

assert cache_hierarchy != None

# Setup the system memory.
# Warning: This must be kept at 3GB for now. X86Motherboard does not support
# anything else right now!
memory = SingleChannelDDR3_1600(size="3GB")

# Setup a Processor.

cpu_type = None
if args.cpu == "kvm":
    cpu_type = CPUTypes.KVM
elif args.cpu == "atomic":
    cpu_type = CPUTypes.ATOMIC
elif args.cpu == "timing":
    cpu_type = CPUTypes.TIMING
elif args.cpu == "o3":
    cpu_type = CPUTypes.O3
else:
    raise NotImplementedError(
        "CPU type '{}' is not supported in the boot tests.".format(args.cpu)
    )

assert cpu_type != None

processor = SimpleProcessor(cpu_type=cpu_type, num_cores=args.num_cpus)

# Setup the motherboard.
motherboard = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    exit_on_work_items=True,
)

motherboard.connect_things()

additional_kernal_args = []
if args.boot_type == "init":
    additional_kernal_args.append("init=/root/exit.sh")

# Set the Full System workload.
motherboard.set_workload(
    kernel=Resource(
        "x86-linux-kernel-5.4.49",
        override=args.override_download,
        resource_directory=args.resource_directory,
    ),
    disk_image=Resource(
        "x86-boot-exit",
        override=args.override_download,
        resource_directory=args.resource_directory,
    ),
    command="m5 exit \n",
    kernel_args=additional_kernal_args,
)


# Begin running of the simulation. This will exit once the Linux system boot
# is complete.
print("Running with ISA: " + get_runtime_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()

root = Root(full_system=True, system=motherboard)

m5.instantiate()

print("Beginning simulation!")
if args.tick_exit != None:
    exit_event = m5.simulate(args.tick_exit)
else:
    exit_event = m5.simulate()
print(
    "Exiting @ tick {} because {}.".format(m5.curTick(), exit_event.getCause())
)
