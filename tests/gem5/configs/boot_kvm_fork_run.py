# Copyright (c) 2021 The University of Texas at Austin
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
# Author: Austin Harris
#

"""
This script tests forking gem5 with the KVM cores and switching cores in the
child process. First, the test boots linux with KVM and tests fast-forwarding
with instruction exit events. Then the test forks the simulation, waits for the
child to simulate until completion, and then simulates to completion in the
parent process.
"""

import argparse
import os
import sys
from textwrap import dedent

import m5
from m5.objects import Root

from gem5.components.boards.x86_board import X86Board
from gem5.coherence_protocol import CoherenceProtocol
from gem5.isas import ISA
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import (
    CPUTypes,
    get_cpu_types_str_set,
    get_cpu_type_from_str,
)
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.resources.resource import Resource
from gem5.runtime import get_runtime_coherence_protocol
from gem5.utils.requires import requires

parser = argparse.ArgumentParser(
    description="A script to test forking gem5 and switching cpus."
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
    default=4,
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
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)
parser.add_argument(
    "-k",
    "--kernel-args",
    type=str,
    default="init=/root/gem5_init.sh",
    help="Additional kernel boot arguments.",
)
parser.add_argument(
    "-f",
    "--num-forks",
    type=int,
    default=4,
    help="The number of times to fork gem5.",
)

args = parser.parse_args()

coherence_protocol_required = None
if args.mem_system == "mi_example":
    coherence_protocol_required = CoherenceProtocol.MI_EXAMPLE
elif args.mem_system == "mesi_two_level":
    coherence_protocol_required = CoherenceProtocol.MESI_TWO_LEVEL

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=coherence_protocol_required,
    kvm_required=True,
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
        "Memory system '{}' is not supported in the boot tests.".format(
            args.mem_system
        )
    )

assert cache_hierarchy != None

# Setup the system memory.

memory = SingleChannelDDR3_1600(size="3GB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=get_cpu_type_from_str(args.cpu),
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

kernel_args = motherboard.get_default_kernel_args() + [args.kernel_args]

# Set the Full System workload.
motherboard.set_kernel_disk_workload(
    kernel=Resource(
        "x86-linux-kernel-5.4.49", resource_directory=args.resource_directory
    ),
    disk_image=Resource(
        "x86-ubuntu-18.04-img", resource_directory=args.resource_directory
    ),
    readfile_contents=dedent(
        """
        m5 exit # signal end of boot
        m5 exit # exit in children and parent
        """
    ),
    kernel_args=kernel_args,
)


# Begin running of the simulation. This will exit once the Linux system boot
# is complete.
print("Running with ISA: " + processor.get_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()

root = Root(full_system=True, system=motherboard)

# TODO: This of annoying. Is there a way to fix this to happen
# automatically when running KVM?
root.sim_quantum = int(1e9)

# Disable the gdb ports. Required for forking.
m5.disableAllListeners()
motherboard._pre_instantiate()
m5.instantiate()

# Simulate the inital boot with the starting KVM cpu
exit_event = m5.simulate()
print("Boot finished", exit_event.getCause())

print("Starting fork and switch processors test")
pids = []
for i in range(args.num_forks):
    pid = m5.fork("%(parent)s/" + str(m5.curTick()))
    if pid == 0:  # in child
        print(f"Switching processors in child {i}.")
        processor.switch()
        exit_event = m5.simulate()
        if exit_event.getCause() != "m5_exit instruction encountered":
            raise Exception(f"Expected m5 exit, got {exit_event.getCause()}")
        print("Child finished, exiting: ", exit_event.getCause())
        sys.exit(0)
    else:
        pids.append(pid)

print("Waiting for children...")
for pid in pids:
    print(os.waitpid(pid, 0))

print("Children finished! Running to completion in parent.")
exit_event = m5.simulate()
if exit_event.getCause() != "m5_exit instruction encountered":
    raise Exception(f"Expected m5 exit, got {exit_event.getCause()}")
