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
A run script for running the parsec benchmark suite in gem5.

Notes
-----

* This will download the PARSEC disk image if not found locally. This image is
  8 GB compressed, and 25 GB decompressed.
* This will only function for the X86 ISA.
"""

import m5.stats

from gem5.resources.resource import Resource
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.components.processors.cpu_types import (
    get_cpu_types_str_set,
    get_cpu_type_from_str,
)
from gem5.isas import ISA
from gem5.runtime import get_runtime_isa, get_runtime_coherence_protocol
from gem5.simulate.simulator import Simulator
from gem5.simulate.exit_event import ExitEvent
from gem5.utils.requires import requires

import time
import argparse

requires(isa_required=ISA.X86)


parser = argparse.ArgumentParser(
    description="A script to run the PARSEC benchmarks on a basic X86 full "
    "system."
)

parser.add_argument(
    "-n",
    "--num-cpus",
    type=int,
    choices=(1, 2, 8),
    required=True,
    help="The number of CPUs. Note: 1, 2, and 8 cores supported on KVM; 1 and "
    "2 supported on TimingSimpleCPU.",
)

parser.add_argument(
    "-b",
    "--boot-cpu",
    type=str,
    choices=get_cpu_types_str_set(),
    required=False,
    help="The CPU type to run before and after the ROI. If not specified will "
    "be equal to that of the CPU type used in the ROI.",
)

parser.add_argument(
    "-c",
    "--cpu",
    type=str,
    choices=get_cpu_types_str_set(),
    required=True,
    help="The CPU type used in the ROI.",
)

parser.add_argument(
    "-m",
    "--mem-system",
    type=str,
    choices=("classic", "mesi_two_level"),
    required=True,
    help="The memory system to be used",
)

parser.add_argument(
    "-e",
    "--benchmark",
    type=str,
    choices=(
        "blackscholes",
        "bodytrack",
        "canneal",
        "dedup",
        "facesim",
        "ferret",
        "fluidanimate",
        "freqmine",
        "raytrace",
        "streamcluster",
        "swaptions",
        "vips",
        "x264",
    ),
    required=True,
    help="The PARSEC benchmark to run.",
)

parser.add_argument(
    "-s",
    "--size",
    type=str,
    choices=("simsmall", "simmedium", "simlarge"),
    required=True,
    help="The size of the PARSEC benchmark input size.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

# Setup the cachie hierarchy.

if args.mem_system == "classic":

    from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )

    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32kB", l1i_size="32kB", l2_size="256kB"
    )
elif args.mem_system == "mesi_two_level":
    from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
        MESITwoLevelCacheHierarchy,
    )

    cache_hierarchy = MESITwoLevelCacheHierarchy(
        l1i_size="32kB",
        l1i_assoc=8,
        l1d_size="32kB",
        l1d_assoc=8,
        l2_size="256kB",
        l2_assoc=16,
        num_l2_banks=1,
    )

# Setup the memory system.
memory = SingleChannelDDR3_1600(size="3GB")

roi_type = get_cpu_type_from_str(args.cpu)
if args.boot_cpu != None:
    boot_type = get_cpu_type_from_str(args.boot_cpu)
else:
    boot_type = roi_type


processor = SimpleSwitchableProcessor(
    starting_core_type=boot_type,
    switch_core_type=roi_type,
    isa=ISA.X86,
    num_cores=args.num_cpus,
)

# Setup the board.
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# The command to run.
command = (
    "cd /home/gem5/parsec-benchmark\n"
    + "source env.sh\n"
    + "parsecmgmt -a run -p {} ".format(args.benchmark)
    + "-c gcc-hooks -i {} ".format(args.size)
    + "-n {}\n".format(str(args.num_cpus))
)

board.set_kernel_disk_workload(
    kernel=Resource(
        "x86-linux-kernel-5.4.49", resource_directory=args.resource_directory
    ),
    disk_image=Resource(
        "x86-parsec", resource_directory=args.resource_directory
    ),
    readfile_contents=command,
)

print("Running with ISA: " + get_runtime_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()


# Here we define some custom workbegin/workend exit event generators. Here we
# want to switch to detailed CPUs at the beginning of the ROI, then continue to
# the end of of the ROI. Then we exit the simulation.
def workbegin():
    processor.switch()
    yield False


def workend():
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        ExitEvent.WORKBEGIN: workbegin(),
        ExitEvent.WORKEND: workend(),
    },
)

global_start = time.time()
simulator.run()
global_end = time.time()
global_time = global_end - global_start

roi_ticks = simulator.get_roi_ticks()
assert len(roi_ticks) == 1


print("Done running the simulation")
print()
print("Performance statistics:")

print("Simulated time in ROI: {}s".format((roi_ticks[0]) / 1e12))
print(
    "Ran a total of {} simulated seconds".format(
        simulator.get_current_tick() / 1e12
    )
)
print(
    "Total wallclock time: {}s, {} min".format(global_time, (global_time) / 60)
)
