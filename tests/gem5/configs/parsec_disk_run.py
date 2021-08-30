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

import m5
import m5.ticks
from m5.objects import Root


from gem5.components.resources.resource import Resource
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.runtime import (
    get_runtime_isa,
    get_runtime_coherence_protocol,
)
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
    choices=("kvm", "timing", "atomic", "o3"),
    required=False,
    help="The CPU type to run before and after the ROI. If not specified will "
    "be equal to that of the CPU type used in the ROI.",
)

parser.add_argument(
    "-c",
    "--cpu",
    type=str,
    choices=("kvm", "timing", "atomic", "o3"),
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

parser.add_argument(
    "-o",
    "--override-download",
    action="store_true",
    help="Override a local resource if the hashes do not match.",
)

args = parser.parse_args()

# Setup the cachie hierarchy.

if args.mem_system == "classic":

    from gem5.components.cachehierarchies.classic.\
        private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )

    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32kB",
        l1i_size="32kB",
        l2_size="256kB",
    )
elif args.mem_system == "mesi_two_level":
    from gem5.components.cachehierarchies.ruby.\
        mesi_two_level_cache_hierarchy import (
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


def input_to_cputype(input: str) -> CPUTypes:
    if input == "kvm":
        return CPUTypes.KVM
    elif input == "timing":
        return CPUTypes.TIMING
    elif input == "atomic":
        return CPUTypes.ATOMIC
    elif input == "o3":
        return CPUTypes.O3
    else:
        raise NotADirectoryError("Unknown CPU type '{}'.".format(input))


roi_type = input_to_cputype(args.cpu)
if args.boot_cpu != None:
    boot_type = input_to_cputype(args.boot_cpu)
else:
    boot_type = roi_type


processor = SimpleSwitchableProcessor(
    starting_core_type=boot_type,
    switch_core_type=roi_type,
    num_cores=args.num_cpus,
)

# Setup the board.
board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    exit_on_work_items=True,
)

board.connect_things()

# The command to run.
command = (
    "cd /home/gem5/parsec-benchmark\n"
    + "source env.sh\n"
    + "parsecmgmt -a run -p {} ".format(args.benchmark)
    + "-c gcc-hooks -i {} ".format(args.size)
    + "-n {}\n".format(str(args.num_cpus))
    + "sleep 5 \n"
    + "m5 exit \n"
)

board.set_workload(
    kernel=Resource(
        "x86-linux-kernel-5.4.49",
        resource_directory=args.resource_directory,
        override=args.override_download,
    ),
    disk_image=Resource(
        "x86-parsec",
        resource_directory=args.resource_directory,
        override=args.override_download,
    ),
    command=command,
)

print("Running with ISA: " + get_runtime_isa().name)
print("Running with protocol: " + get_runtime_coherence_protocol().name)
print()

root = Root(full_system=True, system=board)

if args.cpu == "kvm" or args.boot_cpu == "kvm":
    # TODO: This of annoying. Is there a way to fix this to happen
    # automatically when running KVM?
    root.sim_quantum = int(1e9)

m5.instantiate()

globalStart = time.time()
print("Beginning the simulation")

start_tick = m5.curTick()
end_tick = m5.curTick()

m5.stats.reset()

exit_event = m5.simulate()

if exit_event.getCause() == "workbegin":
    print("Done booting Linux")
    # Reached the start of ROI.
    # The start of the ROI is marked by an m5_work_begin() call.
    print("Resetting stats at the start of ROI!")
    m5.stats.reset()
    start_tick = m5.curTick()

    # Switch to the Timing Processor.
    board.get_processor().switch()
else:
    print("Unexpected termination of simulation!")
    print("Cause: {}".format(exit_event.getCause()))
    print()

    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()
    print("Performance statistics:")
    print("Simulated time: {}s".format((end_tick - start_tick) / 1e12))
    print("Ran a total of", m5.curTick() / 1e12, "simulated seconds")
    print(
        "Total wallclock time: {}s, {} min".format(
            (
                time.time() - globalStart,
                (time.time() - globalStart) / 60,
            )
        )
    )
    exit(1)

# Simulate the ROI.
exit_event = m5.simulate()

if exit_event.getCause() == "workend":
    # Reached the end of ROI
    # The end of the ROI is marked by an m5_work_end() call.
    print("Dumping stats at the end of the ROI!")
    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()

    # Switch back to the Atomic Processor
    board.get_processor().switch()
else:
    print("Unexpected termination of simulation!")
    print("Cause: {}".format(exit_event.getCause()))
    print()
    m5.stats.dump()
    end_tick = m5.curTick()

    m5.stats.reset()
    print("Performance statistics:")
    print("Simulated time: {}s".format((end_tick - start_tick) / 1e12))
    print("Ran a total of", m5.curTick() / 1e12, "simulated seconds")
    print(
        "Total wallclock time: {}s, {} min".format(
            time.time() - globalStart,
            (time.time() - globalStart) / 60,
        )
    )
    exit(1)

# Simulate the remaning part of the benchmark
# Run the rest of the workload until m5 exit

exit_event = m5.simulate()

print("Done running the simulation")
print()
print("Performance statistics:")

print("Simulated time in ROI: {}s".format((end_tick - start_tick) / 1e12))
print("Ran a total of {} simulated seconds".format(m5.curTick() / 1e12))
print(
    "Total wallclock time: {}s, {} min".format(
        time.time() - globalStart, (time.time() - globalStart) / 60
    )
)
