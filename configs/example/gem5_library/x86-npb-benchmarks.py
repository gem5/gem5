# Copyright (c) 2021 The Regents of the University of California.
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
Script to run NAS parallel benchmarks with gem5. The script expects the
benchmark program to run. The input is in the format
<benchmark_prog>.<class>.x .The system is fixed with 2 CPU cores, MESI
Two Level system cache and 3 GB DDR4 memory. It uses the x86 board.

This script will count the total number of instructions executed
in the ROI. It also tracks how much wallclock and simulated time.

Usage:
------

```
scons build/X86/gem5.opt
./build/X86/gem5.opt \
    configs/example/gem5_library/x86-npb-benchmarks.py \
    --benchmark <benchmark_name> \
    --size <benchmark_class>
```
"""
import argparse
import time

import m5
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
from m5.objects import Root
from m5.stats.gem5stats import get_simstat
from m5.util import warn

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
    kvm_required=True,
)

# Following are the list of benchmark programs for npb.

benchmark_choices = ["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp"]

# We are restricting classes of NPB to A, B and C as the other classes (D and
# F) require main memory size of more than 3 GB. The X86Board is currently
# limited to 3 GB of memory. This limitation is explained later in line 136.

# The resource disk has binaries for class D. However, only `ep` benchmark
# works with class D in the current configuration. More information on the
# memory footprint for NPB is available at https://arxiv.org/abs/2010.13216

size_choices = ["A", "B", "C"]

parser = argparse.ArgumentParser(
    description="An example configuration script to run the npb benchmarks."
)

# The only positional argument accepted is the benchmark name in this script.

parser.add_argument(
    "--benchmark",
    type=str,
    required=True,
    help="Input the benchmark program to execute.",
    choices=benchmark_choices,
)

parser.add_argument(
    "--size",
    type=str,
    required=True,
    help="Input the class of the program to simulate.",
    choices=size_choices,
)

parser.add_argument(
    "--ticks",
    type=int,
    help="Optionally put the maximum number of ticks to execute during the "
    "ROI. It accepts an integer value.",
)

args = parser.parse_args()

# The simulation may fail in the case of `mg` with class C as it uses 3.3 GB
# of memory (more information is availabe at https://arxiv.org/abs/2010.13216).
# We warn the user here.

if args.benchmark == "mg" and args.size == "C":
    warn(
        "mg.C uses 3.3 GB of memory. Currently we are simulating 3 GB\
    of main memory in the system."
    )

# The simulation will fail in the case of `ft` with class C. We warn the user
# here.
elif args.benchmark == "ft" and args.size == "C":
    warn(
        "There is not enough memory for ft.C. Currently we are\
    simulating 3 GB of main memory in the system."
    )

# Checking for the maximum number of instructions, if provided by the user.

# Setting up all the fixed system parameters here
# Caches: MESI Two Level Cache Hierarchy

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
# Memory: Dual Channel DDR4 2400 DRAM device.
# The X86 board only supports 3 GB of main memory.

memory = DualChannelDDR4_2400(size="3GB")

# Here we setup the processor. This is a special switchable processor in which
# a starting core type and a switch core type must be specified. Once a
# configuration is instantiated a user may call `processor.switch()` to switch
# from the starting core types to the switch core types. In this simulation
# we start with KVM cores to simulate the OS boot, then switch to the Timing
# cores for the command we wish to run after boot.

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

# Here we setup the board. The X86Board allows for Full-System X86 simulations

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the FS workload, i.e., npb benchmark program
# After simulation has ended you may inspect
# `m5out/system.pc.com_1.device` to the stdout, if any.

# After the system boots, we execute the benchmark program and wait till the
# ROI `workbegin` annotation is reached (m5_work_begin()). We start collecting
# the number of committed instructions till ROI ends (marked by `workend`).
# We then finish executing the rest of the benchmark.

# Also, we sleep the system for some time so that the output is printed
# properly.

command = (
    f"/home/gem5/NPB3.3-OMP/bin/{args.benchmark}.{args.size}.x;"
    + "sleep 5;"
    + "m5 exit;"
)

board.set_kernel_disk_workload(
    # The x86 linux kernel will be automatically downloaded to the
    # `~/.cache/gem5` directory if not already present.
    # npb benchamarks was tested with kernel version 4.19.83
    kernel=obtain_resource("x86-linux-kernel-4.19.83"),
    # The x86-npb image will be automatically downloaded to the
    # `~/.cache/gem5` directory if not already present.
    disk_image=obtain_resource("x86-npb"),
    readfile_contents=command,
)

# The first exit_event ends with a `workbegin` cause. This means that the
# system started successfully and the execution on the program started.
def handle_workbegin():
    print("Done booting Linux")
    print("Resetting stats at the start of ROI!")

    m5.stats.reset()

    # We have completed up to this step using KVM cpu. Now we switch to timing
    # cpu for detailed simulation.

    # # Next, we need to check if the user passed a value for --ticks. If yes,
    # then we limit out execution to this number of ticks during the ROI.
    # Otherwise, we simulate until the ROI ends.
    processor.switch()
    if args.ticks:
        # schedule an exit event for this amount of ticks in the future.
        # The simulation will then continue.
        m5.scheduleTickExitFromCurrent(args.ticks)
    yield False


# The next exit_event is to simulate the ROI. It should be exited with a cause
# marked by `workend`.

# We exepect that ROI ends with `workend` or `simulate() limit reached`.
def handle_workend():
    print("Dump stats at the end of the ROI!")

    m5.stats.dump()
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        ExitEvent.WORKBEGIN: handle_workbegin(),
        ExitEvent.WORKEND: handle_workend(),
    },
)

# We maintain the wall clock time.

globalStart = time.time()

print("Running the simulation")
print("Using KVM cpu")

# We start the simulation.
simulator.run()

# We need to note that the benchmark is not executed completely till this
# point, but, the ROI has. We collect the essential statistics here before
# resuming the simulation again.

# Simulation is over at this point. We acknowledge that all the simulation
# events were successful.
print("All simulation events were successful.")
# We print the final simulation statistics.

print("Done with the simulation")
print()
print("Performance statistics:")

# manually calculate ROI time if ticks arg is used in case the
# entire ROI wasn't simulated
if args.ticks:
    print(f"Simulated time in ROI (to tick): {args.ticks/ 1e12}s")
else:
    print(f"Simulated time in ROI: {simulator.get_roi_ticks()[0] / 1e12}s")

print(
    f"Ran a total of {simulator.get_current_tick() / 1e12} simulated seconds"
)
print(
    "Total wallclock time: %.2fs, %.2f min"
    % (time.time() - globalStart, (time.time() - globalStart) / 60)
)
