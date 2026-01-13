# Copyright (c) 2021-2025 The Regents of the University of California.
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
Script to run NAS parallel benchmarks with gem5. The script expects a benchmark
program and a benchmark size to run. The system is fixed with 2 CPU cores,
MESI Two Level system cache and 3 GiB DDR4 memory. It uses the X86Board.

Usage:
------

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt \
    configs/example/gem5_library/x86-npb-benchmarks.py \
    --benchmark <benchmark_name> \
    --size <benchmark_class>
```
"""

import argparse

import m5
from m5.util import warn

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    WorkBeginExitHandler,
    WorkEndExitHandler,
)
from gem5.simulate.simulator import (
    Simulator,
)
from gem5.utils.override import overrides
from gem5.utils.requires import requires

requires(
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
    kvm_required=True,
)

# We use argparse to select the NPB benchmark and size.

parser = argparse.ArgumentParser(
    description="An example configuration script to run the NPB benchmarks."
)

parser.add_argument(
    "--benchmark",
    type=str,
    required=True,
    help="Input the benchmark program to execute.",
    choices=["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp", "ua"],
)

parser.add_argument(
    "--size",
    type=str,
    required=True,
    help="Input the benchmark size to use.",
    choices=["s", "a", "b", "c", "d"],
)

args = parser.parse_args()


# The simulation may fail when using size "c" and size "d" of the
# benchmarks. This is because the X86Board is currently limited to 3 GB of
# memory.
# We warn the user here.

if args.size.endswith("c") or args.size.endswith("d"):
    warn(
        f"The X86Board is currently limited to 3 GB of memory. The benchmark "
        f"{args.benchmark}, size {args.size} may fail to run."
    )

# Setting up all the fixed system parameters here
# Caches: MESI Two Level Cache Hierarchy

from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)

cache_hierarchy = MESITwoLevelCacheHierarchy(
    l1d_size="32KiB",
    l1d_assoc=8,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="256KiB",
    l2_assoc=16,
    num_l2_banks=2,
)
# Memory: Dual Channel DDR4 2400 DRAM device.
# The X86 board only supports 3 GiB of main memory.

memory = DualChannelDDR4_2400(size="3GiB")

# Here we set up the processor. This is a special switchable processor in which
# a starting core type and a switch core type must be specified. Once a
# configuration is instantiated a user may call `processor.switch()` or
# `simulator.switch_processor()`, if using a hypercall exit handler, to switch
# from the starting core types to the switch core types. In this simulation
# we start with KVM cores to simulate the OS boot, then switch to TIMING cores
# for the command we wish to run after boot, which in this case runs the NPB
# benchmark.

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

# Here we set up the board. The X86Board allows for FS mode (full system) or
# SE mode (syscall emulation) X86 simulations.

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the FS workload, i.e., NPB benchmark program.
# You may inspect `m5out/board.pc.com_1.device` to see the stdout of the
# simulated system.

board.set_workload(
    obtain_resource(
        f"x86-ubuntu-24.04-npb-{args.benchmark}-{args.size}",
        resource_version="3.0.0",
    )
)


# After the system boots, we execute the benchmark until we reach the beginning
# of the ROI, marked by a call to `m5_hypercall_addr(4)` in the benchmark on
# the disk image. We reset stats and switch to TIMING cores to simulate the ROI
# in more detail. Once we encounter the end of the ROI, marked by
# `m5_hypercall_addr(5)`, we dump stats and exit the simulation.
class CustomWorkBeginExitHandler(WorkBeginExitHandler):
    # The default behavior on work begin is to reset stats via
    # m5.stats.reset() and continue simulation. We override `_process`
    # so we can also switch processors.
    @overrides(WorkBeginExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        print("Done booting Linux")
        print("Resetting stats at the start of ROI!")
        m5.stats.reset()
        simulator.switch_processor()


class CustomWorkEndExitHandler(WorkEndExitHandler):
    @overrides(WorkEndExitHandler)
    def _process(self, simulator):
        super()._process(simulator)
        print("Dump stats at the end of the ROI!")
        m5.stats.dump()

    @overrides(WorkEndExitHandler)
    def _exit_simulation(self) -> bool:
        return True


simulator = Simulator(board=board)

print("Running the simulation")
print("Using KVM cpu")

# We start the simulation.
simulator.run()
