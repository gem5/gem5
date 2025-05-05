# Copyright (c) 2024 The Regents of the University of California
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

This script demonstrates how to use global and local instruction trackers to
monitor and control the simulation based on the number of instructions
committed.

This script will create a global instruction tracker to manage each local
instruction tracker and connects each local instruction tracker to a core.
The global instruction tracker will raise an event when the number of
instructions committed across all cores reaches a certain threshold.

In this script, we expect to start monitorning the instruction committed from
the start of the simulation and raise a SIMPOINT_BEGIN exit event when all
cores in combination have committed 100,000,000 instructions. Then, we will
change the threshold to 20,000 instructions and raise another SIMPOINT_BEGIN
event when the new threshold is reached. Finally, we will stop listening to
instructions.

Usage:
------

scons build/X86/gem5.opt
./build/X86/gem5.opt [--debug-flags=InstTracker] \
    configs/example/gem5_library/x86-global-inst-tracker.py

"""

import m5
from m5.objects import (
    GlobalInstTracker,
    LocalInstTracker,
)

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

cache_hierarchy = PrivateL1CacheHierarchy(
    l1d_size="64kB",
    l1i_size="64kB",
)

memory = SingleChannelDDR4_2400("1GB")

# using 9 cores because when using openmp in SE mode, the number of cores
# should be # cores + 1
processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, num_cores=9, isa=ISA.X86)

# setup instruction tracker

# first, we need to create a global instruction tracker
global_inst_tracker = GlobalInstTracker(
    # a list of thresholds to trigger the event
    inst_thresholds=[100_000_000, 100_020_000]
)

# then, we create a local instruction tracker for each core
# and store them in a list
all_trackers = []

for core in processor.get_cores():
    tracker = LocalInstTracker(
        # we pass in the glboal instruction tracker to the local one
        global_inst_tracker=global_inst_tracker,
        # this parameter tells the tracker to start listening to instructions
        # from the beginning of the simulation. If set to False, the tracker
        # will
        start_listening=True,
    )
    # we attach the tracker to the core
    core.core.probeListener = tracker
    # then store the tracker in the list
    all_trackers.append(tracker)


board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_se_binary_workload(
    binary=obtain_resource(resource_id="x86-matrix-multiply-omp"),
    arguments=[100, 8],
)


def max_inst_handler():
    # this handler tests the instruction tracker
    # when it reached this function, it means that it successfully raised
    # the ExitEvent.MAX_INSTS event
    print("Reached MAX_INSTS with 100000000 instructions")
    yield False
    print("Reached MAX_INSTS with 100020000 instructions")
    # we can get the current counter of the global instruction tracker
    print(f"Current counter: {global_inst_tracker.getCounter()}")
    # we can reset the counter
    global_inst_tracker.resetCounter()
    print(f"After reset, current counter: {global_inst_tracker.getCounter()}")
    # we can add new threshold to the global instruction tracker
    global_inst_tracker.addThreshold(20000)
    # we can get the thresholds
    print(f"Current thresholds: {global_inst_tracker.getThresholds()}")
    yield False
    print("Reached MAX_INSTS with 20000 instructions")
    print("Stop listening to instructions")
    # we can stop listening to instructions
    for tracker in all_trackers:
        tracker.stopListening()
    # similarly, we can start listening to instructions again by calling:
    #
    # for tracker in all_trackers:
    #     tracker.startListening()
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        # the global instruction tracker will raise the MAX_INSTS event
        ExitEvent.MAX_INSTS: max_inst_handler()
    },
)

simulator.run()

print("Simulation Done")
