# Copyright (c) 2024-2025 The Regents of the University of California
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
This script demonstrates how to override and extend default hypercall exit
handlers in gem5.

Usage
-----

```sh
./build/ALL/gem5.opt \
    configs/example/gem5_library/exit_handling/user-exit-handler.py
```
"""

import argparse
from pathlib import Path

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import ScheduledExitEventHandler
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

parser = argparse.ArgumentParser()

parser.add_argument(
    "--checkpoint-path",
    type=str,
    required=False,
    default="./",
    help="The directory to store the checkpoint.",
)

args = parser.parse_args()

# Create the system
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600(size="512MB")
processor = SimpleProcessor(cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=1)

# Create the board
board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload
board.set_se_binary_workload(
    obtain_resource("x86-matrix-multiply", resource_version="1.0.0")
)


# The exit handler which will be used to take checkpoints when the scheduled
# exit event is triggered (hypercall ID 6).
class MyExitHandler(ScheduledExitEventHandler):
    def _process(self, simulator: "Simulator") -> None:
        super()._process(simulator)

        # Get the specific information about this exit event: The justification
        # and the tick this exit was scheduled.
        scheduled_at = self.scheduled_at_tick()
        justification = self.justification()

        # Print this information to the console.
        print(
            f"Processing scheduled exit event at tick: {simulator.get_current_tick()}..."
        )
        if scheduled_at:
            print(f"(Exit was originally scheduled at tick: {scheduled_at})")
        if justification:
            print(f"(Justification: {justification})")

        # Take the checkpoint.
        print("Taking checkpoint via scheduled exit event...")
        checkpoint_dir = simulator.get_checkpoint_dir()
        if not checkpoint_dir:
            checkpoint_dir = (
                Path(args.checkpoint_path)
                / f"cpt.{str(simulator.get_current_tick())}"
            )
        simulator.save_checkpoint(checkpoint_dir)
        print(f"Checkpoint taken!")

        # Finally we always schedule another exit 10 billion ticks from now.
        # This means this exit occurs every 10 billion ticks until the program
        # ceases execution.
        print("Scheduling the next checkpoint in 10 billion ticks.")
        simulator.set_hypercall_relative_max_ticks(
            10_000_000_000, "To take checkpoint (every 10 billion ticks)."
        )

    @overrides(ScheduledExitEventHandler)
    def _exit_simulation(self) -> bool:
        # We always want to reenter the simulation loop as we are automating
        # what we wanted to do in the `_process` function.
        return False


# Create the Simulator
simulator = Simulator(board=board)

# Schedule the first exit event. This will be used to take the first
# checkpoint.
simulator.set_hypercall_absolute_max_ticks(
    10_000_000_000, "To take the first checkpoint (@ tick 10 billion)."
)

# Run the simulation.
simulator.run()
