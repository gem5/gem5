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

"""
import argparse

import m5
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser()

parser.add_argument(
    "-t",
    "--tick-exits",
    type=int,
    nargs="+",
    required=True,
    help="Set the tick exits to exit.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

# Setup the system.
motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=SimpleProcessor(
        cpu_type=CPUTypes.TIMING,
        isa=ISA.X86,
        num_cores=1,
    ),
    memory=SingleChannelDDR3_1600(),
    cache_hierarchy=NoCache(),
)

# Set the workload
binary = obtain_resource(
    "x86-hello64-static", resource_directory=args.resource_directory
)
motherboard.set_se_binary_workload(binary)


def scheduled_tick_generator():
    while True:
        print(f"Exiting at: {m5.curTick()}")
        yield False


# Run the simulation
simulator = Simulator(
    board=motherboard,
    on_exit_event={ExitEvent.SCHEDULED_TICK: scheduled_tick_generator()},
)

for tick in args.tick_exits:
    m5.scheduleTickExitFromCurrent(tick)

simulator.run()
