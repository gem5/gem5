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

"""A script which accepts a comma-separated string of integers, each of which
will be returned by the simulation as a hypercall type ID in the order they are
provided. This script will then check that the hypercall type IDs returned are
what is expected given what was passed to the binary (and specified as a
parameter to this script).

Note: This is not the suggested way to use hypercalls in non-testing code.
"""

import argparse
import urllib.request

import m5

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.exit_handler import register_exit_handler
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(description="Hypercall Exit Handler tester")
parser.add_argument(
    "ids",
    type=str,
    help="Comma separated string of integers, each of which will be returned "
    "by the simulation as a hypercall type ID in the order they are "
    "provided",
)
parser.add_argument(
    "--resource-directory",
    type=str,
    help="The gem5-resource download directory",
)
args = parser.parse_args()

# Parse command-separated string integers into a list of integers.
ids_int_list = [int(x) for x in args.ids.split(",")]

# Create the board.
board = SimpleBoard(
    clk_freq="3GHz",
    memory=SingleChannelDDR3_1600(),
    cache_hierarchy=NoCache(),
    processor=SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.X86, num_cores=1
    ),
)

# Set the workload.
# This workload is a simple binary that accepts a sequence of hypercall type
# IDs in which it will iterate too, making a hypercall for each ID.
#
# TODO: This needs to be removed and replaced with a pull from gem5-resources.
# (see commented out code below).
import urllib

binary = BinaryResource(
    urllib.request.urlretrieve(url="http://dist.gem5.org/dist/hypercall-exit")[
        0
    ]
)
# TODO: Uncomment this when the binary is in gem5-resources.
# binary = obtain_resource(
#     "x86-hypercall-exit",
#     resource_directory=args.resource_directory,
# )
board.set_se_binary_workload(binary, arguments=[args.ids])


# Create aa exit handler which checks the hypercall type IDs returned are what
# is expected given what was passed to the binary (as specified by the
# passed to this parameter passed to this script).
class TypeIDEchoExitHandler:
    hypercall_exits_processed = []

    def _process(self, simulator: "Simulator") -> None:
        hypercall_id = simulator.get_hypercall_id()
        print(f"Processing hypercall ID: {hypercall_id}")
        self.hypercall_exits_processed.append(simulator.get_hypercall_id())

        assert len(self.hypercall_exits_processed) > 0, (
            "No hypercall  IDs were appended to list of called. This "
            "shouldn't happen."
        )

        # There are two failure cases we handle here.
        # The first is when hypercalls keep being processes after the expected
        # number of hypercalls.
        # The second is when the hypercall type ID processes does not match the
        # expected hypercall type ID.
        #
        # There is one other which we handle after the simulation loop exits
        # for the last time. That is when the number of hypercalls processed
        # is less than the number of hypercalls expected.
        if len(self.hypercall_exits_processed) > len(ids_int_list):
            m5.fatal(
                "More hypercall IDs were processed than expected.\n"
                f"Expected: {ids_int_list}\n"
                f"Received (thus far): {TypeIDEchoExitHandler.ids}"
            )

        if (
            self.hypercall_exits_processed[-1]
            != ids_int_list[len(self.hypercall_exits_processed) - 1]
        ):
            m5.fatal(
                "The hypercall ID processed does not match the expected "
                "hypercall ID.\n"
                f"Expected: {ids_int_list}\n"
                f"Received (thus far): {TypeIDEchoExitHandler.ids}"
            )
        print(
            f"Hypercall type {hypercall_id} was expected. "
            "Processing complete."
        )

    def _exit_simulation(self) -> bool:
        return False


# Use a single instance of the exit handler to keep state
exit_handler = TypeIDEchoExitHandler()


def exit_event_handler(simulator, payload):
    exit_handler._process(simulator)
    return exit_handler._exit_simulation()


for id in ids_int_list:
    register_exit_handler(
        id, exit_event_handler, f"handler for hypercall type {id}"
    )

# Create the simulator (with exit event handler map)
simulator = Simulator(board=board)

# Run the simulation.
simulator.run()

# Check that the hypercall type IDs processed match what was expected/passed to
# the binary.
if exit_handler.hypercall_exits_processed != ids_int_list:
    m5.fatal(
        "The hypercall IDs processed do not match the expected hypercall IDs\n"
        f"Expected: {ids_int_list}\n"
        f"Received: {exit_handler.hypercall_exits_processed}"
    )


print(
    "All hypercall IDs received match the expected hypercall IDs.\n"
    f"Expected: {ids_int_list}\n"
    f"Received: {exit_handler.hypercall_exits_processed}"
)
