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
This example runs the first workload of the given suite.

Characteristics
---------------
* User needs to specify the isa in lower case.
"""
import argparse

from m5.util import panic

from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser(description="A script to run suite tests.")

parser.add_argument(
    "-i", "--suite-id", type=str, required=True, help="The suite id."
)

parser.add_argument(
    "-v", "--version", type=str, required=False, help="The suite version."
)

parser.add_argument(
    "-t",
    "--tick-exit",
    type=int,
    required=False,
    help="The tick to exit the simulation.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

parser.add_argument(
    "-s",
    "--isa",
    type=str,
    required=True,
    help="The ISA to use.",
)

parser.add_argument(
    "-f",
    "--fs-sim",
    default=False,
    action="store_true",
    required=False,
    help="Whether to run the simulation as full system.",
)

args = parser.parse_args()

# Setup the cache hierarchy.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB"
)

# Setup the system memory.
memory = SingleChannelDDR3_1600(size="3GB")


def get_processor(isa):
    processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, isa=isa, num_cores=1)
    return processor


if args.isa == "riscv":
    requires(isa_required=ISA.RISCV)

    from gem5.prebuilt.riscvmatched.riscvmatched_board import RISCVMatchedBoard

    board = RISCVMatchedBoard()

elif args.isa == "x86":
    requires(isa_required=ISA.X86)

    from gem5.components.boards.x86_board import X86Board

    processor = get_processor(ISA.X86)
    board = X86Board(
        clk_freq="1GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )
else:
    panic(f"ISA {args.isa} does not have a suite.")

# Set the workload.
if args.version:
    suite = obtain_resource(
        args.suite_id,
        resource_version=args.version,
        resource_directory=args.resource_directory,
    )
else:
    suite = obtain_resource(
        args.suite_id, resource_directory=args.resource_directory
    )

board.set_workload(list(suite)[0])


simulator = Simulator(board=board, full_system=args.fs_sim)

if args.tick_exit:
    simulator.run(max_ticks=args.tick_exit)
else:
    simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
