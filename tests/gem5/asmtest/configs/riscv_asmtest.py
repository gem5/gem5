# Copyright (c) 2021 The Regents of the University of California
# Copyright (c) 2022 Google Inc
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
A run script for a very simple Syscall-Execution running simple binaries.
The system has no cache heirarchy and is as "bare-bones" as you can get in
gem5 while still being functinal.
"""
import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import get_cpu_type_from_str
from gem5.components.processors.cpu_types import get_cpu_types_str_set
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(
    description="A gem5 script for testing RISC-V instructions"
)

parser.add_argument(
    "resource", type=str, help="The gem5 resource binary to run."
)

parser.add_argument(
    "cpu", type=str, choices=get_cpu_types_str_set(), help="The CPU type used."
)

parser.add_argument(
    "--riscv-32bits",
    action="store_true",
    help="Use 32 bits core of Riscv CPU",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

parser.add_argument(
    "-n",
    "--num-cores",
    type=int,
    default=1,
    required=False,
    help="The number of CPU cores to run.",
)

args = parser.parse_args()

# Setup the system.
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()

processor = SimpleProcessor(
    cpu_type=get_cpu_type_from_str(args.cpu),
    isa=ISA.RISCV,
    num_cores=args.num_cores,
)

if args.riscv_32bits:
    for simple_core in processor.cores:
        for i in range(len(simple_core.core.isa)):
            simple_core.core.isa[i].riscv_type = "RV32"

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Set the workload
binary = obtain_resource(
    args.resource, resource_directory=args.resource_directory
)
motherboard.set_se_binary_workload(binary)

# Run the simulation
simulator = Simulator(board=motherboard)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
