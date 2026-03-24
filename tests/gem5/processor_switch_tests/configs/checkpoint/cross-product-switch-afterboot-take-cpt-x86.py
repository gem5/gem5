# Copyright (c) 2025 The Regents of the University of California
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

import m5

import gem5.utils.multisim as multisim
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import AfterBootExitHandler
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 20

multisim.set_num_processes(NUM_PROCESSES)


import argparse

parser = argparse.ArgumentParser()

parser.add_argument("--num-cores", type=int, required=True)

args = parser.parse_args()


class AfterBootPrintStatus(AfterBootExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        checkpoint_path = (
            f"./x86-ubuntu-24.04-boot-{args.num_cores}-core-checkpoint"
        )
        print(
            f"Taking a checkpoint at {checkpoint_path}",
        )
        simulator.save_checkpoint(checkpoint_path)
        print("Done taking a checkpoint")
        print("Scheduling exit in 10 million ticks!")
        m5.scheduleTickExitFromCurrent(
            10_000_000
        )  # exit 10 million ticks after system boot

    def _exit_simulation(self) -> bool:
        return False


cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB",
    l1i_size="16KiB",
    l2_size="256KiB",
)
memory = SingleChannelDDR3_1600(size="3GiB")
processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    # The value of switch_core_type doesn't matter, since we never switch cores
    switch_core_type=CPUTypes.KVM,
    isa=ISA.X86,
    num_cores=args.num_cores,
)

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_workload(
    obtain_resource("x86-ubuntu-24.04-npb-cg-s", resource_version="3.0.0")
)

simulator = Simulator(board=board)
simulator.run()
