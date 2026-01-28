# Copyright (c) 2022-2025 The Regents of the University of California
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

import argparse

import m5

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import get_cpu_type_from_str
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import get_isa_from_str
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import ExitHandler
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

parser = argparse.ArgumentParser()

parser.add_argument("--start_cores", type=str, required=True)
parser.add_argument("--switch_cores", type=str, required=True)
parser.add_argument("--isa", type=str, required=True)

args = parser.parse_args()


class ScheduledProcessorSwitchHandler(ExitHandler, hypercall_num=6):
    process_count = 0

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        start_core = (
            simulator._board.get_processor()
            ._switchable_cores["start"][0]
            ._cpu_type.value
        )
        switch_core = (
            simulator._board.get_processor()
            ._switchable_cores["switch"][0]
            ._cpu_type.value
        )
        if self.__class__.process_count == 0:

            print(
                f"processor's start core is {start_core}, "
                f"processor's switch core is {switch_core}"
            )
            print(
                f"Before processor switch, is the processor on the starting "
                f"cores? {simulator._board.get_processor()._current_is_start}"
            )
            print(f"Switching processors!")
            simulator.switch_processor()
            print(
                f"After processor switch, is the processor on the starting "
                f"cores? {simulator._board.get_processor()._current_is_start}"
            )
            print(f"self.process_count: {self.process_count}")
        else:
            isa = simulator._board.get_processor().get_isa().value

            num_cores = len(simulator._board.get_processor()._current_cores)
            print(
                "This is not the first time a scheduled tick exit has been"
                "handled, will exit after stats dump."
            )
            print(
                f"Successful completion of {start_core} to {switch_core} "
                f"{num_cores} core {isa} processor switch test"
            )
        m5.stats.dump()
        self.__class__.process_count += 1

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        if self.process_count < 2:
            return False
        else:
            return True


name = f"{args.start_cores}_to_{args.switch_cores}_{args.isa}_matmul_1core"

cache_hierarchy = NoCache()

memory = SingleChannelDDR3_1600(size="32MiB")

processor = SimpleSwitchableProcessor(
    starting_core_type=get_cpu_type_from_str(args.start_cores),
    switch_core_type=get_cpu_type_from_str(args.switch_cores),
    num_cores=1,
    isa=get_isa_from_str(args.isa),
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

if args.isa == "riscv":
    board.set_workload(
        obtain_resource("riscv-matrix-multiply-run"),
    )
elif args.isa == "arm":
    board.set_workload(
        obtain_resource("arm-matrix-multiply-run"),
    )
else:
    board.set_workload(
        obtain_resource("x86-matrix-multiply-run"),
    )

simulator = Simulator(board=board, id=name)
m5.scheduleTickExitAbsolute(
    1_000_000, "Reached 1 million ticks, switching processors now!"
)
# allow simulations to run for another million ticks after switch to ensure
# simulation doesn't immediately crash
m5.scheduleTickExitAbsolute(2_000_000, "Reached 2 million ticks!")
m5.scheduleTickExitAbsolute(3_000_000, "Reached 3 million ticks!, exiting")

simulator.run()
