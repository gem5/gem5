# Copyright (c) 2021-2025 The Regents of the University of California
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
from pathlib import Path

import m5
from m5.objects import (
    ArmDefaultRelease,
    VExpress_GEM5_V1,
)

import gem5.utils.multisim as multisim
from gem5.components.boards.arm_board import ArmBoard
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.cachehierarchies.classic.private_l1_private_l2_walk_cache_hierarchy import (
    PrivateL1PrivateL2WalkCacheHierarchy,
)
from gem5.components.memory import (
    DualChannelDDR4_2400,
    SingleChannelDDR3_1600,
)
from gem5.components.processors.cpu_types import get_cpu_type_from_str
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    ExitHandler,
    WorkBeginExitHandler,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

parser = argparse.ArgumentParser()
parser.add_argument("--num_cores", type=int, required=True)
parser.add_argument("--start_cores", type=str, required=True)
parser.add_argument("--switch_cores", type=str, required=True)
parser.add_argument("--isa", type=str, required=True)

args = parser.parse_args()


class WorkBeginExit(WorkBeginExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        start_core = (
            simulator._board.get_processor()._current_cores[0]._cpu_type
        )
        switch_core = (
            simulator._board.get_processor()
            ._switchable_cores["switch"][0]
            ._cpu_type
        )
        num_cores = len(simulator._board.get_processor()._current_cores)
        print(
            f"processor's start core is {start_core}, "
            f"processor's switch core is {switch_core}",
            f"processor has {num_cores} cores in total.",
        )
        print(
            f"Before processor switch, is the processor on the starting "
            f"cores? {simulator._board.get_processor()._current_is_start}"
        )
        print("Switching processors at ROI begin! Hypercall 4")
        simulator.switch_processor()
        print(
            f"After processor switch, is the processor on the starting cores? "
            f"{simulator._board.get_processor()._current_is_start}"
        )
        print("Dumping and resetting stats!")
        m5.stats.dump()
        m5.stats.reset()
        print("Scheduling exit event in 1 million ticks")
        m5.scheduleTickExitFromCurrent(
            1_000_000,
            "Scheduling stats dump 1 million ticks after processor switch!",
        )  # run workload for 10 million ticks

    def _exit_simulation(self) -> bool:
        return False


class ScheduledStatsDumpHandler(ExitHandler, hypercall_num=6):
    process_count = 0

    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        print(
            f"In scheduled tick exit event handler, number of times a "
            f"scheduled tick exit has been handled: {self.process_count}"
        )
        if self.__class__.process_count == 0:
            print(
                f"1 million ticks after processor switch, is the processor "
                f"on the starting cores? "
                f"{simulator._board.get_processor()._current_is_start}"
            )
            # run workload for another million ticks
            m5.scheduleTickExitFromCurrent(
                1_000_000,
                "Scheduling stats dump "
                "1 million ticks after previous exit!",
            )
        else:
            isa = simulator._board.get_processor().get_isa().value
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
            num_cores = len(simulator._board.get_processor()._current_cores)
            print(
                "This is not the first time a scheduled tick exit has been "
                "handled, will exit after stats dump."
            )
            print(
                f"Successful completion of {start_core} to {switch_core} "
                f"{num_cores} core {isa} processor switch test"
            )
            m5.scheduleTickExitFromCurrent(
                10, "Scheduling stats dump " "10 ticks after previous exit!"
            )
        print("Dumping and resetting stats!")
        m5.stats.dump()
        m5.stats.reset()
        self.__class__.process_count += 1

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        if self.process_count < 2:
            return False
        else:
            return True


name = (
    f"{args.start_cores}_to_{args.switch_cores}_{args.isa}_systemboot_"
    f"{args.num_cores}core"
)

if args.isa == "x86":
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB",
        l1i_size="16KiB",
        l2_size="256KiB",
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleSwitchableProcessor(
        starting_core_type=get_cpu_type_from_str(args.start_cores),
        switch_core_type=get_cpu_type_from_str(args.switch_cores),
        isa=ISA.X86,
        num_cores=args.num_cores,
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    board.set_kernel_disk_workload(
        kernel=obtain_resource(
            "x86-linux-kernel-6.8.0-52-generic", resource_version="1.0.0"
        ),
        disk_image=obtain_resource(
            "x86-ubuntu-24.04-npb-img", resource_version="5.0.0"
        ),
        kernel_args=[
            "earlyprintk=ttyS0",
            "console=ttyS0",
            "lpj=7999923",
            "root=/dev/sda2",
        ],
        readfile_contents=f"/home/gem5/NPB3.4-OMP/bin/cg.S.x; sleep 5;",
        checkpoint=obtain_resource(
            f"x86-ubuntu-24.04-boot-{args.num_cores}-core-checkpoint"
        ),
    )

elif args.isa == "arm":
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
    )

    memory = DualChannelDDR4_2400(size="2GiB")
    processor = SimpleSwitchableProcessor(
        starting_core_type=get_cpu_type_from_str(args.start_cores),
        switch_core_type=get_cpu_type_from_str(args.switch_cores),
        isa=ISA.ARM,
        num_cores=args.num_cores,
    )

    release = ArmDefaultRelease.for_kvm()

    platform = VExpress_GEM5_V1()

    board = ArmBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
        release=release,
        platform=platform,
    )

    board.set_kernel_disk_workload(
        kernel=obtain_resource(
            "arm64-linux-kernel-6.8.12", resource_version="1.0.0"
        ),
        disk_image=obtain_resource(
            "arm-ubuntu-24.04-npb-img", resource_version="4.0.0"
        ),
        bootloader=obtain_resource(
            "arm64-bootloader-foundation", resource_version="2.0.0"
        ),
        readfile_contents="/home/gem5/NPB3.4-OMP/bin/cg.S.x; sleep 5;",
        checkpoint=obtain_resource(
            f"arm-ubuntu-24.04-boot-{args.num_cores}-core-checkpoint"
        ),
    )

elif args.isa == "riscv":
    cache_hierarchy = PrivateL1PrivateL2WalkCacheHierarchy(
        l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
    )

    memory = DualChannelDDR4_2400(size="3GiB")

    processor = SimpleSwitchableProcessor(
        starting_core_type=get_cpu_type_from_str(args.start_cores),
        switch_core_type=get_cpu_type_from_str(args.switch_cores),
        isa=ISA.RISCV,
        num_cores=args.num_cores,
    )

    board = RiscvBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    board.set_kernel_disk_workload(
        kernel=obtain_resource(
            "riscv-linux-6.8.12-kernel", resource_version="1.0.0"
        ),
        disk_image=obtain_resource(
            "riscv-ubuntu-24.04-npb-img", resource_version="1.0.0"
        ),
        bootloader=obtain_resource(
            "riscv-bootloader-opensbi-1.3.1", resource_version="1.0.0"
        ),
        readfile_contents="/home/gem5/NPB3.4-OMP/bin/cg.S.x; sleep 5;",
        checkpoint=obtain_resource(
            f"riscv-ubuntu-24.04-boot-{args.num_cores}-core-checkpoint"
        ),
    )

else:
    print("Unrecognized ISA! Valid options are x86, arm, and riscv")


simulator = Simulator(board=board, id=name)

simulator.run()
