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
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
)
from gem5.simulate.exit_handler import (
    AfterBootExitHandler,
    AfterBootScriptExitHandler,
    KernelBootedExitHandler,
    WorkBeginExitHandler,
    WorkEndExitHandler,
)
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 8

multisim.set_num_processes(NUM_PROCESSES)


class KernelBootedDumpReset(KernelBootedExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after kernel boot! Hypercall 1")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


class AfterBootDumpReset(AfterBootExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after Ubuntu boot! Hypercall 2")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptDumpReset(AfterBootScriptExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        print(
            "Dumping and resetting stats before exiting simulation! Hypercall 3"
        )
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return True


class WorkBeginDumpReset(WorkBeginExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at ROI begin! Hypercall 4")
        print("Switching processors at ROI begin! Hypercall 4")
        processor.switch()

    def _exit_simulation(self) -> bool:
        return False


class WorkEndDumpReset(WorkEndExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats at ROI end! Hypercall 5")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


for npb_workload in ["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp", "ua"]:

    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB",
        l1i_size="16KiB",
        l2_size="256KiB",
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleSwitchableProcessor(
        starting_core_type=CPUTypes.KVM,
        switch_core_type=CPUTypes.TIMING,
        isa=ISA.X86,
        num_cores=1,
    )

    # if isa == ISA.X86:
    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    board.set_kernel_disk_workload(
        kernel=KernelResource(
            "/projects/gem5/new-base-imgs-w-hypercalls/x86-disk-image-24-04/6.8.0-52-generic-x86-ubuntu"
        ),
        disk_image=DiskImageResource(
            "/projects/gem5/new-base-imgs-w-hypercalls/disk-image-x86-npb/x86-ubuntu-npb"
        ),
        kernel_args=[
            "earlyprintk=ttyS0",
            "console=ttyS0",
            "lpj=7999923",
            "root=/dev/sda2",
        ],
        readfile_contents=f"/home/gem5/NPB3.4-OMP/bin/{npb_workload}.S.x; sleep 5;",
    )
    multisim.add_simulator(
        Simulator(board=board, id=f"process_x86_npb_{npb_workload}_s")
    )
