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

import os
import shutil

import m5

from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import KernelBootedExitHandler
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

print(f"Current working directory is: {os.getcwd()}")


class KernelBootProcessorSwitch(KernelBootedExitHandler):
    @overrides(KernelBootedExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at kernel boot! Hypercall 1")

    @overrides(KernelBootedExitHandler)
    def _exit_simulation(self) -> bool:
        return True


def read_stats_files(filepath: str, stats_dict: dict) -> None:
    with open(filepath) as stats:
        print(f"Opening stats file at path {filepath}")
        for line in stats:
            # print(line)
            tmp = line.split()
            if len(tmp) > 1:
                stats_dict[tmp[0]] = tmp[1]
    stats_dict.pop("----------", None)


cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB",
    l1i_size="16KiB",
    l2_size="256KiB",
)
memory = SingleChannelDDR3_1600(size="3GiB")
# processor = SimpleSwitchableProcessor(
#     starting_core_type=CPUTypes.KVM, #ATOMIC
#     switch_core_type=CPUTypes.TIMING,
#     isa=ISA.X86,
#     num_cores=1,
# )
processor = SimpleProcessor(
    # cpu_type=CPUTypes.KVM,  # switch to ATOMIC for final
    cpu_type=CPUTypes.ATOMIC,
    isa=ISA.X86,
    num_cores=1,
)

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_workload(
    obtain_resource(
        "x86-ubuntu-24.04-boot-no-systemd", resource_version="4.0.0"
    )
)

simulator = Simulator(board=board)

simulator.run()


print("starting data analysis: IPC comparison")

cached_stats = {}
curr_stats = {}

read_stats_files("./gem5/regression_tests/cached_stats.txt", cached_stats)
read_stats_files(f"{m5.options.outdir}/stats.txt", curr_stats)

print(f"{m5.options.outdir}/stats.txt")

print(curr_stats)
try:
    old_ipc = float(cached_stats["board.processor.cores.core.ipc"])
    new_ipc = float(curr_stats["board.processor.cores.core.ipc"])

    print(f"old IPC: {old_ipc}")
    print(f"new IPC: {new_ipc}")

    if (new_ipc / old_ipc) < 0.8:
        print(f"New IPC is less than 80% of the old IPC!")
        exit(1)

finally:
    shutil.move(
        f"{m5.options.outdir}/stats.txt",
        "./gem5/regression_tests/cached_stats.txt",
    )
