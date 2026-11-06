# Copyright (c) 2026 The Regents of The University of California
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

"""Resolve the host virtualization CPU and switch it to a simulated CPU."""

import m5
from m5.objects import (
    ArmDefaultRelease,
    VExpress_GEM5_V1,
)

from gem5.components.boards.arm_board import ArmBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.virtualized_processor import (
    VirtualizedSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    AfterBootExitHandler,
    ExitHandler,
    ExitHypercall,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides
from gem5.utils.requires import requires

requires(isa_required=ISA.ARM)


class SwitchToTimingHandler(AfterBootExitHandler):
    @overrides(AfterBootExitHandler)
    def _process(self, simulator: Simulator) -> None:
        simulator.switch_processor()
        print("Switched from the host virtualization CPU to Timing")
        m5.scheduleTickExitFromCurrent(
            10_000_000, "Timing takeover interval complete"
        )


class VirtualizedSwitchCompleteHandler(
    ExitHandler, hypercall=ExitHypercall.SCHEDULED_EXIT
):
    @overrides(ExitHandler)
    def _process(self, simulator: Simulator) -> None:
        if processor.get_total_instructions() <= 0:
            raise AssertionError(
                "Timing CPU did not execute instructions after takeover"
            )
        print("Host-resolved virtualized processor completed successfully")

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
)
memory = DualChannelDDR4_2400(size="2GiB")
processor = VirtualizedSwitchableProcessor(
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.ARM,
    num_cores=1,
    clk_freq="3GHz",
)

board = ArmBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    release=ArmDefaultRelease.for_kvm(),
    platform=VExpress_GEM5_V1(),
)
workload = obtain_resource(
    "arm-ubuntu-24.04-boot-with-systemd", resource_version="3.0.0"
)
workload.set_parameter(
    "readfile_contents", "echo 'Virtualized processor booted'\n"
)
board.set_workload(workload)

Simulator(board=board).run()
