# Copyright (c) 2022-25 The Regents of the University of California
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
from m5.objects import (
    ArmDefaultRelease,
    VExpress_GEM5_V1,
)

from gem5.components.boards.arm_board import ArmBoard
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import AfterBootExitHandler
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# This runs a check to ensure the gem5 binary is compiled for ARM.
requires(isa_required=ISA.ARM)

from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)

parser = argparse.ArgumentParser()

parser.add_argument("--num-cores", type=int, required=True)

args = parser.parse_args()


class AfterBootTakeCheckpoint(AfterBootExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        checkpoint_path = (
            f"./arm-ubuntu-24.04-boot-{args.num_cores}-core-checkpoint"
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


# Here we setup the parameters of the l1 and l2 caches.
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
)

# Memory: Dual Channel DDR4 2400 DRAM device.
memory = DualChannelDDR4_2400(size="2GiB")

# Here we setup the processor. This is a special switchable processor in which
# a starting core type and a switch core type must be specified. Once a
# configuration is instantiated a user may call `processor.switch()` to switch
# from the starting core types to the switch core types. In this simulation
# we start with KVM cores to simulate the OS boot, then switch to the Timing
# cores for the command we wish to run after boot.
processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    # The value of switch_core_type doesn't matter, since we never switch cores
    switch_core_type=CPUTypes.ATOMIC,
    isa=ISA.ARM,
    num_cores=args.num_cores,
)

# The ArmBoard requires a `release` to be specified. This adds all the
# extensions or features to the system. We are setting this to for_kvm()
# to enable KVM simulation.
release = ArmDefaultRelease.for_kvm()

# The platform sets up the memory ranges of all the on-chip and off-chip
# devices present on the ARM system. ARM KVM only works with VExpress_GEM5_V1
# on the ArmBoard at the moment.
platform = VExpress_GEM5_V1()

board = ArmBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    release=release,
    platform=platform,
)

board.set_workload(
    obtain_resource("arm-ubuntu-24.04-npb-cg-s", resource_version="2.0.0")
)

simulator = Simulator(board=board)

simulator.run()
