# Copyright (c) 2021-2026 The Regents of the University of California
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

import gem5.utils.multisim as multisim
from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 10

multisim.set_num_processes(NUM_PROCESSES)

# RISCV boot using Atomic cores, with and without systemd. Longer
# workloads.
for no_systemd in [True, False]:
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB",
        l1i_size="16KiB",
        l2_size="256KiB",
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.RISCV, num_cores=1
    )

    board = RiscvBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    if no_systemd:
        board.set_workload(
            obtain_resource(
                "riscv-ubuntu-24.04-boot-no-systemd", resource_version="2.0.0"
            )
        )
        name = f"process_riscv-atomic-24-04-boot-no-systemd"
    else:
        board.set_workload(
            obtain_resource(
                "riscv-ubuntu-24.04-boot", resource_version="2.0.0"
            )
        )
        name = f"process_riscv-atomic-24-04-boot-with-systemd"

    multisim.add_simulator(Simulator(board=board, id=name))

# Run X86 and Arm hello world binaries. Shorter workloads.
# Run these 6 times each so we test having more jobs
# (2 RISCV boots + 12 hello world binaries = 14) than the
# number of processes (10).

for isa in [ISA.X86, ISA.ARM]:
    for i in range(0, 6):
        cache_hierarchy = NoCache()

        memory = SingleChannelDDR3_1600(size="32MiB")

        processor = SimpleProcessor(
            cpu_type=CPUTypes.TIMING, isa=isa, num_cores=1
        )

        board = SimpleBoard(
            clk_freq="3GHz",
            processor=processor,
            memory=memory,
            cache_hierarchy=cache_hierarchy,
        )

        board.set_se_binary_workload(
            obtain_resource(
                f"{isa.value}-hello64-static", resource_version="1.0.0"
            )
        )
        multisim.add_simulator(
            Simulator(board=board, id=f"process_{isa.value}-hello-{i}")
        )
