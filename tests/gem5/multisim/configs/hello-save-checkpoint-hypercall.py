# Copyright (c) 2022-2026 The Regents of the University of California
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
This gem5 configuation script is used to test taking checkpoints while using
MultiSim. It runs three simulations using the Arm, X86, and RISCV "hello world"
binaries and takes a checkpoint for each simulation after 10^6 ticks.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt \
    tests/gem5/multisim/configs/hello-save-checkpoint-hypercall.py
```

or

```
cd tests
./main.py run gem5/multisim
```
"""

import m5
import m5.options

import gem5.utils.multisim as multisim
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    ExitHandler,
    ScheduledExitEventHandler,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

multisim.set_num_processes(5)


for isa in [ISA.RISCV, ISA.ARM, ISA.X86]:

    cache_hierarchy = NoCache()

    memory = SingleChannelDDR3_1600(size="32MiB")

    processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, isa=isa, num_cores=1)

    board = SimpleBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    if isa == ISA.RISCV:
        board.set_se_binary_workload(obtain_resource("riscv-hello"))
    elif isa == ISA.ARM:
        board.set_se_binary_workload(obtain_resource("arm-hello64-static"))
    else:
        board.set_se_binary_workload(obtain_resource("x86-hello64-static"))

    max_ticks = 10**6
    simulator = Simulator(board=board, id=f"process_{isa.name}")
    simulator.set_hypercall_absolute_max_ticks(
        max_ticks, "Taking checkpoint now!"
    )
    multisim.add_simulator(simulator=simulator)


class ScheduledCheckpointTakingHandler(ScheduledExitEventHandler):
    @overrides(ScheduledExitEventHandler)
    def _process(self, simulator: "Simulator") -> None:
        checkpoint_path = (
            f"{m5.options.outdir}/{simulator.get_id()}-hello-checkpoint/"
        )
        print(f"Taking a checkpoint at {checkpoint_path}")
        simulator.save_checkpoint(checkpoint_path)
        print("Done taking a checkpoint")

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True
