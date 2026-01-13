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

"""
This script shows an example of running a full system RISCV Ubuntu boot
simulation using the gem5 library. This simulation boots Ubuntu 24.04 using
2 TIMING CPU cores. The simulation ends when the startup is completed
successfully.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt configs/example/gem5_library/riscv-ubuntu-run.py
```
"""

from gem5.components.boards.riscv_board import RiscvBoard

# With RISCV, we use simple caches.
from gem5.components.cachehierarchies.classic.private_l1_private_l2_walk_cache_hierarchy import (
    PrivateL1PrivateL2WalkCacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    ExitHandler,
    KernelBootedExitHandler,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

# Here we set up the parameters of the l1 and l2 caches.
cache_hierarchy = PrivateL1PrivateL2WalkCacheHierarchy(
    l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
)

# Memory: Dual Channel DDR4 2400 DRAM device.
memory = DualChannelDDR4_2400(size="3GiB")

# Here we set up the processor. We use a simple processor.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=2
)

# Here we set up the board. The RiscvBoard allows for FS mode (full system) and
# SE mode (syscall emulation) RISCV simulations.
board = RiscvBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set a full system workload, "riscv-ubuntu-24.04-boot", which boots
# Ubuntu 24.04. Once the system successfully boots it encounters a
# `gem5-bridge hypercall 3` command which stops the simulation.

# The simulated system's stdout can be viewed in
# `m5out/board.platform.terminal`.

board.set_workload(
    obtain_resource("riscv-ubuntu-24.04-boot", resource_version="2.0.0")
)

# Examples of how you can override the default hypercall exit handler
# behaviors.
# Exit handlers don't have to be specified in the config script if you don't
# want to modify/override their default behaviors.


# You can inherit from either the class that handles a certain hypercall, or
# inherit directly from ExitHandler and specify a hypercall number.
# See src/python/gem5/simulate/exit_handler.py for more information on which
# handlers map to which hypercalls, and what the default behaviors are.
class CustomKernelBootedExitHandler(KernelBootedExitHandler):
    @overrides(KernelBootedExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        print("First exit: kernel booted")

    @overrides(KernelBootedExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class CustomAfterBootExitHandler(ExitHandler, hypercall_num=2):
    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        print("Second exit: Started `after_boot.sh` script")

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptExitHandler(ExitHandler, hypercall_num=3):
    @overrides(ExitHandler)
    def _process(self, simulator: "Simulator") -> None:
        print(f"Third exit: {self.get_handler_description()}")

    @overrides(ExitHandler)
    def _exit_simulation(self) -> bool:
        return True


simulator = Simulator(board=board)
simulator.run()
