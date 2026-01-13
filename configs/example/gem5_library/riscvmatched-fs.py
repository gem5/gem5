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

"""
This gem5 configuration script boots Ubuntu 24.04 on the RISCVMatchedBoard in
FS mode. If -i or --to-kernel-init is passed, the simulation will exit after
the kernel is booted. Otherwise, it will exit after Ubuntu boots.

Usage
---

```
scons build/ALL/gem5.opt

./build/ALL/gem5.opt configs/example/gem5_library/riscvmatched-fs.py
```
"""

import argparse

from gem5.prebuilt.riscvmatched.riscvmatched_board import RISCVMatchedBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_handler import (
    KernelBootedExitHandler,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.override import overrides

parser = argparse.ArgumentParser(
    description="A script which uses the RISCVMatchedBoard in FS mode."
)

parser.add_argument(
    "-i",
    "--to-kernel-init",
    action="store_true",
    help="Exit the simulation after the Linux Kernel boot.",
)

args = parser.parse_args()


# If the `-i` flag is passed, exit the simulation after kernel boot.
class CustomKernelBootedExitHandler(KernelBootedExitHandler):
    @overrides(KernelBootedExitHandler)
    def _exit_simulation(self) -> bool:
        if args.to_kernel_init:
            return True
        return False


# instantiate the RISCV Matched board with default parameters
board = RISCVMatchedBoard(
    clk_freq="1.2GHz",
    l2_size="2MiB",
    is_fs=True,
)

# Here we a full system workload: "riscv-ubuntu-24.04-boot" which boots
# Ubuntu 24.04. Once the system successfully boots it encounters hypercall 3,
# which stops the simulation. You may inspect `m5out/board.platform.terminal`
# to see the stdout of the simulated system.

workload = obtain_resource("riscv-ubuntu-24.04-boot", resource_version="2.0.0")

board.set_workload(workload)

simulator = Simulator(board=board)
simulator.run()
