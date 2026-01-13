# Copyright (c) 2024-2025 The Regents of the University of California
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
This script can be used to run a simple Ubuntu boot with the RiscvDemoBoard.
The script will boot the OS to login before exiting the simulation.

A detailed terminal output can be found in `m5out/board.platform.terminal`.

**Warning:** The RiscvDemoBoard uses the Timing CPU. The boot may take
considerable time to complete execution.

Usage
-----

```
scons build/ALL/gem5.opt

./build/ALL/gem5.opt configs/example/gem5_library/riscv-demo-board-run.py \
--workload=riscv-ubuntu-24.04-boot-no-systemd
```
"""

import argparse

from gem5.prebuilt.demo.riscv_demo_board import RiscvDemoBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser()

parser.add_argument(
    "--workload",
    type=str,
    required=True,
    help="Enter the name of the workload you would like to run. You can browse"
    " through the available workloads and resources at "
    "https://resources.gem5.org",
    default="riscv-ubuntu-24.04-boot-no-systemd",
)

parser.add_argument(
    "--version",
    type=str,
    default=None,
    help="Enter the workload version you would like to use. The latest version"
    " will be used if this is left blank.",
)

args = parser.parse_args()


board = RiscvDemoBoard()

board.set_workload(
    obtain_resource(resource_id=args.workload, resource_version=args.version)
)

simulator = Simulator(board=board)

simulator.run()
