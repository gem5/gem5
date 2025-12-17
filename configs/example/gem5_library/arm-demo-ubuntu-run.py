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
This script boots an ARM Ubuntu disk image in FS (full system) mode using the
ArmDemoBoard.

Usage
-----

```bash
scons build/ALL/gem5.opt -j $(nproc)
./build/ALL/gem5.opt configs/example/gem5_library/arm-demo-ubuntu-run.py
```
"""
import argparse

from gem5.prebuilt.demo.arm_demo_board import ArmDemoBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(
    description="An example configuration script to run the ArmDemoBoard."
)

parser.add_argument(
    "--use-kvm",
    action="store_true",
    help="Use KVM cores instead of Timing.",
)
args = parser.parse_args()

board = ArmDemoBoard(use_kvm=args.use_kvm)

board.set_workload(
    obtain_resource(
        "arm-ubuntu-24.04-boot-with-systemd", resource_version="3.0.0"
    )
)

simulator = Simulator(board=board)

simulator.run()
