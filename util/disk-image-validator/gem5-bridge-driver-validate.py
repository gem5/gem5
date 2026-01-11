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

import argparse

from gem5.prebuilt.demo.arm_demo_board import ArmDemoBoard
from gem5.prebuilt.demo.riscv_demo_board import RiscvDemoBoard
from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
from gem5.resources.resource import (
    BinaryResource,
    obtain_resource,
)
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

"""
gem5 Bridge Driver Test Script

This script tests whether the gem5-bridge driver is correctly installed by
running a simple C program that makes an m5 hypercall (hypercall number 1234)
without requiring superuser privileges.

Usage:
    build/ALL/gem5.opt util/disk-image-validator/gem5-bridge-driver-validate.py --isa <ISA> --workload <WORKLOAD_ID> --resource-version <RESOURCE_VERSION>

Arguments:
    --isa                The instruction set architecture (ISA) for the simulation.
                         Options: x86, arm, riscv (Required).
    --workload           The workload ID to run (Required).
    --resource-version   The version of the workload resource (Optional).

Example:
     build/ALL/gem5.opt util/disk-image-validator/gem5-bridge-driver-validate.py --isa x86 --workload x86-ubuntu-24.04-boot-no-systemd --resource-version 4.0.0

Requirements:
    - gem5 v25.0 or higher must be installed and properly configured.
    - The necessary prebuilt demo board classes must be available.
    - The gem5 resource system should be able to fetch the specified workload.
    - Python 3 must be installed.

Functionality:
    - Selects the appropriate demo board (X86, Arm, or RISC-V) based on the given ISA.
    - Loads the specified workload and resource version (if provided).
    - Runs a shell script (`test_gem5_bridge.sh`) to check whether m5 hypercall 1234
      executes successfully without requiring `sudo`.
    - Prints a success message if the test passes.

Exit Conditions:
    - If the hypercall executes successfully, the script prints a success message.
    - If an invalid ISA is provided, the script raises an exception.

"""


parser = argparse.ArgumentParser()
parser.add_argument("--workload", help="The workload to run")
parser.add_argument(
    "--resource-version", help="The version of the workload to run"
)
parser.add_argument(
    "--isa",
    help="The isa of the simulation to run. Options: x86, arm and riscv",
    choices=["x86", "riscv", "arm"],
    required=True,
)

args = parser.parse_args()
workload_id = args.workload
resource_version = args.resource_version
isa = args.isa

board = None

match isa:
    case "x86":
        board = X86DemoBoard()
    case "arm":
        board = ArmDemoBoard()
    case "riscv":
        board = RiscvDemoBoard()
    case _:
        raise Exception("The isa must be arm, x86 or riscv")

board.set_workload(
    obtain_resource(workload_id, resource_version=resource_version)
)
board.set_binary_to_run(
    application=BinaryResource(
        "./util/disk-image-validator/test_gem5_bridge.sh"
    ),
    args=[],
)

from gem5.simulate.exit_handler import ExitHandler


class Gem5BridgeDriverTestExitHandler(ExitHandler, hypercall_num=1234):
    def _process(self, simulator):
        print("Successfully called m5 hypercall without superuser privileges")
        print("Test passed")

    def _exit_simulation(self):
        return False


simulator = Simulator(board=board)

simulator.run()
