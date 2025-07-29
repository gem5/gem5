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
import re
from pathlib import Path

import m5.options

from gem5.prebuilt.demo.arm_demo_board import ArmDemoBoard
from gem5.prebuilt.demo.riscv_demo_board import RiscvDemoBoard
from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator

config_dir = Path(__file__).resolve().parent
script_path = str(config_dir / "test_script.sh")

parser = argparse.ArgumentParser()
parser.add_argument(
    "--isa",
    help="The isa of the simulation to run. Options: x86, arm and riscv",
    choices=["x86", "riscv", "arm"],
    required=True,
)
parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)
args = parser.parse_args()
isa = args.isa
resource_directory = args.resource_directory
board = None

match isa:
    case "x86":
        board = X86DemoBoard()
        workload = obtain_resource(
            "x86-ubuntu-24.04-boot-no-systemd",
            resource_version="4.0.0",
            resource_directory=resource_directory,
        )
    case "arm":
        board = ArmDemoBoard()
        workload = obtain_resource(
            "arm-ubuntu-24.04-boot-no-systemd",
            resource_version="2.0.0",
            resource_directory=resource_directory,
        )
    case "riscv":
        board = RiscvDemoBoard()
        workload = obtain_resource(
            "riscv-ubuntu-24.04-boot-no-systemd",
            resource_version="2.0.0",
            resource_directory=resource_directory,
        )
    case _:
        raise Exception("The isa must be arm, x86 or riscv")

workload.set_parameter("readfile", script_path)
board.set_workload(workload=workload)
simulator = Simulator(board=board)

simulator.run()

out_dir = m5.options.outdir
terminal_out_path = None

match isa:
    case "x86":
        terminal_out_path = Path(out_dir) / "board.pc.com_1.device"

    case "arm":
        terminal_out_path = Path(out_dir) / "board.terminal"

    case "riscv":
        terminal_out_path = Path(out_dir) / "board.platform.terminal"

    case _:
        raise Exception("The isa must be arm, x86 or riscv")

expected_script_output = "running test script from readfile_tests"

with open(terminal_out_path) as file:
    terminal_output = file.read()

    if re.search(expected_script_output, terminal_output):
        print("Readfile test passed!")
    else:
        print("Readfile test failed!")
