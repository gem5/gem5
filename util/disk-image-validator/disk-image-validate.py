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
from pathlib import Path

from helper import (
    parse_stats,
    update_json_with_stats,
    validate_npb_output,
)

import m5.options

from gem5.prebuilt.demo.arm_demo_board import ArmDemoBoard
from gem5.prebuilt.demo.riscv_demo_board import RiscvDemoBoard
from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

"""
gem5 Disk Image Validation Script

This script tests disk images to ensure they boot correctly and execute
hypercalls in the intended order.

Usage:
    build/ALL/gem5.opt util/disk-image-validator/disk-image-validate.py --isa <ISA> --workload <WORKLOAD_ID> --resource_version <RESOURCE_VERSION> [--validate-npb]

Arguments:
    --isa                The instruction set architecture (ISA) for the simulation.
                         Options: x86, arm, riscv.
    --workload           The workload ID to run.
    --resource_version   The version of the workload resource.
    --validate-npb       Validate the NAS Parallel Benchmarks (NPB) output (Optional).

Example:
    build/ALL/gem5.opt util/disk-img-validator/disk-image-validate.py --isa x86 --workload x86-ubuntu-24.04-npb-is-s --resource_version 3.0.0 --validate-npb

Requirements:
    - gem5 must be installed and properly configured.
    - The necessary prebuilt demo board classes must be available.
    - The gem5 resource system should be able to fetch the specified workload.
    - Python 3 must be installed.
    - Required Python modules: argparse, pathlib.

Functionality:
    - Selects the appropriate demo board (X86, Arm, or RISC-V) based on the given ISA.
    - Loads the specified workload and resource version (if provided).
    - Monitors key hypercalls:
        * Hypercall 1: Kernel boot
        * Hypercall 2: In `after_boot.sh`
        * Hypercall 3: Done running `after_boot.sh`
        * Hypercall 4: Start of Region of Interest (ROI)
        * Hypercall 5: End of ROI
    - Dumps and resets gem5 statistics at each hypercall.
    - Checks whether the hypercalls are executed in the correct order.
    - If `--validate-npb` is passed, it verifies the correctness of the NPB output.

Exit Conditions:
    - If the hypercalls are executed in the expected order, a success message is printed.
    - If the order is incorrect, an error message is printed.
    - If `--validate-npb` is used and validation fails, an error message is printed.

"""


parser = argparse.ArgumentParser()
parser.add_argument("--workload", help="The workload to run")
parser.add_argument(
    "--resource_version", help="The version of the workload to run"
)
parser.add_argument(
    "--validate-npb", action="store_true", help="Validate the NPB output"
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

simulator = Simulator(board=board)

exit_order = []
from gem5.simulate.exit_handler import ExitHandler


class KernelBootedDumpReset(ExitHandler, hypercall_num=1):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after kernel boot! Hypercall 1")
        m5.stats.dump()
        m5.stats.reset()
        exit_order.append(1)

    def _exit_simulation(self) -> bool:
        return False


class AfterBootDumpReset(ExitHandler, hypercall_num=2):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after Ubuntu boot! Hypercall 2")
        m5.stats.dump()
        m5.stats.reset()
        exit_order.append(2)

    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptDumpReset(ExitHandler, hypercall_num=3):
    def _process(self, simulator: "Simulator") -> None:
        print(
            "Dumping and resetting stats before exiting simulation! Hypercall 3"
        )
        m5.stats.dump()
        m5.stats.reset()
        exit_order.append(3)

    def _exit_simulation(self) -> bool:
        return True


class WorkBeginDumpReset(ExitHandler, hypercall_num=4):
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at ROI begin! Hypercall 4")
        exit_order.append(4)

    def _exit_simulation(self) -> bool:
        return False


class WorkEndDumpReset(ExitHandler, hypercall_num=5):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats at ROI end! Hypercall 5")
        m5.stats.dump()
        m5.stats.reset()
        exit_order.append(5)

    def _exit_simulation(self) -> bool:
        return False


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

if exit_order == [1, 2, 3] or exit_order == [1, 2, 4, 5, 3]:
    print("All exit events are called in expected order")
else:
    print("Exit events are not called in expected order")

if args.validate_npb:
    is_valid = validate_npb_output(
        output_file=terminal_out_path, workload=workload_id
    )
    if not is_valid:
        print("Validation failed.")
