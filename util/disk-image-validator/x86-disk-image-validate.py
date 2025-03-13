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

from gem5.prebuilt.demo.x86_demo_board import X86DemoBoard
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

"""
This script is used to test disk images and make sure that the disks boot and
call the hypercalls in intended order.
"""

parser = argparse.ArgumentParser()
parser.add_argument("--workload", help="The workload to run")
parser.add_argument(
    "--resource_version", help="The version of the workload to run"
)
parser.add_argument(
    "--validate-npb", action="store_true", help="Validate the NPB output"
)

args = parser.parse_args()
workload_id = args.workload
resource_version = args.resource_version

# Setup the board for Full-System X86 simulation
board = X86DemoBoard()
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
terminal_out_path = Path(out_dir) / "board.pc.com_1.device"

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
