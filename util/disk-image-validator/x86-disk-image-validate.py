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

# Accept command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--workload", help="The workload to run")
parser.add_argument(
    "--resource_version", help="The version of the workload to run"
)
parser.add_argument(
    "--validate-npb", action="store_true", help="Validate the NPB output"
)
parser.add_argument(
    "--output-stats", action="store_true", help="Output stats to JSON"
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

from gem5.simulate.exit_handler import ExitHandler


class KernelBootedDumpReset(ExitHandler, hypercall_num=1):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after kernel boot! Hypercall 1")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


class AfterBootDumpReset(ExitHandler, hypercall_num=2):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats after Ubuntu boot! Hypercall 2")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


class AfterBootScriptDumpReset(ExitHandler, hypercall_num=3):
    def _process(self, simulator: "Simulator") -> None:
        print(
            "Dumping and resetting stats before exiting simulation! Hypercall 3"
        )
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return True


class WorkBeginDumpReset(ExitHandler, hypercall_num=4):
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at ROI begin! Hypercall 4")

    def _exit_simulation(self) -> bool:
        return False


class WorkEndDumpReset(ExitHandler, hypercall_num=5):
    def _process(self, simulator: "Simulator") -> None:
        print("Dumping and resetting stats at ROI end! Hypercall 5")
        m5.stats.dump()
        m5.stats.reset()

    def _exit_simulation(self) -> bool:
        return False


simulator.run()

out_dir = m5.options.outdir
terminal_out_path = Path(out_dir) / "board.pc.com_1.device"

if args.validate_npb:
    is_valid = validate_npb_output(
        output_file=terminal_out_path, workload=workload_id
    )
    if not is_valid:
        print("Validation failed.")

if args.output_stats:
    stats_file = Path(out_dir) / "stats.txt"
    stats = parse_stats(stats_file)
    print(stats)
    update_json_with_stats(
        json_file="/home/harshilp/worktrees/disk-image-validate/hypercall_workloads.json",
        workload_id=workload_id,
        version=resource_version,
        stats=stats,
        output_file="updated_out_with_stats.json",
    )
