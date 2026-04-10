# Copyright (c) 2026
# All rights reserved.

"""Run a local RISC-V performance benchmark binary in SE mode."""

import argparse
import sys
from pathlib import Path

from m5.objects import BranchPredictor, LTAGE, SimpleBTB

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import (
    get_cpu_type_from_str,
    get_cpu_types_str_set,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator


def _enable_riscv_o3_decoupled_frontend(processor) -> None:
    """
    O3 decoupled front-end (FDP), aligned with stdlib DecoupledProcessor.
    Enables BAC BTB-guided fetch targets and commit-time hw-loop BTB priming.
    TournamentBP does not implement branchPlaceholder — LTAGE is required.
    """
    for core in processor.cores:
        cpu = core.core
        if not hasattr(cpu, "decoupledFrontEnd"):
            continue
        cpu.minInstSize = 2
        cpu.fetchBufferSize = 16
        cpu.fetchTargetWidth = 32
        cpu.decoupledFrontEnd = True
        cpu.branchPred = BranchPredictor(
            instShiftAmt=1,
            btb=SimpleBTB(numEntries=8 * 1024, associativity=8),
            conditionalBranchPred=LTAGE(),
            requiresBTBHit=True,
            takenOnlyHistory=True,
        )


parser = argparse.ArgumentParser(
    description="Run a local RISC-V performance benchmark binary."
)
parser.add_argument("binary", type=str, help="Path to a local ELF binary.")
parser.add_argument(
    "cpu", type=str, choices=get_cpu_types_str_set(), help="The CPU type used."
)
parser.add_argument(
    "--no-o3-fdp",
    action="store_true",
    help="Keep the classic coupled O3 front-end (disables BTB-based fetch "
    "targets; hardware-loop BTB priming has no effect).",
)
parser.add_argument(
    "--no-cache",
    action="store_true",
    help="No L1/L2: CPU ports go straight to memory (old behavior). "
    "Default: 32KiB L1I + 32KiB L1D (8-way) + 256KiB shared L2 (16-way).",
)
args = parser.parse_args()

binary_path = Path(args.binary).resolve()
if not binary_path.is_file():
    raise FileNotFoundError(f"Binary does not exist: {binary_path}")

if args.no_cache:
    cache_hierarchy = NoCache()
else:
    # Classic two-level: matches many single-core SE studies; eases
    # memory-bound IPC (vs NoCache + DDR) for dot4-style kernels.
    cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
        l1i_size="32KiB",
        l1d_size="32KiB",
        l2_size="256KiB",
        l1i_assoc=8,
        l1d_assoc=8,
        l2_assoc=16,
    )
memory = SingleChannelDDR3_1600()
processor = SimpleProcessor(
    cpu_type=get_cpu_type_from_str(args.cpu), isa=ISA.RISCV, num_cores=1
)
if args.cpu == "o3" and not args.no_o3_fdp:
    _enable_riscv_o3_decoupled_frontend(processor)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_se_binary_workload(BinaryResource(local_path=str(binary_path)))

simulator = Simulator(board=board)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
sys.exit(simulator.get_last_exit_event_code())
