# Copyright (c) 2026
# All rights reserved.

"""Run a local RISC-V performance benchmark binary in SE mode."""

import argparse
import sys
from pathlib import Path

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import (
    get_cpu_type_from_str,
    get_cpu_types_str_set,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator


parser = argparse.ArgumentParser(
    description="Run a local RISC-V performance benchmark binary."
)
parser.add_argument("binary", type=str, help="Path to a local ELF binary.")
parser.add_argument(
    "cpu", type=str, choices=get_cpu_types_str_set(), help="The CPU type used."
)
args = parser.parse_args()

binary_path = Path(args.binary).resolve()
if not binary_path.is_file():
    raise FileNotFoundError(f"Binary does not exist: {binary_path}")

cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()
processor = SimpleProcessor(
    cpu_type=get_cpu_type_from_str(args.cpu), isa=ISA.RISCV, num_cores=1
)

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
