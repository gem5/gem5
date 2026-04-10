# Copyright (c) 2026
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

"""Run a local RISC-V binary in SE mode."""

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
    description="Run a local RISC-V custom ISA regression binary."
)
parser.add_argument("binary", type=str, help="Path to a local ELF binary.")
parser.add_argument(
    "cpu", type=str, choices=get_cpu_types_str_set(), help="The CPU type used."
)
parser.add_argument(
    "--no-o3-fdp",
    action="store_true",
    help="Use coupled O3 front-end instead of decoupled (FDP).",
)
parser.add_argument(
    "--no-cache",
    action="store_true",
    help="No L1/L2 (direct-to-memory). Default: 32KiB L1I/L1D + 256KiB L2.",
)
args = parser.parse_args()

binary_path = Path(args.binary).resolve()
if not binary_path.is_file():
    raise FileNotFoundError(f"Binary does not exist: {binary_path}")

if args.no_cache:
    cache_hierarchy = NoCache()
else:
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
