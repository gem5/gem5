# SPDX-License-Identifier: BSD-3-Clause
"""Run an ARM program with McPAT power attached to its CPU and caches.

Usage:
    build/ARM/gem5.opt configs/example/gem5_library/mcpat-power-model-example.py \
        --cpu-type minor /path/to/arm/program [program arguments ...]
"""

import argparse

from mcpat_power_model.gem5_adapter import derive_coefficients
from mcpat_power_model.inorder_mcpat_power_model.inorder_mcpat_cpu_power_model import (
    InorderMcPATCpuPowerModel,
)
from mcpat_power_model.l1l2_cache_pm.l1d_power_model import L1DMcPATPower
from mcpat_power_model.l1l2_cache_pm.l1i_power_model import L1IMcPATPower
from mcpat_power_model.l1l2_cache_pm.l2_power_model import L2McPATPower
from mcpat_power_model.l1l2_cache_pm.mcpat_cache_power_model import (
    CachePowerModel,
)
from mcpat_power_model.o3_mcpat_power_model.o3_mcpat_cpu_power_model import (
    O3McPATCpuPowerModel,
)
from mcpat_power_model.stage_duty_cycles import ARM_SCALAR_FP_STAT_ALIASES

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--cpu-type", choices=("timing", "minor", "o3"), default="minor"
)
parser.add_argument("binary", help="Local ARM executable")
parser.add_argument("program_args", nargs=argparse.REMAINDER)
args = parser.parse_args()

processor = SimpleProcessor(
    cpu_type=getattr(CPUTypes, args.cpu_type.upper()),
    isa=ISA.ARM,
    num_cores=1,
    clk_freq="2GHz",
)
cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="32KiB", l1i_size="32KiB", l2_size="256KiB"
)
board = SimpleBoard(
    clk_freq="2GHz",
    processor=processor,
    memory=SingleChannelDDR3_1600(size="256MiB"),
    cache_hierarchy=cache_hierarchy,
)
board.set_se_binary_workload(
    BinaryResource(local_path=args.binary), arguments=args.program_args
)

# The stdlib constructs the cache SimObjects during board connection.
# Attach power immediately afterward, before gem5 instantiates the objects.
connect_board = board._connect_things


def attach_power_models():
    connect_board()
    _, static_coefficients = derive_coefficients(board)
    model = static_coefficients.model
    cpu = processor.get_cores()[0].core
    cpu_model = (
        O3McPATCpuPowerModel
        if args.cpu_type == "o3"
        else InorderMcPATCpuPowerModel
    )
    cpu.power_state.default_state = "ON"
    cpu.power_model = cpu_model(
        cpu, model, stat_aliases=ARM_SCALAR_FP_STAT_ALIASES
    )
    for cache, power in (
        (cache_hierarchy.l1icaches[0], L1IMcPATPower),
        (cache_hierarchy.l1dcaches[0], L1DMcPATPower),
        (cache_hierarchy.l2cache, L2McPATPower),
    ):
        cache.power_state.default_state = "ON"
        cache.power_model = CachePowerModel(power, cache, model)


board._connect_things = attach_power_models
Simulator(board=board).run()
