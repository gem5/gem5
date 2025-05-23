"""
The original was
gem5-6th-worktree/gem5-dev/staging-24.1.1.0/multisim-testing-sprint-2/processor-switch/x86-npb-ind-handlers/processor-switch-x86-npb-ind-handlers.py
"""

import m5

import gem5.utils.multisim as multisim
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import (
    DiskImageResource,
    KernelResource,
    obtain_resource,
)
from gem5.simulate.exit_handler import KernelBootedExitHandler
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 10

multisim.set_num_processes(NUM_PROCESSES)


for npb_workload in ["bt", "cg", "ep"]:  # , "ft", "is", "lu", "mg", "sp", "ua"

    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="16KiB",
        l1i_size="16KiB",
        l2_size="256KiB",
    )
    memory = SingleChannelDDR3_1600(size="3GiB")
    processor = SimpleSwitchableProcessor(
        starting_core_type=CPUTypes.KVM,
        switch_core_type=CPUTypes.TIMING,
        isa=ISA.X86,
        num_cores=1,
    )

    board = X86Board(
        clk_freq="3GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    board.set_workload(
        obtain_resource(
            f"x86-ubuntu-24.04-npb-{npb_workload}-s", resource_version="3.0.0"
        )
    )

    simulator = Simulator(board=board, id=f"process_x86_npb_{npb_workload}_s")
    print(f"In config script. simulator: {simulator}")

    multisim.add_simulator(simulator)


class KernelBootProcessorSwitch(KernelBootedExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at kernel boot! Hypercall 1")
        print("Switching processors at kernel boot! Hypercall 1")
        simulator.switch_processor()
        m5.scheduleTickExitFromCurrent(1_000_000)

    def _exit_simulation(self) -> bool:
        return False
