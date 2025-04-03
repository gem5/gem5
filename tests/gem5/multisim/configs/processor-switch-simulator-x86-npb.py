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
from gem5.simulate.exit_handler import WorkBeginExitHandler
from gem5.simulate.simulator import Simulator

NUM_PROCESSES = 10

multisim.set_num_processes(NUM_PROCESSES)


for npb_workload in ["bt", "cg", "ep", "ft", "is", "lu", "mg", "sp", "ua"]:

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

    board.set_kernel_disk_workload(
        kernel=KernelResource(
            "/projects/gem5/new-base-imgs-w-hypercalls/x86-disk-image-24-04/6.8.0-52-generic-x86-ubuntu"
        ),
        disk_image=DiskImageResource(
            "/projects/gem5/new-base-imgs-w-hypercalls/disk-image-x86-npb/x86-ubuntu-npb"
        ),
        kernel_args=[
            "earlyprintk=ttyS0",
            "console=ttyS0",
            "lpj=7999923",
            "root=/dev/sda2",
        ],
        readfile_contents=f"/home/gem5/NPB3.4-OMP/bin/{npb_workload}.S.x; sleep 5;",
    )

    simulator = Simulator(board=board, id=f"process_x86_npb_{npb_workload}_s")
    print(f"In config script. simulator: {simulator}")

    multisim.add_simulator(simulator)


class WorkBeginDumpReset(WorkBeginExitHandler):
    def _process(self, simulator: "Simulator") -> None:
        m5.stats.dump()
        m5.stats.reset()
        print("Dumping and resetting stats at ROI begin! Hypercall 4")
        print("Switching processors at ROI begin! Hypercall 4")
        simulator.switch_processor()
        m5.scheduleTickExitFromCurrent(1_000_000)

    def _exit_simulation(self) -> bool:
        return False
