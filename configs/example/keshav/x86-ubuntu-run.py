from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
    kvm_required=True,
)

cache_hierarchy = MESITwoLevelCacheHierarchy(
    l1d_size="32KiB",
    l1d_assoc=8,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="256KiB",
    l2_assoc=16,
    num_l2_banks=1,
)

memory = SingleChannelDDR3_1600(size="2GiB")

processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=2,
)

for proc in processor.get_cores():
    proc.core.usePerf = False

board = X86Board(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# workload=obtain_resource("x86-ubuntu-24.04-boot-with-systemd");
workload = obtain_resource(
    "x86-ubuntu-24.04-npb-cg-s", resource_version="1.0.0"
)
board.set_workload(workload)

# to use our own resource that is not available
# here we can seperately specify kernal, disk image
# we can also paass an optional "readfile_contents" argument.
# This will be run a sbash script after system boots up
# board.set_kernal_disk_workload()


def exit_event_handler():
    print("First Exit: Kernal Booted")
    yield False
    # gem5 is now executing systemd startup
    print("Second Exit: Started 'after_boot.sh' script")
    # The after_boot.sh script is executed after the kernal and
    # systemd have booted. Here we switch the cpu type to TIMING
    print("Switching to TIMING CPU")
    processor.switch()
    yield False
    # gem5 is now executing 'after_boot.sh' script
    print("Third Exit: Finished 'after_boot.sh' script")
    # The after_boot.sh script will run a scrpt if it is passed via
    # readfile_contents. This is the last exit event before simulation exits
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        ExitEvent.EXIT: exit_event_handler(),
    },
)

simulator.run()
