from gem5.utils.requires import requires
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory.multi_channel import DualChannelDDR4_2400
from gem5.components.cachehierarchies.ruby.mesi_three_level_cache_hierarchy import MESIThreeLevelCacheHierarchy
from gem5.components.processors.simple_processor import SimpleProcessor
# from gem5.components.processors.simple_switchable_processor import SimpleSwitchableProcessor
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.simulate.exit_event import ExitEvent

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_THREE_LEVEL
)

cache_hierarchy = MESIThreeLevelCacheHierarchy(
    l1i_size="32KiB",
    l1i_assoc="8",
    l1d_size="32KiB",
    l1d_assoc="8",
    l2_size="256KiB",
    l2_assoc="4",
    l3_size="2MiB",
    l3_assoc="16",
    num_l3_banks=1,
)

memory = DualChannelDDR4_2400("2GiB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3,
    num_cores=4, 
    isa=ISA.X86
)

# processor = SimpleSwitchableProcessor(
#     starting_core_type=CPUTypes.TIMING,
#     switch_core_type=CPUTypes.O3,
#     num_cores=4,
#     isa=ISA.X86
# )

board = X86Board(
    clk_freq="3GHz", 
    processor=processor, 
    memory=memory, 
    cache_hierarchy=cache_hierarchy
)

# command = "m5 exit;" \
#         + "echo 'This is running on O3 CPU cores.';" \
#         + "sleep 1;" \
#         + "m5 exit;"

# board.set_kernel_disk_workload(
#     kernel=obtain_resource("x86-linux-kernel-5.4.49"),
#     disk_image=obtain_resource("x86-ubuntu-18.04-img"),
#     readfile_contents=command
# )

binary = obtain_resource("x86-hello64-static")
board.set_se_binary_workload(binary) 

simulator = Simulator(
    board=board,
    # on_exit_event={
    #     ExitEvent.EXIT: (func() for func in [processor.switch])
    # }
)
simulator.run()



