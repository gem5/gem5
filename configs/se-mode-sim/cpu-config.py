from gem5.utils.requires import requires
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.memory.multi_channel import DualChannelDDR4_2400
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import MESITwoLevelCacheHierarchy
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.resources.workload import CustomWorkload
from gem5.simulate.simulator import Simulator
from gem5.simulate.exit_event import ExitEvent

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL
)

cache_hierarchy = MESITwoLevelCacheHierarchy(
    l1i_size="32KiB",
    l1i_assoc="8",
    l1d_size="32KiB",
    l1d_assoc="8",
    l2_size="256KiB",
    l2_assoc="4",
    num_l2_banks=1,
)
memory = DualChannelDDR4_2400("2GiB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3,
    num_cores=4, 
    isa=ISA.X86
)

board = SimpleBoard(
    clk_freq="3GHz", 
    processor=processor, 
    memory=memory, 
    cache_hierarchy=cache_hierarchy
)

workload = CustomWorkload(
    function = "set_se_binary_workload",
    parameters = {
    "binary" : Resource("x86-print-this"),
    "arguments" : ["hello", 6]
    },
)

board.set_se_binary_workload(workload)
simulator = Simulator(board=board)

simulator.run()