from gem5.isas import ISA
from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.memory.multi_channel import DualChannelDDR4_2400
from gem5.components.cachehierarchies.ruby.mesi_three_level_cache_hierarchy import MESIThreeLevelCacheHierarchy
from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.se_binary_workload import *
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.components.processors.simple_processor import  SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes

requires(
        coherence_protocol_required=CoherenceProtocol.MESI_THREE_LEVEL,
        isa_required=ISA.X86,
)

memory = DualChannelDDR4_2400("2GiB")

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

processor=SimpleProcessor(
    num_cores=1,
    cpu_type=CPUTypes.O3,
    isa=ISA.X86,
)

board = AbstractBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

binary = obtain_resource("x86-hello64-static")
board.set_se_binary_workload(binary)

simulator = Simulator(
    board=board,
    full_system=False,
)

simulator.run()
