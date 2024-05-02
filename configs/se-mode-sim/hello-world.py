from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from m5.objects.BranchPredictor import *
from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

# Obtain the components.
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600("1GiB")
processor = SimpleProcessor(
    cpu_type=CPUTypes.O3, 
    num_cores=2,
    isa=ISA.X86,
)

# Add them to the board.
board = SimpleBoard(
    clk_freq="3GHz", 
    processor=processor, 
    memory=memory, 
    cache_hierarchy=cache_hierarchy
)

# bp = Param.BranchPredictor(
#         BiModeBP(numThreads=Parent.numThreads), "Branch Predictor"
#     )

# for cores in processor.get_cores():
#     cores.core.branchPred = bp

# Set the workload.
binary = obtain_resource("x86-hello64-static")
board.set_se_binary_workload(binary)

# Setup the Simulator and run the simulation.
simulator = Simulator(board=board)
simulator.run()