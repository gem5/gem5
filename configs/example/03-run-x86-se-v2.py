from pathlib import Path

import m5

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.boards.x86_board import X86Board
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    BinaryResource,
    obtain_resource,
)
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

processor = SimpleProcessor(
    isa=ISA.X86,
    # isa=ISA.ARM,
    # cpu_type=CPUTypes.TIMING,
    cpu_type=CPUTypes.O3,
    num_cores=1,
)
"""
cache_hierarchy=MESITwoLevelCacheHierarchy(
        l1i_size="32KiB",
        l1i_assoc=8,
        l1d_size="32KiB",
        l1d_assoc=8,
        l2_size="256KiB",
        l2_assoc=8,
        num_l2_banks=2,
        )
"""

cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="32KiB",
    l1d_assoc=8,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="256KiB",
    # num_l2_banks=2,
)

memory = SingleChannelDDR4_2400(size="1GiB")

# "SimpleBoard" works only with classic cache hierarchy
# board=SimpleBoard(
board = X86Board(
    clk_freq="1GHz",
    processor=processor,
    cache_hierarchy=cache_hierarchy,
    memory=memory,
)

binary_path = Path(
    "/home/docker_share/gem5_mar15/materials/02-using-gem5/\
    03-running-in-gem5/02-annotate-this-x86"
)

# for cross compiler dynamic compilation. we need to redirect
# the library path to the actual cross compiled library in the host
# link_types=['static','dynamic']; link_type=link_types[1]
link_types = ["static", "dynamic"]
link_type = link_types[0]
if link_type == "dynamic":
    from m5.core import setInterpDir
    from m5.objects import RedirectPath

    print("Time to redirect the library path at host to cross compile library")
    setInterpDir("/usr/aarch64-linux-gnu/")
    board.redirect_paths[
        RedirectPath(app_path="lib", host_paths=["usr/aarch64-linux-gnu/lib"])
    ]
# binary_path=Path('/home/docker_share/gem5_mar15/materials/02-using-gem5/\
# 03-running-in-gem5/02-annotate-this-arm64'+'-'+link_type)

# as_posix() provides path independency over platforms such as windows, linux
binary = BinaryResource(local_path=binary_path.as_posix())

board.set_se_binary_workload(binary)


# dfine a workbegin handler
def workbegin_handler():
    print("Keshav Workbegin handler")
    print(simulator.get_last_exit_event_cause())
    m5.debug.flags["ExecAll"].enable()
    # m5.debug.flags['O3PipeView'].enable()
    # if we yield true from a exit event the simulation will
    # be terminated immedealy after that
    yield False


#
# define a workend handler
def workend_handler():
    print("keshav Worend handler")
    # m5.debug.flags['ExecAll'].disable()
    m5.debug.flags["O3PipeView"].disable()
    yield False


#

simulator = Simulator(
    board=board,
    # setup handler for workbegin and workend
    # specfying the task to be executed on exit
    # events of type WORKBEGIN, WORKEND
    on_exit_event={
        ExitEvent.WORKBEGIN: workbegin_handler(),
        ExitEvent.WORKEND: workend_handler(),
    },
)

simulator.run(20_000_000_000)
# simulator.run(20_000)
