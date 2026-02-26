# Copyright (c) 2024 The Regents of the University of California
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

from pathlib import Path

import m5.debug

# system components
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400

# simulation components
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

"""

Usage:

gem5 -re 03-run-x86-SE.py

"""


# binary_path = Path("/workspaces/2024/materials/02-Using-gem5/\
# 03-running-in-gem5/02-annotate-this/complete/02-annotate-this")
binary_path = Path("/home/docker_share/gem5_mar20/ca_2025/roi")

# cache_hierarchy = PrivateL1CacheHierarchy(
#    l1d_size="64kB",
#    l1i_size="64kB",
# )

cache_hierarchy = PrivateL1SharedL2CacheHierarchy(
    l1d_size="64kB",
    l1i_size="64kB",
    l2_size="128kB",
)

memory = SingleChannelDDR4_2400("1GB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.O3,
    # cpu_type = CPUTypes.TIMING,
    num_cores=1,
    isa=ISA.X86,
)
print(f"----FUPool----\n{processor.cores[0]}")
print(f"----FUPool----\n{processor.cores[0].core}")
print(f"----FUPool----\n{processor.cores[0].core.fuPool}")
# print(f"----FUPool----\n{processor.cores[0].core.fuPool.maxOpLatencies}")
for unit in processor.cores[0].core.fuPool.FUList:
    for op in unit.opList:
        print(f"unit:{unit},op:{op.opClass},{op.opLat},{op.pipelined}")
        if str(op.opClass) == "IntMult":
            op.opLat = 100
            # op.pipelined=False;
            print(
                f"--Updated unit:{unit},op:{op.opClass},\
                        {op.opLat},{op.pipelined}"
            )
            # unit.count=1;

board = SimpleBoard(
    clk_freq="1GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_se_binary_workload(
    binary=BinaryResource(local_path=binary_path.as_posix())
)


# define a workbegin handler
def workbegin_handler():
    print("Workbegin handler")
    m5.debug.flags["ExecAll"].enable()
    yield False


#


# define a workend handler
def workend_handler():
    m5.debug.flags["ExecAll"].disable()
    yield False


#

simulator = Simulator(
    board=board,
    # setup handler for ExitEvent.WORKBEGIN and ExitEvent.WORKEND
    on_exit_event={
        ExitEvent.WORKBEGIN: workbegin_handler(),
        ExitEvent.WORKEND: workend_handler(),
    },
    #
)

simulator.run()

print("Simulation Done")
