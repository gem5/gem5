# Copyright (c) 2023 The Regents of the University of California
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
"""
This script is used for running a traffic generator connected to the
DRAMSys simulator.

**Important Note**: DRAMSys must be compiled into the gem5 binary to use the
DRRAMSys simulator. Please consult 'ext/dramsys/README' on how to compile
correctly. If this is not done correctly this script will run with error.
"""
import m5
from gem5.components.boards.test_board import TestBoard
from gem5.components.memory import DRAMSysMem
from gem5.components.processors.linear_generator import LinearGenerator
from m5.objects import Root

memory = DRAMSysMem(
    configuration="ext/dramsys/DRAMSys/DRAMSys/"
    "library/resources/simulations/ddr4-example.json",
    resource_directory="ext/dramsys/DRAMSys/DRAMSys/library/resources",
    recordable=True,
    size="4GB",
)

generator = LinearGenerator(
    duration="250us",
    rate="40GB/s",
    num_cores=1,
    max_addr=memory.get_size(),
)
board = TestBoard(
    clk_freq="3GHz",
    generator=generator,
    memory=memory,
    cache_hierarchy=None,
)

root = Root(full_system=False, system=board)
board._pre_instantiate()
m5.instantiate()
generator.start_traffic()
exit_event = m5.simulate()
