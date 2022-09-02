# Copyright (c) 2022 The Regents of the University of California
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
This configuation script shows an example of how to restore a checkpoint that
was taken for SimPoints in the
configs/example/gem5_library/checkpoints/simpoints-se-checkpoint.py.
The SimPoints, SimPoints interval length, and the warmup instruction length
are passed into the SimPoint module, so the SimPoint object will store and
calculate the warmup instruction length for each SimPoints based on the
avaliable instructions before reaching the start of the SimPoint. With the
Simulator module, exit event will be generated to stop when the warmup session
ends and the SimPoints interval ends.

This scipt builds a more complex board than the board used for taking
checkpoint.

Usage
-----

```
scons build/X86/gem5.opt
./build/X86/gem5.opt \
    configs/example/gem5_library/checkpoints/simpoints-se-checkpoint.py

./build/X86/gem5.opt \
    configs/example/gem5_library/checkpoints/simpoints-se-restore.py
```
"""

import imp
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.resources.resource import Resource

from gem5.utils.simpoint import SimPoint
from pathlib import Path
from m5.stats import reset, dump

requires(isa_required=ISA.X86)

# The cache hierarchy can be different from the cache hierarchy used in taking
# the checkpoints
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="256kB",
)

# The memory structure can be different from the memory structure used in
# taking the checkpoints, but the size of the memory must be maintained
memory = DualChannelDDR4_2400(size="2GB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=1,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

simpoint = SimPoint(
    simpoint_list=[2, 3, 5, 15],
    weight_list=[0.1, 0.2, 0.4, 0.3],
    simpoint_interval=1000000,
    warmup_interval=1000000,
)

board.set_se_binary_workload(
    binary=Resource("x86-print-this"), arguments=["print this", 15000]
)

# configs/example/gem5_library/checkpoints/simpoints-se-checkpoint.py has to
# run before running this script.
# In here, it will get the path of the first SimPoint checkpoint taken by the
# simpoints-se-checkpoint.py
dir = Path("se_checkpoint_folder")
subfolder = [int(str(name).split(".")[1]) for name in dir.iterdir()]
dir = Path(dir / f"cpt.{min(subfolder)}").as_posix()


def max_inst():
    warmed_up = False
    while True:
        if warmed_up:
            print("end of SimPoint interval")
            yield True
        else:
            print("end of warmup, starting to simulate SimPoint")
            warmed_up = True
            # Schedule a MAX_INSTS exit event during the simulation
            simulator.schedule_max_insts(simpoint.get_simpoint_interval())
            dump()
            reset()
            yield False


simulator = Simulator(
    board=board,
    checkpoint_path=dir,
    on_exit_event={ExitEvent.MAX_INSTS: max_inst()},
)

# Schedule a MAX_INSTS exit event before the simulation begins the
# schedule_max_insts function only schedule event when the instruction length
# is greater than 0.
# In here, it schedules an exit event for the first SimPoint's warmup
# instructions
simulator.schedule_max_insts(simpoint.get_warmup_list()[0], True)
simulator.run()
