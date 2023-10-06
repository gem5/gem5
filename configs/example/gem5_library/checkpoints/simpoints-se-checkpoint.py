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
This configuration script shows an example of how to take checkpoints for
SimPoints using the gem5 stdlib. Simpoints are set via a Workload and the
gem5 SimPoint module will calculate where to take checkpoints based of the
SimPoints, SimPoints interval length, and the warmup instruction length.

This scipt builds a simple board with the gem5 stdlib with no cache and a
simple memory structure to take checkpoints. Some of the components, such as
cache hierarchy, can be changed when restoring checkpoints.

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
import argparse
from pathlib import Path

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.resources.resource import SimpointResource
from gem5.resources.workload import Workload
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.exit_event_generators import (
    save_checkpoint_generator,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(isa_required=ISA.X86)

parser = argparse.ArgumentParser(
    description="An example simpoint workload file path",
)

# The lone arguments is a file path to a directory to store the checkpoints.

parser.add_argument(
    "--checkpoint-path",
    type=str,
    required=False,
    default="se_checkpoint_folder/",
    help="The directory to store the checkpoint.",
)

args = parser.parse_args()

# When taking a checkpoint, the cache state is not saved, so the cache
# hierarchy can be changed completely when restoring from a checkpoint.
# By using NoCache() to take checkpoints, it can slightly improve the
# performance when running in atomic mode, and it will not put any restrictions
# on what people can do with the checkpoints.
cache_hierarchy = NoCache()

# Using simple memory to take checkpoints might slightly imporve the
# performance in atomic mode. The memory structure can be changed when
# restoring from a checkpoint, but the size of the memory must be maintained.
memory = SingleChannelDDR3_1600(size="2GB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=ISA.X86,
    # SimPoints only works with one core
    num_cores=1,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# board.set_workload(
#    Workload("x86-print-this-15000-with-simpoints")
#
# **Note: This has been removed until we update the resources.json file to
# encapsulate the new Simpoint format.
# Below we set the simpount manually.

board.set_se_simpoint_workload(
    binary=obtain_resource("x86-print-this"),
    arguments=["print this", 15000],
    simpoint=SimpointResource(
        simpoint_interval=1000000,
        simpoint_list=[2, 3, 4, 15],
        weight_list=[0.1, 0.2, 0.4, 0.3],
        warmup_interval=1000000,
    ),
)

dir = Path(args.checkpoint_path)
dir.mkdir(exist_ok=True)

simulator = Simulator(
    board=board,
    on_exit_event={
        # using the SimPoints event generator in the standard library to take
        # checkpoints
        ExitEvent.SIMPOINT_BEGIN: save_checkpoint_generator(dir),
    },
)

simulator.run()
