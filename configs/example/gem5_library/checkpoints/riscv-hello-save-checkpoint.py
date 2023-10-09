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
This gem5 configuation script creates a simple board to run the first
10^6 ticks of "riscv-hello" binary simulation and saves a checkpoint.
This configuration serves as an example of taking a checkpoint.

This is setup is the close to the simplest setup possible using the gem5
library. It does not contain any kind of caching, IO, or any non-essential
components.

Usage
-----

```
scons build/RISCV/gem5.opt
./build/RISCV/gem5.opt \
    configs/example/gem5_library/checkpoint/riscv-hello-save-checkpoint.py
```
"""
import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser()

parser.add_argument(
    "--checkpoint-path",
    type=str,
    required=False,
    default="riscv-hello-checkpoint/",
    help="The directory to store the checkpoint.",
)

args = parser.parse_args()

# This check ensures the gem5 binary is compiled to the RISCV ISA target.
# If not, an exception will be thrown.
requires(isa_required=ISA.RISCV)

# In this setup we don't have a cache. `NoCache` can be used for such setups.
cache_hierarchy = NoCache()

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MB")

# We use a simple Timing processor with one core.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=1
)

# The gem5 library simble board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a simple "Hello World!"
# program compiled to the RISCV ISA. The `Resource` class will automatically
# download the binary from the gem5 Resources cloud bucket if it's not already
# present.
board.set_se_binary_workload(
    # The `Resource` class reads the `resources.json` file from the gem5
    # resources repository:
    # https://github.com/gem5/gem5-resources.
    # Any resource specified in this file will be automatically retrieved.
    # At the time of writing, this file is a WIP and does not contain all
    # resources. Jira ticket: https://gem5.atlassian.net/browse/GEM5-1096
    obtain_resource("riscv-hello")
)

# Lastly we run the simulation.
max_ticks = 10**6
simulator = Simulator(board=board, full_system=False)
simulator.run(max_ticks=max_ticks)

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)

print("Taking a checkpoint at", args.checkpoint_path)
simulator.save_checkpoint(args.checkpoint_path)
print("Done taking a checkpoint")
