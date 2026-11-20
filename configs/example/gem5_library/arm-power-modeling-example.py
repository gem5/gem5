# Copyright (c) 2026 - The Unviersity of Wisconsin
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
This gem5 configuation script creates and attaches a Power Model onto
an ARM CPU and then runs `arm-hello` from gem5-resources.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt configs/example/gem5_library/arm-power-modeling-example.py
./build/ALL/gem5.opt configs/example/gem5_library/arm-power-modeling-example.py \
    --pwr-interval 1000
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
from gem5.simulate.power_models.example_power_model.example_cpu_power_model import (
    ExampleCpuPowerModel,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser()
parser.add_argument(
    "--pwr-interval",
    type=int,
    default=0,
    help="Cycles between power samples. 0 disables sampling.",
)
args = parser.parse_args()

requires(isa_required=ISA.ARM)

cache_hierarchy = NoCache()

memory = SingleChannelDDR3_1600(size="32MiB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.MINOR, isa=ISA.ARM, num_cores=1, clk_freq="3GHz"
)


board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# In order to apply a power model onto a SimObject, the object you
# want to attach to must be an instance of `ClockedObject`. We directly
# get the object we want to model and attach a power model onto it.
# We also ensure that we use the ExampleCpuPowerOn by enforcing the
# defaults state to be "ON"
core0 = processor.get_cores()[0].core
core0.power_state.default_state = "ON"
power_model = ExampleCpuPowerModel(core0, args.pwr_interval)
core0.power_model = power_model

board.set_se_binary_workload(
    obtain_resource("arm-hello64-static", resource_version="1.0.0")
)

simulator = Simulator(board=board)
simulator.run()

if args.pwr_interval > 0:
    power_model.pm[0].stopSampling()
