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

# m5 includes for power model utilities
from m5.objects import (
    PowerModel,
    PowerModelPyFunc,
    Root,
)

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# Below is the definition for a Power Model State
# (i.e., if a device is on, off or clock gated, this defines
# what dynamic/static power should look like for each of these states).

# With PowerModelPyFunc, you can do arithmetic operations very easily,
# or for a more interesting use might be using parameters of a component
# to scale certain energy values for your model.
# In this example, we try to grab the all of the IntegerALU functional units for Minor and scale some static power by the number of number of IntegerALUs.


class CpuPowerOn(PowerModelPyFunc):
    def __init__(
        self, cpu_obj, pwr_interval, alu_energy=1.5e-9, base_static_power=3
    ):
        super().__init__()
        self._cpu = cpu_obj
        self._alu_ae = alu_energy
        self._base_st_power = base_static_power
        self._prev_alu = 0
        self.pwr_interval = pwr_interval
        self.auto_start = pwr_interval > 0
        self.dyn = self.dynamic_power
        self.st = self.static_power

    def dynamic_power(self):
        alu = self._cpu.resolveStat("executeStats0.numIntAluAccesses").total
        if self.inPowerAtInterval():
            energy = (alu - self._prev_alu) * self._alu_ae
            self._prev_alu = alu
            return energy / self.getSampleDurationSeconds()
        time = Root.getInstance().resolveStat("simSeconds").total
        return alu * self._alu_ae / time

    def static_power(self):
        return self._base_st_power * self.count_fus()

    def count_fus(self):
        return sum(
            1
            for fu in self._cpu.executeFuncUnits.funcUnits
            for oc in fu.opClasses.opClasses
            if str(oc.opClass) == "IntAlu"
        )


class CpuPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


# CpuPowerModel takes on the actual definition of our PowerModel, where
# we need to be explicit what power model to use per state. In this case,
# becuase we only care about the ON state, we have explicitly defined a
# different, non-zero, power model.


class CpuPowerModel(PowerModel):
    def __init__(self, cpu, pwr_interval):
        super().__init__()
        self.pm = [
            CpuPowerOn(cpu, pwr_interval),
            CpuPowerOff(),
            CpuPowerOff(),
            CpuPowerOff(),
        ]


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
# We also ensure that we use the CpuPowerModelOn by enforcing the
# defaults state to be "ON"
core0 = processor.get_cores()[0].core
core0.power_state.default_state = "ON"
power_model = CpuPowerModel(core0, args.pwr_interval)
core0.power_model = power_model

board.set_se_binary_workload(
    obtain_resource("arm-hello64-static", resource_version="1.0.0")
)

simulator = Simulator(board=board)
simulator.run()

if args.pwr_interval > 0:
    power_model.pm[0].stopSampling()
