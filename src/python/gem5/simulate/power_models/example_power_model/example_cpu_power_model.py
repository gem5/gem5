# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from m5.objects import (
    BaseCPU,
    PowerModel,
    PowerModelPyFunc,
)

from .example_alu_power_model import ExampleAluPowerModel


class ExampleCpuPowerOn(PowerModelPyFunc):
    def __init__(
        self,
        cpu_obj: BaseCPU,
        pwr_interval: int,
        alu_energy: float = 1.5e-9,
        base_static_power: float = 3,
    ):
        super().__init__()
        self._alu = ExampleAluPowerModel(
            cpu_obj, alu_energy, base_static_power
        )
        self.pwr_interval = pwr_interval
        self.auto_start = pwr_interval > 0
        self.dyn = self.dynamic_power
        self.st = self.static_power

    def dynamic_power(self):
        if self.inPowerAtInterval():
            return self._alu.sampled_dynamic_power(
                self.getSampleDurationSeconds()
            )
        return self._alu.dynamic_power()

    def static_power(self):
        return self._alu.static_power()


class ExampleCpuPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class ExampleCpuPowerModel(PowerModel):
    def __init__(self, cpu: BaseCPU, pwr_interval: int):
        super().__init__()
        self.pm = [
            ExampleCpuPowerOn(cpu, pwr_interval),
            ExampleCpuPowerOff(),
            ExampleCpuPowerOff(),
            ExampleCpuPowerOff(),
        ]
