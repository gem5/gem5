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
    Root,
)

from ..abstract_power_model import AbstractPowerModel


class ExampleAluPowerModel(AbstractPowerModel):
    def __init__(
        self,
        cpu: BaseCPU,
        alu_energy: float = 1.5e-9,
        base_static_power: float = 3,
    ):
        super().__init__(cpu)
        self.name = "ExampleAluPowerModel"
        self._alu_energy = alu_energy
        self._base_static_power = base_static_power
        self._prev_alu = 0

    def alu_accesses(self) -> float:
        return self.get_stat("executeStats0.numIntAluAccesses").total

    def alu_delta(self) -> float:
        alu = self.alu_accesses()
        delta = alu - self._prev_alu
        self._prev_alu = alu
        return delta

    def dynamic_power(self) -> float:
        return self.convert_to_watts(self.alu_accesses() * self._alu_energy)

    def sampled_dynamic_power(self, seconds: float) -> float:
        return self.alu_delta() * self._alu_energy / seconds

    def static_power(self) -> float:
        return self._base_static_power * self.count_int_alus()

    def count_int_alus(self) -> int:
        return sum(
            1
            for fu in self._simobj.executeFuncUnits.funcUnits
            for oc in fu.opClasses.opClasses
            if str(oc.opClass) == "IntAlu"
        )

    def convert_to_watts(self, value: float) -> float:
        return value / Root.getInstance().resolveStat("simSeconds").total
