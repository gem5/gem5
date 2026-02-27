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
)

from ..base_mcpat_power_model import (
    ActEnergyType,
    BaseMcPATPowerModel,
)


class McPATCpuRfPowerModel(BaseMcPATPowerModel):
    def __init__(self, cpu: BaseCPU, act_energies: ActEnergyType):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuRFPowerModel"

    def print_mcpat(self, indent):
        int_rf_energy = self.int_energy()
        fp_rf_energy = self.fp_energy()
        print(" " * indent + f"Register Files:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(int_rf_energy + fp_rf_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Integer RF:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_rf_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Floating Point RF:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_rf_energy)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.int_energy() + self.fp_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        # Placeholder for static power
        return 1.0

    def int_energy(self) -> float:
        reads = self.get_stat("executeStats0.numIntRegReads").total
        writes = self.get_stat("executeStats0.numIntRegWrites").total
        return (
            reads * self._act_energies["IntRegFile"]["Read"]
            + writes * self._act_energies["IntRegFile"]["Write"]
        )

    def fp_energy(self) -> float:
        reads = self.get_stat("executeStats0.numFpRegReads").total
        writes = self.get_stat("executeStats0.numFpRegWrites").total

        return (
            reads * self._act_energies["FpRegFile"]["Read"]
            + writes * self._act_energies["FpRegFile"]["Write"]
        )
