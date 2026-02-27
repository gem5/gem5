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


class McPATCpuBtbPowerModel(BaseMcPATPowerModel):
    def __init__(
        self, cpu: BaseCPU, act_energies: ActEnergyType, has_predictor: bool
    ):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuBtbPowerModel"
        self._has_predictor = has_predictor

    def print_mcpat(self, indent):
        energy = self.total_btb_energy()
        print(" " * indent + f"Branch Target Buffer:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(energy)} W\n"
        )

    def static_power(self) -> float:
        # Static Power in Watts, 1.0 is a placeholder.
        return 1.0

    def dynamic_power(self) -> float:
        energy = self.total_btb_energy()
        return self.convert_to_watts(energy)

    def total_btb_energy(self) -> float:
        if not self._has_predictor:
            return 0.0
        btb_reads = self.get_stat("branchPred.BTBLookups").total
        btb_writes = self.get_stat("branchPred.BTBHits").total
        return (
            self._act_energies["BTB"]["Read"] * btb_reads
            + self._act_energies["BTB"]["Write"] * btb_writes
        )
