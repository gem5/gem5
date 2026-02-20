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
    BaseO3CPU,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuMmuPowerModel(BaseMcPATPowerModel):
    # avoid the use of default values
    def __init__(
        self, cpu: BaseCPU, act_energies, pipeline_act_factor, mmu_act_factor
    ):
        super().__init__(cpu, act_energies)

        self.name = "McPATCpuMmuPower"

        """ The Activity Factor of the MMU (default: 0.71): """
        self._mmu_act_factor = mmu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 4.0
        if isinstance(cpu, BaseO3CPU):
            self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        itlb_energy = self.itlb_energy()
        dtlb_energy = self.dtlb_energy()
        total_energy = itlb_energy + dtlb_energy + self.mmu_pipeline_energy()

        print(" " * indent + f"Memory Management Unit")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Itlb")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(itlb_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Dtlb")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(dtlb_energy)} W\n"
        )

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        energy = (
            self.itlb_energy()
            + self.dtlb_energy()
            + self.mmu_pipeline_energy()
        )
        return self.convert_to_watts(energy)

    def mmu_pipeline_energy(self) -> float:
        """
        McPAT Considers the LSU to be in its' own stage
        Combined with the DCache and LSQ. So, the LSU
        and the MMU have their own pipeline costs.
        """
        cycles = self.get_stat("numCycles").total
        """ McPAT uses 0.5 + 0.5*lsu_act_factor for figuring out MMU pipeline costs... """
        rtp_pipeline_coe = (
            self._pipeline_act_factor
            * cycles
            * (0.5 + 0.5 * self._mmu_act_factor)
        )
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]

    def itlb_energy(self) -> float:
        misses = self.get_stat("mmu.itb.misses").total
        accesses = self.get_stat("mmu.itb.accesses").total
        return (
            accesses * self._act_energies["ITLB"]["Search"]
            + misses * self._act_energies["ITLB"]["Write"]
        )

    def dtlb_energy(self) -> float:
        misses = self.get_stat("mmu.dtb.misses").total
        accesses = self.get_stat("mmu.dtb.accesses").total
        return (
            accesses * self._act_energies["DTLB"]["Search"]
            + misses * self._act_energies["DTLB"]["Write"]
        )
