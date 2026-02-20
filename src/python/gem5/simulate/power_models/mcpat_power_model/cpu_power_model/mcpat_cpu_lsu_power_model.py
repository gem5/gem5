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


class McPATCpuLsuPowerModel(BaseMcPATPowerModel):
    # avoid the use of default values
    def __init__(
        self, cpu: BaseCPU, act_energies, pipeline_act_factor, lsu_act_factor
    ):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuLsuPower"
        """ The Activity Factor of the LSU (default: 0.71): """
        self._lsu_act_factor = lsu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 4.0
        if isinstance(cpu, BaseO3CPU):
            self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        loadq_energy = self.loadq_energy()
        storeq_energy = self.storeq_energy()
        total_energy = (
            self.lsu_pipeline_energy() + loadq_energy + storeq_energy
        )
        """ LoadQ should effectively be half of StoreQ's energy """
        print(" " * indent + f"Load Store Unit")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"LoadQ")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(loadq_energy)} W\n"
        )
        print(" " * (indent + 4) + f"StoreQ")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(storeq_energy)} W\n"
        )

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        energy = self.lsu_pipeline_energy() + self.storeq_energy()
        if isinstance(self._simobj, BaseO3CPU):
            energy += self.loadq_energy()

        return self.convert_to_watts(energy)

    def storeq_energy(self) -> float:
        loads = self.get_stat("commitStats0.numLoadInsts").total
        stores = self.get_stat("commitStats0.numStoreInsts").total
        """ 'Accesses' considers the overhead for flush """
        accesses = (loads + stores) * 2
        return (
            accesses
            * (
                self._act_energies["LoadStoreQueue"]["Read"]
                + self._act_energies["LoadStoreQueue"]["Search"]
            )
            + accesses * self._act_energies["LoadStoreQueue"]["Write"]
        )

    def loadq_energy(self) -> float:
        loads = self.get_stat("commitStats0.numLoadInsts").total
        stores = self.get_stat("commitStats0.numStoreInsts").total
        accesses = loads + stores
        return (
            accesses
            * (
                self._act_energies["LoadStoreQueue"]["Read"]
                + self._act_energies["LoadStoreQueue"]["Search"]
            )
            + accesses * self._act_energies["LoadStoreQueue"]["Write"]
        )

    def lsu_pipeline_energy(self) -> float:
        cycles = self.get_stat("numCycles").total
        rtp_pipeline_coe = (
            self._pipeline_act_factor * cycles * self._lsu_act_factor
        )
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
