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

from m5.objects import BaseO3CPU

from ..base_mcpat_power_model import (
    ActEnergyType,
    BaseMcPATPowerModel,
)


class McPATCpuRenamingUnitPowerModel(BaseMcPATPowerModel):
    """
    This class implements the power model for McPAT's renaming stage.
    This is separate from the execute stage, and is only for Out-of-Order
    CPUs.

    Even though McPAT supports Physical Register Files and Reservation Station
    based renaming, gem5 only implements the latter combined with a free list,
    so this implementation assumes that.
    """

    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies: ActEnergyType,
        pipeline_act_factor: float,
    ):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuRenamingUnitPower"

        # The Activity Factor of the Pipeline itself (default: 1.0)
        self._pipeline_act_factor = pipeline_act_factor
        # Number of Pipeline Stages for any Inorder CPU in McPAT
        self._num_units = 5.0

        # The number of pipelines our CPU has (assume 1)
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        int_frat = self.int_frat_energy()
        fp_frat = self.fp_frat_energy()
        int_fl = self.int_fl_energy()
        fp_fl = self.fp_fl_energy()
        total_energy = (
            int_frat + fp_frat + int_fl + fp_fl + self.rnu_pipeline_energy()
        )
        print(" " * indent + f"Renaming Unit:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Int Front End RAT:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_frat)} W\n"
        )
        print(" " * (indent + 4) + f"FP Front End RAT:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_frat)} W\n"
        )
        print(" " * (indent + 4) + f"Int Free List:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_fl)} W\n"
        )
        print(" " * (indent + 4) + f"FP Free List:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_fl)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = (
            self.int_frat_energy()
            + self.fp_frat_energy()
            + self.int_fl_energy()
            + self.fp_fl_energy()
            + self.rnu_pipeline_energy()
        )
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        # Placeholder value for static power
        return 1.0

    def int_frat_energy(self) -> float:
        int_rename_reads = self.get_stat("rename.intLookups").total
        int_rename_writes = self.get_stat("rename.intReturned").total

        return (
            int_rename_reads
            * (
                self._act_energies["IntFRAT"]["Search"]
                + self._act_energies["IntDCL"]
            )
            + int_rename_writes * self._act_energies["IntFRAT"]["Write"]
        )

    def fp_frat_energy(self) -> float:
        fp_rename_reads = self.get_stat("rename.fpLookups").total
        fp_rename_writes = self.get_stat("rename.fpReturned").total

        return (
            fp_rename_reads
            * (
                self._act_energies["FpFRAT"]["Search"]
                + self._act_energies["FpDCL"]
            )
            + fp_rename_writes * self._act_energies["FpFRAT"]["Write"]
        )

    def int_fl_energy(self) -> float:
        int_rename_reads = self.get_stat("rename.intLookups").total
        int_rename_writes = self.get_stat("rename.intReturned").total
        return (
            int_rename_reads * self._act_energies["IntFreeList"]["Read"]
            + 2
            * int_rename_writes
            * self._act_energies["IntFreeList"]["Write"]
        )

    def fp_fl_energy(self) -> float:
        fp_rename_reads = self.get_stat("rename.fpLookups").total
        fp_rename_writes = self.get_stat("rename.fpReturned").total
        return (
            fp_rename_reads * self._act_energies["FpFreeList"]["Read"]
            + 2 * fp_rename_writes * self._act_energies["FpFreeList"]["Write"]
        )

    def rnu_pipeline_energy(self) -> float:
        cycles = self.get_stat("numCycles").total
        rtp_pipeline_coe = self._pipeline_act_factor * cycles
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
