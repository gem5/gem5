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
    BaseO3CPU,
    Root,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuInstructionSchedulerPowerModel(BaseMcPATPowerModel):
    # avoid the use of default values
    def __init__(self, cpu: BaseO3CPU, act_energies):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuInstructionSchedulerPowerModel"

    def print_mcpat(self, indent):
        int_iw = self.int_inst_window_energy()
        fp_iw = self.fp_inst_window_energy()
        total_energy = int_iw + fp_iw
        print(" " * indent + f"Instruction Scheduler:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Instruction Window:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_iw)} W\n"
        )
        print(" " * (indent + 4) + f"FP Instruction Window:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_iw)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.int_inst_window_energy()
        energy += self.fp_inst_window_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def int_inst_window_energy(self) -> float:
        int_iw_reads = self.get_stat("intInstQueueReads").total
        int_iw_writes = self.get_stat("intInstQueueWrites").total
        int_iw_wakeups = self.get_stat("intInstQueueWakeupAccesses").total

        return (
            int_iw_reads * self._act_energies["IntInstWindow"]["Read"]
            + int_iw_writes * self._act_energies["IntInstWindow"]["Write"]
            + int_iw_wakeups * self._act_energies["IntInstWindow"]["Search"]
            + int_iw_reads * self._act_energies["SelLogic"]
        )

    def fp_inst_window_energy(self) -> float:
        fp_iw_reads = self.get_stat("fpInstQueueReads").total
        fp_iw_writes = self.get_stat("fpInstQueueWrites").total
        fp_iw_wakeups = self.get_stat("fpInstQueueWakeupAccesses").total

        return (
            fp_iw_reads * self._act_energies["FpInstWindow"]["Read"]
            + fp_iw_writes * self._act_energies["FpInstWindow"]["Write"]
            + fp_iw_wakeups * self._act_energies["FpInstWindow"]["Search"]
            + fp_iw_reads * self._act_energies["SelLogic"]
        )
