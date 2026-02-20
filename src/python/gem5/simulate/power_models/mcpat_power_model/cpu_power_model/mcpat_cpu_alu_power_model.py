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

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuAluPowerModel(BaseMcPATPowerModel):
    def __init__(self, cpu: BaseCPU, act_energies):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuAluPowerModel"
        """
        Note that for vec access energy I'm using
        McPAT's access energy for Mul/Div ops.
        I suspect this is actually larger, but this
        is a placeholder. Surprisingly, McPAT doesn't
        actually account for SIMD insts.
        """

    def print_mcpat(self, indent):
        issued_insts = self.get_stat("issuedInstType")
        int_energy = self.int_energy(issued_insts)
        fp_energy = self.fp_energy(issued_insts)
        mul_energy = self.mul_energy(issued_insts)
        print(" " * indent + f"Integer ALUs:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(int_energy)} W\n"
        )
        print(" " * indent + f"Floating Point Units (FPU):")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_energy)} W\n"
        )
        print(" " * indent + f"Complex ALUs (Mul/Div):")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(mul_energy)} W\n"
        )

    def dynamic_power(self) -> float:
        issued_insts = self.get_stat("issuedInstType")
        energy = (
            self.int_energy(issued_insts)
            + self.fp_energy(issued_insts)
            + self.mul_energy(issued_insts)
        )
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 1.0

    def int_energy(self, issued_insts) -> float:
        """Note: below is used to sort btwn mult/non-mult insts"""
        int_accesses = issued_insts.value[
            issued_insts.ysubnames.index("IntAlu")
        ]
        return int_accesses * self._act_energies["IntAlu"]

    def fp_energy(self, issued_insts) -> float:
        fp_adds = issued_insts.value[issued_insts.ysubnames.index("FloatAdd")]
        fp_mults = issued_insts.value[
            issued_insts.ysubnames.index("FloatMult")
        ]
        fp_maccs = issued_insts.value[
            issued_insts.ysubnames.index("FloatMultAcc")
        ]
        fp_divs = issued_insts.value[issued_insts.ysubnames.index("FloatDiv")]
        fp_misc = issued_insts.value[issued_insts.ysubnames.index("FloatMisc")]
        fp_cmp = issued_insts.value[issued_insts.ysubnames.index("FloatCmp")]
        fp_accesses = (
            fp_adds + fp_mults + fp_maccs + fp_divs + fp_misc + fp_cmp
        )
        return fp_accesses * self._act_energies["FpAlu"]

    def mul_energy(self, issued_insts) -> float:
        int_mults = issued_insts.value[issued_insts.ysubnames.index("IntMult")]
        int_divs = issued_insts.value[issued_insts.ysubnames.index("IntDiv")]
        return (int_mults + int_divs) * self._act_energies["ComplexAlu"]
