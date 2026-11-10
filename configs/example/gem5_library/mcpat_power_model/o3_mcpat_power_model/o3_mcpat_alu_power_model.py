from m5.objects import BaseO3CPU

from .base_power_model import AbstractPowerModel
from .mcpat_power_model import McPATPowerModel


class O3McPATAluPower(McPATPowerModel):
    STATIC_UNITS = ("IntAlu", "FpAlu", "ComplexAlu")

    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies,
        interval=0,
        interval_ticks=0,
        isEmbedded=True,
        machine_model=None,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATAluPower"
        """
        Note that for vec access energy I'm using
        McPAT's access energy for Mul/Div ops.
        I suspect this is actually larger, but this
        is a placeholder. Surprisingly, McPAT doesn't
        actually account for SIMD insts.
        """
        self._embedded: bool = isEmbedded

    def print_mcpat(self, indent):
        issued_insts = self.get_stat("issuedInstType")
        rows = (
            ("Integer ALUs", self.int_energy(issued_insts), "IntAluBase"),
            (
                "Floating Point Units (FPU)",
                self.fp_energy(issued_insts),
                "FpAluBase",
            ),
            (
                "Complex ALUs (Mul/Div)",
                self.mul_energy(issued_insts),
                "ComplexAluBase",
            ),
        )
        for name, energy, key in rows:
            power = self.convert_to_watts(energy)
            if not self._embedded and self.getExecutionTime() > 0:
                if key in self._act_energies:
                    power += self._act_energies[key]
            print(" " * indent + f"{name}:")
            print(" " * (indent + 2) + f"Runtime Dynamic = {power} W\n")

    def dynamic_power(self) -> float:
        issued_insts = self.get_stat("issuedInstType")
        power = self.convert_to_watts(
            self.int_energy(issued_insts)
            + self.fp_energy(issued_insts)
            + self.mul_energy(issued_insts)
        )
        # McPAT (logic.cc:548-583, 684): a non-embedded OOO core also adds a
        # constant base_energy*sckt_co_eff W per FU type to runtime dynamic.
        if not self._embedded and self.getExecutionTime() > 0:
            for key in ("IntAluBase", "FpAluBase", "ComplexAluBase"):
                if key in self._act_energies:
                    power += self._act_energies[key]
        return power

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
