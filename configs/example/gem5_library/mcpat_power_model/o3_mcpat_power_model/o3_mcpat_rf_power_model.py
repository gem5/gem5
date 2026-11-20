from m5.objects import BaseO3CPU

from .base_power_model import AbstractPowerModel
from .mcpat_power_model import McPATPowerModel


class O3McPATRfPower(McPATPowerModel):
    STATIC_UNITS = ("IntRegFile", "FpRegFile")

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies,
        interval=0,
        interval_ticks=0,
        machine_model=None,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        # _rf isn't really needed since you have `_simobj`
        self.name = "InorderMcPatRFPowerModel"

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

    def int_energy(self) -> float:
        reads = sum(
            self.get_stat(f"executeStats{t}.numIntRegReads").total
            for t in range(self._simobj.numThreads)
        )
        writes = sum(
            self.get_stat(f"executeStats{t}.numIntRegWrites").total
            for t in range(self._simobj.numThreads)
        )
        return (
            reads * self._act_energies["IntRegFile"]["Read"]
            + writes * self._act_energies["IntRegFile"]["Write"]
        )

    def fp_energy(self) -> float:
        reads = sum(
            self.get_stat(f"executeStats{t}.numFpRegReads").total
            for t in range(self._simobj.numThreads)
        )
        writes = sum(
            self.get_stat(f"executeStats{t}.numFpRegWrites").total
            for t in range(self._simobj.numThreads)
        )

        return (
            reads * self._act_energies["FpRegFile"]["Read"]
            + writes * self._act_energies["FpRegFile"]["Write"]
        )
