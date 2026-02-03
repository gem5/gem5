from m5.objects import (
    BaseCPU,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuBtbPowerModel(BaseMcPATPowerModel):
    def __init__(self, cpu: BaseCPU, act_energies, has_predictor):
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
        """Returns static power in Watts"""
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
