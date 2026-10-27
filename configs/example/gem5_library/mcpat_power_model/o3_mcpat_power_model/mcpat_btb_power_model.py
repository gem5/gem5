from m5.objects import (
    BaseO3CPU,
    PowerModel,
    PowerModelPyFunc,
)

from .mcpat_power_model import McPATPowerModel


class McPATBtbPower(McPATPowerModel):
    STATIC_UNITS = ("BTB",)

    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies,
        has_predictor,
        interval=0,
        interval_ticks=0,
        machine_model=None,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATBtbPower"
        self._has_predictor = has_predictor

    def print_mcpat(self, indent):
        energy = self.total_btb_energy()
        print(" " * indent + f"Branch Target Buffer:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(energy)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.total_btb_energy()
        return self.convert_to_watts(energy)

    def total_btb_energy(self) -> float:
        if not self._has_predictor:
            return 0
        btb_reads = self.get_stat("branchPred.BTBLookups").total
        btb_writes = self.get_stat("branchPred.BTBUpdates").total
        return (
            self._act_energies["BTB"]["Read"] * btb_reads
            + self._act_energies["BTB"]["Write"] * btb_writes
        )
