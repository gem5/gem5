from m5.objects import (
    BaseCPU,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuTournamentBpPowerModel(BaseMcPATPowerModel):
    def __init__(self, cpu: BaseCPU, act_energies, has_predictor):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuTournamentBpPowerModel"
        self._has_predictor = has_predictor

    def print_mcpat(self, indent):
        gp_energy = self.total_glob_pred_energy()
        l1_energy = self.total_l1_local_pred_energy()
        l2_energy = self.total_l2_local_pred_energy()
        chooser_energy = self.total_chooser_pred_energy()
        ras_energy = self.total_ras_pred_energy()

        total_energy = (
            gp_energy + l1_energy + l2_energy + chooser_energy + ras_energy
        )

        print(" " * indent + f"Branch Predictor:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )

        print(" " * (indent + 4) + f"Global Predictor:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(gp_energy)} W\n"
        )

        print(" " * (indent + 4) + f"L1_Local Predictor:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(l1_energy)} W\n"
        )

        print(" " * (indent + 4) + f"L2_Local Predictor:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(l2_energy)} W\n"
        )

        print(" " * (indent + 4) + f"Chooser:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(chooser_energy)} W\n"
        )

        print(" " * (indent + 4) + f"RAS:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(ras_energy)} W\n"
        )

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        total_energy = (
            +self.total_glob_pred_energy()
            + self.total_l1_local_pred_energy()
            + self.total_l2_local_pred_energy()
            + self.total_chooser_pred_energy()
            + self.total_ras_pred_energy()
        )

        return self.convert_to_watts(total_energy)

    def total_glob_pred_energy(self) -> float:
        if not self._has_predictor:
            return 0
        predicts = self.get_stat("branchPred.condPredicted").total
        mispredicts = (
            self.get_stat("branchPred.condIncorrect").total + 0.1 * predicts
        )
        return (
            self._act_energies["GlobalPred"]["Read"] * predicts
            + self._act_energies["GlobalPred"]["Write"] * mispredicts
        )

    def total_l1_local_pred_energy(self) -> float:
        if not self._has_predictor:
            return 0
        predicts = self.get_stat("branchPred.condPredicted").total
        mispredicts = (
            self.get_stat("branchPred.condIncorrect").total + 0.1 * predicts
        )
        return (
            self._act_energies["L1LocalPred"]["Read"] * predicts
            + self._act_energies["L1LocalPred"]["Write"] * mispredicts
        )

    def total_l2_local_pred_energy(self) -> float:
        if not self._has_predictor:
            return 0
        predicts = self.get_stat("branchPred.condPredicted").total
        mispredicts = (
            self.get_stat("branchPred.condIncorrect").total + 0.1 * predicts
        )
        return (
            self._act_energies["L2LocalPred"]["Read"] * predicts
            + self._act_energies["L2LocalPred"]["Write"] * mispredicts
        )

    def total_chooser_pred_energy(self) -> float:
        if not self._has_predictor:
            return 0
        predicts = self.get_stat("branchPred.condPredicted").total
        mispredicts = (
            self.get_stat("branchPred.condIncorrect").total + 0.1 * predicts
        )
        return (
            self._act_energies["ChooserPred"]["Read"] * predicts
            + self._act_energies["ChooserPred"]["Write"] * mispredicts
        )

    def total_ras_pred_energy(self) -> float:
        ras_rws = self.get_stat("commitStats0.functionCalls").total
        return (
            self._act_energies["RAS"]["Read"] * ras_rws
            + self._act_energies["RAS"]["Write"] * ras_rws
        )
