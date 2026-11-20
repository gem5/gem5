from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchSchedulerPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchSchedulerPowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        energy = self.int_inst_window_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def int_inst_window_energy(self) -> float:
        int_insts = (
            sum(
                self.get_stat(f"CUs{i}.decodedIntInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_INT"]
        )
        fp_insts = (
            sum(
                self.get_stat(f"CUs{i}.decodedFpInsts")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["FP_INT"]
        )
        reads = writes = int_insts + fp_insts
        searches = 2 * (int_insts + fp_insts)

        return (
            reads * self._act_energies["IntInstWindow"]["Read"]
            + writes * self._act_energies["IntInstWindow"]["Write"]
            + searches * self._act_energies["IntInstWindow"]["Search"]
            + writes * self._act_energies["InstSel"]
        )
