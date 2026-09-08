from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchFPUPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchFPUPowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        energy = self.fpu_energy() + self.dpu_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def fp_static_coeff(self) -> float:
        fp_accs = self.get_fp_accesses()
        tot_accs = fp_accs + self.get_dp_accesses()
        if tot_accs == 0:
            return 0
        coeff = self._act_energies["FPU"] * (fp_accs / tot_accs)
        return self.convert_to_watts(coeff)

    def dp_static_coeff(self) -> float:
        dp_accs = self.get_dp_accesses()
        tot_accs = dp_accs + self.get_fp_accesses()
        if tot_accs == 0:
            return 0
        coeff = self._act_energies["FPU"] * (dp_accs / tot_accs)
        return self.convert_to_watts(coeff)

    def get_fp_accesses(self) -> float:
        fpu_accesses = (
            sum(self.get_stat(f"CUs{i}.fpInsts") for i in range(self._num_cus))
            * self._scaling_factors["FP_ACC"]
        )
        return fpu_accesses

    def get_dp_accesses(self) -> float:
        dpu_accesses = (
            sum(self.get_stat(f"CUs{i}.dpInsts") for i in range(self._num_cus))
            * self._scaling_factors["DP_ACC"]
        )
        return dpu_accesses

    def fpu_energy(self) -> float:
        fpu_accesses = self.get_fp_accesses()
        return fpu_accesses * self._act_energies["FPU"]

    def dpu_energy(self) -> float:
        dpu_accesses = self.get_dp_accesses()
        return dpu_accesses * self._act_energies["FPU"]
