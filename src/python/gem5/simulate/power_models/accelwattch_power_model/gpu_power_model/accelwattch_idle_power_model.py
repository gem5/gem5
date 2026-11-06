from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchIdlePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchIdlePowerModel"
        self._num_units = 4.0
        self._num_pipelines = 1.0

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        energy = self.idle_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def idle_energy(self) -> float:
        time = self.get_time()
        return (
            self.get_avg_idle_cores()
            * self._scaling_factors["IDLE_CORE_POWER"]
            * time
        )
