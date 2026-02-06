from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class BaseAccelwattchMemoryPowerModel(BaseAccelwattchPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "BaseAccelwattchMemoryPowerModel"
        self._gpu_mem = gpu_memory

    def get_num_memory_controllers(self):
        return len(self._gpu_mem.get_memory_controllers())
