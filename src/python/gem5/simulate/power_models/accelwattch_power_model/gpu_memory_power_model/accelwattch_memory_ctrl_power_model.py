from math import ceil

from m5.objects import Shader

from .accelwattch_base_memory_power_model import (
    AccelwattchBaseMemoryPowerModel,
)
from .accelwattch_memory_backend_power_model import (
    AccelwattchMemoryBackendPower,
)
from .accelwattch_memory_frontend_power_model import (
    AccelwattchMemoryFrontendPower,
)
from .accelwattch_memory_phy_power_model import (
    AccelwattchMemoryPhyPower,
)


class AccelwattchMemoryCtrlPowerModel(AccelwattchBaseMemoryPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self.name = "AccelwattchMemoryCtrlPowerModel"
        self._llc_block_size = (
            int(ceil(gpu._cache_line_size / 8.0)) + gpu._cache_line_size
        )
        self._data_bus_width = int(
            ceil(self._gpu_mem._dram[0].device_bus_width / 8.0)
        ) + int(self._gpu_mem._dram[0].device_bus_width)
        self._frontend = AccelwattchMemoryFrontendPowerModel(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self._backend = AccelwattchMemoryBackendPowerModel(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self._phy = AccelwattchMemoryPhyPowerModel(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )

    def dynamic_power(self) -> float:
        return (
            self._frontend.dynamic_power()
            + self._backend.dynamic_power()
            + self._phy.dynamic_power()
        )

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self._frontend.reset_stats_dict()
        self._backend.reset_stats_dict()
        self._phy.reset_stats_dict()
