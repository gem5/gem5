from math import ceil

from m5.objects import (
    Shader,
)

from .base_accelwattch_memory_power_model import (
    BaseAccelwattchMemoryPowerModel,
)


class AccelwattchMemoryBackendPowerModel(BaseAccelwattchMemoryPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self.name = "AccelwattchMemoryBackendPowerModel"
        self._llc_block_size = (
            int(ceil(gpu._cache_line_size / 8.0)) + gpu._cache_line_size
        )
        self._data_bus_width = int(
            ceil(self._gpu_mem._dram[0].device_bus_width / 8.0)
        ) + int(self._gpu_mem._dram[0].device_bus_width)
        self._tdp = 0.0

        # Value below comes from AW's initial power/area estimates
        self._power_constant = 1.1512e-12 * self._data_bus_width

    def calc_tdp(self) -> float:
        reads = writes = (
            0.5
            * self.get_num_memory_controllers()
            / self._gpu_mem._num_channels
        )
        return (reads + writes) * self._power_constant

    def mem_refresh_overhead(self) -> float:
        execTime = self.get_time()
        # AW grabs the clock rate of the DIMM-IO bus, then multiplies it
        # by 2 because of double-pumping
        mem_clk_rate = 2 * int(1 / float(str(self._gpu_mem._dram[0].tCK)))
        return (
            self.calc_tdp()
            * 0.1
            * mem_clk_rate
            * self.get_num_memory_controllers()
            * execTime
        )

    def dynamic_power(self) -> float:
        mem_ctrls = self.get_num_memory_controllers()
        mem_reads = (
            sum(
                self.get_stat(f"mem_ctrl{i}.readReqs", self._gpu_mem)
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_RD"]
        )
        mem_writes = (
            sum(
                self.get_stat(f"mem_ctrl{i}.writeReqs", self._gpu_mem)
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_WR"]
        )
        energy = (
            (mem_reads + mem_writes)
            * self._llc_block_size
            * 8.0
            / self._data_bus_width
            * self._power_constant
        ) + self.mem_refresh_overhead()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()
