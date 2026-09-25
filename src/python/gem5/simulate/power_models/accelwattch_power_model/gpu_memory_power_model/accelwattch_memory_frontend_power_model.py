from math import ceil

from m5.objects import (
    Shader,
)

from .base_accelwattch_memory_power_model import (
    BaseAccelwattchMemoryPowerModel,
)


class AccelwattchMemoryFrontendPowerModel(BaseAccelwattchMemoryPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self.name = "AccelwattchMemoryFrontendPowerModel"
        self._llc_block_size = (
            int(ceil(gpu._cache_line_size / 8.0)) + gpu._cache_line_size
        )
        self._data_bus_width = int(
            ceil(self._gpu_mem._dram[0].device_bus_width / 8.0)
        ) + int(self._gpu_mem._dram[0].device_bus_width)

    def dynamic_power(self) -> float:
        energy = (
            self.frontend_buffer_energy()
            + self.read_buffer_energy()
            + self.write_buffer_energy()
        )
        return self.convert_to_watts(energy)

    def frontend_buffer_energy(self) -> float:
        mem_ctrls = self.get_num_memory_controllers()
        mem_reads = (
            sum(
                self.get_memory_stat(f"mem_ctrl{i}.readReqs")
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_RD"]
        )
        mem_writes = (
            sum(
                self.get_memory_stat(f"mem_ctrl{i}.writeReqs")
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_WR"]
        )

        mem_reads = (
            mem_reads
            * self._llc_block_size
            * 8.0
            / self._data_bus_width
            * self._data_bus_width
            / 72
        )
        mem_writes = (
            mem_writes
            * self._llc_block_size
            * 8.0
            / self._data_bus_width
            * self._data_bus_width
            / 72
        )
        energy = (
            self._act_energies["memFrontendBuffer"]["Search"]
            * (mem_reads + mem_writes)
            + mem_reads * self._act_energies["memFrontendBuffer"]["Read"]
            + mem_writes * self._act_energies["memFrontendBuffer"]["Write"]
        )
        return energy

    def read_buffer_energy(self) -> float:
        mem_ctrls = self.get_num_memory_controllers()
        mem_reads = (
            sum(
                self.get_memory_stat(f"mem_ctrl{i}.readReqs")
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_RD"]
        )

        mem_reads = (
            mem_reads * self._llc_block_size * 8.0 / self._data_bus_width
        )
        energy = mem_reads * (
            self._act_energies["memRWBuffer"]["Read"]
            + self._act_energies["memRWBuffer"]["Write"]
        )
        return energy

    def write_buffer_energy(self) -> float:
        mem_ctrls = self.get_num_memory_controllers()
        mem_writes = (
            sum(
                self.get_memory_stat(f"mem_ctrl{i}.writeReqs")
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_WR"]
        )

        mem_writes = (
            mem_writes * self._llc_block_size * 8.0 / self._data_bus_width
        )
        energy = mem_writes * (
            self._act_energies["memRWBuffer"]["Read"]
            + self._act_energies["memRWBuffer"]["Write"]
        )
        return energy

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()
