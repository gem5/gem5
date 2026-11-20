from m5.objects import Shader

from .base_accelwattch_memory_power_model import (
    BaseAccelwattchMemoryPowerModel,
)


class AccelwattchDRAMPowerModel(BaseAccelwattchMemoryPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self.name = "AccelwattchDRAMPowerModel"

    def dynamic_power(self) -> float:
        energy = self.dram_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def dram_energy(self) -> float:
        mem_ctrls = self.get_num_memory_controllers()
        reads = (
            sum(
                self.get_stat(f"mem_ctrl{i}.readReqs", self._gpu_mem)
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_RD"]
        )
        writes = (
            sum(
                self.get_stat(f"mem_ctrl{i}.writeReqs", self._gpu_mem)
                for i in range(mem_ctrls)
            )
            * self._scaling_factors["MEM_WR"]
        )

        """ Precharge stat added exclusively for DRAM below.
            Don't really know what the difference between
            dram and dram_2 is? Turns out, the
            number of R/Ws of dram and dram 2 sum to the
            above of ReadReqs and WriteReqs respectively.
        """
        precharges = (
            sum(
                self.get_stat(f"mem_ctrl{i}.dram.precharges", self._gpu_mem)
                for i in range(mem_ctrls)
            )
            + sum(
                self.get_stat(f"mem_ctrl{i}.dram_2.precharges", self._gpu_mem)
                for i in range(mem_ctrls)
            )
        ) * self._scaling_factors["MEM_PRE"]

        return (
            reads * self._act_energies["dram_rd_coeff"]
            + writes * self._act_energies["dram_wr_coeff"]
            + precharges * self._act_energies["dram_pre_coeff"]
        )
