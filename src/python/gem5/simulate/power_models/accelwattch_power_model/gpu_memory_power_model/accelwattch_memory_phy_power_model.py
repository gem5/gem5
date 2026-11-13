from math import ceil

from m5.objects import (
    Shader,
)

from .base_accelwattch_memory_power_model import (
    BaseAccelwattchMemoryPowerModel,
)


class AccelwattchMemoryPhyPowerModel(BaseAccelwattchMemoryPowerModel):
    def __init__(
        self, gpu: Shader, gpu_memory, act_energies, scaling_factors, interval
    ):
        super().__init__(
            gpu, gpu_memory, act_energies, scaling_factors, interval
        )
        self.name = "AccelwattchMemoryPhyPowerModel"
        self._llc_block_size = (
            int(ceil(gpu._cache_line_size / 8.0)) + gpu._cache_line_size
        )
        self._data_bus_width = int(
            ceil(self._gpu_mem._dram[0].device_bus_width / 8.0)
        ) + int(self._gpu_mem._dram[0].device_bus_width)

        """ Below value comes from Accelwattch's following comment:
            ----
            This is from curve fitting based on Niagara 1 and 2's PHY
            die photo.This is power not energy, 10mw/Gb/s @90nm
            for each channel and scaling down power.readOp.dynamic
            = 0.02*memAccesses*llcBlocksize*8;
            //change from Bytes to bits.
        """
        self._power_constant = 0.0023033

    def calc_tdp(self) -> float:
        reads = writes = (
            0.5
            * self.get_num_memory_controllers()
            / self._gpu_mem._num_channels
        )
        # Below is strictly for GDDR5, AW uses this
        data_transfer_unit = 72
        peakBW = (
            self.get_stat("mem_ctrl0.dram.peakBW", self._gpu_mem)
            * self.get_num_memory_controllers()
        )
        mem_clk_rate = 2 * int(1 / float(str(self._gpu_mem._dram[0].tCK)))
        # The extra '1' below is for the # of channels per controller.
        # You can assume this to be 1.
        return (
            self._power_constant
            * (peakBW * 8 * 1e6 / 1e9)
            * self._data_bus_width
            / data_transfer_unit
            * 1
            / mem_clk_rate
        )

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
        execTime = self.get_time()
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
            self._power_constant
            * (mem_reads + mem_writes)
            * self._llc_block_size
            * 8
            / 1e9
            / execTime
            * execTime
        ) + self.mem_refresh_overhead()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()
