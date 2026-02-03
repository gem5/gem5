from math import ceil

from m5.objects import (
    MemCtrl,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATMemPhyPowerModel(BaseMcPATPowerModel):
    def __init__(self, mem_ctrl: MemCtrl, act_energies):
        super().__init__(mem_ctrl, act_energies)
        self.name = "McPATMemPhyPowerModel"
        self._curve_fitted_constant = 0.00135072
        """ TODO: The above value is just for LPDDR3, could change for other DRAM types?
            McPAT also doesn't support higher than DDR3...
        """
        self._data_bus_width = 0
        self._llc_block_size = 0
        self._clock_rate = 0

    def set_mem_vars(self):
        self._llc_block_size = ceil(
            int(self._simobj._parent.get_cache_line_size()) / 8.0
        ) + int(self._simobj._parent.get_cache_line_size())
        self._data_bus_width = ceil(
            self._simobj.dram.device_bus_width / 8.0
        ) + int(self._simobj.dram.device_bus_width)
        clk = 10**6 / self._simobj.clk_domain.clock.getValue()[0]
        self._clock_rate = clk * 2 * 1e6

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        self.set_mem_vars()
        execTime = self.getExecutionTime()
        reads = self.get_stat("readReqs").total
        writes = self.get_stat("writeReqs").total
        energy = (
            self._curve_fitted_constant
            * (reads + writes)
            * self._llc_block_size
            * 8
            / 1e9
            / execTime
            * execTime
        )
        energy += self.mem_refresh_overhead()
        return self.convert_to_watts(energy)

    def calc_tdp(self):
        # obj's path: board.memory.mem_ctrl
        peakBW = self.get_stat("dram.peakBW").total
        energy = (
            self._curve_fitted_constant
            * (peakBW * 8 * 1e6 / 1e9)
            * self._data_bus_width
            / 72
            * self._simobj._parent._num_channels
        ) / self._clock_rate
        return energy

    def mem_refresh_overhead(self) -> float:
        # The constant 1 below is the number of memory controllers. Should be
        # fine to assume 1?
        return (
            self.calc_tdp()
            * 0.1
            * self._clock_rate
            * 1
            * self.getExecutionTime()
        )

    def print_mcpat(self, indent):
        power = self.dynamic_power()
        print(" " * indent + f"PHY:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {power} W\n")
