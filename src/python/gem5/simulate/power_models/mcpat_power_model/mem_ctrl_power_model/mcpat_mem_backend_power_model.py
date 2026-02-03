from math import ceil

from m5.objects import (
    MemCtrl,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATMemBackendPowerModel(BaseMcPATPowerModel):
    def __init__(self, mem_ctrl: MemCtrl, act_energies):
        super().__init__(mem_ctrl, act_energies)
        self.name = "McPATMemBackendPower"
        self._curve_fitted_constant = 0.00135072
        self._data_bus_width = 0
        self._llc_block_size = 0
        self._clock_rate = 0
        self._backend_dyn = 0

    def set_mem_vars(self):
        self._llc_block_size = ceil(
            int(self._simobj._parent.get_cache_line_size()) / 8.0
        ) + int(self._simobj._parent.get_cache_line_size())
        self._data_bus_width = ceil(
            self._simobj.dram.device_bus_width / 8.0
        ) + int(self._simobj.dram.device_bus_width)
        clk = 10**6 / self._simobj.clk_domain.clock.getValue()[0]
        self._clock_rate = clk * 2 * 1e6

    def calc_backend_dyn(self):
        low_power = True
        if low_power:
            peakBW = self.get_stat("dram.peakBW").total
            Vdd = 0.661538
            tech_node = 40
            backend_dyn = (
                0.9e-9
                / 800e6
                * self._clock_rate
                / 12800
                * peakBW
                * self._data_bus_width
                / 72
                * Vdd
                / 1.1
                * Vdd
                / 1.1
                * (tech_node / 65)
            )
        self._backend_dyn = backend_dyn

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        self.set_mem_vars()
        self.calc_backend_dyn()
        reads = self.get_stat("readReqs").total
        writes = self.get_stat("writeReqs").total
        energy = (
            (reads + writes)
            * self._llc_block_size
            * 8
            / self._data_bus_width
            * self._backend_dyn
        ) + self.mem_refresh_overhead()
        return self.convert_to_watts(energy)

    def mem_refresh_overhead(self) -> float:
        execTime = self.getExecutionTime()
        return self.calc_tdp() * 0.1 * self._clock_rate * 1 * execTime

    def calc_tdp(self) -> float:
        tdp_reads = 0.5 * self._simobj._parent._num_channels
        tdp_writes = tdp_reads
        return self._backend_dyn * (tdp_reads + tdp_writes)

    def print_mcpat(self, indent):
        power = self.dynamic_power()
        print(" " * indent + f"Transaction Engine:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {power} W\n")
