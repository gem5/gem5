# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
        # The above value is just for LPDDR3, could change for other DRAM types?
        # McPAT also doesn't support higher than DDR3...
        self._data_bus_width = 0
        self._llc_block_size = 0
        self._clock_rate = 0
        self._dimm_data_width = 72

    def set_mem_vars(self):
        self._llc_block_size = ceil(
            int(self._simobj._parent.get_cache_line_size()) / 8.0
        ) + int(self._simobj._parent.get_cache_line_size())
        self._data_bus_width = ceil(
            self._simobj.dram.device_bus_width / 8.0
        ) + int(self._simobj.dram.device_bus_width)
        # The clock rate below is converted to MHz and then
        # multiplied by 2 because it is double-pumped.
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
        peakBW = self.get_stat("dram.peakBW").total
        energy = (
            self._curve_fitted_constant
            * (peakBW * 8 * 1e6 / 1e9)  # convert peak bandwidth into Gb/s
            * self._data_bus_width
            / self._dimm_data_width
            * self._simobj._parent._num_channels
        ) / self._clock_rate
        return energy

    def mem_refresh_overhead(self) -> float:
        # 1 below refers to the number of memory controllers, which should
        # be 1.
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
