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

from ..base_mcpat_power_model import (
    ActEnergyType,
    BaseMcPATPowerModel,
)


class McPATMemBackendPowerModel(BaseMcPATPowerModel):
    def __init__(self, mem_ctrl: MemCtrl, act_energies: ActEnergyType):
        super().__init__(mem_ctrl, act_energies)
        self.name = "McPATMemBackendPower"
        self._curve_fitted_constant = 0.00135072
        self._data_bus_width = 0
        self._llc_block_size = 0
        self._clock_rate = 0
        self._backend_dyn = 0
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

    def calc_backend_dyn(self):
        """
        The following function calculates the backend dynamic energy based
        on several things:
        (1) If the memory is low power or not
        (2) the technology node, if the technology node is between two known
            ones (e.g., 32 <= 40 <= 45), then the Vdd is obtained by
            linearly interpolating between the two known technology nodes.
        (3) the peak bandwidth of a MC.
        """
        low_power = True
        if low_power:
            peakBW = self.get_stat("dram.peakBW").total
            Vdd = 0.661538
            tech_node = 40
            # For below, the justification for constants comes from
            # McPAT's memoryctrl.cc, L839:
            # "Average on DDR2/3 protocol controller and DDRC 1600/800A
            # in Cadence ChipEstimate. Scaling to technology and DIMM
            # feature. The base IP support DDR3-1600(PC3 12800)"
            backend_dyn = (
                0.9e-9  # Empirical estimates from Cadence ChipEstimate?
                / 800e6
                * self._clock_rate
                / 12800  # Refers to peak bandwidth
                * peakBW
                * self._data_bus_width
                / self._dimm_data_width
                * Vdd  # Vdd / 1.1 refers to voltage scaling
                / 1.1
                * Vdd
                / 1.1
                * (tech_node / 65)  # scale w.r.t technology node
            )
        self._backend_dyn = backend_dyn

    def static_power(self) -> float:
        # Placeholder value for static power.
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
