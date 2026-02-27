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
    PowerModel,
    PowerModelPyFunc,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATMemFrontendPowerModel(BaseMcPATPowerModel):
    def __init__(self, mem_ctrl: MemCtrl, act_energies):
        super().__init__(mem_ctrl, act_energies)
        self.name = "McPATMemFrontendPowerModel"
        self._curve_fitted_constant = 0.00135072
        self._data_bus_width = 0
        self._llc_block_size = 0
        self._dimm_data_width = 72
        # this is the number of read/write/search ports, in McPAT
        # they are set to all the same value (and this is just
        # the number of mem channels on the MC)
        self._num_rws_ports = 0
        self._clock_rate = 0

    def set_mem_vars(self):
        self._llc_block_size = ceil(
            int(self._simobj._parent.get_cache_line_size()) / 8.0
        ) + int(self._simobj._parent.get_cache_line_size())
        self._data_bus_width = ceil(
            self._simobj.dram.device_bus_width / 8.0
        ) + int(self._simobj.dram.device_bus_width)
        self._num_rws_ports = self._simobj._parent._num_channels
        clk = 10**6 / self._simobj.clk_domain.clock.getValue()[0]
        self._clock_rate = clk * 2 * 1e6

    def static_power(self) -> float:
        # Placeholder value for static power.
        return 1.0

    def dynamic_power(self) -> float:
        self.set_mem_vars()
        sched_result_modifier = self._llc_block_size * 8 / self._data_bus_width
        reads = self.get_stat("readReqs").total
        writes = self.get_stat("writeReqs").total
        frontendBufReads = (
            reads
            * sched_result_modifier
            * self._data_bus_width
            / self._dimm_data_width
        )
        frontendBufWrites = (
            writes * sched_result_modifier * self._data_bus_width / 72
        )
        readBufReads = reads * sched_result_modifier
        readBufWrites = readBufReads
        writeBufReads = writes * sched_result_modifier
        writeBufWrites = writeBufReads

        frontendBufEnergy = (
            self._act_energies["frontendBuffer"]["Search"]
            * (frontendBufReads + frontendBufWrites)
            + self._act_energies["frontendBuffer"]["Read"] * frontendBufReads
            + self._act_energies["frontendBuffer"]["Write"] * frontendBufWrites
        )

        readBufEnergy = (
            self._act_energies["readBuffer"]["Read"] * readBufReads
            + self._act_energies["readBuffer"]["Write"] * readBufWrites
        )
        writeBufEnergy = (
            self._act_energies["writeBuffer"]["Read"] * writeBufReads
            + self._act_energies["writeBuffer"]["Write"] * writeBufWrites
        )
        total_energy = (
            frontendBufEnergy
            + readBufEnergy
            + writeBufEnergy
            + self.mem_refresh_overhead()
        )
        return self.convert_to_watts(total_energy)

    def mem_refresh_overhead(self) -> float:
        execTime = self.getExecutionTime()
        return self.calc_tdp() * 0.1 * self._clock_rate * 1 * execTime

    def calc_tdp(self) -> float:
        mc_duty_cycle = 0.5
        frontendBufferRW = self._num_rws_ports
        readBufferRW = mc_duty_cycle * self._num_rws_ports
        writeBufferRW = mc_duty_cycle * self._num_rws_ports
        frontendBufferTDP = self._act_energies["frontendBuffer"]["Search"] * (
            frontendBufferRW * 2
        ) + frontendBufferRW * (
            self._act_energies["frontendBuffer"]["Read"]
            + self._act_energies["frontendBuffer"]["Write"]
        )

        readBufferTDP = readBufferRW * (
            self._act_energies["readBuffer"]["Read"]
            + self._act_energies["readBuffer"]["Write"]
        )
        writeBufferTDP = writeBufferRW * (
            self._act_energies["writeBuffer"]["Read"]
            + self._act_energies["writeBuffer"]["Write"]
        )

        total_tdp = frontendBufferTDP + readBufferTDP + writeBufferTDP
        return total_tdp

    def print_mcpat(self, indent):
        power = self.dynamic_power()
        print(" " * indent + f"Front End Engine:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {power} W\n")
