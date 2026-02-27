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

from m5.objects import (
    Cache,
    PowerModel,
    PowerModelPyFunc,
)

from ...base_mcpat_power_model import (
    ActEnergyType,
    BaseMcPATPowerModel,
)


class L1DPowerOn(PowerModelPyFunc, BaseMcPATPowerModel):
    """
    This class implements McPAT's power model for an L1D$.
    This is not a part of the LSU stage in gem5 (it is in McPAT)
    because gem5 decouples the cache hierarchy from the CPU.
    """

    def __init__(
        self, l1dcache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()
        self._simobj = l1dcache
        self._act_energies = act_energies
        self._writeback = writeback

        self.dyn = lambda: self.dynamic_power()
        self.st = lambda: self.static_power()

    def static_power(self):
        # Placeholder for static power.
        return 1.0

    def dynamic_power(self):
        # Dynamic power returned in Watts.
        total_energy = self.dcache_energy()
        total_energy += self.miss_buffer_energy()
        total_energy += self.inst_fill_buffer_energy()
        total_energy += self.prefetch_buffer_energy()
        total_energy += self.writeback_buffer_energy()
        return self.convert_to_watts(total_energy)

    def dcache_energy(self):
        read_accesses = self.get_stat("ReadReq.accesses").total
        write_accesses = self.get_stat("WriteReq.accesses").total

        read_misses = self.get_stat("ReadReq.misses").total
        write_misses = self.get_stat("WriteReq.misses").total

        read_hits = read_accesses - read_misses

        energy = (
            read_hits * self._act_energies["DataCache"]["Read"]
            + read_misses * self._act_energies["DataCache"]["Read"]
            + write_misses * self._act_energies["DataCacheTag"]
            + write_accesses * self._act_energies["DataCache"]["Write"]
        )
        if self._writeback:
            # if the cache is writeback, then this accounts for
            # extra energy costs for an extra write.
            energy += write_misses * self._act_energies["DataCache"]["Write"]
        return energy

    def miss_buffer_energy(self):
        if self._writeback:
            read_accesses = write_accesses = self.get_stat(
                "WriteReq.misses"
            ).total
        else:
            read_accesses = write_accesses = self.get_stat(
                "ReadReq.misses"
            ).total

        return (
            read_accesses
            * self._act_energies["DataCacheMissb"]["Search"]  # CAM Energy
            + write_accesses
            * self._act_energies["DataCacheMissb"]["Write"]  # Miss Energy
        )

    def inst_fill_buffer_energy(self):
        if self._writeback:
            read_accesses = write_accesses = self.get_stat(
                "WriteReq.misses"
            ).total
        else:
            read_accesses = write_accesses = self.get_stat(
                "ReadReq.misses"
            ).total
        return (
            read_accesses * self._act_energies["DataCacheIfb"]["Search"]
            + write_accesses * self._act_energies["DataCacheIfb"]["Write"]
        )

    def prefetch_buffer_energy(self):
        if self._writeback:
            read_accesses = write_accesses = self.get_stat(
                "WriteReq.misses"
            ).total
        else:
            read_accesses = write_accesses = self.get_stat(
                "ReadReq.misses"
            ).total
        return (
            read_accesses * self._act_energies["DataCachePrefetchb"]["Search"]
            + write_accesses
            * self._act_energies["DataCachePrefetchb"]["Write"]
        )

    def writeback_buffer_energy(self):
        if not self._writeback:
            return 0
        read_accesses = write_accesses = self.get_stat("WriteReq.misses").total
        return (
            read_accesses * self._act_energies["DataCacheWritebackb"]["Search"]
            + write_accesses
            * self._act_energies["DataCacheWritebackb"]["Write"]
        )


class L1DPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATClassicL1DPowerModel(PowerModel):
    def __init__(
        self, L1Dcache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L1DPowerOn(L1Dcache, writeback, act_energies),  # ON
            L1DPowerOff(),  # CLK_GATED
            L1DPowerOff(),  # SRAM_RETENTION
            L1DPowerOff(),  # OFF
        ]
