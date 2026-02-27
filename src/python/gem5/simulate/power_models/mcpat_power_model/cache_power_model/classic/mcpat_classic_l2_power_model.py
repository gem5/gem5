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


class L2PowerOn(PowerModelPyFunc, BaseMcPATPowerModel):
    """
    This class implements McPAT's power model for an L2$.
    Even though McPAT has options for a private L2 cache, and
    a cache which uses Ruby, this assumes that the cache is
    a classic cache and is shared.
    """

    def __init__(
        self, l1dcache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()
        # The SimObject that contains the stats we need.
        self._simobj = l1dcache
        self._act_energies = act_energies
        self._writeback = writeback

        self.dyn = lambda: self.dynamic_power()
        self.st = lambda: self.static_power()

    def static_power(self):
        # Placeholder value for static power.
        return 1.0

    def dynamic_power(self):
        total_energy = self.l2cache_energy()
        total_energy += self.miss_buffer_energy()
        total_energy += self.inst_fill_buffer_energy()
        total_energy += self.prefetch_buffer_energy()
        total_energy += self.writeback_buffer_energy()
        return self.convert_to_watts(total_energy)

    def l2cache_energy(self):
        # Writeback is ALWAYS assumed in shared caches, according to McPAT
        read_accesses = self.get_stat("ReadExReq.accesses").total
        write_accesses = (
            self.get_stat("overallAccesses").total
            + self.get_stat("WritebackClean.accesses").total
        )

        read_misses = self.get_stat("ReadExReq.misses").total
        write_misses = self.get_stat("overallMisses").total - read_misses

        read_hits = read_accesses - read_misses
        return (
            read_hits * self._act_energies["L2Cache"]["Read"]
            + read_misses * self._act_energies["L2CacheTag"]["Read"]
            + write_misses * self._act_energies["L2CacheTag"]["Write"]
            + write_accesses * self._act_energies["L2Cache"]["Write"]
        )

    def miss_buffer_energy(self):
        read_accesses = write_accesses = (
            self.get_stat("overallMisses").total
            - self.get_stat("ReadExReq.misses").total
        )

        return (
            read_accesses
            * self._act_energies["L2CacheMissb"]["Search"]  # CAM Energy
            + write_accesses
            * self._act_energies["L2CacheMissb"]["Write"]  # Miss Energy
        )

    def inst_fill_buffer_energy(self):
        read_accesses = write_accesses = (
            self.get_stat("overallMisses").total
            - self.get_stat("ReadExReq.misses").total
        )
        return (
            read_accesses * self._act_energies["L2CacheIfb"]["Search"]
            + write_accesses * self._act_energies["L2CacheIfb"]["Write"]
        )

    def prefetch_buffer_energy(self):
        read_accesses = write_accesses = (
            self.get_stat("overallMisses").total
            - self.get_stat("ReadExReq.misses").total
        )
        return (
            read_accesses * self._act_energies["L2CachePrefetchb"]["Search"]
            + write_accesses * self._act_energies["L2CachePrefetchb"]["Write"]
        )

    def writeback_buffer_energy(self):
        read_accesses = write_accesses = (
            self.get_stat("overallMisses").total
            - self.get_stat("ReadExReq.misses").total
        )
        return (
            read_accesses * self._act_energies["L2CacheWritebackb"]["Search"]
            + write_accesses * self._act_energies["L2CacheWritebackb"]["Write"]
        )


class L2PowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATClassicL2PowerModel(PowerModel):
    def __init__(
        self, L1Dcache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L2PowerOn(L1Dcache, writeback, act_energies),  # ON
            L2PowerOff(),  # CLK_GATED
            L2PowerOff(),  # SRAM_RETENTION
            L2PowerOff(),  # OFF
        ]
