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


class L1ICachePowerModel(BaseMcPATPowerModel):
    """
    This class implements McPAT's power model for an L1I$
    This is not a part of the IF stage in gem5 (it is in McPAT)
    because gem5 decouples the cache hierarchy from the CPU.
    """

    def __init__(
        self, l1dcache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__(simobj=l1dcache, act_energies=act_energies)
        self._writeback = writeback

    def dynamic_power(self):
        energy = self.icache_energy()
        energy += self.miss_buffer_energy()
        energy += self.inst_fill_buffer_energy()
        energy += self.prefetch_buffer_energy()
        return self.convert_to_watts(energy)

    def static_power(self):
        # Placeholder value for static power.
        return 1.0

    def icache_energy(self):
        read_accesses = self.get_stat("ReadReq.accesses").total
        read_misses = self.get_stat("ReadReq.misses").total
        read_hits = read_accesses - read_misses

        return (
            read_hits * self._act_energies["InstCache"]["Read"]
            + read_misses * self._act_energies["InstCache"]["Read"]
            + read_misses * self._act_energies["InstCache"]["Write"]
        )

    def miss_buffer_energy(self):
        read_accesses = write_accesses = self.get_stat("ReadReq.misses").total

        return (
            read_accesses
            * self._act_energies["InstCacheMissb"]["Search"]  # CAM Energy
            + write_accesses
            * self._act_energies["InstCacheMissb"]["Write"]  # Miss Energy
        )

    def inst_fill_buffer_energy(self):
        read_accesses = write_accesses = self.get_stat("ReadReq.misses").total
        return (
            read_accesses * self._act_energies["InstCacheIfb"]["Search"]
            + write_accesses * self._act_energies["InstCacheIfb"]["Write"]
        )

    def prefetch_buffer_energy(self):
        read_accesses = write_accesses = self.get_stat("ReadReq.misses").total
        return (
            read_accesses * self._act_energies["InstCachePrefetchb"]["Search"]
            + write_accesses
            * self._act_energies["InstCachePrefetchb"]["Write"]
        )


class L1IPowerOn(PowerModelPyFunc):
    def __init__(
        self, l1icache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()

        self._cache_pm = L1ICachePowerModel(l1icache, writeback, act_energies)
        self.dyn = lambda: self.dynamic_power()
        self.st = lambda: self.static_power()

    def dynamic_power(self):
        return self._cache_pm.dynamic_power()

    def static_power(self):
        return self._cache_pm.static_power()


class L1IPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATClassicL1IPowerModel(PowerModel):
    def __init__(
        self, l1icache: Cache, writeback: bool, act_energies: ActEnergyType
    ):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L1IPowerOn(l1icache, writeback, act_energies),  # ON
            L1IPowerOff(),  # CLK_GATED
            L1IPowerOff(),  # SRAM_RETENTION
            L1IPowerOff(),  # OFF
        ]
