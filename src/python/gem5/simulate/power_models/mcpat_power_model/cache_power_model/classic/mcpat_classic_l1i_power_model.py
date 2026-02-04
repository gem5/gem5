from m5.objects import (
    Cache,
    PowerModel,
    PowerModelPyFunc,
)

from ...base_mcpat_power_model import BaseMcPATPowerModel


class L1IPowerOn(PowerModelPyFunc, BaseMcPATPowerModel):
    def __init__(self, l1icache: Cache, writeback: bool, act_energies):
        super().__init__()
        # The SimObject that contains the stats we need.
        self._simobj = l1icache
        self._act_energies = act_energies
        self._writeback = writeback

        self.dyn = lambda: self.dynamic_power()
        self.st = lambda: self.static_power()

    def static_power(self):
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self):
        """Returns dynamic power in Watts"""
        energy = self.icache_energy()
        energy += self.miss_buffer_energy()
        energy += self.inst_fill_buffer_energy()
        energy += self.prefetch_buffer_energy()
        return self.convert_to_watts(energy)

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
        read_misses = self.get_stat("ReadReq.misses").total

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


class L1IPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATClassicL1IPowerModel(PowerModel):
    def __init__(self, l1icache, writeback, act_energies):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L1IPowerOn(l1icache, writeback, act_energies),  # ON
            L1IPowerOff(),  # CLK_GATED
            L1IPowerOff(),  # SRAM_RETENTION
            L1IPowerOff(),  # OFF
        ]
