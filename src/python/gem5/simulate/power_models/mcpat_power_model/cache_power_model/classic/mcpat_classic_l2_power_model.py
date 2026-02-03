from m5.objects import (
    Cache,
    PowerModel,
    PowerModelPyFunc,
)

from ...base_mcpat_power_model import BaseMcPATPowerModel


class L2PowerOn(PowerModelPyFunc, BaseMcPATPowerModel):
    def __init__(self, l1dcache: Cache, writeback: bool, act_energies):
        super().__init__()
        # The SimObject that contains the stats we need.
        self._simobj = l1dcache
        self._act_energies = act_energies
        self._writeback = writeback

        self.dyn = lambda: self.dynamic_power()
        self.st = lambda: self.static_power()

    def static_power(self):
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self):
        """Returns dynamic power in Watts"""
        total_energy = self.l2cache_energy()
        print(f"L2cache energy: {total_energy}")
        total_energy += self.miss_buffer_energy()
        print(f"\t+ mb energy: {total_energy}")
        total_energy += self.inst_fill_buffer_energy()
        print(f"\t+ ifb energy: {total_energy}")
        total_energy += self.prefetch_buffer_energy()
        print(f"\t+ prefetch energy: {total_energy}")
        total_energy += self.writeback_buffer_energy()
        print(f"\t+ wbb energy: {total_energy}")
        print(f"L2 power: {self.convert_to_watts(total_energy)}")
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
        write_hits = write_accesses - write_misses
        return (
            read_hits * self._l2cache_read_ae
            + read_misses * self._act_energies["L2CacheTag"]["Read"]
            + write_misses * self._act_energies["L2CacheTag"]["Write"]
            + write_accesses * self._act_energies["L2Cache"]["Write"]
        )

    def miss_buffer_energy(self):
        # by default, there is WB in caches (to my knowledge)
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
    def __init__(self, L1Dcache, writeback, act_energies):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L2PowerOn(L1Dcache, writeback, act_energies),  # ON
            L2PowerOff(),  # CLK_GATED
            L2PowerOff(),  # SRAM_RETENTION
            L2PowerOff(),  # OFF
        ]
