from m5.objects import (
    Cache,
    PowerModel,
    PowerModelPyFunc,
)

from ...base_mcpat_power_model import BaseMcPATPowerModel


class L1DPowerOn(PowerModelPyFunc, BaseMcPATPowerModel):
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
        total_energy = self.dcache_energy()
        print(f"L1Dcache energy: {total_energy}")
        total_energy += self.miss_buffer_energy()
        print(f"\t+ mb energy: {total_energy}")
        total_energy += self.inst_fill_buffer_energy()
        print(f"\t+ ifb energy: {total_energy}")
        total_energy += self.prefetch_buffer_energy()
        print(f"\t+ prefetch energy: {total_energy}")
        total_energy += self.writeback_buffer_energy()
        print(f"\t+ wbb energy: {total_energy}")
        print(f"L1D power: {self.convert_to_watts(total_energy)}")
        return self.convert_to_watts(total_energy)

    def dcache_energy(self):
        read_accesses = self.get_stat("ReadReq.accesses").total
        write_accesses = self.get_stat("WriteReq.accesses").total

        read_misses = self.get_stat("ReadReq.misses").total
        write_misses = self.get_stat("WriteReq.misses").total

        read_hits = read_accesses - read_misses
        write_hits = write_accesses - write_misses

        energy = (
            read_hits * self._act_energies["DataCache"]["Read"]
            + read_misses * self._act_energies["DataCache"]["Read"]
            + write_misses * self._dcache_tag_ae
            + write_accesses * self._act_energies["DataCache"]["Write"]
        )
        if self._writeback:
            # extra write for write misses since wb policy
            energy += write_misses * self._act_energies["DataCache"]["Write"]
        return energy

    def miss_buffer_energy(self):
        # by default, there is WB in caches (to my knowledge)
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
    def __init__(self, L1Dcache, writeback, act_energies):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            L1DPowerOn(L1Dcache, writeback, act_energies),  # ON
            L1DPowerOff(),  # CLK_GATED
            L1DPowerOff(),  # SRAM_RETENTION
            L1DPowerOff(),  # OFF
        ]
