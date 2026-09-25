from m5.objects import Shader

from gem5.components.cachehierarchies.ruby.caches.viper.tcp import TCPCache

from ..base_accelwattch_power_model import (
    BaseAccelwattchPowerModel,
)

""" Note that this is also referred to as TCP """


class AccelwattchDataCachePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchDataCachePowerModel"
        self._caches = []

    def dynamic_power(self) -> float:
        self._caches = [
            tcp
            for tcp in self._simobj._parent.gpu_caches._controllers
            if isinstance(tcp, TCPCache)
        ]
        self.dcache_static_coeff()
        return self.convert_to_watts(
            self.dcache_hit_energy()
            + self.dcache_miss_energy()
            + self.dcache_access_energy()
            + self.dcache_prt_energy()
            + self.dcache_prc_energy()
            + self.dcache_thread_mask_energy()
        )

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def dcache_static_coeff(self) -> float:
        read_coalescing = (
            self._act_energies["PRT"]["Read"]
            + self._act_energies["PRC"]["Read"]
            + self._act_energies["threadMasks"]["Read"]
            + self._act_energies["perAccessCoalescing"]
        )
        write_coalescing = (
            self._act_energies["PRT"]["Write"]
            + self._act_energies["PRC"]["Write"]
            + self._act_energies["threadMasks"]["Write"]
            + self._act_energies["perAccessCoalescing"]
        )
        read_hit_coeff = (
            self._act_energies["dcache"]["Read"]
            + self._act_energies["xbar_shared"]
        ) + read_coalescing
        write_hit_coeff = (
            self._act_energies["dcache"]["Write"]
            + self._act_energies["xbar_shared"]
        ) + write_coalescing
        write_miss_coeff = (
            self._act_energies["dcache"]["Write"]
            + self._act_energies["dcacheTag"]
        ) + write_coalescing
        read_miss_coeff = (
            self._act_energies["dcache"]["Read"]
            + self._act_energies["dcacheMissb"]["Search"]
            + self._act_energies["dcacheIfb"]["Search"]
            + self._act_energies["dcachePrefetchb"]["Search"]
            + self._act_energies["dcacheMissb"]["Write"]
            + self._act_energies["dcacheIfb"]["Write"]
            + self._act_energies["dcachePrefetchb"]["Write"]
        ) + read_coalescing
        coeff = (
            read_hit_coeff
            + read_miss_coeff
            + write_hit_coeff
            + write_miss_coeff
        )
        return coeff

    def dcache_hit_energy(self) -> float:
        assert all(isinstance(cache, TCPCache) for cache in self._caches)
        # IFB R/W -> read_misses
        # PFB R/W -> read_misses
        # WBB R/W -> read_misses

        read_hits = (
            sum(
                self.get_stat("L1cache.read_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_RH"]
        )
        write_hits = self.get_dcache_write_hits()
        energy = (
            read_hits
            * (
                self._act_energies["dcache"]["Read"]
                + self._act_energies["xbar_shared"]
            )
            + write_hits * self._act_energies["xbar_shared"]
        )
        return energy

    def dcache_miss_energy(self) -> float:
        assert all(isinstance(cache, TCPCache) for cache in self._caches)
        read_misses = (
            sum(
                self.get_stat("L1cache.read_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_RM"]
        )
        write_misses = self.get_dcache_write_misses()

        energy = (
            read_misses * self._act_energies["dcache"]["Read"]
            + write_misses * self._act_energies["dcacheTag"]
            + read_misses * self._act_energies["dcacheMissb"]["Write"]
            + read_misses * self._act_energies["dcacheMissb"]["Search"]
        )

        return energy

    def dcache_access_energy(self) -> float:
        assert all(isinstance(cache, TCPCache) for cache in self._caches)
        write_accesses = (
            self.get_dcache_write_misses() + self.get_dcache_write_hits()
        )
        energy = write_accesses * (self._act_energies["dcache"]["Write"])
        return energy

    """ PRT/PRC are under Mem in AW, but have added them here instead """

    def dcache_prc_energy(self) -> float:
        """You could include C/Tex caches, but they aren't in gem5."""
        read_accesses = (
            self.get_dcache_read_misses() + self.get_dcache_read_hits()
        )
        write_accesses = (
            self.get_dcache_write_misses() + self.get_dcache_write_hits()
        )

        energy = (
            read_accesses * self._act_energies["PRC"]["Read"]
            + write_accesses * self._act_energies["PRC"]["Write"]
        )
        return energy

    def dcache_prt_energy(self) -> float:
        read_accesses = (
            self.get_dcache_read_misses() + self.get_dcache_read_hits()
        )
        write_accesses = (
            self.get_dcache_write_misses() + self.get_dcache_write_hits()
        )
        energy = (
            read_accesses * self._act_energies["PRT"]["Read"]
            + write_accesses * self._act_energies["PRT"]["Write"]
        )
        return energy

    def dcache_thread_mask_energy(self) -> float:
        read_accesses = (
            self.get_dcache_read_misses() + self.get_dcache_read_hits()
        )
        write_accesses = (
            self.get_dcache_write_misses() + self.get_dcache_write_hits()
        )
        energy = (
            read_accesses * self._act_energies["threadMasks"]["Read"]
            + write_accesses * self._act_energies["threadMasks"]["Write"]
            + (read_accesses + write_accesses)
            * self._act_energies["perAccessCoalescing"]
        )
        return energy

    def get_dcache_read_hits(self) -> float:
        read_hits = (
            sum(
                self.get_stat("L1cache.read_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_RH"]
        )
        return read_hits

    def get_dcache_read_misses(self) -> float:
        read_misses = (
            sum(
                self.get_stat("L1cache.read_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_RM"]
        )
        return read_misses

    def get_dcache_write_hits(self) -> float:
        write_hits = (
            sum(
                self.get_stat("L1cache.write_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_WH"]
        )
        return write_hits

    def get_dcache_write_misses(self) -> float:
        write_misses = (
            sum(
                self.get_stat("L1cache.write_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["DC_WM"]
        )
        return write_misses

    def get_dcache_accesses(self) -> float:
        assert all(isinstance(cache, TCPCache) for cache in self._caches)
        accesses = (
            self.get_dcache_read_hits()
            + self.get_dcache_read_misses()
            + self.get_dcache_write_hits()
            + self.get_dcache_write_misses()
        )

        return accesses
