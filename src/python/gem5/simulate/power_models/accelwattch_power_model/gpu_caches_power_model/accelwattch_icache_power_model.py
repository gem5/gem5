from m5.objects import Shader

from gem5.components.cachehierarchies.ruby.caches.viper.sqc import SQCCache

from ..base_accelwattch_power_model import (
    BaseAccelwattchPowerModel,
)

""" Note that this is also referred to as SQC """


class AccelwattchInstCachePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchInstCachePowerModel"
        self._caches = []

    def dynamic_power(self) -> float:
        self._caches = [
            sqc
            for sqc in self._simobj._parent.gpu_caches._controllers
            if isinstance(sqc, SQCCache)
        ]
        return self.convert_to_watts(
            self.icache_hit_energy()
            + self.icache_miss_energy()
            + self.icache_access_energy()
        )

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def icache_hit_energy(self) -> float:
        assert all(isinstance(cache, SQCCache) for cache in self._caches)
        read_hits = (
            sum(
                self.get_stat("L1cache.read_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["IC_H"]
        )
        energy = read_hits * self._act_energies["icache"]["Read"]
        return energy

    def icache_miss_energy(self) -> float:
        assert all(isinstance(cache, SQCCache) for cache in self._caches)
        read_misses = (
            sum(
                self.get_stat("L1cache.read_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["IC_M"]
        )
        energy = (
            read_misses * self._act_energies["icache"]["Read"]
            + read_misses * self._act_energies["icache"]["Write"]
            + read_misses * self._act_energies["icacheMissb"]["Search"]
        )

        return energy

    def icache_access_energy(self) -> float:
        assert all(isinstance(cache, SQCCache) for cache in self._caches)
        read_hits = (
            sum(
                self.get_stat("L1cache.read_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["IC_H"]
        )
        read_misses = (
            sum(
                self.get_stat("L1cache.read_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["IC_M"]
        )

        accesses = read_hits + read_misses

        energy = accesses * (
            self._act_energies["icacheMissb"]["Search"]
            + self._act_energies["icacheIfb"]["Search"]
            + self._act_energies["icachePrefetchb"]["Search"]
            + self._act_energies["icacheMissb"]["Write"]
            + self._act_energies["icacheIfb"]["Write"]
            + self._act_energies["icachePrefetchb"]["Write"]
        )
        return energy
