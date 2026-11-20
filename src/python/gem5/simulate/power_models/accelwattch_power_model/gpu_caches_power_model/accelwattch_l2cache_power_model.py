from m5.objects import Shader

from gem5.components.cachehierarchies.ruby.caches.viper.tcc import TCCCache

from ..base_accelwattch_power_model import (
    BaseAccelwattchPowerModel,
)

""" Note that this is also referred to as TCC """


class AccelwattchL2CachePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchL2CachePowerModel"
        self._caches = []

    def dynamic_power(self) -> float:
        self._caches = [
            tcc
            for tcc in self._simobj._parent.gpu_caches._controllers
            if isinstance(tcc, TCCCache)
        ]
        return self.convert_to_watts(self.l2cache_energy())

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def l2cache_static_coeff(self) -> float:
        read_hit_coeff = self._act_energies["l2cache"]["Read"]
        write_hit_coeff = self._act_energies["l2cache"]["Write"]
        read_miss_coeff = self._act_energies["l2cacheTag"]["Read"]
        write_miss_coeff = (
            self._act_energies["l2cacheTag"]["Write"]
            + self._act_energies["l2cache"]["Write"]
            + self._act_energies["l2cacheMissb"]["Search"]
            + self._act_energies["l2cacheMissb"]["Write"]
            + self._act_energies["l2cacheIfb"]["Search"]
            + self._act_energies["l2cacheIfb"]["Write"]
            + self._act_energies["l2cachePrefetchb"]["Search"]
            + self._act_energies["l2cachePrefetchb"]["Write"]
            + self._act_energies["l2cacheWbb"]["Search"]
            + self._act_energies["l2cacheWbb"]["Write"]
        )
        coeff = (
            read_hit_coeff
            + write_hit_coeff
            + read_miss_coeff
            + write_miss_coeff
        )
        return self.convert_to_watts(coeff)

    def l2cache_energy(self) -> float:
        read_hits = (
            sum(
                self.get_stat("L2cache.read_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["L2_RH"]
        )
        read_misses = (
            sum(
                self.get_stat("L2cache.read_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["L2_RM"]
        )
        write_hits = (
            sum(
                self.get_stat("L2cache.write_hits", cache)
                for cache in self._caches
            )
            * self._scaling_factors["L2_WH"]
        )
        write_misses = (
            sum(
                self.get_stat("L2cache.write_misses", cache)
                for cache in self._caches
            )
            * self._scaling_factors["L2_WM"]
        )

        energy = (
            read_hits * self._act_energies["l2cache"]["Read"]
            + read_misses * self._act_energies["l2cacheTag"]["Read"]
            + write_misses * self._act_energies["l2cacheTag"]["Write"]
            + (write_hits + write_misses)
            * self._act_energies["l2cache"]["Write"]
        )
        return energy
