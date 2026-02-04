from m5.objects import BaseXBar

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.simulate.power_models.mcpat_power_model.cache_power_model.classic.mcpat_classic_l1d_power_model import (
    McPATClassicL1DPowerModel,
)
from gem5.simulate.power_models.mcpat_power_model.cache_power_model.classic.mcpat_classic_l1i_power_model import (
    McPATClassicL1IPowerModel,
)
from gem5.simulate.power_models.mcpat_power_model.cache_power_model.classic.mcpat_classic_l2_power_model import (
    McPATClassicL2PowerModel,
)


class PrivateL1SharedL2CacheHierarchyWithPowerModel(
    PrivateL1SharedL2CacheHierarchy
):
    def __init__(self, l1d_size: str, l1i_size: str, l2_size: str) -> None:
        """
        Same the parent, however, adds a McPATPowerModel
        """
        # Initialize the parent class with the standard arguments
        super().__init__(
            l1d_size=l1d_size,
            l1i_size=l1i_size,
            l2_size=l2_size,
        )
        self._act_energies = {}

    def incorporate_cache(self, board: AbstractBoard) -> None:
        super().incorporate_cache(board)
        self.init_mcpat_act_energies()
        for cache in self.l1icaches:
            cache.power_model = McPATClassicL1IPowerModel(
                cache, cache.writeback_clean, self._act_energies
            )
        for cache in self.l1dcaches:
            cache.power_model = McPATClassicL1DPowerModel(
                cache, cache.writeback_clean, self._act_energies
            )
        for cache in self.l2cache:
            cache.power_model = McPATClassicL2PowerModel(
                cache, cache.writeback_clean, self._act_energies
            )

    def init_mcpat_act_energies(self) -> None:
        self._act_energies = {
            "DataCacheData": 7.80998e-12,
            "DataCacheTag": 1.57793e-12,
            "DataCache": {"Write": 1.57793e-12},
            "DataCacheMissb": {
                "Read": 6.13188e-12,
                "Write": 6.03686e-12,
                "Search": 5.31524e-12,
            },
            "DataCacheIfb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "DataCachePrefetchb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "DataCacheWritebackb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "InstCacheData": 7.80998e-12,
            "InstCacheTag": 1.57793e-12,
            "InstCache": {"Write": 1.77257e-11},
            "InstCacheMissb": {
                "Read": 6.13188e-12,
                "Write": 6.03686e-12,
                "Search": 5.31524e-12,
            },
            "InstCacheIfb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "InstCachePrefetchb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "InstCacheWritebackb": {
                "Read": 3.19263e-12,
                "Write": 3.17392e-12,
                "Search": 2.93613e-12,
            },
            "L2Cache": {"Write": 1.80156e-10},
            "L2CacheData": 1.52711e-10,
            "L2CacheTag": {"Read": 7.0062e-12, "Write": 2.10746e-11},
            "L2CacheMissb": {
                "Read": 7.44104e-12,
                "Write": 7.67662e-12,
                "Search": 7.79179e-12,
            },
            "L2CacheIfb": {
                "Read": 3.72729e-11,
                "Write": 3.7854e-11,
                "Search": 3.5083e-11,
            },
            "L2CachePrefetchb": {
                "Read": 3.72729e-11,
                "Write": 3.7854e-11,
                "Search": 3.5083e-11,
            },
            "L2CacheWritebackb": {
                "Read": 3.72729e-11,
                "Write": 3.7854e-11,
                "Search": 3.5083e-11,
            },
        }

        self._act_energies["InstCache"]["Read"] = (
            self._act_energies["InstCacheData"]
            + self._act_energies["InstCacheTag"]
        )

        self._act_energies["DataCache"]["Read"] = (
            self._act_energies["DataCacheData"]
            + self._act_energies["DataCacheTag"]
        )
        self._act_energies["L2Cache"]["Read"] = (
            self._act_energies["L2CacheData"]
            + self._act_energies["L2CacheTag"]["Read"]
        )
