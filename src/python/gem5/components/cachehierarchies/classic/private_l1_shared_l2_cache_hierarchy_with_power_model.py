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

from m5.objects import BaseXBar

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.simulate.power_models.mcpat_power_model.cache_power_model.cache_act_energy_constants import (
    cache_act_energies,
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
    def __init__(self) -> None:
        # Models power for an L1D/L1I/L2 caches using the McPAT Power Model
        # for a pre-defined size/assoc. for the caches.
        super().__init__(
            l1d_size="32kB",
            l1i_size="32kB",
            l2_size="1MB",
            l1d_assoc=4,
            l1i_assoc=8,
            l2_assoc=8,
        )
        self._act_energies = cache_act_energies

    def incorporate_cache(self, board: AbstractBoard) -> None:
        super().incorporate_cache(board)
        # Apply the power model to each cache, and assume
        # the caches are always turned on.
        for cache in self.l1icaches:
            cache.power_model = McPATClassicL1IPowerModel(
                cache, cache.writeback_clean, self._act_energies
            )
            cache.power_state.default_state = "ON"

        for cache in self.l1dcaches:
            cache.power_model = McPATClassicL1DPowerModel(
                cache, cache.writeback_clean, self._act_energies
            )
            cache.power_state.default_state = "ON"

        self.l2cache.power_model = McPATClassicL2PowerModel(
            self.l2cache, self.l2cache.writeback_clean, self._act_energies
        )
        self.l2cache.power_state.default_state = "ON"
