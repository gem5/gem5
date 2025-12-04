# Copyright (c) 2022 The Regents of the Yonsei University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import Optional

from m5.objects import (
    BaseXBar,
    FetchDirectedPrefetcher,
    MultiPrefetcher,
    TaggedPrefetcher,
)

from ....utils.override import *
from ...boards.abstract_board import AbstractBoard
from .private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)


class TwoLevelFDPCacheHierarchy(PrivateL1SharedL2CacheHierarchy):
    """
    A two-level cache setup based on the PrivateL1SharedL2CacheHierarchy
    where each core has a private L1 Data and  Instruction Cache, and a L2
    cache is shared with all cores.
    Each instruction cache is equipped with fetch-direct prefetcher (FDP)
    and TaggedPrefetcher (next-line)
    """

    def __init__(
        self,
        l1d_size: str,
        l1i_size: str,
        l2_size: str,
        l1d_assoc: int = 8,
        l1i_assoc: int = 8,
        l2_assoc: int = 16,
        decoupled: bool = True,
        membus: Optional[BaseXBar] = None,
    ) -> None:
        """
        :param l1d_size: The size of the L1 Data Cache (e.g., "32KiB").
        :param  l1i_size: The size of the L1 Instruction Cache (e.g., "32KiB").
        :param l2_size: The size of the L2 Cache (e.g., "256KiB").
        :param l1d_assoc: The associativity of the L1 Data Cache.
        :param l1i_assoc: The associativity of the L1 Instruction Cache.
        :param l2_assoc: The associativity of the L2 Cache.
        :param decoupled: Enable/Disable the decoupled frontend.
        :param membus: The memory bus. This parameter is optional parameter and
                       will default to a 64 bit width SystemXBar is not
                       specified.
        """

        super().__init__(
            l1i_size=l1i_size,
            l1i_assoc=l1i_assoc,
            l1d_size=l1d_size,
            l1d_assoc=l1d_assoc,
            l2_size=l2_size,
            l2_assoc=l2_assoc,
            membus=membus,
        )
        self._decoupled = decoupled

    @overrides(PrivateL1SharedL2CacheHierarchy)
    def incorporate_cache(self, board: AbstractBoard) -> None:
        super().incorporate_cache(board)

        # Add the prefetchers to the L1I caches and register the MMU.
        for i in range(board.get_processor().get_num_cores()):
            cpu = board.get_processor().cores[i].core

            self.l1icaches[i].prefetcher = MultiPrefetcher()
            if self._decoupled:
                pf = FetchDirectedPrefetcher(
                    use_virtual_addresses=True, cpu=cpu
                )
                # Optionally register the cache to prefetch into to enable
                # cache snooping
                pf.registerCache(self.l1icaches[i])
                self.l1icaches[i].prefetcher.prefetchers.append(pf)

            self.l1icaches[i].prefetcher.prefetchers.append(
                TaggedPrefetcher(use_virtual_addresses=True)
            )

            for pf in self.l1icaches[i].prefetcher.prefetchers:
                pf.registerMMU(cpu.mmu)
