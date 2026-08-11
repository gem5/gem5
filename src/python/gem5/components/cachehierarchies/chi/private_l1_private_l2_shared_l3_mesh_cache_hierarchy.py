# Copyright (c) 2026 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

import math
from typing import List

from m5.objects import RubyCache
from m5.params import NULL

from gem5.components.boards.abstract_board import AbstractBoard

from .nodes.directory import (
    BaseDirectory,
    CachedDirectory,
)
from .nodes.hnf import CHI_HNF
from .private_l1_private_l2_mesh_cache_hierarchy import (
    CustomMeshNoC_Params,
    PrivateL1PrivateL2MeshCacheHierarchy,
)


class PrivateL1PrivateL2SharedL3MeshCacheHierarchy(
    PrivateL1PrivateL2MeshCacheHierarchy
):
    """Mesh CHI hierarchy identical to PrivateL1PrivateL2Mesh with shared L3 HNF cache slices."""

    def __init__(
        self,
        l1i_size: str,
        l1i_assoc: int,
        l1d_size: str,
        l1d_assoc: int,
        l2_size: str,
        l2_assoc: int,
        l3_size: str,
        l3_assoc: int,
        noc_params: CustomMeshNoC_Params,
    ):
        super().__init__(
            l1i_size=l1i_size,
            l1i_assoc=l1i_assoc,
            l1d_size=l1d_size,
            l1d_assoc=l1d_assoc,
            l2_size=l2_size,
            l2_assoc=l2_assoc,
            noc_params=noc_params,
        )
        self._l3_size = l3_size
        self._l3_assoc = l3_assoc

    def _create_hnfs(
        self, board: AbstractBoard, num_hnfs: int
    ) -> List[CHI_HNF]:
        hnfs = []
        mem_ranges = list(board.get_mem_ranges())
        for idx in range(num_hnfs):
            # Apply AddrRange interleaving for each HNF/L3 slice.
            ranges = BaseDirectory.create_addr_ranges(
                num_directories=num_hnfs,
                dir_idx=idx,
                mem_ranges=mem_ranges,
                cache_line_size=board.get_cache_line_size(),
            )
            block_size_bits = int(math.log(board.get_cache_line_size(), 2))
            llc_bits = int(math.log(num_hnfs, 2))
            intlv_high_bit = block_size_bits + llc_bits - 1

            ll_cache = RubyCache(
                size=self._l3_size,
                assoc=self._l3_assoc,
                dataAccessLatency=10,
                tagAccessLatency=2,
                start_index_bit=intlv_high_bit + 1,
            )

            directory = CachedDirectory(
                network=self.ruby_system.network,
                cache_line_size=board.get_cache_line_size(),
                clk_domain=board.get_clock_domain(),
                cache=ll_cache,
                prefetcher=NULL,
                addr_ranges=ranges,
            )
            directory.ruby_system = self.ruby_system

            hnf = CHI_HNF(self.ruby_system, directory)
            downstream_nodes = self.snf_nodes + getattr(
                self, "snf_boot_nodes", []
            )
            hnf.setDownstreamNodes(downstream_nodes)
            hnfs.append(hnf)

        return hnfs
