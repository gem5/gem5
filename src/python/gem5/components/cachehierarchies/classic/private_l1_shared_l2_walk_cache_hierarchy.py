# Copyright (c) 2025 The Regents of the University of California
# Copyright (c) 2024 Arm Limited
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

from m5.objects import BaseCPU

from ....utils.override import *
from ...boards.abstract_board import AbstractBoard
from .caches.mmu_cache import MMUCache
from .private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)


class PrivateL1SharedL2WalkCacheHierarchy(PrivateL1SharedL2CacheHierarchy):
    """Shared L2 hierarchy variant that places MMU caches on walker ports."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    @overrides(PrivateL1SharedL2CacheHierarchy)
    def incorporate_cache(self, board: AbstractBoard) -> None:
        self._tmp_iptw_caches = []
        self._tmp_dptw_caches = []
        super().incorporate_cache(board)
        if self._tmp_iptw_caches:
            self.iptw_caches = self._tmp_iptw_caches
        self.dptw_caches = self._tmp_dptw_caches

    def _connect_table_walker(self, cpu_id: int, cpu: BaseCPU) -> None:
        walker_ports = cpu.get_mmu().walkerPorts() if cpu.has_mmu() else []
        if len(walker_ports) > 2:
            raise RuntimeError(
                "Unexpected number of walker ports "
                f"from CPU {cpu_id}: {len(walker_ports)}.\n"
                "Expected 0, 1, or 2"
            )

        if len(walker_ports) == 0:
            return

        dptw_cache = MMUCache(size="8KiB", writeback_clean=False)
        dptw_cache.mem_side = self.l2bus.cpu_side_ports

        if len(walker_ports) == 2:
            iptw_cache = MMUCache(size="8KiB", writeback_clean=False)
            iptw_cache.mem_side = self.l2bus.cpu_side_ports
            cpu.connect_walker_ports(iptw_cache.cpu_side, dptw_cache.cpu_side)
            self._tmp_iptw_caches.append(iptw_cache)
        else:
            assert (
                len(walker_ports) == 1
            ), f"This branch expects 1 walker_port, got {len(walker_ports)}."
            cpu.connect_walker_ports(dptw_cache.cpu_side, dptw_cache.cpu_side)

        self._tmp_dptw_caches.append(dptw_cache)
