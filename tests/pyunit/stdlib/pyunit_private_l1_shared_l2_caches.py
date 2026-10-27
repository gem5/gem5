# Copyright (c) 2026 Akanksha Chaudhari
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


import unittest

from m5.objects import L2XBar

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.caches.l1dcache import L1DCache
from gem5.components.cachehierarchies.classic.caches.l1icache import L1ICache
from gem5.components.cachehierarchies.classic.caches.l2cache import L2Cache
from gem5.components.cachehierarchies.classic.private_l1_shared_l2_cache_hierarchy import (
    PrivateL1SharedL2CacheHierarchy,
)
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA


def _connect(hierarchy, num_cores):
    board = SimpleBoard(
        clk_freq="1GHz",
        processor=SimpleProcessor(
            cpu_type=CPUTypes.TIMING,
            isa=ISA.ARM,
            num_cores=num_cores,
            clk_freq="1GHz",
        ),
        memory=SingleChannelDDR4_2400(size="1GiB"),
        cache_hierarchy=hierarchy,
    )
    board._set_fullsystem(False)
    board._connect_things()
    return board


class TestPrivateL1SharedL2Caches(unittest.TestCase):
    def test_defaults(self):
        hierarchy = PrivateL1SharedL2CacheHierarchy(
            l1d_size="16KiB", l1i_size="8KiB", l2_size="256KiB"
        )
        _connect(hierarchy, 2)
        for cache in hierarchy.l1icaches:
            self.assertIsInstance(cache, L1ICache)
            self.assertEqual(int(cache.size), 8 * 1024)
            self.assertEqual(int(cache.assoc), 8)
            self.assertFalse(cache.writeback_clean)
        for cache in hierarchy.l1dcaches:
            self.assertIsInstance(cache, L1DCache)
            self.assertEqual(int(cache.size), 16 * 1024)
        self.assertIsInstance(hierarchy.l2cache, L2Cache)
        self.assertEqual(int(hierarchy.l2cache.size), 256 * 1024)
        self.assertEqual(int(hierarchy.l2cache.assoc), 16)
        self.assertIsInstance(hierarchy.l2bus, L2XBar)
