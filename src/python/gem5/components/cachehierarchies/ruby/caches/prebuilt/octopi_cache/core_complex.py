# Copyright (c) 2022-2023 The Regents of the University of California
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

from typing import (
    List,
    Tuple,
)

from m5.objects import (
    RubySequencer,
    SubSystem,
)

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.l1_cache import (
    L1Cache,
)
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.l2_cache import (
    L2Cache,
)
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.l3_cache import (
    L3Cache,
)
from gem5.components.processors.abstract_core import AbstractCore
from gem5.isas import ISA

from .ruby_network_components import (
    RubyExtLink,
    RubyIntLink,
    RubyNetworkComponent,
    RubyRouter,
)


class CoreComplex(SubSystem, RubyNetworkComponent):
    _core_id = 0
    _core_complex_id = 0

    @classmethod
    def _get_core_id(cls):
        cls._core_id += 1
        return cls._core_id - 1

    @classmethod
    def _get_core_complex_id(cls):
        cls._core_complex_id += 1
        return cls._core_complex_id - 1

    def __init__(
        self,
        board: AbstractBoard,
        cores: List[AbstractCore],
        ruby_system,
        l1i_size: str,
        l1i_assoc: int,
        l1d_size: str,
        l1d_assoc: int,
        l2_size: str,
        l2_assoc: int,
        l3_size: str,
        l3_assoc: int,
    ):
        SubSystem.__init__(self=self)
        RubyNetworkComponent.__init__(self=self)

        self._l1i_size = l1i_size
        self._l1i_assoc = l1i_assoc
        self._l1d_size = l1d_size
        self._l1d_assoc = l1d_assoc
        self._l2_size = l2_size
        self._l2_assoc = l2_assoc
        self._l3_size = l3_size
        self._l3_assoc = l3_assoc

        self._board = board
        self._cores = cores
        self._ruby_system = ruby_system
        self._cache_line_size = 64

        self._directory_controllers = []

        self._core_complex_id = self._get_core_complex_id()
        self.main_router = RubyRouter(
            self._ruby_system
        )  # this will be connect to component outside the core complex
        self._add_router(self.main_router)
        self._create_core_complex()

    def get_main_router(self):
        return self.main_router

    def _create_core_complex(self):
        # Create L1 caches, L2 cache, and corresponding controllers per core
        self.core_clusters = [
            self._create_core_cluster(core) for core in self._cores
        ]
        # Create L3 cache and its corresponding controller
        self._create_shared_cache()
        # Setting up one router and one external link per controller
        self._create_external_links()
        # Setting up L1/L2 links, L2/main links, L3/main link
        self._create_internal_links()

    def _create_core_cluster(self, core: AbstractCore):
        cluster = SubSystem()
        core_id = self._get_core_id()

        cluster.l1_cache = L1Cache(
            l1i_size=self._l1i_size,
            l1i_assoc=self._l1i_assoc,
            l1d_size=self._l1d_size,
            l1d_assoc=self._l1d_assoc,
            network=self._ruby_system.network,
            core=core,
            cache_line_size=self._cache_line_size,
            target_isa=self._board.processor.get_isa(),
            clk_domain=self._board.get_clock_domain(),
        )
        cluster.l1_cache.sequencer = RubySequencer(
            version=core_id,
            dcache=cluster.l1_cache.Dcache,
            clk_domain=cluster.l1_cache.clk_domain,
        )

        if self._board.has_io_bus():
            cluster.l1_cache.sequencer.connectIOPorts(self._board.get_io_bus())
        cluster.l1_cache.ruby_system = self._ruby_system
        core.connect_icache(cluster.l1_cache.sequencer.in_ports)
        core.connect_dcache(cluster.l1_cache.sequencer.in_ports)
        core.connect_walker_ports(
            cluster.l1_cache.sequencer.in_ports,
            cluster.l1_cache.sequencer.in_ports,
        )
        if self._board.get_processor().get_isa() == ISA.X86:
            core.connect_interrupt(
                cluster.l1_cache.sequencer.interrupt_out_port,
                cluster.l1_cache.sequencer.in_ports,
            )
        else:
            core.connect_interrupt()

        cluster.l2_cache = L2Cache(
            l2_size=self._l2_size,
            l2_assoc=self._l2_assoc,
            network=self._ruby_system.network,
            core=core,
            num_l3Caches=1,  # each core complex has 1 slice of L3 Cache
            cache_line_size=self._cache_line_size,
            cluster_id=self._core_complex_id,
            target_isa=self._board.processor.get_isa(),
            clk_domain=self._board.get_clock_domain(),
        )
        cluster.l2_cache.ruby_system = self._ruby_system
        # L0Cache in the ruby backend is l1 cache in stdlib
        # L1Cache in the ruby backend is l2 cache in stdlib
        cluster.l2_cache.bufferFromL0 = cluster.l1_cache.bufferToL1
        cluster.l2_cache.bufferToL0 = cluster.l1_cache.bufferFromL1

        return cluster

    def _create_shared_cache(self):
        self.l3_cache = L3Cache(
            l3_size=self._l3_size,
            l3_assoc=self._l3_assoc,
            network=self._ruby_system.network,
            num_l3Caches=1,
            cache_line_size=self._cache_line_size,
            cluster_id=self._core_complex_id,
        )
        self.l3_cache.ruby_system = self._ruby_system

    # This is where all routers and links are created
    def _create_external_links(self):
        # create a router per cache controller
        #  - there is one L3 per ccd
        self.l3_router = RubyRouter(self._ruby_system)
        self._add_router(self.l3_router)
        #  - there is one L1 and one L2 per cluster
        for cluster in self.core_clusters:
            cluster.l1_router = RubyRouter(self._ruby_system)
            self._add_router(cluster.l1_router)
            cluster.l2_router = RubyRouter(self._ruby_system)
            self._add_router(cluster.l2_router)

        # create an ext link from a controller to a router
        self.l3_router_link = RubyExtLink(
            ext_node=self.l3_cache,
            int_node=self.l3_router,
            bandwidth_factor=64,
        )
        self._add_ext_link(self.l3_router_link)
        for cluster in self.core_clusters:
            cluster.l1_router_link = RubyExtLink(
                ext_node=cluster.l1_cache, int_node=cluster.l1_router
            )
            self._add_ext_link(cluster.l1_router_link)
            cluster.l2_router_link = RubyExtLink(
                ext_node=cluster.l2_cache, int_node=cluster.l2_router
            )
            self._add_ext_link(cluster.l2_router_link)

    def _create_internal_links(self):
        # create L1/L2 links
        for cluster in self.core_clusters:
            l1_to_l2, l2_to_l1 = RubyIntLink.create_bidirectional_links(
                cluster.l1_router, cluster.l2_router
            )
            cluster.l1_to_l2_link = l1_to_l2
            cluster.l2_to_l1_link = l2_to_l1
            self._add_int_link(l1_to_l2)
            self._add_int_link(l2_to_l1)
        # create L2/main_router links
        for cluster in self.core_clusters:
            l2_to_main, main_to_l2 = RubyIntLink.create_bidirectional_links(
                cluster.l2_router, self.main_router
            )
            cluster.l2_to_main_link = l2_to_main
            cluster.main_to_l2_link = main_to_l2
            self._add_int_link(l2_to_main)
            self._add_int_link(main_to_l2)
        # create L3/main_router link
        l3_to_main, main_to_l3 = RubyIntLink.create_bidirectional_links(
            self.l3_router, self.main_router, bandwidth_factor=64
        )
        self.l3_to_main_link = l3_to_main
        self.main_to_l3_link = main_to_l3
        self._add_int_link(l3_to_main)
        self._add_int_link(main_to_l3)
