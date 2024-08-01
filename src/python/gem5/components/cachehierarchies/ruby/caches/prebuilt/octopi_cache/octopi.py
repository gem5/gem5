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

from typing import List

from m5.objects.RubySystem import RubySystem
from m5.objects.Sequencer import (
    DMASequencer,
    RubyPortProxy,
)

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.directory import (
    Directory,
)
from gem5.components.cachehierarchies.ruby.caches.mesi_three_level.dma_controller import (
    DMAController,
)
from gem5.utils.requires import requires

from .....abstract_three_level_cache_hierarchy import (
    AbstractThreeLevelCacheHierarchy,
)
from ....abstract_ruby_cache_hierarchy import AbstractRubyCacheHierarchy
from .core_complex import CoreComplex
from .octopi_network import OctopiNetwork
from .ruby_network_components import (
    RubyExtLink,
    RubyIntLink,
    RubyRouter,
)


# CoreComplex sub-systems own the L1, L2, L3 controllers
# OctopiCache owns the directory controllers
# RubySystem owns the DMA Controllers
class OctopiCache(
    AbstractRubyCacheHierarchy, AbstractThreeLevelCacheHierarchy
):
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
        num_core_complexes: int,
        is_fullsystem: bool,
    ):
        AbstractRubyCacheHierarchy.__init__(self=self)
        AbstractThreeLevelCacheHierarchy.__init__(
            self=self,
            l1i_size=l1i_size,
            l1i_assoc=l1i_assoc,
            l1d_size=l1d_size,
            l1d_assoc=l1d_assoc,
            l2_size=l2_size,
            l2_assoc=l2_assoc,
            l3_size=l3_size,
            l3_assoc=l3_assoc,
        )

        self._directory_controllers: List = []
        self._dma_controllers: List = []
        self._io_controllers: List = []
        self._core_complexes: List = []
        self._num_core_complexes = num_core_complexes
        self._is_fullsystem = is_fullsystem

    def incorporate_cache(self, board: AbstractBoard) -> None:
        requires(
            coherence_protocol_required=CoherenceProtocol.MESI_THREE_LEVEL
        )

        cache_line_size = board.get_cache_line_size()

        self.ruby_system = RubySystem()
        # MESI_Three_Level needs 3 virtual networks
        self.ruby_system.number_of_virtual_networks = 3
        self.ruby_system.network = OctopiNetwork(self.ruby_system)

        # Setting up the core complex
        all_cores = board.get_processor().get_cores()
        num_cores_per_core_complex = len(all_cores) // self._num_core_complexes

        self.core_complexes = [
            CoreComplex(
                board=board,
                cores=all_cores[
                    core_complex_idx
                    * num_cores_per_core_complex : (core_complex_idx + 1)
                    * num_cores_per_core_complex
                ],
                ruby_system=self.ruby_system,
                l1i_size=self._l1i_size,
                l1i_assoc=self._l1i_assoc,
                l1d_size=self._l1d_size,
                l1d_assoc=self._l1d_assoc,
                l2_size=self._l2_size,
                l2_assoc=self._l2_assoc,
                l3_size=self._l3_size,
                l3_assoc=self._l3_assoc,
            )
            for core_complex_idx in range(self._num_core_complexes)
        ]

        self.ruby_system.network.incorporate_ccds(self.core_complexes)

        self._create_directory_controllers(board)
        self._create_dma_controllers(board, self.ruby_system)

        self.ruby_system.num_of_sequencers = (
            len(all_cores)
            + len(self._dma_controllers)
            + len(self._io_controllers)
        )
        # SimpleNetwork requires .int_links and .routers to exist
        # if we want to call SimpleNetwork.setup_buffers()
        self.ruby_system.network.int_links = (
            self.ruby_system.network._int_links
        )
        self.ruby_system.network.ext_links = (
            self.ruby_system.network._ext_links
        )
        self.ruby_system.network.routers = self.ruby_system.network._routers
        self.ruby_system.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.ruby_system.sys_port_proxy = RubyPortProxy()
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)

    def _create_directory_controllers(self, board):
        # Adding controllers
        self.directory_controllers = [
            Directory(
                self.ruby_system.network,
                board.get_cache_line_size(),
                addr_range,
                mem_port,
            )
            for addr_range, mem_port in board.get_mem_ports()
        ]
        for ctrl in self.directory_controllers:
            ctrl.ruby_system = self.ruby_system
        # Adding controller routers
        self.directory_controller_routers = [
            RubyRouter(self.ruby_system.network)
            for _ in range(len(self.directory_controllers))
        ]
        for router in self.directory_controller_routers:
            self.ruby_system.network._add_router(router)
        # Adding an external link for each controller and its router
        self.directory_controller_ext_links = [
            RubyExtLink(ext_node=dir_ctrl, int_node=dir_router)
            for dir_ctrl, dir_router in zip(
                self.directory_controllers, self.directory_controller_routers
            )
        ]
        for ext_link in self.directory_controller_ext_links:
            self.ruby_system.network._add_ext_link(ext_link)
        _directory_controller_int_links = []
        for router in self.directory_controller_routers:
            int_link_1, int_link_2 = RubyIntLink.create_bidirectional_links(
                router, self.ruby_system.network.cross_ccd_router
            )
            _directory_controller_int_links.extend([int_link_1, int_link_2])
            self.ruby_system.network._add_int_link(int_link_1)
            self.ruby_system.network._add_int_link(int_link_2)
        self.directory_controller_int_links = _directory_controller_int_links

    def _create_dma_controllers(self, board, ruby_system):
        # IOController for full system simulation
        if self._is_fullsystem:
            self.io_sequencer = DMASequencer(
                version=0, ruby_system=self.ruby_system
            )
            self.io_sequencer.in_ports = board.get_mem_side_coherent_io_port()
            self.ruby_system.io_controller = DMAController(
                dma_sequencer=self.io_sequencer, ruby_system=self.ruby_system
            )
            self._io_controllers.append(self.ruby_system.io_controller)
            self.io_controller_router = RubyRouter(self.ruby_system.network)
            self.ruby_system.network._add_router(self.io_controller_router)
            self.io_controller_ext_link = RubyExtLink(
                ext_node=self._io_controllers[0],
                int_node=self.io_controller_router,
            )
            self.ruby_system.network._add_ext_link(self.io_controller_ext_link)
            self.io_controller_int_links = (
                RubyIntLink.create_bidirectional_links(
                    self.io_controller_router,
                    self.ruby_system.network.cross_ccd_router,
                )
            )
            self.ruby_system.network._add_int_link(
                self.io_controller_int_links[0]
            )
            self.ruby_system.network._add_int_link(
                self.io_controller_int_links[1]
            )

        self._dma_controllers = []
        if board.has_dma_ports():
            self.ruby_system.dma_controllers = [
                DMAController(
                    dma_sequencer=DMASequencer(version=i + 1, in_ports=port),
                    ruby_system=self.ruby_system,
                )
                for i, port in enumerate(board.get_dma_ports())
            ]
            self._dma_controllers = self.ruby_system.dma_controllers
            self.dma_routers = [
                RubyRouter(self.ruby_system.network)
                for dma_controller in self._dma_controllers
            ]
            for dma_router in self.dma_routers:
                self.ruby_system.network._add_router(dma_router)
            self.dma_ext_links = [
                RubyExtLink(ext_node=dma_controller, int_node=dma_router)
                for dma_controller, dma_router in zip(
                    self._dma_controllers, self.dma_routers
                )
            ]
            for link in self.dma_ext_links:
                self.ruby_system.network._add_ext_link(link)
            self.dma_int_links = [
                RubyIntLink(
                    dma_router, self.ruby_system.network.cross_ccd_router
                )
                for dma_router in self.dma_routers
            ] + [
                RubyIntLink(
                    self.ruby_system.network.cross_ccd_router, dma_router
                )
                for dma_router in self.dma_routers
            ]
            for link in self.dma_int_links:
                self.ruby_system.network._add_int_link(link)
