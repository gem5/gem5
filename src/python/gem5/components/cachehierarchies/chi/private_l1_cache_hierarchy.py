# Copyright (c) 2021 The Regents of the University of California
# All Rights Reserved.
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
from itertools import chain
from typing import List

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.abstract_ruby_cache_hierarchy import (
    AbstractRubyCacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.topologies.simple_pt2pt import (
    SimplePt2Pt,
)
from gem5.components.processors.abstract_core import AbstractCore
from gem5.isas import ISA
from gem5.utils.override import overrides
from gem5.utils.requires import requires
from m5.objects import NULL
from m5.objects import RubyPortProxy
from m5.objects import RubySequencer
from m5.objects import RubySystem
from m5.objects.SubSystem import SubSystem

from .nodes.directory import SimpleDirectory
from .nodes.dma_requestor import DMARequestor
from .nodes.memory_controller import MemoryController
from .nodes.private_l1_moesi_cache import PrivateL1MOESICache


class PrivateL1CacheHierarchy(AbstractRubyCacheHierarchy):
    """A single level cache based on CHI for RISC-V

    This hierarchy has a split I/D L1 caches per CPU, a single directory (HNF),
    and as many memory controllers (SNF) as memory channels. The directory does
    not have an associated cache.

    The network is a simple point-to-point between all of the controllers.
    """

    def __init__(self, size: str, assoc: int) -> None:
        """
        :param size: The size of the priavte I/D caches in the hierarchy.
        :param assoc: The associativity of each cache.
        """
        super().__init__()

        self._size = size
        self._assoc = assoc

    @overrides(AbstractCacheHierarchy)
    def incorporate_cache(self, board: AbstractBoard) -> None:

        requires(coherence_protocol_required=CoherenceProtocol.CHI)

        self.ruby_system = RubySystem()

        # Ruby's global network.
        self.ruby_system.network = SimplePt2Pt(self.ruby_system)

        # Network configurations
        # virtual networks: 0=request, 1=snoop, 2=response, 3=data
        self.ruby_system.number_of_virtual_networks = 4
        self.ruby_system.network.number_of_virtual_networks = 4

        # Create a single centralized directory
        self.directory = SimpleDirectory(
            self.ruby_system.network,
            cache_line_size=board.get_cache_line_size(),
            clk_domain=board.get_clock_domain(),
        )
        self.directory.ruby_system = self.ruby_system

        # Create one core cluster with a split I/D cache for each core
        self.core_clusters = [
            self._create_core_cluster(core, i, board)
            for i, core in enumerate(board.get_processor().get_cores())
        ]

        # Create the coherent side of the memory controllers
        self.memory_controllers = self._create_memory_controllers(board)
        self.directory.downstream_destinations = self.memory_controllers

        # Create the DMA Controllers, if required.
        if board.has_dma_ports():
            self.dma_controllers = self._create_dma_controllers(board)
            self.ruby_system.num_of_sequencers = len(
                self.core_clusters
            ) * 2 + len(self.dma_controllers)
        else:
            self.ruby_system.num_of_sequencers = len(self.core_clusters) * 2

        self.ruby_system.network.connectControllers(
            list(
                chain.from_iterable(  # Grab the controllers from each cluster
                    [
                        (cluster.dcache, cluster.icache)
                        for cluster in self.core_clusters
                    ]
                )
            )
            + self.memory_controllers
            + [self.directory]
            + (self.dma_controllers if board.has_dma_ports() else [])
        )

        self.ruby_system.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.ruby_system.sys_port_proxy = RubyPortProxy()
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)

    def _create_core_cluster(
        self, core: AbstractCore, core_num: int, board: AbstractBoard
    ) -> SubSystem:
        """Given the core and the core number this function creates a cluster
        for the core with a split I/D cache
        """
        cluster = SubSystem()
        cluster.dcache = PrivateL1MOESICache(
            size=self._size,
            assoc=self._assoc,
            network=self.ruby_system.network,
            core=core,
            cache_line_size=board.get_cache_line_size(),
            target_isa=board.get_processor().get_isa(),
            clk_domain=board.get_clock_domain(),
        )
        cluster.icache = PrivateL1MOESICache(
            size=self._size,
            assoc=self._assoc,
            network=self.ruby_system.network,
            core=core,
            cache_line_size=board.get_cache_line_size(),
            target_isa=board.get_processor().get_isa(),
            clk_domain=board.get_clock_domain(),
        )

        cluster.icache.sequencer = RubySequencer(
            version=core_num, dcache=NULL, clk_domain=cluster.icache.clk_domain
        )
        cluster.dcache.sequencer = RubySequencer(
            version=core_num,
            dcache=cluster.dcache.cache,
            clk_domain=cluster.dcache.clk_domain,
        )

        if board.has_io_bus():
            cluster.dcache.sequencer.connectIOPorts(board.get_io_bus())

        cluster.dcache.ruby_system = self.ruby_system
        cluster.icache.ruby_system = self.ruby_system

        core.connect_icache(cluster.icache.sequencer.in_ports)
        core.connect_dcache(cluster.dcache.sequencer.in_ports)

        core.connect_walker_ports(
            cluster.dcache.sequencer.in_ports,
            cluster.icache.sequencer.in_ports,
        )

        # Connect the interrupt ports
        if board.get_processor().get_isa() == ISA.X86:
            int_req_port = cluster.dcache.sequencer.interrupt_out_port
            int_resp_port = cluster.dcache.sequencer.in_ports
            core.connect_interrupt(int_req_port, int_resp_port)
        else:
            core.connect_interrupt()

        cluster.dcache.downstream_destinations = [self.directory]
        cluster.icache.downstream_destinations = [self.directory]

        return cluster

    def _create_memory_controllers(
        self, board: AbstractBoard
    ) -> List[MemoryController]:
        memory_controllers = []
        for rng, port in board.get_mem_ports():
            mc = MemoryController(self.ruby_system.network, rng, port)
            mc.ruby_system = self.ruby_system
            memory_controllers.append(mc)
        return memory_controllers

    def _create_dma_controllers(
        self, board: AbstractBoard
    ) -> List[DMARequestor]:
        dma_controllers = []
        for i, port in enumerate(board.get_dma_ports()):
            ctrl = DMARequestor(
                self.ruby_system.network,
                board.get_cache_line_size(),
                board.get_clock_domain(),
            )
            version = len(board.get_processor().get_cores()) + i
            ctrl.sequencer = RubySequencer(version=version, in_ports=port)
            ctrl.sequencer.dcache = NULL

            ctrl.ruby_system = self.ruby_system
            ctrl.sequencer.ruby_system = self.ruby_system

            ctrl.downstream_destinations = [self.directory]

            dma_controllers.append(ctrl)

        return dma_controllers
