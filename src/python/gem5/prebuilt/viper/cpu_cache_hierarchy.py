# Copyright (c) 2021 The Regents of the University of California
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

from ...components.cachehierarchies.ruby.abstract_ruby_cache_hierarchy import (
    AbstractRubyCacheHierarchy,
)
from ...components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from ...components.boards.abstract_board import AbstractBoard
from ...coherence_protocol import CoherenceProtocol
from ...isas import ISA
from ...utils.override import overrides
from ...utils.requires import requires

from .viper_network import DisjointSimplePt2Pt

from ...components.cachehierarchies.ruby.caches.viper.dma_controller import (
    ViperDMAController,
)
from ...components.cachehierarchies.ruby.caches.viper.directory import (
    ViperDirectory,
)
from ...components.cachehierarchies.ruby.caches.viper.corepair_cache import (
    CorePairCache,
)

from m5.objects import (
    RubySystem,
    RubySequencer,
    DMASequencer,
    RubyPortProxy,
    RubyCache,
    Directory_Controller,
)

import math


class ViperCPUCacheHierarchy(AbstractRubyCacheHierarchy):
    _seqs = 0

    @classmethod
    def seqCount(cls):
        # Use SeqCount not class since we need global count
        cls._seqs += 1
        return cls._seqs - 1

    _cntrls = 0

    @classmethod
    def cntrlCount(cls):
        # Use CntlCount not class since we need global count
        cls._cntrls += 1
        return cls._cntrls - 1

    _version = 0

    @classmethod
    def versionCount(cls):
        cls._version += 1  # Use count for this particular type
        return cls._version - 1

    """
    The MI_Example cache hierarchy creates a Ruby cache for each code in a
    simple point-to-point topology.
    """

    def __init__(
        self,
        l1d_size: str,
        l1d_assoc: int,
        l1i_size: str,
        l1i_assoc: int,
        l2_size: str,
        l2_assoc: int,
    ):
        """
        :param size: The size of each cache in the heirarchy.
        :param assoc: The associativity of each cache.
        """
        super().__init__()

        self._l1d_size = l1d_size
        self._l1d_assoc = l1d_assoc
        self._l1i_size = l1i_size
        self._l1i_assoc = l1i_assoc
        self._l2_size = l2_size
        self._l2_assoc = l2_assoc

        self.ruby_system = RubySystem()

    @overrides(AbstractCacheHierarchy)
    def incorporate_cache(self, board: AbstractBoard) -> None:

        requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)

        # Ruby networks for CPU
        self.ruby_system.network_cpu = DisjointSimplePt2Pt(self.ruby_system)

        # MI Example users 5 virtual networks.
        self.ruby_system.number_of_virtual_networks = 11
        self.ruby_system.network_cpu.number_of_virtual_networks = 11

        # There is a single global list of all of the controllers to make it
        # easier to connect everything to the global network. This can be
        # customized depending on the topology/network requirements.
        # Create one controller for each L1 cache (and the cache mem obj.)
        # Create a single directory controller (Really the memory cntrl).
        self._controllers = []

        num_cores = len(board.get_processor().get_cores())
        for i in range((num_cores + 1) // 2):
            cache = CorePairCache(
                l1d_size=self._l1d_size,
                l1d_assoc=self._l1d_assoc,
                l1i_size=self._l1i_size,
                l1i_assoc=self._l1i_assoc,
                l2_size=self._l2_size,
                l2_assoc=self._l2_assoc,
                network=self.ruby_system.network_cpu,
                cache_line_size=board.get_cache_line_size(),
                clk_domain=board.get_clock_domain(),
            )

            cache.version = self.versionCount()

            cache.sequencer = RubySequencer(
                version=self.seqCount(),
                dcache=cache.L1D0cache,
                ruby_system=self.ruby_system,
                coreid=0,
                is_cpu_sequencer=True,
                clk_domain=board.get_clock_domain(),
            )

            cache.sequencer1 = RubySequencer(
                version=self.seqCount(),
                dcache=cache.L1D1cache,
                ruby_system=self.ruby_system,
                coreid=1,
                is_cpu_sequencer=True,
                clk_domain=board.get_clock_domain(),
            )

            # Needed ?
            # if board.has_io_bus():
            #    cache.sequencer.connectIOPorts(board.get_io_bus())

            cache.ruby_system = self.ruby_system

            # ???
            # core.connect_icache(cache.sequencer.in_ports)
            # core.connect_dcache(cache.sequencer.in_ports)

            # ??
            # core.connect_walker_ports(
            #    cache.sequencer.in_ports, cache.sequencer.in_ports
            # )

            # ??
            # Connect the interrupt ports
            # int_req_port = cache.sequencer.interrupt_out_port
            # int_resp_port = cache.sequencer.in_ports
            # core.connect_interrupt(int_req_port, int_resp_port)

            self._controllers.append(cache)

        # Create the CPU directory controllers
        self._directory_controllers = []

        # Automatically determine the numa bit. This can be changed to
        # increase the number of bytes to each memory channel before
        # going to the next channels
        dir_bits = int(math.log(len(board.get_mem_ports()), 2))
        block_size_bits = int(math.log(board.get_cache_line_size()))

        # Stuck here for now.  There doesn't seem to be a way to differentiate
        # between device mem ports and cpu mem ports
        for addr_range, port in board.get_mem_ports():
            dir = ViperDirectory(
                self.ruby_system.network_cpu,
                board.get_cache_line_size(),
                addr_range,
                port,
            )
            dir.ruby_system = self.ruby_system
            self._directory_controllers.append(dir)

        # Create the DMA Controllers, if required.
        self._dma_controllers = []
        if board.has_dma_ports():
            dma_ports = board.get_dma_ports()
            for i, port in enumerate(dma_ports):
                ctrl = ViperDMAController(
                    self.ruby_system.network_cpu, board.get_cache_line_size()
                )
                ctrl.dma_sequencer = DMASequencer(version=i, in_ports=port)

                ctrl.ruby_system = self.ruby_system
                ctrl.dma_sequencer.ruby_system = self.ruby_system

                self._dma_controllers.append(ctrl)

        self.ruby_system.num_of_sequencers = len(self._controllers) + len(
            self._dma_controllers
        )

        # Connect the controllers.
        # self.ruby_system.controllers = self._controllers
        # self.ruby_system.directory_controllers = self._directory_controllers

        if len(self._dma_controllers) != 0:
            self.ruby_system.dma_controllers = self._dma_controllers

        self.ruby_system.network_cpu.connectCPU(
            self._controllers
            + self._directory_controllers
            + self._dma_controllers
        )
        self.ruby_system.network_cpu.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.ruby_system.sys_port_proxy = RubyPortProxy()
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)

    # Due to the way the 'disjoint' network is setup, we need to share
    # the RubySystem.
    def get_ruby(self):
        return self.ruby_system
