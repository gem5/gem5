# Copyright (c) 2024 Advanced Micro Devices, Inc.
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

import math

from m5.objects import (
    DMASequencer,
    RubyCache,
    RubyPortProxy,
    RubySequencer,
    RubySystem,
    TreePLRURP,
)

from ...coherence_protocol import CoherenceProtocol
from ...components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from ...components.cachehierarchies.ruby.abstract_ruby_cache_hierarchy import (
    AbstractRubyCacheHierarchy,
)
from ...components.cachehierarchies.ruby.caches.viper.corepair_cache import (
    CorePairCache,
)
from ...components.cachehierarchies.ruby.caches.viper.directory import (
    ViperCPUDirectory,
)
from ...components.cachehierarchies.ruby.caches.viper.dma_controller import (
    ViperCPUDMAController,
)
from ...prebuilt.viper.board import ViperBoard
from ...utils.override import overrides
from ...utils.requires import requires
from .viper_network import SimplePt2Pt


class ViperCPUCacheHierarchy(AbstractRubyCacheHierarchy):
    """
    The VIPER CPU cache hierarchy creates CPU-side Ruby caches and connects
    the nodes using a simple point-to-point topology.
    """

    def __init__(
        self,
        l1d_size: str,
        l1d_assoc: int,
        l1i_size: str,
        l1i_assoc: int,
        l2_size: str,
        l2_assoc: int,
        l3_size: str,
        l3_assoc: int,
    ):
        """
        :param size: The size of each cache in the heirarchy.
        :param assoc: The associativity of each cache.
        :param device_dmas: Optional list of CPU connect device DMAs
        """
        super().__init__()

        self._l1d_size = l1d_size
        self._l1d_assoc = l1d_assoc
        self._l1i_size = l1i_size
        self._l1i_assoc = l1i_assoc
        self._l2_size = l2_size
        self._l2_assoc = l2_assoc
        self._l3_size = l3_size
        self._l3_assoc = l3_assoc

        self.ruby_system = RubySystem()

    @overrides(AbstractCacheHierarchy)
    def incorporate_cache(self, board: ViperBoard) -> None:
        requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)

        # Ruby networks for CPU
        self.ruby_system.network = SimplePt2Pt(self.ruby_system)

        # MOESI_AMD_Base uses 5 virtual networks.
        self.ruby_system.number_of_virtual_networks = 5
        self.ruby_system.network.number_of_virtual_networks = 5

        # There is a single local list of all of the controllers to make it
        # easier to connect everything to the CPU network. This can be
        # customized depending on the topology/network requirements.
        # Create one controller for each L1 cache (and the cache mem obj.)
        # Create a single directory controller (Really the memory cntrl).
        self._controllers = []

        cores = board.get_processor().get_cores()
        num_cores = len(cores)
        for i in range(0, num_cores, 2):
            cache = CorePairCache(
                l1d_size=self._l1d_size,
                l1d_assoc=self._l1d_assoc,
                l1i_size=self._l1i_size,
                l1i_assoc=self._l1i_assoc,
                l2_size=self._l2_size,
                l2_assoc=self._l2_assoc,
                network=self.ruby_system.network,
                cache_line_size=board.get_cache_line_size(),
                core=cores[i],
            )

            cache.version = i // 2
            cache.ruby_system = self.ruby_system
            cache.clk_domain = board.get_clock_domain()

            cache.sequencer = RubySequencer(
                version=i,
                dcache=cache.L1D0cache,
                ruby_system=self.ruby_system,
                coreid=0,
                is_cpu_sequencer=True,
                clk_domain=board.get_clock_domain(),
            )

            cache.sequencer1 = RubySequencer(
                version=i + 1,
                dcache=cache.L1D1cache,
                ruby_system=self.ruby_system,
                coreid=1,
                is_cpu_sequencer=True,
                clk_domain=board.get_clock_domain(),
            )

            cache.sequencer.connectIOPorts(board.get_io_bus())
            cache.sequencer1.connectIOPorts(board.get_io_bus())

            cores[i].connect_icache(cache.sequencer.in_ports)
            cores[i].connect_dcache(cache.sequencer.in_ports)

            cores[i].connect_walker_ports(
                cache.sequencer.in_ports, cache.sequencer.in_ports
            )

            # Connect the interrupt ports
            int_req_port = cache.sequencer.interrupt_out_port
            int_resp_port = cache.sequencer.in_ports
            cores[i].connect_interrupt(int_req_port, int_resp_port)

            if i + 1 < num_cores:
                cores[i + 1].connect_icache(cache.sequencer1.in_ports)
                cores[i + 1].connect_dcache(cache.sequencer1.in_ports)

                cores[i + 1].connect_walker_ports(
                    cache.sequencer.in_ports, cache.sequencer1.in_ports
                )

                # Connect the interrupt ports
                cores[i + 1].connect_interrupt(int_req_port, int_resp_port)

            self._controllers.append(cache)

        # Create the CPU directory controllers
        self._directory_controllers = []

        # Automatically determine the numa bit. This can be changed to
        # increase the number of bytes to each memory channel before
        # going to the next channels
        dir_bits = int(math.log(len(board.get_mem_ports()), 2))
        block_size_bits = int(math.log(board.get_cache_line_size()))

        for addr_range, port in board.get_mem_ports():
            dir = ViperCPUDirectory(
                self.ruby_system.network,
                board.get_cache_line_size(),
                addr_range,
                port,
            )
            dir.ruby_system = self.ruby_system
            dir.version = len(self._directory_controllers)
            self._directory_controllers.append(dir)

            dir.L3CacheMemory = RubyCache(
                size=self._l3_size,
                assoc=self._l3_assoc,
                replacement_policy=TreePLRURP(),
                resourceStalls=False,
                dataArrayBanks=16,
                tagArrayBanks=16,
                dataAccessLatency=20,
                tagAccessLatency=15,
            )

        # Create the DMA Controllers, if required.
        self._dma_controllers = []
        if board.has_dma_ports():
            dma_ports = board.get_dma_ports()
            for i, port in enumerate(dma_ports):
                ctrl = ViperCPUDMAController(
                    self.ruby_system.network, board.get_cache_line_size()
                )
                ctrl.dma_sequencer = DMASequencer(version=i, in_ports=port)

                ctrl.version = len(self._dma_controllers)
                ctrl.ruby_system = self.ruby_system
                ctrl.dma_sequencer.ruby_system = self.ruby_system

                self._dma_controllers.append(ctrl)

        # Create DMA Controllers requires for any devices in the system.
        device_dmas = []
        if board.get_devices() is not None:
            for device in board.get_devices():
                device_dmas += device.get_cpu_dma_ports()

        if len(device_dmas) > 0:
            for _, port in enumerate(device_dmas):
                ctrl = ViperCPUDMAController(
                    self.ruby_system.network, board.get_cache_line_size()
                )
                ctrl.dma_sequencer = DMASequencer(
                    version=len(self._dma_controllers), in_ports=port
                )

                ctrl.version = len(self._dma_controllers)
                ctrl.ruby_system = self.ruby_system
                ctrl.dma_sequencer.ruby_system = self.ruby_system

                self._dma_controllers.append(ctrl)

        # Number of sequencers = one per core pair + one per DMA
        self.ruby_system.num_of_sequencers = len(self._controllers) * 2 + len(
            self._dma_controllers
        )

        # Assign the controllers to their parent objects.
        self.ruby_system.controllers = self._controllers
        self.ruby_system.directory_controllers = self._directory_controllers

        if len(self._dma_controllers) != 0:
            self.ruby_system.dma_controllers = self._dma_controllers

        # Connect the controllers using the network topology
        self.ruby_system.network.connect(
            self._controllers
            + self._directory_controllers
            + self._dma_controllers
        )
        self.ruby_system.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.ruby_system.sys_port_proxy = RubyPortProxy(
            ruby_system=self.ruby_system
        )
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)
