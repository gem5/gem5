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

from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

from m5.objects import (
    NULL,
    RubyPortProxy,
    RubySequencer,
    RubySystem,
)
from m5.objects.SubSystem import SubSystem

from gem5.coherence_protocol import CoherenceProtocol
from gem5.utils.requires import requires

requires(coherence_protocol_required=CoherenceProtocol.CHI)

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.cachehierarchies.abstract_cache_hierarchy import (
    AbstractCacheHierarchy,
)
from gem5.components.cachehierarchies.abstract_two_level_cache_hierarchy import (
    AbstractTwoLevelCacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.abstract_ruby_cache_hierarchy import (
    AbstractRubyCacheHierarchy,
)
from gem5.components.cachehierarchies.ruby.topologies.custom_mesh import (
    CustomMesh,
)
from gem5.components.cachehierarchies.ruby.topologies.custom_mesh import (
    NoC_Params as CustomMeshNoC_Params,
)
from gem5.components.processors.abstract_core import AbstractCore
from gem5.isas import ISA
from gem5.utils.override import overrides

from .nodes.abstract_node import (
    CHI_NodeType,
    Node_Params,
)
from .nodes.directory import (
    BaseDirectory,
    SimpleDirectory,
)
from .nodes.dma_requestor import DMARequestor
from .nodes.hnf import (
    CHI_HNF,
)
from .nodes.l1_cache import L1CacheController
from .nodes.l2_cache import L2CacheController
from .nodes.memory_controller import MemoryController
from .nodes.rnf import (
    CHI_RNF,
    CHI_RNI_DMA,
)
from .nodes.snf import (
    CHI_SNF_BootMem,
    CHI_SNF_MainMem,
)


class PrivateL1PrivateL2MeshCacheHierarchy(
    AbstractRubyCacheHierarchy, AbstractTwoLevelCacheHierarchy
):
    """A CHI private L1/private L2 hierarchy wired with a configurable mesh."""

    def __init__(
        self,
        l1i_size: str,
        l1i_assoc: int,
        l1d_size: str,
        l1d_assoc: int,
        l2_size: str,
        l2_assoc: int,
        noc_params: CustomMeshNoC_Params,
    ):
        AbstractRubyCacheHierarchy.__init__(self=self)
        AbstractTwoLevelCacheHierarchy.__init__(
            self,
            l1i_size=l1i_size,
            l1i_assoc=l1i_assoc,
            l1d_size=l1d_size,
            l1d_assoc=l1d_assoc,
            l2_size=l2_size,
            l2_assoc=l2_assoc,
        )

        self._noc_params = noc_params
        self._node_params: Dict[CHI_NodeType, Node_Params] = {}

    def add_nodes(self, node_params: Node_Params):
        self._node_params[node_params.node_type] = node_params

    def _get_node_params(
        self, node_type: CHI_NodeType
    ) -> Optional[Node_Params]:
        mandatory_node_types = (
            CHI_NodeType.CHI_RNF,
            CHI_NodeType.CHI_HNF,
            CHI_NodeType.CHI_SNF_MainMem,
        )

        node_params = self._node_params.get(node_type)
        if node_params is None and node_type in mandatory_node_types:
            raise ValueError(
                f"Node parameters for {node_type.name} must be provided via add_nodes()."
            )
        return node_params

    @overrides(AbstractCacheHierarchy)
    def get_coherence_protocol(self):
        return CoherenceProtocol.CHI

    @overrides(AbstractCacheHierarchy)
    def incorporate_cache(self, board: AbstractBoard) -> None:
        super().incorporate_cache(board)
        self.ruby_system = RubySystem()

        # Ruby's global network.
        self.ruby_system.network = CustomMesh(
            ruby_system=self.ruby_system,
            noc_params=self._noc_params,
        )

        self.ruby_system.network.physical_vnets_channels = [
            self._noc_params.req_int_channels,
            self._noc_params.snp_int_channels,
            self._noc_params.rsp_int_channels,
            self._noc_params.dat_int_channels,
        ]

        self.ruby_system.network.physical_vnets_bandwidth = [
            self._noc_params.cntrl_msg_size,
            self._noc_params.cntrl_msg_size,
            self._noc_params.cntrl_msg_size,
            self._noc_params.cntrl_msg_size + self._noc_params.data_width,
        ]

        # Network configurations
        # virtual networks: 0=request, 1=snoop, 2=response, 3=data
        self.ruby_system.number_of_virtual_networks = 4
        self.ruby_system.network.number_of_virtual_networks = 4

        rnf_params = self._get_node_params(CHI_NodeType.CHI_RNF)
        hnf_params = self._get_node_params(CHI_NodeType.CHI_HNF)
        snf_params = self._get_node_params(CHI_NodeType.CHI_SNF_MainMem)
        snf_boot_params = self._get_node_params(CHI_NodeType.CHI_SNF_BootMem)
        rni_params = self._get_node_params(CHI_NodeType.CHI_RNI_DMA)

        num_hnfs = hnf_params.num_nodes()
        num_rnfs = rnf_params.num_nodes()
        num_snfs = snf_params.num_nodes()
        num_boot_snfs = (
            snf_boot_params.num_nodes() if snf_boot_params is not None else 0
        )
        num_rnis = rni_params.num_nodes() if rni_params is not None else 0

        if num_hnfs < 1:
            raise ValueError("Number of HNFs must be at least 1.")
        if num_hnfs & (num_hnfs - 1):
            raise ValueError("Number of HNFs must be a power of 2.")

        cores = list(board.get_processor().get_cores())

        # This is where we start instantiating CHI nodes
        # The logic might appear weird, because of how
        # we first generate a list and then we assign
        # it to a member variable immediately after.
        # This is because we need to allow for the following
        # gem5 constraints
        #
        # 1) A SimObject cannot have empty list as a child
        # (unless it is a Param), therefore we cannot do
        # direct assignment for optional nodes like for example
        # self.rni_nodes = self._create_dma_controllers(..)
        #
        # 2) If we do the member assignment at the end, all
        # CHI nodes names will be wrong. This is because
        # the first assignment specifies the name. However
        # as we are creating and connecting nodes at the same
        # time, doing something like
        #
        # rnf.setDownstreamNodes(self._hnf_nodes)
        #
        # will name all hnf nodes as rnf.upstream_destination
        # Therefore by immediately doing self.hnf_nodes = hnf_nodes
        # We make sure they are properly named as "hnf_nodes"

        # Create the coherent side of the memory controllers
        snf_nodes, snf_boot_nodes = self._create_snfs(
            board, num_snfs, num_boot_snfs
        )
        self.snf_nodes = snf_nodes
        if num_boot_snfs > 0:
            self.snf_boot_nodes = snf_boot_nodes

        hnf_nodes = self._create_hnfs(board, num_hnfs)
        self.hnf_nodes = hnf_nodes

        # Create one core cluster with a split I/D cache for each core
        rnf_nodes = self._create_rnfs(board, cores, num_rnfs)
        self.rnf_nodes = rnf_nodes

        # Create the DMA Controllers, if required.
        rni_nodes = self._create_dma_controllers(board, num_rnis)
        if num_rnis > 0:
            self.rni_nodes = rni_nodes

        self.ruby_system.num_of_sequencers = len(self.rnf_nodes) * 2 + len(
            rni_nodes
        )

        self.ruby_system.network.connectControllers(
            rnf=(rnf_nodes, rnf_params),
            hnf=(hnf_nodes, hnf_params),
            snf=(snf_nodes, snf_params),
            snf_boot=(snf_boot_nodes, snf_boot_params),
            rni=(rni_nodes, rni_params),
        )

        self._configure_controllers(
            (rnf_nodes, rnf_params),
            (hnf_nodes, hnf_params),
            (snf_nodes, snf_params),
            (snf_boot_nodes, snf_boot_params),
            (rni_nodes, rni_params),
        )

        self.ruby_system.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.ruby_system.sys_port_proxy = RubyPortProxy(
            ruby_system=self.ruby_system
        )
        board.connect_system_port(self.ruby_system.sys_port_proxy.in_ports)

    def _configure_controllers(
        self, *node_groups: Tuple[List[SubSystem], Optional[Node_Params]]
    ) -> None:
        req_channels = self._noc_params.req_ext_channels
        snp_channels = self._noc_params.snp_ext_channels
        rsp_channels = self._noc_params.rsp_ext_channels
        dat_channels = self._noc_params.dat_ext_channels

        for node_list, node_params in node_groups:
            if node_params is None:
                continue

            # Keep aligned with CustomMesh.distribute_nodes latency handling.
            inbound_latency = (
                node_params.inbound_link_latency
                if node_params.inbound_link_latency
                else self._noc_params.node_link_latency
            )
            outbound_latency = (
                node_params.outbound_link_latency
                if node_params.outbound_link_latency
                else self._noc_params.node_link_latency
            )

            for node in node_list:
                for ctrl in node.getNetworkSideControllers():
                    # Enforces the number of channels or ports between
                    # the node and the router by limiting the number of
                    # messages that can be consumed in a cycle
                    ctrl.reqIn.max_dequeue_rate = req_channels
                    ctrl.snpIn.max_dequeue_rate = snp_channels
                    ctrl.rspIn.max_dequeue_rate = rsp_channels
                    ctrl.datIn.max_dequeue_rate = dat_channels
                    ctrl.reqOut.max_dequeue_rate = req_channels
                    ctrl.snpOut.max_dequeue_rate = snp_channels
                    ctrl.rspOut.max_dequeue_rate = rsp_channels
                    ctrl.datOut.max_dequeue_rate = dat_channels

                    # Fine tuned the buffers size to properly create contention
                    # and network stalls.  Sizes are set similarly to
                    # src/mem/ruby/network/simple/SimpleLink.py
                    ctrl.reqIn.buffer_size = req_channels * (
                        inbound_latency + 1
                    )
                    ctrl.snpIn.buffer_size = snp_channels * (
                        inbound_latency + 1
                    )
                    ctrl.rspIn.buffer_size = rsp_channels * (
                        inbound_latency + 1
                    )
                    ctrl.datIn.buffer_size = dat_channels * (
                        inbound_latency + 1
                    )
                    ctrl.reqOut.buffer_size = req_channels * (
                        outbound_latency + 1
                    )
                    ctrl.snpOut.buffer_size = snp_channels * (
                        outbound_latency + 1
                    )
                    ctrl.rspOut.buffer_size = rsp_channels * (
                        outbound_latency + 1
                    )
                    ctrl.datOut.buffer_size = dat_channels * (
                        outbound_latency + 1
                    )

                    # The latency for outbound messages to the network is defined by
                    # these parameters when enqueueing messages in the SLICC code
                    cntrl.request_latency = outbound_latency
                    cntrl.response_latency = outbound_latency
                    cntrl.snoop_latency = outbound_latency
                    cntrl.data_latency = outbound_latency

    def _create_hnfs(
        self, board: AbstractBoard, num_hnfs: int
    ) -> List[CHI_HNF]:
        hnfs = []
        mem_ranges = list(board.get_mem_ranges())
        for idx in range(num_hnfs):
            ranges = BaseDirectory.create_addr_ranges(
                num_directories=num_hnfs,
                dir_idx=idx,
                mem_ranges=mem_ranges,
                cache_line_size=board.get_cache_line_size(),
            )
            directory = SimpleDirectory(
                self.ruby_system.network,
                cache_line_size=board.get_cache_line_size(),
                clk_domain=board.get_clock_domain(),
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

    def _create_rnfs(
        self,
        board: AbstractBoard,
        cores: List[AbstractCore],
        num_rnfs: int,
    ) -> List[CHI_RNF]:
        """Given the core and core number, create a split I/D + private L2."""
        rnfs = []
        for core_num, core in enumerate(cores):
            dcache = L1CacheController(
                size=self._l1d_size,
                assoc=self._l1d_assoc,
                network=self.ruby_system.network,
                requires_send_evicts=core.requires_send_evicts(),
                cache_line_size=board.get_cache_line_size(),
                target_isa=board.get_processor().get_isa(),
                clk_domain=board.get_clock_domain(),
            )
            dcache.sc_lock_enabled = True
            icache = L1CacheController(
                size=self._l1i_size,
                assoc=self._l1i_assoc,
                network=self.ruby_system.network,
                requires_send_evicts=core.requires_send_evicts(),
                cache_line_size=board.get_cache_line_size(),
                target_isa=board.get_processor().get_isa(),
                clk_domain=board.get_clock_domain(),
            )

            icache.sequencer = RubySequencer(
                version=core_num,
                dcache=NULL,
                clk_domain=icache.clk_domain,
                ruby_system=self.ruby_system,
            )
            dcache.sequencer = RubySequencer(
                version=core_num,
                dcache=dcache.cache,
                clk_domain=dcache.clk_domain,
                ruby_system=self.ruby_system,
            )

            l2 = L2CacheController(
                size=self._l2_size,
                assoc=self._l2_assoc,
                network=self.ruby_system.network,
                cache_line_size=board.get_cache_line_size(),
                clk_domain=board.get_clock_domain(),
            )

            rnf = CHI_RNF(self.ruby_system, [dcache, icache, l2])

            if board.has_io_bus():
                dcache.sequencer.connectIOPorts(board.get_io_bus())

            dcache.ruby_system = self.ruby_system
            icache.ruby_system = self.ruby_system
            l2.ruby_system = self.ruby_system

            core.connect_icache(icache.sequencer.in_ports)
            core.connect_dcache(dcache.sequencer.in_ports)

            core.connect_walker_ports(
                dcache.sequencer.in_ports,
                icache.sequencer.in_ports,
            )

            # Connect the interrupt ports
            core.connect_interrupt()

            dcache.downstream_destinations = [l2]
            icache.downstream_destinations = [l2]

            rnf.setDownstreamNodes(self.hnf_nodes)

            rnfs.append(rnf)

        return rnfs

    def _create_snfs(
        self,
        board: AbstractBoard,
        num_snfs: int,
        num_boot_snfs: int,
    ) -> Tuple[List[CHI_SNF_MainMem], List[CHI_SNF_BootMem]]:

        mem_ports = board.get_mem_ports()

        if num_snfs + num_boot_snfs != len(mem_ports):
            raise ValueError(
                f"Defining {num_snfs + num_boot_snfs} CHI_SNF nodes with "
                f"{len(mem_ports)} memory ports."
            )

        snfs_main = []
        snfs_boots = []
        for idx, (rng, port) in enumerate(mem_ports):
            mc = MemoryController(self.ruby_system.network, [rng], port)
            mc.ruby_system = self.ruby_system
            # Clear hack, assuming bootmem ranges are appended at the end
            # of normal memory ranges
            if idx < num_snfs:
                snf = CHI_SNF_MainMem(self.ruby_system, mc)
                snfs_main.append(snf)
            else:
                snf = CHI_SNF_BootMem(self.ruby_system, mc)
                snfs_boots.append(snf)

        return snfs_main, snfs_boots

    def _create_dma_controllers(
        self,
        board: AbstractBoard,
        num_rnis: int,
    ) -> List[CHI_RNI_DMA]:
        rnis = []

        dma_ports = board.get_dma_ports()
        if num_rnis != len(dma_ports):
            raise ValueError(
                f"Defining {num_rnis} CHI_RNI nodes with "
                f"{len(dma_ports)} dma ports."
            )

        for i, port in enumerate(dma_ports):
            ctrl = DMARequestor(
                self.ruby_system.network,
                board.get_cache_line_size(),
                board.get_clock_domain(),
            )
            ctrl.ruby_system = self.ruby_system

            version = len(board.get_processor().get_cores()) + i
            ctrl.sequencer = RubySequencer(
                version=version,
                dcache=NULL,
                in_ports=port,
                ruby_system=self.ruby_system,
            )

            rni = CHI_RNI_DMA(self.ruby_system, ctrl)
            rni.setDownstreamNodes(self.hnf_nodes)

            rnis.append(rni)

        return rnis

    @overrides(AbstractRubyCacheHierarchy)
    def _reset_version_numbers(self):
        from .nodes.abstract_node import CacheController

        CacheController._version = 0
        MemoryController._version = 0
