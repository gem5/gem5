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
    AddrRange,
    DMASequencer,
    HBM_2000_4H_1x64,
    MemCtrl,
    RubyCache,
    RubySequencer,
    RubySystem,
    SrcClockDomain,
    TreePLRURP,
    VIPERCoalescer,
    VoltageDomain,
)

from ...coherence_protocol import CoherenceProtocol
from ...components.cachehierarchies.ruby.abstract_ruby_cache_hierarchy import (
    AbstractRubyCacheHierarchy,
)
from ...components.cachehierarchies.ruby.caches.viper.directory import (
    ViperGPUDirectory,
)
from ...components.cachehierarchies.ruby.caches.viper.dma_controller import (
    ViperGPUDMAController,
)
from ...components.cachehierarchies.ruby.caches.viper.sqc import SQCCache
from ...components.cachehierarchies.ruby.caches.viper.tcc import TCCCache
from ...components.cachehierarchies.ruby.caches.viper.tcp import TCPCache
from ...components.devices.gpus.viper_shader import ViperShader
from ...utils.requires import requires
from .viper_network import (
    SimpleDoubleCrossbar,
    SimplePt2Pt,
)


class ViperGPUCacheHierarchy(AbstractRubyCacheHierarchy):
    _seqs = 0

    @classmethod
    def seqCount(cls):
        # Use SeqCount not class since we need global count
        cls._seqs += 1
        return cls._seqs - 1

    def __init__(
        self,
        tcp_size: str,
        tcp_assoc: int,
        sqc_size: str,
        sqc_assoc: int,
        scalar_size: str,
        scalar_assoc: int,
        tcc_size: str,
        tcc_assoc: int,
        tcc_count: int,
        cu_per_sqc: int,
        num_memory_channels: int,
        cache_line_size: int,
        shader: ViperShader,
    ):
        """
        :param size: The size of each cache in the heirarchy.
        :param assoc: The associativity of each cache.
        """
        super().__init__()

        self._tcp_size = tcp_size
        self._tcp_assoc = tcp_assoc
        self._sqc_size = sqc_size
        self._sqc_assoc = sqc_assoc
        self._scalar_size = scalar_size
        self._scalar_assoc = scalar_assoc
        self._tcc_size = tcc_size
        self._tcc_assoc = tcc_assoc
        self._cache_line_size = cache_line_size

        # We have everything we need to know to create the GPU cache hierarchy
        # immediately. Therefore, an incorporate_cache method is not part of
        # this cache hierarchy. Go ahead and incorporate everything now.
        requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)

        self.ruby_gpu = RubySystem()
        self.ruby_gpu.block_size_bytes = cache_line_size

        # Ruby network for this GPU
        self.ruby_gpu.network = SimpleDoubleCrossbar(self.ruby_gpu)

        # VIPER uses 6 virtual networks.
        self.ruby_gpu.number_of_virtual_networks = 6
        self.ruby_gpu.network.number_of_virtual_networks = 6

        # There is a single local list of all of the controllers to make it
        # easier to connect everything to the GPU network. This can be
        # customized depending on the topology/network requirements.
        self._controllers = []
        self._directory_controllers = []
        self._dma_controllers = []
        self._mem_ctrls = []

        self.clk_domain = SrcClockDomain(
            clock="1801MHz",
            voltage_domain=VoltageDomain(),
        )

        # Variables used by multiple objects are defined once here
        tcc_bits = int(math.log(tcc_count, 2))
        deadlock_threshold = 500000

        # Create one TCP per CU
        compute_units = shader.get_compute_units()
        for idx, cu in enumerate(compute_units):
            tcp = TCPCache(
                tcp_size=self._tcp_size,
                tcp_assoc=self._tcp_assoc,
                network=self.ruby_gpu.network,
                cache_line_size=self._cache_line_size,
            )

            tcp.version = idx

            tcp.sequencer = RubySequencer(
                version=self.seqCount(),
                dcache=tcp.L1cache,
                ruby_system=self.ruby_gpu,
                is_cpu_sequencer=True,
            )

            tcp.coalescer = VIPERCoalescer(
                version=self.seqCount(),
                icache=tcp.L1cache,
                dcache=tcp.L1cache,
                ruby_system=self.ruby_gpu,
                support_inst_reqs=False,
                is_cpu_sequencer=False,
                deadlock_threshold=deadlock_threshold,
                max_coalesces_per_cycle=1,
                gmTokenPort=cu.gmTokenPort,
            )

            for port_idx in range(cu.wf_size):
                cu.memory_port[port_idx] = tcp.coalescer.in_ports

            tcp.ruby_system = self.ruby_gpu
            tcp.TCC_select_num_bits = tcc_bits
            tcp.use_seq_not_coal = False
            tcp.issue_latency = 1
            tcp.clk_domain = self.clk_domain
            tcp.recycle_latency = 10
            tcp.WB = False
            tcp.disableL1 = False

            self._controllers.append(tcp)

        # This check ensures there are a same number of CUs with shared SQC
        # and Scalar caches.
        num_cus = len(shader.get_compute_units())
        assert (num_cus % cu_per_sqc) == 0
        num_sqcs = num_cus // cu_per_sqc

        for idx in range(num_sqcs):
            sqc = SQCCache(
                sqc_size=self._sqc_size,
                sqc_assoc=self._sqc_assoc,
                network=self.ruby_gpu.network,
                cache_line_size=self._cache_line_size,
            )

            sqc.version = idx

            sqc.sequencer = RubySequencer(
                version=self.seqCount(),
                dcache=sqc.L1cache,
                ruby_system=self.ruby_gpu,
                support_data_reqs=False,
                is_cpu_sequencer=False,
                deadlock_threshold=deadlock_threshold,
            )

            # SQC is shared across {cu_per_sqc} CUs.
            cu_base = cu_per_sqc * idx
            for cu_num in range(cu_per_sqc):
                cu_id = cu_base + cu_num
                compute_units[cu_id].sqc_port = sqc.sequencer.in_ports

            sqc.ruby_system = self.ruby_gpu
            sqc.TCC_select_num_bits = tcc_bits
            sqc.clk_domain = self.clk_domain
            sqc.recycle_latency = 10

            self._controllers.append(sqc)

        num_scalars = num_sqcs
        for idx in range(num_scalars):
            scalar = SQCCache(
                sqc_size=self._scalar_size,
                sqc_assoc=self._scalar_assoc,
                network=self.ruby_gpu.network,
                cache_line_size=self._cache_line_size,
            )

            # Scalar uses same controller as SQC, so add SQC count
            scalar.version = idx + num_sqcs

            scalar.sequencer = RubySequencer(
                version=self.seqCount(),
                dcache=scalar.L1cache,
                ruby_system=self.ruby_gpu,
                support_data_reqs=False,
                is_cpu_sequencer=False,
                deadlock_threshold=deadlock_threshold,
            )

            # Scalar cache is shared across {cu_per_sqc} CUs.
            cu_base = cu_per_sqc * idx
            for cu_num in range(cu_per_sqc):
                cu_id = cu_base + cu_num
                compute_units[cu_id].scalar_port = scalar.sequencer.in_ports

            scalar.ruby_system = self.ruby_gpu
            scalar.TCC_select_num_bits = tcc_bits
            scalar.clk_domain = self.clk_domain
            scalar.recycle_latency = 10

            self._controllers.append(scalar)

        # Create TCCs (GPU L2 cache)
        for idx in range(tcc_count):
            tcc = TCCCache(
                tcc_size=self._tcc_size,
                tcc_assoc=self._tcc_assoc,
                network=self.ruby_gpu.network,
                cache_line_size=self._cache_line_size,
            )

            tcc.version = idx

            tcc.ruby_system = self.ruby_gpu
            tcc.WB = False
            tcc.clk_domain = self.clk_domain
            tcc.recycle_latency = 10

            self._controllers.append(tcc)

        # Create DMA controllers
        for i, port in enumerate(shader.get_gpu_dma_ports()):
            ctrl = ViperGPUDMAController(
                self.ruby_gpu.network, self._cache_line_size
            )
            ctrl.dma_sequencer = DMASequencer(version=i, in_ports=port)

            ctrl.ruby_system = self.ruby_gpu
            ctrl.dma_sequencer.ruby_system = self.ruby_gpu

            self._dma_controllers.append(ctrl)

        # Create GPU memories. Currently fixed to HBM2.
        mem_type_cls = HBM_2000_4H_1x64

        # AMDGPUDevice currently tells the driver there is 16GiB for memory.
        # Until that is a parameter, this need to be fixed to 16GiB.
        gpu_mem_range = AddrRange(0, size="16GiB")
        intlv_low_bit = int(math.log(self._cache_line_size, 2))
        intlv_bits = int(math.log(num_memory_channels, 2))

        for idx in range(num_memory_channels):
            addr_range = AddrRange(
                gpu_mem_range.start,
                size=gpu_mem_range.size(),
                intlvHighBit=intlv_low_bit + intlv_bits - 1,
                intlvBits=intlv_bits,
                intlvMatch=idx,
                xorHighBit=0,
            )

            mem_ctrl = MemCtrl(dram=mem_type_cls(range=addr_range))
            self._mem_ctrls.append(mem_ctrl)

            dir = ViperGPUDirectory(
                self.ruby_gpu.network,
                self._cache_line_size,
                addr_range,
                self._mem_ctrls[idx].port,
            )

            dir.ruby_system = self.ruby_gpu
            dir.TCC_select_num_bits = tcc_bits
            dir.version = len(self._directory_controllers)
            self._directory_controllers.append(dir)

            dir.L3CacheMemory = RubyCache(
                size="16MiB",
                assoc=16,
                atomicALUs=64,
                replacement_policy=TreePLRURP(),
                resourceStalls=False,
                dataArrayBanks=16,
                tagArrayBanks=16,
                dataAccessLatency=20,
                tagAccessLatency=15,
            )

        # Number of sequencers = one per TCP, SQC, and Scalar + one per DMA.
        self.ruby_gpu.num_of_sequencers = len(self._controllers) + len(
            self._dma_controllers
        )

        # Assign the controllers to their parent objects.
        self.ruby_gpu.controllers = self._controllers
        self.ruby_gpu.directory_controllers = self._directory_controllers

        # Connect the controllers using the network topology
        self.ruby_gpu.network.connect(
            self._controllers
            + self._directory_controllers
            + self._dma_controllers
        )
        self.ruby_gpu.network.setup_buffers()

    def get_mem_ctrls(self):
        return self._mem_ctrls
