# Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
# All rights reserved.
#
# This file contains modifications and/or code derived from:
# gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

from m5.objects.Bridge import Bridge
from m5.objects.Cache import Cache
from m5.objects.CommInterface import CommInterface
from m5.objects.Device import (
    BadAddr,
    BasicPioDevice,
    DmaDevice,
    IsaFake,
    PioDevice,
)
from m5.objects.NoncoherentDma import NoncoherentDma
from m5.objects.Platform import Platform
from m5.objects.SimpleMemory import SimpleMemory
from m5.objects.StreamDma import StreamDma
from m5.objects.SubSystem import SubSystem
from m5.objects.XBar import *
from m5.params import *
from m5.proxy import *


class ClusterCache(Cache):
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    write_buffers = 8


class AccCluster(Platform):
    type = "AccCluster"
    cxx_header = "salam/acc_cluster.hh"
    system = Param.System(Parent.any, "system")

    # System Cache Parameter
    cache_size = Param.String("32kB", "cache size in bytes")
    local_range_min = Param.Unsigned(
        0x2F000000, "minimal address of local range"
    )
    local_range_max = Param.Unsigned(
        0x7FFFFFFF, "maximum address of local range"
    )
    external_range_low_min = Param.Unsigned(
        0x00000000, "minimal address of external range low"
    )
    external_range_low_max = Param.Unsigned(
        0x2EFFFFFF, "maximum address of external range low"
    )
    external_range_hi_min = Param.Unsigned(
        0x80000000, "minimal address of external range high"
    )
    external_range_hi_max = Param.Unsigned(
        0xFFFFFFFF, "maximum address of external range high"
    )

    local_bus = NoncoherentXBar(
        width=2, frontend_latency=1, forward_latency=0, response_latency=1
    )
    coherency_bus = CoherentXBar(
        width=2, frontend_latency=1, forward_latency=0, response_latency=1
    )
    coherency_bus.snoop_filter = SnoopFilter()
    coherency_bus.snoop_response_latency = 4
    coherency_bus.point_of_coherency = True
    coherency_bus.point_of_unification = True

    def _add_spm(self, spm_range, spm_latency):
        self.spm = SimpleMemory(
            range=spm_range, conf_table_reported=False, latency=spm_latency
        )
        self.spm.port = self.local_bus.mem_side_ports

    def _connect_spm(self, spm):
        spm.port = self.local_bus.mem_side_ports

    def _attach_bridges(self, system, mem_range, ext_ranges):
        self.mem2cls = Bridge(delay="1ns", ranges=mem_range)
        self.mem2cls.mem_side_ports = self.local_bus.cpu_side_ports
        self.mem2cls.cpu_side_ports = system.membus.mem_side_ports

        # self.cls2mem = Bridge(delay='1ns', ranges = ext_ranges)
        # self.cls2mem.mem_side_ports = system.membus.cpu_side_ports
        # self.cls2mem.cpu_side_ports = self.local_bus.mem_side_ports

    def _connect_hwacc(self, hwacc):
        hwacc.pio = self.local_bus.mem_side_ports

    def _connect_caches(self, system, options, l2coherent, cache_size=0):
        if options.acc_cache and (cache_size != 0):
            self.cluster_cache = ClusterCache()
            self.cluster_cache.size = cache_size

            if options.l2cache and l2coherent:
                self.cluster_cache.mem_side = system.tol2bus.cpu_side_ports
            else:
                self.cluster_cache.mem_side = system.membus.cpu_side_ports
            self.coherency_bus.mem_side_ports = self.cluster_cache.cpu_side
        else:
            if options.l2cache and l2coherent:
                self.coherency_bus.mem_side_ports = (
                    system.tol2bus.cpu_side_ports
                )
            else:
                self.coherency_bus.mem_side_ports = (
                    system.membus.cpu_side_ports
                )

    def _connect_dma(self, system, dma):
        dma.pio = self.local_bus.mem_side_ports
        dma.dma = self.coherency_bus.cpu_side_ports

    def _connect_cluster_dma(self, system, dma):
        self._connect_dma(system, dma)
        dma.cluster_dma = self.local_bus.cpu_side_ports
