# Copyright (c) 2016-2017, 2019 ARM Limited
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
#
# Authors: Andreas Sandberg
#          Gabor Dozsa

# System components used by the bigLITTLE.py configuration script

from __future__ import print_function
from __future__ import absolute_import

import m5
from m5.objects import *
m5.util.addToPath('../../')
from common.Caches import *
from common import CpuConfig

have_kvm = "ArmV8KvmCPU" in CpuConfig.cpu_names()

class L1I(L1_ICache):
    tag_latency = 1
    data_latency = 1
    response_latency = 1
    mshrs = 4
    tgts_per_mshr = 8
    size = '48kB'
    assoc = 3


class L1D(L1_DCache):
    tag_latency = 2
    data_latency = 2
    response_latency = 1
    mshrs = 16
    tgts_per_mshr = 16
    size = '32kB'
    assoc = 2
    write_buffers = 16


class WalkCache(PageTableWalkerCache):
    tag_latency = 4
    data_latency = 4
    response_latency = 4
    mshrs = 6
    tgts_per_mshr = 8
    size = '1kB'
    assoc = 8
    write_buffers = 16


class L2(L2Cache):
    tag_latency = 12
    data_latency = 12
    response_latency = 5
    mshrs = 32
    tgts_per_mshr = 8
    size = '1MB'
    assoc = 16
    write_buffers = 8
    clusivity='mostly_excl'


class L3(Cache):
    size = '16MB'
    assoc = 16
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    clusivity='mostly_excl'


class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio


class CpuCluster(SubSystem):
    def __init__(self, system,  num_cpus, cpu_clock, cpu_voltage,
                 cpu_type, l1i_type, l1d_type, wcache_type, l2_type):
        super(CpuCluster, self).__init__()
        self._cpu_type = cpu_type
        self._l1i_type = l1i_type
        self._l1d_type = l1d_type
        self._wcache_type = wcache_type
        self._l2_type = l2_type

        assert num_cpus > 0

        self.voltage_domain = VoltageDomain(voltage=cpu_voltage)
        self.clk_domain = SrcClockDomain(clock=cpu_clock,
                                         voltage_domain=self.voltage_domain)

        self.cpus = [ self._cpu_type(cpu_id=system.numCpus() + idx,
                                     clk_domain=self.clk_domain)
                      for idx in range(num_cpus) ]

        for cpu in self.cpus:
            cpu.createThreads()
            cpu.createInterruptController()
            cpu.socket_id = system.numCpuClusters()
        system.addCpuCluster(self, num_cpus)

    def requireCaches(self):
        return self._cpu_type.require_caches()

    def memoryMode(self):
        return self._cpu_type.memory_mode()

    def addL1(self):
        for cpu in self.cpus:
            l1i = None if self._l1i_type is None else self._l1i_type()
            l1d = None if self._l1d_type is None else self._l1d_type()
            iwc = None if self._wcache_type is None else self._wcache_type()
            dwc = None if self._wcache_type is None else self._wcache_type()
            cpu.addPrivateSplitL1Caches(l1i, l1d, iwc, dwc)

    def addL2(self, clk_domain):
        if self._l2_type is None:
            return
        self.toL2Bus = L2XBar(width=64, clk_domain=clk_domain)
        self.l2 = self._l2_type()
        for cpu in self.cpus:
            cpu.connectAllPorts(self.toL2Bus)
        self.toL2Bus.master = self.l2.cpu_side

    def connectMemSide(self, bus):
        bus.slave
        try:
            self.l2.mem_side = bus.slave
        except AttributeError:
            for cpu in self.cpus:
                cpu.connectAllPorts(bus)


class AtomicCluster(CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock, cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("AtomicSimpleCPU"), None, None, None, None ]
        super(AtomicCluster, self).__init__(system, num_cpus, cpu_clock,
                                            cpu_voltage, *cpu_config)
    def addL1(self):
        pass

class KvmCluster(CpuCluster):
    def __init__(self, system, num_cpus, cpu_clock, cpu_voltage="1.0V"):
        cpu_config = [ CpuConfig.get("ArmV8KvmCPU"), None, None, None, None ]
        super(KvmCluster, self).__init__(system, num_cpus, cpu_clock,
                                         cpu_voltage, *cpu_config)
    def addL1(self):
        pass


class SimpleSystem(LinuxArmSystem):
    cache_line_size = 64

    def __init__(self, caches, mem_size, platform=None, **kwargs):
        super(SimpleSystem, self).__init__(**kwargs)

        self.voltage_domain = VoltageDomain(voltage="1.0V")
        self.clk_domain = SrcClockDomain(clock="1GHz",
                                         voltage_domain=Parent.voltage_domain)

        if platform is None:
            self.realview = VExpress_GEM5_V1()
        else:
            self.realview = platform

        if hasattr(self.realview.gic, 'cpu_addr'):
            self.gic_cpu_addr = self.realview.gic.cpu_addr
        self.flags_addr = self.realview.realview_io.pio_addr + 0x30

        self.membus = MemBus()

        self.intrctrl = IntrControl()
        self.terminal = Terminal()
        self.vncserver = VncServer()

        self.iobus = IOXBar()
        # CPUs->PIO
        self.iobridge = Bridge(delay='50ns')
        # Device DMA -> MEM
        mem_range = self.realview._mem_regions[0]
        assert long(mem_range.size()) >= long(Addr(mem_size))
        self.mem_ranges = [ AddrRange(start=mem_range.start, size=mem_size) ]
        self._caches = caches
        if self._caches:
            self.iocache = IOCache(addr_ranges=[self.mem_ranges[0]])
        else:
            self.dmabridge = Bridge(delay='50ns',
                                    ranges=[self.mem_ranges[0]])

        self._pci_devices = 0
        self._clusters = []
        self._num_cpus = 0

    def attach_pci(self, dev):
        dev.pci_bus, dev.pci_dev, dev.pci_func = (0, self._pci_devices + 1, 0)
        self._pci_devices += 1
        self.realview.attachPciDevice(dev, self.iobus)

    def connect(self):
        self.iobridge.master = self.iobus.slave
        self.iobridge.slave = self.membus.master

        if self._caches:
            self.iocache.mem_side = self.membus.slave
            self.iocache.cpu_side = self.iobus.master
        else:
            self.dmabridge.master = self.membus.slave
            self.dmabridge.slave = self.iobus.master

        if hasattr(self.realview.gic, 'cpu_addr'):
            self.gic_cpu_addr = self.realview.gic.cpu_addr
        self.realview.attachOnChipIO(self.membus, self.iobridge)
        self.realview.attachIO(self.iobus)
        self.system_port = self.membus.slave

    def numCpuClusters(self):
        return len(self._clusters)

    def addCpuCluster(self, cpu_cluster, num_cpus):
        assert cpu_cluster not in self._clusters
        assert num_cpus > 0
        self._clusters.append(cpu_cluster)
        self._num_cpus += num_cpus

    def numCpus(self):
        return self._num_cpus

    def addCaches(self, need_caches, last_cache_level):
        if not need_caches:
            # connect each cluster to the memory hierarchy
            for cluster in self._clusters:
                cluster.connectMemSide(self.membus)
            return

        cluster_mem_bus = self.membus
        assert last_cache_level >= 1 and last_cache_level <= 3
        for cluster in self._clusters:
            cluster.addL1()
        if last_cache_level > 1:
            for cluster in self._clusters:
                cluster.addL2(cluster.clk_domain)
        if last_cache_level > 2:
            max_clock_cluster = max(self._clusters,
                                    key=lambda c: c.clk_domain.clock[0])
            self.l3 = L3(clk_domain=max_clock_cluster.clk_domain)
            self.toL3Bus = L2XBar(width=64)
            self.toL3Bus.master = self.l3.cpu_side
            self.l3.mem_side = self.membus.slave
            cluster_mem_bus = self.toL3Bus

        # connect each cluster to the memory hierarchy
        for cluster in self._clusters:
            cluster.connectMemSide(cluster_mem_bus)
