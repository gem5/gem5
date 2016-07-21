# Copyright (c) 2016 ARM Limited
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

import m5
from m5.objects import *
m5.util.addToPath('../../common')
from Caches import *

class L1I(L1_ICache):
    hit_latency = 1
    response_latency = 1
    mshrs = 4
    tgts_per_mshr = 8
    size = '48kB'
    assoc = 3


class L1D(L1_DCache):
    hit_latency = 2
    response_latency = 1
    mshrs = 16
    tgts_per_mshr = 16
    size = '32kB'
    assoc = 2
    write_buffers = 16


class WalkCache(PageTableWalkerCache):
    hit_latency = 4
    response_latency = 4
    mshrs = 6
    tgts_per_mshr = 8
    size = '1kB'
    assoc = 8
    write_buffers = 16


class L2(L2Cache):
    hit_latency = 12
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
    hit_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12
    clusivity='mostly_excl'


class MemBus(SystemXBar):
    badaddr_responder = BadAddr(warn_access="warn")
    default = Self.badaddr_responder.pio


class SimpleSystem(LinuxArmSystem):
    cache_line_size = 64

    voltage_domain = VoltageDomain(voltage="1.0V")
    clk_domain = SrcClockDomain(clock="1GHz",
                                voltage_domain=Parent.voltage_domain)

    realview = VExpress_GEM5_V1()

    gic_cpu_addr = realview.gic.cpu_addr
    flags_addr = realview.realview_io.pio_addr + 0x30

    membus = MemBus()

    intrctrl = IntrControl()
    terminal = Terminal()
    vncserver = VncServer()

    iobus = IOXBar()
    # CPUs->PIO
    iobridge = Bridge(delay='50ns')
    # Device DMA -> MEM
    dmabridge = Bridge(delay='50ns', ranges=realview._mem_regions)

    _pci_devices = 0
    _clusters = []
    _cpus = []

    def attach_pci(self, dev):
        dev.pci_bus, dev.pci_dev, dev.pci_func = (0, self._pci_devices + 1, 0)
        self._pci_devices += 1
        self.realview.attachPciDevice(dev, self.iobus)

    def connect(self):
        self.iobridge.master = self.iobus.slave
        self.iobridge.slave = self.membus.master

        self.dmabridge.master = self.membus.slave
        self.dmabridge.slave = self.iobus.master

        self.gic_cpu_addr = self.realview.gic.cpu_addr
        self.realview.attachOnChipIO(self.membus, self.iobridge)
        self.realview.attachIO(self.iobus)
        self.system_port = self.membus.slave
