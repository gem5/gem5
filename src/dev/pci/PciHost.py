# Copyright (c) 2015-2016 ARM Limited
#  All rights reserved
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
#          Glenn Bergmans

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5.objects.Device import PioDevice
from m5.objects.Platform import Platform

class PciHost(PioDevice):
    type = 'PciHost'
    cxx_class = 'PciHost'
    cxx_header = "dev/pci/host.hh"
    abstract = True

class GenericPciHost(PciHost):
    type = 'GenericPciHost'
    cxx_class = 'GenericPciHost'
    cxx_header = "dev/pci/host.hh"

    platform = Param.Platform(Parent.any, "Platform to use for interrupts")

    conf_base = Param.Addr("Config space base address")
    conf_size = Param.Addr("Config space base address")
    conf_device_bits = Param.UInt8(8, "Number of bits used to as an "
                                      "offset a devices address space")

    pci_pio_base = Param.Addr(0, "Base address for PCI IO accesses")
    pci_mem_base = Param.Addr(0, "Base address for PCI memory accesses")
    pci_dma_base = Param.Addr(0, "Base address for DMA memory accesses")

    def pciFdtAddr(self, bus=0, device=0, function=0, register=0, space=0,
                   aliased=0, prefetchable=0, relocatable=0, addr=0):

        busf = bus & 0xff
        devicef = device & 0x1f
        functionf = function & 0x7
        registerf = register & 0xff
        spacef = space & 0x3
        aliasedf = aliased & 0x1
        prefetchablef = prefetchable & 0x1
        relocatablef = relocatable & 0x1

        if  busf != bus or \
            devicef != device or \
            functionf != function or \
            registerf != register or \
            spacef != space or \
            aliasedf != aliased or \
            prefetchablef != prefetchable or \
            relocatablef != relocatable:
            fatal("One of the fields for the PCI address is out of bounds")

        address = registerf | (functionf << 8) | (devicef << 11) | \
                (busf << 16) | (spacef << 24) | (aliasedf << 29) | \
                (prefetchablef << 30) | (relocatablef << 31)

        low_addr = addr & 0xffffffff
        high_addr = (addr >> 32) & 0xffffffff

        return [address, high_addr, low_addr]
