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

from m5.objects.Bridge import (
    Bridge,
    BridgeBase,
)
from m5.objects.ClockedObject import ClockedObject
from m5.objects.PciDevice import PciDevice
from m5.objects.PciUpstream import (
    PciBus,
    PciConfigError,
    PciUpDownBridge,
)
from m5.objects.Platform import Platform
from m5.params import *
from m5.proxy import *


class PciHost(ClockedObject):
    type = "PciHost"
    cxx_class = "gem5::PciHost"
    cxx_header = "dev/pci/host.hh"
    abstract = True

    up_to_down = Param.PciUpDownBridge(
        PciUpDownBridge(), "Bridge upstream -> downstream"
    )
    down_to_up = Param.BridgeBase(Bridge(), "Bridge downstream -> upstream")

    internal_bus = Param.PciBus(
        PciBus(),
        "Internal PCI bus XBar used to transmit packets between devices",
    )

    config_error = Param.PciConfigError(
        PciConfigError(), "Device to handle config errors"
    )

    devices = VectorParam.PciDevice(
        [], "List of all PCI device connected to this upstream"
    )

    def internal_connect(self):
        """Helper function to connect the host bridge internal ports"""
        self.up_to_down.mem_side_port = self.internal_bus.cpu_side_ports
        self.down_to_up.cpu_side_port = self.internal_bus.default
        self.config_error.pio = self.internal_bus.config_error_port

    def connect_upper_bus(self, bus, connect_dma):
        """
        Connect the host bridge to given bus. If connect_dma is set to False,
        the DMA port will not be connected to the bus and it will be returned,
        allowing it to be connected on a ruby hierarchy. Otherwise, the port
        is connected to the bus and None is returned.
        """
        self.up_to_down.cpu_side_port = bus.mem_side_ports

        if not connect_dma:
            return self.down_to_up.mem_side_port

        self.down_to_up.mem_side_port = bus.cpu_side_ports
        return None

    def dma_port(self):
        return self.down_to_up.mem_side_port

    def connect_device(self, device: PciDevice):
        """Connect given PCI device to the host bridge"""
        if device in self.devices:
            raise ValueError(
                f"Device {device} is already connected to this PCI host"
            )
        for existing in self.devices:
            if (
                existing.pci_dev == device.pci_dev
                and existing.pci_func == device.pci_func
            ):
                raise ValueError(
                    f"PCI slot (dev={existing.pci_dev}, func={existing.pci_func}) is already "
                    f"occupied by {existing}"
                )

        self.devices.append(device)

        device.pio = self.internal_bus.mem_side_ports
        device.dma = self.internal_bus.cpu_side_ports


class GenericPciHost(PciHost):
    type = "GenericPciHost"
    cxx_class = "gem5::GenericPciHost"
    cxx_header = "dev/pci/host.hh"

    platform = Param.Platform(Parent.any, "Platform to use for interrupts")

    conf_base = Param.Addr("Config space base address")
    conf_size = Param.Addr("Config space base address")
    conf_device_bits = Param.UInt8(
        8, "Number of bits used to as an offset a devices address space"
    )

    pci_pio_base = Param.Addr(0, "Base address for PCI IO accesses")
    pci_mem_base = Param.Addr(0, "Base address for PCI memory accesses")
    pci_dma_base = Param.Addr(0, "Base address for DMA memory accesses")

    def pciFdtAddr(
        self,
        bus=0,
        device=0,
        function=0,
        register=0,
        space=0,
        aliased=0,
        prefetchable=0,
        relocatable=0,
        addr=0,
    ):
        busf = bus & 0xFF
        devicef = device & 0x1F
        functionf = function & 0x7
        registerf = register & 0xFF
        spacef = space & 0x3
        aliasedf = aliased & 0x1
        prefetchablef = prefetchable & 0x1
        relocatablef = relocatable & 0x1

        if (
            busf != bus
            or devicef != device
            or functionf != function
            or registerf != register
            or spacef != space
            or aliasedf != aliased
            or prefetchablef != prefetchable
            or relocatablef != relocatable
        ):
            fatal("One of the fields for the PCI address is out of bounds")

        address = (
            registerf
            | (functionf << 8)
            | (devicef << 11)
            | (busf << 16)
            | (spacef << 24)
            | (aliasedf << 29)
            | (prefetchablef << 30)
            | (relocatablef << 31)
        )

        low_addr = addr & 0xFFFFFFFF
        high_addr = (addr >> 32) & 0xFFFFFFFF

        return [address, high_addr, low_addr]
