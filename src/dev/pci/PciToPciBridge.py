# Copyright (c) 2026 REDS institute of the HEIG-VD
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
from m5.objects.PciDevice import (
    PciDevice,
    PciType1Device,
)
from m5.objects.PciUpstream import (
    PciBus,
    PciConfigError,
    PciUpDownBridge,
)
from m5.params import *
from m5.proxy import *


class PciToPciBridge(PciType1Device):
    type = "PciToPciBridge"
    cxx_class = "gem5::PciToPciBridge"
    cxx_header = "dev/pci/p2p_bridge.hh"

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
        """Helper function to connect the PCI to PCI bridge internal ports"""
        self.up_to_down.mem_side_port = self.internal_bus.cpu_side_ports
        self.down_to_up.cpu_side_port = self.internal_bus.default
        self.config_error.pio = self.internal_bus.config_error_port

    def connect_upper_bridge(self, bridge):
        """
        Connect the PCI to PCI bridge to given upstream bridge.

        bridge should be either a PCI host bridge (PciHost)
        or another PCI to PCI bridge, with connect_device() function
        and internal_bus member of type PciBus.
        """
        bridge.connect_device(self)

        self.up_to_down.cpu_side_port = bridge.internal_bus.mem_side_ports
        self.down_to_up.mem_side_port = bridge.internal_bus.cpu_side_ports

    def connect_device(self, device: PciDevice):
        """Connect given PCI device to the PCI to PCI bridge"""
        if device in self.devices:
            raise ValueError(
                f"Device {device} is already connected to this PCI-to-PCI bridge"
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
