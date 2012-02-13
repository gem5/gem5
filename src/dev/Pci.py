# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
#
# Authors: Nathan Binkert

from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, DmaDevice, PioDevice

class PciConfigAll(PioDevice):
    type = 'PciConfigAll'
    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    pio_latency = Param.Tick(1, "Programmed IO latency in simticks")
    bus = Param.UInt8(0x00, "PCI bus to act as config space for")
    size = Param.MemorySize32('16MB', "Size of config space")


class PciDevice(DmaDevice):
    type = 'PciDevice'
    abstract = True
    platform = Param.Platform(Parent.any, "Platform this device is part of.")
    config = SlavePort("PCI configuration space port")
    pci_bus = Param.Int("PCI bus")
    pci_dev = Param.Int("PCI device number")
    pci_func = Param.Int("PCI function code")
    pio_latency = Param.Latency('1ns', "Programmed IO latency in simticks")
    config_latency = Param.Latency('20ns', "Config read or write latency")

    VendorID = Param.UInt16("Vendor ID")
    DeviceID = Param.UInt16("Device ID")
    Command = Param.UInt16(0, "Command")
    Status = Param.UInt16(0, "Status")
    Revision = Param.UInt8(0, "Device")
    ProgIF = Param.UInt8(0, "Programming Interface")
    SubClassCode = Param.UInt8(0, "Sub-Class Code")
    ClassCode = Param.UInt8(0, "Class Code")
    CacheLineSize = Param.UInt8(0, "System Cacheline Size")
    LatencyTimer = Param.UInt8(0, "PCI Latency Timer")
    HeaderType = Param.UInt8(0, "PCI Header Type")
    BIST = Param.UInt8(0, "Built In Self Test")

    BAR0 = Param.UInt32(0x00, "Base Address Register 0")
    BAR1 = Param.UInt32(0x00, "Base Address Register 1")
    BAR2 = Param.UInt32(0x00, "Base Address Register 2")
    BAR3 = Param.UInt32(0x00, "Base Address Register 3")
    BAR4 = Param.UInt32(0x00, "Base Address Register 4")
    BAR5 = Param.UInt32(0x00, "Base Address Register 5")
    BAR0Size = Param.MemorySize32('0B', "Base Address Register 0 Size")
    BAR1Size = Param.MemorySize32('0B', "Base Address Register 1 Size")
    BAR2Size = Param.MemorySize32('0B', "Base Address Register 2 Size")
    BAR3Size = Param.MemorySize32('0B', "Base Address Register 3 Size")
    BAR4Size = Param.MemorySize32('0B', "Base Address Register 4 Size")
    BAR5Size = Param.MemorySize32('0B', "Base Address Register 5 Size")
    BAR0LegacyIO = Param.Bool(False, "Whether BAR0 is hardwired legacy IO")
    BAR1LegacyIO = Param.Bool(False, "Whether BAR1 is hardwired legacy IO")
    BAR2LegacyIO = Param.Bool(False, "Whether BAR2 is hardwired legacy IO")
    BAR3LegacyIO = Param.Bool(False, "Whether BAR3 is hardwired legacy IO")
    BAR4LegacyIO = Param.Bool(False, "Whether BAR4 is hardwired legacy IO")
    BAR5LegacyIO = Param.Bool(False, "Whether BAR5 is hardwired legacy IO")

    CardbusCIS = Param.UInt32(0x00, "Cardbus Card Information Structure")
    SubsystemID = Param.UInt16(0x00, "Subsystem ID")
    SubsystemVendorID = Param.UInt16(0x00, "Subsystem Vendor ID")
    ExpansionROM = Param.UInt32(0x00, "Expansion ROM Base Address")
    InterruptLine = Param.UInt8(0x00, "Interrupt Line")
    InterruptPin = Param.UInt8(0x00, "Interrupt Pin")
    MaximumLatency = Param.UInt8(0x00, "Maximum Latency")
    MinimumGrant = Param.UInt8(0x00, "Minimum Grant")


