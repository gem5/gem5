# Copyright (c) 2013 ARM Limited
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

from m5.objects.Device import DmaDevice
from m5.objects.PciCapability import PciCapabilityBase
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class PciBar(SimObject):
    type = "PciBar"
    cxx_class = "gem5::PciBar"
    cxx_header = "dev/pci/device.hh"
    abstract = True


class PciBarNone(PciBar):
    type = "PciBarNone"
    cxx_class = "gem5::PciBarNone"
    cxx_header = "dev/pci/device.hh"


class PciIoBar(PciBar):
    type = "PciIoBar"
    cxx_class = "gem5::PciIoBar"
    cxx_header = "dev/pci/device.hh"

    size = Param.MemorySize32("IO region size")


class PciLegacyIoBar(PciIoBar):
    type = "PciLegacyIoBar"
    cxx_class = "gem5::PciLegacyIoBar"
    cxx_header = "dev/pci/device.hh"

    addr = Param.UInt32("Legacy IO address")


# To set up a 64 bit memory BAR, put a PciMemUpperBar immediately after
# a PciMemBar. The pair will take up the right number of BARs, and will be
# recognized by the device and turned into a 64 bit BAR when the config is
# consumed.
class PciMemBar(PciBar):
    type = "PciMemBar"
    cxx_class = "gem5::PciMemBar"
    cxx_header = "dev/pci/device.hh"

    size = Param.MemorySize("Memory region size")


class PciMemUpperBar(PciBar):
    type = "PciMemUpperBar"
    cxx_class = "gem5::PciMemUpperBar"
    cxx_header = "dev/pci/device.hh"


class PciDevice(DmaDevice):
    type = "PciDevice"
    cxx_class = "gem5::PciDevice"
    cxx_header = "dev/pci/device.hh"
    abstract = True

    pci_dev = Param.Int("PCI device number")
    pci_func = Param.Int("PCI function code")

    pio_latency = Param.Latency("30ns", "Programmed IO latency")
    config_latency = Param.Latency("20ns", "Config read or write latency")

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
    BIST = Param.UInt8(0, "Built In Self Test")

    ExpansionROM = Param.UInt32(0, "Expansion ROM Base Address")

    InterruptLine = Param.UInt8(0x00, "Interrupt Line")
    InterruptPin = Param.UInt8(0x00, "Interrupt Pin")

    capabilities = VectorParam.PciCapabilityBase(
        [], "Capabilities supported by the device"
    )


class PciEndpoint(PciDevice):
    type = "PciEndpoint"
    cxx_class = "gem5::PciEndpoint"
    cxx_header = "dev/pci/device.hh"
    abstract = True

    BAR0 = Param.PciBar(PciBarNone(), "Base address register 0")
    BAR1 = Param.PciBar(PciBarNone(), "Base address register 1")
    BAR2 = Param.PciBar(PciBarNone(), "Base address register 2")
    BAR3 = Param.PciBar(PciBarNone(), "Base address register 3")
    BAR4 = Param.PciBar(PciBarNone(), "Base address register 4")
    BAR5 = Param.PciBar(PciBarNone(), "Base address register 5")

    CardbusCIS = Param.UInt32(0x00, "Cardbus Card Information Structure")
    SubsystemID = Param.UInt16(0x00, "Subsystem ID")
    SubsystemVendorID = Param.UInt16(0x00, "Subsystem Vendor ID")
    MaximumLatency = Param.UInt8(0x00, "Maximum Latency")
    MinimumGrant = Param.UInt8(0x00, "Minimum Grant")


class PciType1Device(PciDevice):
    type = "PciType1Device"
    cxx_class = "gem5::PciType1Device"
    cxx_header = "dev/pci/device.hh"
    abstract = True

    BAR0 = Param.PciBar(PciBarNone(), "Base address register 0")
    BAR1 = Param.PciBar(PciBarNone(), "Base address register 1")

    PrimaryBusNumber = Param.UInt8(0, "Primary bus number")
    SecondaryBusNumber = Param.UInt8(0, "Secondary bus number")
    SubordinateBusNumber = Param.UInt8(0, "Subordinate bus number")
    SecondaryLatencyTimer = Param.UInt8(0, "Secondary Latency Timer")
    IOBase = Param.UInt8(0, "I/O Base")
    IOLimit = Param.UInt8(0, "I/O Limit")
    SecondaryStatus = Param.UInt16(0, "Secondary status")
    MemoryBase = Param.UInt16(0, "Memory base")
    MemoryLimit = Param.UInt16(0, "Memory limit")
    PrefetchableMemoryBase = Param.UInt16(0, "Prefetchable Memory Base")
    PrefetchableMemoryLimit = Param.UInt16(0, "Prefetchable Memory Limit")
    PrefetchableBaseUpper = Param.UInt32(0, "Prefetchable Base Upper")
    PrefetchableLimitUpper = Param.UInt32(0, "Prefetchable Limit Upper")
    IOBaseUpper = Param.UInt16(0, "I/O Base Upper")
    IOLimitUpper = Param.UInt16(0, "I/O Limit Upper")
    BridgeControl = Param.UInt16(0, "Bridge Control")
