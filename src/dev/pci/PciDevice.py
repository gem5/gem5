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
from m5.objects.PciHost import PciHost
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

    host = Param.PciHost(Parent.any, "PCI host")
    pci_bus = Param.Int("PCI bus")
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
    HeaderType = Param.UInt8(0, "PCI Header Type")
    BIST = Param.UInt8(0, "Built In Self Test")

    CapabilityPtr = Param.UInt8(0x00, "Capability List Pointer offset")
    InterruptLine = Param.UInt8(0x00, "Interrupt Line")
    InterruptPin = Param.UInt8(0x00, "Interrupt Pin")

    # Capabilities List structures for PCIe devices
    # PMCAP - PCI Power Management Capability
    PMCAPBaseOffset = Param.UInt8(
        0x00, "Base offset of PMCAP in PCI Config space"
    )
    PMCAPNextCapability = Param.UInt8(0x00, "Pointer to next capability block")
    PMCAPCapId = Param.UInt8(
        0x00, "Specifies this is the Power Management capability"
    )
    PMCAPCapabilities = Param.UInt16(
        0x0000, "PCI Power Management Capabilities Register"
    )
    PMCAPCtrlStatus = Param.UInt16(
        0x0000, "PCI Power Management Control and Status"
    )

    # MSICAP - Message Signaled Interrupt Capability
    MSICAPBaseOffset = Param.UInt8(
        0x00, "Base offset of MSICAP in PCI Config space"
    )
    MSICAPNextCapability = Param.UInt8(
        0x00, "Pointer to next capability block"
    )
    MSICAPCapId = Param.UInt8(0x00, "Specifies this is the MSI Capability")
    MSICAPMsgCtrl = Param.UInt16(0x0000, "MSI Message Control")
    MSICAPMsgAddr = Param.UInt32(0x00000000, "MSI Message Address")
    MSICAPMsgUpperAddr = Param.UInt32(0x00000000, "MSI Message Upper Address")
    MSICAPMsgData = Param.UInt16(0x0000, "MSI Message Data")
    MSICAPMaskBits = Param.UInt32(0x00000000, "MSI Interrupt Mask Bits")
    MSICAPPendingBits = Param.UInt32(0x00000000, "MSI Pending Bits")

    # MSIXCAP - MSI-X Capability
    MSIXCAPBaseOffset = Param.UInt8(
        0x00, "Base offset of MSIXCAP in PCI Config space"
    )
    MSIXCAPNextCapability = Param.UInt8(
        0x00, "Pointer to next capability block"
    )
    MSIXCAPCapId = Param.UInt8(0x00, "Specifices this the MSI-X Capability")
    MSIXMsgCtrl = Param.UInt16(0x0000, "MSI-X Message Control")
    MSIXTableOffset = Param.UInt32(
        0x00000000, "MSI-X Table Offset and Table BIR"
    )
    MSIXPbaOffset = Param.UInt32(0x00000000, "MSI-X PBA Offset and PBA BIR")

    # PXCAP - PCI Express Capability
    PXCAPBaseOffset = Param.UInt8(
        0x00, "Base offset of PXCAP in PCI Config space"
    )
    PXCAPNextCapability = Param.UInt8(0x00, "Pointer to next capability block")
    PXCAPCapId = Param.UInt8(0x00, "Specifies this is the PCIe Capability")
    PXCAPCapabilities = Param.UInt16(0x0000, "PCIe Capabilities")
    PXCAPDevCapabilities = Param.UInt32(0x00000000, "PCIe Device Capabilities")
    PXCAPDevCtrl = Param.UInt16(0x0000, "PCIe Device Control")
    PXCAPDevStatus = Param.UInt16(0x0000, "PCIe Device Status")
    PXCAPLinkCap = Param.UInt32(0x00000000, "PCIe Link Capabilities")
    PXCAPLinkCtrl = Param.UInt16(0x0000, "PCIe Link Control")
    PXCAPLinkStatus = Param.UInt16(0x0000, "PCIe Link Status")
    PXCAPSlotCap = Param.UInt32(0x00000000, "PCIe Slot Capabilities")
    PXCAPSlotCtrl = Param.UInt16(0x0000, "PCIe Slot Control")
    PXCAPSlotStatus = Param.UInt16(0x0000, "PCIe Slot Status")
    PXCAPRootCap = Param.UInt16(0x0000, "PCIe Root Capabilities")
    PXCAPRootCtrl = Param.UInt16(0x0000, "PCIe Root Control")
    PXCAPRootStatus = Param.UInt32(0x00000000, "PCIe Root Status")
    PXCAPDevCap2 = Param.UInt32(0x00000000, "PCIe Device Capabilities 2")
    PXCAPDevCtrl2 = Param.UInt16(0x0000, "PCIe Device Control 2")
    PXCAPDevStatus2 = Param.UInt16(0x0000, "PCIe Device Status 2")
    PXCAPLinkCap2 = Param.UInt32(0x00000000, "PCIe Link Capabilities 2")
    PXCAPLinkCtrl2 = Param.UInt16(0x0000, "PCIe Link Control 2")
    PXCAPLinkStatus2 = Param.UInt16(0x0000, "PCIe Link Status 2")
    PXCAPSlotCap2 = Param.UInt32(0x00000000, "PCIe Slot Capabilities 2")
    PXCAPSlotCtrl2 = Param.UInt16(0x0000, "PCIe Slot Control 2")
    PXCAPSlotStatus2 = Param.UInt16(0x0000, "PCIe Slot Status 2")


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
    ExpansionROM = Param.UInt32(0x00, "Expansion ROM Base Address")
    MaximumLatency = Param.UInt8(0x00, "Maximum Latency")
    MinimumGrant = Param.UInt8(0x00, "Minimum Grant")


class PciBridge(PciDevice):
    type = "PciBridge"
    cxx_class = "gem5::PciBridge"
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
    ExpansionROM = Param.UInt32(0, "Expansion ROM Base Address")
    BridgeControl = Param.UInt16(0, "Bridge Control")
