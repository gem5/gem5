# Copyright (c) 2025 REDS institute of the HEIG-VD
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

from m5.params import *
from m5.SimObject import SimObject


class PciCapabilityBase(SimObject):
    type = "PciCapabilityBase"
    cxx_class = "gem5::PciCapabilityBase"
    cxx_header = "dev/pci/capabilities.hh"
    abstract = True

    BaseOffset = Param.UInt8(0x00, "Base offset of the capability")
    CapId = Param.UInt8(0x00, "Capability identifier")


class PciPowerManagementCap(PciCapabilityBase):
    type = "PciPowerManagementCap"
    cxx_class = "gem5::PciPowerManagementCap"
    cxx_header = "dev/pci/capabilities.hh"

    CapId = 0x01

    Capabilities = Param.UInt16(
        0x0000, "PCI Power Management Capabilities Register"
    )
    CtrlStatus = Param.UInt16(
        0x0000, "PCI Power Management Control and Status"
    )


class PciMsiCap(PciCapabilityBase):
    type = "PciMsiCap"
    cxx_class = "gem5::PciMsiCap"
    cxx_header = "dev/pci/capabilities.hh"

    CapId = 0x05

    MsgCtrl = Param.UInt16(0x0000, "MSI Message Control")
    MsgAddr = Param.UInt32(0x00000000, "MSI Message Address")
    MsgUpperAddr = Param.UInt32(0x00000000, "MSI Message Upper Address")
    MsgData = Param.UInt16(0x0000, "MSI Message Data")
    ExtendedMsgData = Param.UInt16(0x0000, "MSI Extended Message Data")
    MaskBits = Param.UInt32(0x00000000, "MSI Interrupt Mask Bits")
    PendingBits = Param.UInt32(0x00000000, "MSI Pending Bits")


class PciMsiXCap(PciCapabilityBase):
    type = "PciMsiXCap"
    cxx_class = "gem5::PciMsiXCap"
    cxx_header = "dev/pci/capabilities.hh"

    CapId = 0x11

    MsgCtrl = Param.UInt16(0x0000, "MSI-X Message Control")
    TableOffset = Param.UInt32(0x00000000, "MSI-X Table Offset and Table BIR")
    PbaOffset = Param.UInt32(0x00000000, "MSI-X PBA Offset and PBA BIR")


class PciExpressCap(PciCapabilityBase):
    type = "PciExpressCap"
    cxx_class = "gem5::PciExpressCap"
    cxx_header = "dev/pci/capabilities.hh"

    CapId = 0x10

    Capabilities = Param.UInt16(0x0000, "PCIe Capabilities")
    DevCapabilities = Param.UInt32(0x00000000, "PCIe Device Capabilities")
    DevCtrl = Param.UInt16(0x0000, "PCIe Device Control")
    DevStatus = Param.UInt16(0x0000, "PCIe Device Status")
    LinkCap = Param.UInt32(0x00000000, "PCIe Link Capabilities")
    LinkCtrl = Param.UInt16(0x0000, "PCIe Link Control")
    LinkStatus = Param.UInt16(0x0000, "PCIe Link Status")
    SlotCap = Param.UInt32(0x00000000, "PCIe Slot Capabilities")
    SlotCtrl = Param.UInt16(0x0000, "PCIe Slot Control")
    SlotStatus = Param.UInt16(0x0000, "PCIe Slot Status")
    RootCap = Param.UInt16(0x0000, "PCIe Root Capabilities")
    RootCtrl = Param.UInt16(0x0000, "PCIe Root Control")
    RootStatus = Param.UInt32(0x00000000, "PCIe Root Status")
    DevCap2 = Param.UInt32(0x00000000, "PCIe Device Capabilities 2")
    DevCtrl2 = Param.UInt16(0x0000, "PCIe Device Control 2")
    DevStatus2 = Param.UInt16(0x0000, "PCIe Device Status 2")
    LinkCap2 = Param.UInt32(0x00000000, "PCIe Link Capabilities 2")
    LinkCtrl2 = Param.UInt16(0x0000, "PCIe Link Control 2")
    LinkStatus2 = Param.UInt16(0x0000, "PCIe Link Status 2")
    SlotCap2 = Param.UInt32(0x00000000, "PCIe Slot Capabilities 2")
    SlotCtrl2 = Param.UInt16(0x0000, "PCIe Slot Control 2")
    SlotStatus2 = Param.UInt16(0x0000, "PCIe Slot Status 2")
