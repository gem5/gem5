# Copyright (c) 2013, 2018-2020 ARM Limited
# All rights reserved
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
from m5.proxy import *
from m5.util.fdthelper import *
from m5.SimObject import *
from m5.objects.ClockedObject import ClockedObject


class SMMUv3DeviceInterface(ClockedObject):
    type = "SMMUv3DeviceInterface"
    cxx_header = "dev/arm/smmu_v3_deviceifc.hh"
    cxx_class = "gem5::SMMUv3DeviceInterface"

    device_port = ResponsePort("Device port")
    slave = DeprecatedParam(device_port, "`slave` is now called `device_port`")
    ats_mem_side_port = RequestPort(
        "ATS mem side port,sends requests and receives responses"
    )
    ats_master = DeprecatedParam(
        ats_mem_side_port, "`ats_master` is now called `ats_mem_side_port`"
    )
    ats_dev_side_port = ResponsePort(
        "ATS dev_side_port,sends responses and receives requests"
    )
    ats_slave = DeprecatedParam(
        ats_dev_side_port, "`ats_slave` is now called `ats_dev_side_port`"
    )

    port_width = Param.Unsigned(16, "Port width in bytes (= 1 beat)")
    wrbuf_slots = Param.Unsigned(16, "Write buffer size (in beats)")
    xlate_slots = Param.Unsigned(16, "Translation slots")

    utlb_entries = Param.Unsigned(32, "Micro TLB size (entries)")
    utlb_assoc = Param.Unsigned(0, "Micro TLB associativity (0=full)")
    utlb_policy = Param.String("rr", "Micro TLB replacement policy")
    utlb_enable = Param.Bool(True, "Micro TLB enable")
    utlb_lat = Param.Cycles(1, "Micro TLB lookup latency")
    utlb_slots = Param.Cycles(1, "Micro TLB lookup slots")

    tlb_entries = Param.Unsigned(2048, "Main TLB size (entries)")
    tlb_assoc = Param.Unsigned(4, "Main TLB associativity (0=full)")
    tlb_policy = Param.String("rr", "Main TLB replacement policy")
    tlb_enable = Param.Bool(True, "Main TLB enable")
    tlb_lat = Param.Cycles(3, "Main TLB lookup latency")
    tlb_slots = Param.Cycles(3, "Main TLB lookup slots")

    prefetch_enable = Param.Bool(False, "Enable prefetch")
    prefetch_reserve_last_way = Param.Bool(
        True, "Reserve last way of the main TLB for prefetched entries"
    )


class SMMUv3(ClockedObject):
    type = "SMMUv3"
    cxx_header = "dev/arm/smmu_v3.hh"
    cxx_class = "gem5::SMMUv3"

    request = RequestPort("Request port")
    walker = RequestPort(
        "Request port for SMMU initiated HWTW requests (optional)"
    )
    control = ResponsePort(
        "Control port for accessing memory-mapped registers"
    )
    sample_period = Param.Clock("10us", "Stats sample period")
    reg_map = Param.AddrRange("Address range for control registers")
    system = Param.System(Parent.any, "System this device is part of")

    irq_interface_enable = Param.Bool(
        False,
        "This flag enables software to program SMMU_IRQ_CTRL and "
        "SMMU_IRQ_CTRLACK as if the model implemented architectural "
        "interrupt sources",
    )

    device_interfaces = VectorParam.SMMUv3DeviceInterface(
        [], "Responder interfaces"
    )

    # RESPONDER INTERFACE<->SMMU link parameters
    ifc_smmu_lat = Param.Cycles(8, "IFC to SMMU communication latency")
    smmu_ifc_lat = Param.Cycles(8, "SMMU to IFC communication latency")

    # SMMU parameters
    xlate_slots = Param.Unsigned(64, "SMMU translation slots")
    ptw_slots = Param.Unsigned(16, "SMMU page table walk slots")

    request_port_width = Param.Unsigned(
        16, "Request port width in bytes (= 1 beat)"
    )

    tlb_entries = Param.Unsigned(2048, "TLB size (entries)")
    tlb_assoc = Param.Unsigned(4, "TLB associativity (0=full)")
    tlb_policy = Param.String("rr", "TLB replacement policy")
    tlb_enable = Param.Bool(False, "TLB enable")
    tlb_lat = Param.Cycles(3, "TLB lookup latency")
    tlb_slots = Param.Cycles(3, "TLB lookup slots")

    cfg_entries = Param.Unsigned(64, "Config cache size (entries)")
    cfg_assoc = Param.Unsigned(4, "Config cache associativity (0=full)")
    cfg_policy = Param.String("rr", "Config cache replacement policy")
    cfg_enable = Param.Bool(True, "Config cache enable")
    cfg_lat = Param.Cycles(3, "Config cache lookup latency")
    cfg_slots = Param.Cycles(3, "Config cache lookup slots")

    ipa_entries = Param.Unsigned(128, "IPA cache size (entries)")
    ipa_assoc = Param.Unsigned(4, "IPA cache associativity (0=full)")
    ipa_policy = Param.String("rr", "IPA cache replacement policy")
    ipa_enable = Param.Bool(False, "IPA cache enable")
    ipa_lat = Param.Cycles(3, "IPA cache lookup lantency")
    ipa_slots = Param.Cycles(3, "IPA cache lookup slots")

    walk_S1L0 = Param.Unsigned(4, "Walk cache S1L0 size (entries)")
    walk_S1L1 = Param.Unsigned(28, "Walk cache S1L1 size (entries)")
    walk_S1L2 = Param.Unsigned(348, "Walk cache S1L2 size (entries)")
    walk_S1L3 = Param.Unsigned(4, "Walk cache S1L3 size (entries)")
    walk_S2L0 = Param.Unsigned(4, "Walk cache S2L0 size (entries)")
    walk_S2L1 = Param.Unsigned(28, "Walk cache S2L1 size (entries)")
    walk_S2L2 = Param.Unsigned(92, "Walk cache S2L2 size (entries)")
    walk_S2L3 = Param.Unsigned(4, "Walk cache S2L3 size (entries)")
    walk_assoc = Param.Unsigned(4, "Walk cache associativity (0=full)")
    walk_policy = Param.String("rr", "Walk cache replacement policy")
    walk_enable = Param.Bool(True, "Walk cache enable")
    wc_nonfinal_enable = Param.Bool(
        False, "Nonfinal translations use walk cache"
    )
    wc_s1_levels = Param.Unsigned(
        7, "S1 PT levels cached in walk cache (bit 0 is L0, bit 1 is L1, etc)"
    )
    wc_s2_levels = Param.Unsigned(
        7, "S2 PT levels cached in walk cache (bit 0 is L0, bit 1 is L1, etc)"
    )

    walk_lat = Param.Cycles(4, "Walk cache lookup latency")
    walk_slots = Param.Cycles(4, "Walk cache lookup slots")

    # [28:27] ST_LEVEL = 0b01, 2-level Stream Table supported in addition
    # to Linear Stream table.
    # [25:24] STALL_MODEL = 0b01, Stall is not supported, all faults
    # terminate transaction.
    # [22:21] TTENDIAN = 0b10, Endianness support for translation table walks
    # (0b10 = Little-endian).
    # [19] CD2L = 0b1, 2-level CD table supported.
    # [18] VMID16 = 0b1, 16-bit VMID supported.
    # [12] ASID16 = 0b1, 16-bit ASID supported.
    # [3:2] TTF = 0b10, Translation Table Formats (Stage 1/2)
    # (0b10 = AArch64).
    # [1] S1P = 0b1, Stage 1 translation supported.
    # [0] S2P = 0b1, Stage 2 translation supported.
    smmu_idr0 = Param.UInt32(0x094C100F, "SMMU_IDR0 register")

    # [25:21] CMDQS = 0b00111, Maximum number of Command queue entries
    # as log 2 (entries) (0b00111 = 128 entries).
    smmu_idr1 = Param.UInt32(0x00E00000, "SMMU_IDR1 register")

    smmu_idr2 = Param.UInt32(0, "SMMU_IDR2 register")
    smmu_idr3 = Param.UInt32(0, "SMMU_IDR3 register")
    smmu_idr4 = Param.UInt32(0, "SMMU_IDR4 register")

    # [6] GRAN64K = 0b1, 64KB translation granule supported.
    # [4] GRAN4K = 0b1, 4KB translation granule supported.
    # [2:0] OAS = 0b101, Output Address Size (0b101 = 48-bit).
    smmu_idr5 = Param.UInt32(0x55, "SMMU_IDR5 register")
    smmu_iidr = Param.UInt32(0, "SMMU_IIDR register")

    # [7:0] (0 = SMMUv3.0) (1 = SMMUv3.1)
    smmu_aidr = Param.UInt32(0, "SMMU_AIDR register")

    def generateDeviceTree(self, state):
        reg_addr = self.reg_map.start
        reg_size = self.reg_map.size()
        node = FdtNode(f"smmuv3@{int(reg_addr):x}")
        node.appendCompatible("arm,smmu-v3")
        node.append(
            FdtPropertyWords(
                "reg", state.addrCells(reg_addr) + state.sizeCells(reg_size)
            )
        )
        node.append(FdtPropertyWords("#iommu-cells", [1]))

        node.appendPhandle(self)
        yield node

    def connect(self, device):
        """
        Helper method used to connect the SMMU. The requestor could
        be either a dma port (if the SMMU is attached directly to a
        dma device), or to a request port (this is the case where the SMMU
        is attached to a bridge).
        """

        device_interface = SMMUv3DeviceInterface()

        if hasattr(device, "request_port"):
            device_interface.device_port = device.request_port
        elif hasattr(device, "dma"):
            device_interface.device_port = device.dma
        else:
            print("Unable to attach SMMUv3\n")
            sys.exit(1)

        self.device_interfaces.append(device_interface)

        # Storing a reference to the smmu to be used when generating
        # the binding in the device DTB.
        device._iommu = self
