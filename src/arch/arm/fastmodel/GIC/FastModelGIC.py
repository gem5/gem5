# Copyright (c) 2020 ARM Limited
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
# Copyright 2019 Google, Inc.
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
from m5.util.fdthelper import *
from m5.SimObject import SimObject

from m5.objects.FastModel import AmbaInitiatorSocket, AmbaTargetSocket
from m5.objects.Gic import BaseGic
from m5.objects.SystemC import SystemC_ScModule

GICV3_COMMS_TARGET_ROLE = 'GICV3 COMMS TARGET'
GICV3_COMMS_INITIATOR_ROLE = 'GICV3 COMMS INITIATOR'

Port.compat(GICV3_COMMS_TARGET_ROLE, GICV3_COMMS_INITIATOR_ROLE)

class Gicv3CommsTargetSocket(Port):
    def __init__(self, desc):
        super(Gicv3CommsTargetSocket, self).__init__(
                GICV3_COMMS_INITIATOR_ROLE, desc)

class Gicv3CommsInitiatorSocket(Port):
    def __init__(self, desc):
        super(Gicv3CommsInitiatorSocket, self).__init__(
                GICV3_COMMS_TARGET_ROLE, desc, is_source=True)

class VectorGicv3CommsInitiatorSocket(VectorPort):
    def __init__(self, desc):
        super(VectorGicv3CommsInitiatorSocket, self).__init__(
                GICV3_COMMS_TARGET_ROLE, desc, is_source=True)


class SCFastModelGIC(SystemC_ScModule):
    type = 'SCFastModelGIC'
    cxx_class = 'FastModel::SCGIC'
    cxx_header = 'arch/arm/fastmodel/GIC/gic.hh'

    enabled = Param.Bool(True, "Enable GICv3 functionality; when false the "
            "component is inactive. has_gicv3 will replace this when GIC_IRI "
            "replaces GICv3IRI.")
    has_gicv3 = Param.Bool(False, "Enable GICv3 functionality; when false "
            "the component is inactive. This will replace \"enabled\" "
            "parameter.")
    has_gicv4_1 = Param.Bool(False, "Enable GICv4.1 functionality; when "
            "false the component is inactive.")
    vPEID_bits = Param.Unsigned(16, "Number of bits of vPEID with GICv4.1.")
    print_mmap = Param.Bool(False, "Print memory map to stdout")
    monolithic = Param.Bool(False, "Indicate that the implementation is not "
            "distributed")
    direct_lpi_support = Param.Bool(False, "Enable support for LPI "
            "operations through GICR registers")
    cpu_affinities = Param.String("", "A comma separated list of dotted quads "
            "containing the affinities of all PEs connected to this IRI.")
    non_ARE_core_count = Param.Unsigned(8, "Maximum number of non-ARE cores; "
            "normally used to pass the cluster-level NUM_CORES parameter to "
            "the top-level redistributor.")
    reg_base = Param.Addr(0x2c010000, "Base for decoding GICv3 registers.")
    reg_base_per_redistributor = Param.String("", "Base address for each "
            "redistributor in the form "
            "'0.0.0.0=0x2c010000, 0.0.0.1=0x2c020000'.  All redistributors "
            "must be specified and this overrides the reg-base parameter "
            "(except that reg-base will still be used for the top-level "
            "redistributor).")
    gicd_alias = Param.Addr(0x0, "In GICv2 mode: the base address for a 4k "
            "page alias of the first 4k of the Distributor page, in GICv3 "
            "mode. the base address of a 64KB page containing message based "
            "SPI signalling register aliases(0:Disabled)")
    has_two_security_states = Param.Bool(True, "If true, has two security "
            "states")
    DS_fixed_to_zero = Param.Bool(False, "Enable/disable support of single "
            "security state")
    IIDR = Param.UInt32(0x0, "GICD_IIDR and GICR_IIDR value")
    gicv2_only = Param.Bool(False, "If true, when using the GICv3 model, "
            "pretend to be a GICv2 system")
    STATUSR_implemented = Param.Bool(True, "Determines whether the "
            "GICR_STATUSR register is implemented.")
    priority_bits_implemented = Param.Unsigned(5, "Number of implemented "
            "priority bits")
    itargets_razwi = Param.Bool(False, "If true, the GICD_ITARGETS registers "
            "are RAZ/WI")
    icfgr_sgi_mask = Param.UInt32(0x0, "Mask for writes to ICFGR registers "
            "that configure SGIs")
    icfgr_ppi_mask = Param.UInt32(0xaaaaaaaa, "Mask for writes to ICFGR "
            "registers that configure PPIs")
    icfgr_spi_mask = Param.UInt32(0xaaaaaaaa, "Mask for writes to ICFGR "
            "registers that configure SPIs")
    icfgr_sgi_reset = Param.UInt32(0xaaaaaaaa, "Reset value for ICFGR "
            "registers that configure SGIs")
    icfgr_ppi_reset = Param.UInt32(0x0, "Reset value for ICFGR regesters "
            "that configure PPIs")
    icfgr_spi_reset = Param.UInt32(0x0, "Reset value for ICFGR regesters "
            "that configure SPIs")
    icfgr_ppi_rsvd_bit = Param.Bool(False, "If ARE=0, the value of reserved "
            "bits i.e. bit 0,2,4..30 of ICFGRn for n>0")
    igroup_sgi_mask = Param.UInt16(0xffff, "Mask for writes to SGI bits in "
            "IGROUP registers")
    igroup_ppi_mask = Param.UInt16(0xffff, "Mask for writes to PPI bits in "
            "IGROUP registers")
    igroup_sgi_reset = Param.UInt16(0x0, "Reset value for SGI bits in IGROUP "
            "registers")
    igroup_ppi_reset = Param.UInt16(0x0, "Reset value for SGI bits in IGROUP "
            "registers")
    ppi_implemented_mask = Param.UInt16(0xffff, "Mask of PPIs that are "
            "implemented. One bit per PPI bit 0 == PPI 16 (first PPI). This "
            "will affect other masks.")
    spi_count = Param.UInt16(224, "Number of SPIs that are implemented.")
    lockable_spi_count = Param.Unsigned(0, "Number of SPIs that are locked "
            "down when CFGSDISABLE signal is asserted.  Only applies for "
            "GICv2.")
    iri_id_bits = Param.Unsigned(16, "Number of bits used to represent "
            "interrupts IDs in the Distributor and Redistributors, forced to "
            "10 if LPIs are not supported")
    delay_redistributor_accesses = Param.Bool(True, "Delay memory accesses "
            "from the redistributor until GICR_SYNCR is read.")
    gicd_pidr = Param.UInt64(0x0, "The value for the GICD_PIDR registers, if "
            "non-zero. Note: fixed fields (device type etc.) will be "
            "overriden in this value.")
    gicr_pidr = Param.UInt64(0x0, "The value for the GICR_PIDR registers, if "
            "non-zero. Note: fixed fields (device type etc.) will be "
            "overriden in this value.")
    its_count = Param.Unsigned(0, "Number of Interrupt Translation Services "
            "to be instantiated (0=none)")
    its0_base = Param.Addr(0, "Register base address for ITS0 "
            "(automatic if 0).")
    its1_base = Param.Addr(0, "Register base address for ITS1 "
            "(automatic if 0).")
    its2_base = Param.Addr(0, "Register base address for ITS2 "
            "(automatic if 0).")
    its3_base = Param.Addr(0, "Register base address for ITS3 "
            "(automatic if 0).")
    gits_pidr = Param.UInt64(0x0, "The value for the GITS_PIDR registers, if "
            "non-zero. Note: fixed fields (device type etc.) will be "
            "overriden in this value.")
    gits_baser0_type = Param.Unsigned(0, "Type field for GITS_BASER0 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser1_type = Param.Unsigned(0, "Type field for GITS_BASER1 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser2_type = Param.Unsigned(0, "Type field for GITS_BASER2 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser3_type = Param.Unsigned(0, "Type field for GITS_BASER3 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser4_type = Param.Unsigned(0, "Type field for GITS_BASER4 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser5_type = Param.Unsigned(0, "Type field for GITS_BASER5 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser6_type = Param.Unsigned(0, "Type field for GITS_BASER6 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser7_type = Param.Unsigned(0, "Type field for GITS_BASER7 "
            "register. 0 = Unimplemented; 1 = Devices; "
            "2 = Virtual Processors; 3 = Physical Processors; 4 = Collections")
    gits_baser0_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER0 register.")
    gits_baser1_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER1 register.")
    gits_baser2_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER2 register.")
    gits_baser3_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER3 register.")
    gits_baser4_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER4 register.")
    gits_baser5_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER5 register.")
    gits_baser6_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER6 register.")
    gits_baser7_entry_bytes = Param.Unsigned(8, "Number of bytes required per "
            "entry for GITS_BASER7 register.")
    gits_baser0_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER0 register is RAZ/WI.")
    gits_baser1_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER1 register is RAZ/WI.")
    gits_baser2_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER2 register is RAZ/WI.")
    gits_baser3_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER3 register is RAZ/WI.")
    gits_baser4_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER4 register is RAZ/WI.")
    gits_baser5_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER5 register is RAZ/WI.")
    gits_baser6_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER6 register is RAZ/WI.")
    gits_baser7_indirect_raz = Param.Bool(False, "Indirect field for "
            "GITS_BASER7 register is RAZ/WI.")
    its_baser_force_page_alignement = Param.Bool(True, "Force alignement of "
            "address writen to a GITS_BASER register to the page size "
            "configured")
    processor_numbers = Param.String("", "Specify processor numbers (as "
            "appears in GICR_TYPER) in the form 0.0.0.0=0,0.0.0.1=1 etc.) If "
            "not specified, will number processors starting at 0.")
    supports_shareability = Param.Bool(True, "Device supports shareability "
            "attributes on outgoing memory bus (i.e. is modelling an ACElite "
            "port rather than an AXI4 port).")
    a3_affinity_supported = Param.Bool(False, "Device supports affinity "
            "level 3 values that are non-zero.")
    SGI_RSS_support = Param.Bool(False, "Device has support for the Range "
            "Selector feature for SGI")
    gicr_propbaser_read_only = Param.Bool(False, "GICR_PROPBASER register is "
            "read-only.")
    gicr_propbaser_reset = Param.UInt64(0x0, "Value of GICR_PROPBASER on "
            "reset.")
    its_device_bits = Param.Unsigned(16, "Number of bits supported for ITS "
            "device IDs.")
    its_entry_size = Param.Unsigned(8, "Number of bytes required to store "
            "each entry in the ITT tables.")
    its_id_bits = Param.Unsigned(16, "Number of interrupt bits supported by "
            "ITS.")
    its_collection_id_bits = Param.Unsigned(0, "Number of collection bits "
            "supported by ITS (optional parameter, 0 => 16bits support and "
            "GITS_TYPER.CIL=0")
    its_cumulative_collection_tables = Param.Bool(True, "When true, the "
            "supported amount of collections is the sum of GITS_TYPER.HCC and "
            "the number of collections supported in memory, otherwise, simply "
            "the number supported in memory only. Irrelevant when HCC=0")
    delay_ITS_accesses = Param.Bool(True, "Delay accesses from the ITS until "
            "GICR_SYNCR is read.")
    local_SEIs = Param.Bool(False, "Generate SEI to signal internal issues")
    local_VSEIs = Param.Bool(False, "Generate VSEI to signal internal issues")
    ITS_use_physical_target_addresses = Param.Bool(True, "Use physical "
            "hardware adresses for targets in ITS commands -- must be true "
            "for distributed implementations")
    ITS_hardware_collection_count = Param.Unsigned(0, "Number of hardware "
            "collections held exclusively in the ITS")
    ITS_MOVALL_update_collections = Param.Bool(False, "Whether MOVALL command "
            "updates the collection entires")
    ITS_TRANSLATE64R = Param.Bool(False, "Add an implementation specific "
            "register at 0x10008 supporting 64 bit TRANSLATER (dev[63:32], "
            "interupt[31:0])")
    enable_protocol_checking = Param.Bool(False, "Enable/disable protocol "
            "checking at cpu interface")
    fixed_routed_spis = Param.String("", "Value of IROUTER[n] register in the "
            "form 'n=a.b.c.d, n=*'. The RM bit of IROUTER is 0 when n=a.b.c.d "
            "is used else 1 when n=* is used. n can be >= 32 and <= 1019")
    irouter_default_mask = Param.String("", "Default Mask value for "
            "IROUTER[32..1019] register in the form 'a.b.c.d'")
    irouter_default_reset = Param.String("", "Default Reset Value of "
            "IROUTER[32..1019] register in the form 'a.b.c.d' or *")
    irouter_reset_values = Param.String("", "Reset Value of IROUTER[n] "
            "register in the form 'n=a.b.c.d or n=*'.n can be >= 32 and "
            "<= 1019")
    irouter_mask_values = Param.String("", "Mask Value of IROUTER[n] register "
            "in the form 'n=a.b.c.d'.n can be >= 32 and <= 1019")
    ITS_threaded_command_queue = Param.Bool(True, "Enable execution of ITS "
            "commands in a separate thread which is sometimes required for "
            "cosimulation")
    ITS_legacy_iidr_typer_offset = Param.Bool(False, "Put the GITS_IIDR and "
            "GITS_TYPER registers at their older offset of 0x8 and 0x4 "
            "respectively")
    redistributor_threaded_command_queue = Param.Bool(True, "Enable execution "
            "of redistributor delayed transactions in a separate thread which "
            "is sometimes required for cosimulation")
    ignore_generate_sgi_when_no_are = Param.Bool(False, "Ignore GenerateSGI "
            "packets coming form the CPU interface if both ARE_S and ARE_NS "
            "are 0")
    trace_speculative_lpi_property_updates = Param.Bool(False, "Trace LPI "
            "propery updates performed on speculative accesses (useful for "
            "debuging LPI)")
    virtual_lpi_support = Param.Bool(False, "GICv4 Virtual LPIs and Direct "
            "injection of Virtual LPIs supported")
    virtual_priority_bits = Param.Unsigned(5, "Number of implemented virtual "
            "priority bits")
    LPI_cache_type = Param.Unsigned(1, "Cache type for LPIs, 0:No caching, "
            "1:Full caching")
    LPI_cache_check_data = Param.Bool(False, "Enable Cached LPI data against "
             "memory checking when available for cache type")
    DPG_bits_implemented = Param.Bool(False, "Enable implementation of "
            "interrupt group participation bits or DPG bits in GICR_CTLR")
    DPG_ARE_only = Param.Bool(False, "Limit application of DPG bits to "
            "interrupt groups for which ARE=1")
    ARE_fixed_to_one = Param.Bool(False, "GICv2 compatibility is not "
            "supported and GICD_CTLR.ARE_* is always one")
    legacy_sgi_enable_rao = Param.Bool(False, "Enables for SGI associated "
            "with an ARE=0 regime are RAO/WI")
    pa_size = Param.Unsigned(48, "Number of valid bits in physical address")
    MSI_IIDR = Param.UInt32(0x0, "Value returned in MSI_IIDR registers.")
    MSI_NS_frame0_base = Param.Addr(0x0, "If non-zero, sets the base "
            "address used for non-secure MSI frame 0 registers.")
    MSI_NS_frame0_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 0. Set to 0 to disable frame.")
    MSI_NS_frame0_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 0. Set to 0 to disable frame.")
    MSI_NS_frame1_base = Param.Addr(0x0, "If non-zero, sets the base "
            "address used for non-secure MSI frame 1 registers.")
    MSI_NS_frame1_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 1. Set to 0 to disable frame.")
    MSI_NS_frame1_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 1. Set to 0 to disable frame.")
    MSI_NS_frame2_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 2 registers.")
    MSI_NS_frame2_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 2. Set to 0 to disable frame.")
    MSI_NS_frame2_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 2. Set to 0 to disable frame.")
    MSI_NS_frame3_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 3 registers.")
    MSI_NS_frame3_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 3. Set to 0 to disable frame.")
    MSI_NS_frame3_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 3. Set to 0 to disable frame.")
    MSI_NS_frame4_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 4 registers.")
    MSI_NS_frame4_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 4. Set to 0 to disable frame.")
    MSI_NS_frame4_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 4. Set to 0 to disable frame.")
    MSI_NS_frame5_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 5 registers.")
    MSI_NS_frame5_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 5. Set to 0 to disable frame.")
    MSI_NS_frame5_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 5. Set to 0 to disable frame.")
    MSI_NS_frame6_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 6 registers.")
    MSI_NS_frame6_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 6. Set to 0 to disable frame.")
    MSI_NS_frame6_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 6. Set to 0 to disable frame.")
    MSI_NS_frame7_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for non-secure MSI frame 7 registers.")
    MSI_NS_frame7_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "non-secure MSI frame 7. Set to 0 to disable frame.")
    MSI_NS_frame7_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "non-secure MSI frame 7. Set to 0 to disable frame.")
    MSI_PIDR = Param.UInt64(0x0, "The value for the MSI_PIDR registers, if "
            "non-zero and distributor supports GICv2m. Note: fixed fields "
            "(device type etc.) will be overriden in this value.")
    MSI_S_frame0_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 0 registers.")
    MSI_S_frame0_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 0. Set to 0 to disable frame.")
    MSI_S_frame0_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 0. Set to 0 to disable frame.")
    MSI_S_frame1_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 1 registers.")
    MSI_S_frame1_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 1. Set to 0 to disable frame.")
    MSI_S_frame1_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 1. Set to 0 to disable frame.")
    MSI_S_frame2_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 2 registers.")
    MSI_S_frame2_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 2. Set to 0 to disable frame.")
    MSI_S_frame2_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 2. Set to 0 to disable frame.")
    MSI_S_frame3_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 3 registers.")
    MSI_S_frame3_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 3. Set to 0 to disable frame.")
    MSI_S_frame3_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 3. Set to 0 to disable frame.")
    MSI_S_frame4_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 4 registers.")
    MSI_S_frame4_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 4. Set to 0 to disable frame.")
    MSI_S_frame4_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 4. Set to 0 to disable frame.")
    MSI_S_frame5_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 5 registers.")
    MSI_S_frame5_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 5. Set to 0 to disable frame.")
    MSI_S_frame5_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 5. Set to 0 to disable frame.")
    MSI_S_frame6_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 6 registers.")
    MSI_S_frame6_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 6. Set to 0 to disable frame.")
    MSI_S_frame6_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 6. Set to 0 to disable frame.")
    MSI_S_frame7_base = Param.Addr(0x0, "If non-zero, sets the base address "
            "used for secure MSI frame 7 registers.")
    MSI_S_frame7_max_SPI = Param.UInt16(0, "Maximum SPI ID supported by "
            "secure MSI frame 7. Set to 0 to disable frame.")
    MSI_S_frame7_min_SPI = Param.UInt16(0, "Minimum SPI ID supported by "
            "secure MSI frame 7. Set to 0 to disable frame.")
    outer_cacheability_support = Param.Bool(False, "Allow configuration of "
            "outer cachability attributes in ITS and Redistributor")
    wakeup_on_reset = Param.Bool(False, "Go against specification and start "
            "redistributors in woken-up state at reset. This allows software "
            "that was written for previous versions of the GICv3 "
            "specification to work correctly. This should not be used for "
            "production code or when the distributor is used separately from "
            "the core fast model.")
    SPI_MBIS = Param.Bool(True, "Distributor supports meassage based "
            "signaling of SPI")
    SPI_unimplemented = Param.String("", "A comma spearated list of "
            "unimplemented SPIs ranges for sparse SPI defintion(for ex: "
            "'35, 39-42, 73)'")
    irm_razwi = Param.Bool(False, "GICD_IROUTERn.InterruptRoutingMode is "
            "RAZ/WI")
    common_LPI_configuration = Param.Unsigned(0, "Describes which "
            "re-distributors share (and must be configured with the same) "
            "LPI configuration table as described in GICR_TYPER( 0:All, "
            "1:A.x.x.x, 2:A.B.x.x, 3:A.B.C.x")
    single_set_support = Param.Bool(False, "When true, forces redistributors "
            "to recall interrupts with a clear rather than issue a second Set "
            "command")
    has_mpam = Param.Unsigned(0, "Implement ARMv8.4 MPAM Registers and "
            "associated functionality.\n\nPossible values of this parameter "
            "are:\n  - 0, feature is not enabled.\n  - 1, feature is "
            "implemented if ARMv8.4 is enabled.\n  - 2, feature is "
            "implemented.")
    mpam_max_partid = Param.UInt16(0xffff, "MPAM Maximum PARTID Supported")
    mpam_max_pmg = Param.Unsigned(255, "MPAM Maximum PMG Supported")
    output_attributes = Param.String("ExtendedID[62:55]=MPAM_PMG, "
            "ExtendedID[54:39]=MPAM_PARTID, ExtendedID[38]=MPAM_NS",
            "User-defined transform to be applied to bus attributes like "
            "RequestorID, ExtendedID or UserFlags. Currently, only works for "
            "MPAM Attributes encoding into bus attributes.")
    has_DirtyVLPIOnLoad = Param.Bool(False, "GICR_VPENDBASER.Dirty reflects "
            "transient loading state when valid=1")
    allow_LPIEN_clear = Param.Bool(False, "Allow RW behaviour on "
            "GICR_CTLR.LPIEN isntead of set once")
    GICD_legacy_reg_reserved = Param.Bool(False, "When ARE is RAO/WI, makes "
            "superfluous registers in GICD reserved (including for the "
            "purpose of STATUSR updates)")
    extended_spi_count = Param.Unsigned(0, "Number of extended SPI supported")
    extended_ppi_count = Param.Unsigned(0, "Number of extended PPI supported")
    consolidators = Param.String("", "Specify consolidators' base addresses, "
            "interrupt line counts and base interrupt IDs, in the form "
            "'baseAddr0:itlineCount0:baseINTID0, "
            "baseAddr1:itlineCount1:baseINTID1, [etc]' "
            "(eg '0x3f100000:64:4096, 0x3f200000:64:4224'). The "
            "consolidators' count is inferred from the list (maximum of 4). "
            "If not specified, the component contains no consolidators.")

class FastModelGIC(BaseGic):
    type = 'FastModelGIC'
    cxx_class = 'FastModel::GIC'
    cxx_header = 'arch/arm/fastmodel/GIC/gic.hh'

    sc_gic = Param.SCFastModelGIC(SCFastModelGIC(),
                                  'SystemC version of the GIC')

    amba_m = AmbaInitiatorSocket(64, 'Memory initiator socket')
    amba_s = AmbaTargetSocket(64, 'Memory target socket')

    redistributor = VectorGicv3CommsInitiatorSocket(
            'GIC communication initiator')

    # Used for DTB autogeneration
    _state = FdtState(addr_cells=2, size_cells=2, interrupt_cells=3)

    def get_redist_bases(self):
        """
        The format of reg_base_per_redistributor is
        '0.0.0.0=0x2c010000,0.0.0.1=0x2c020000...'
        Return an array of base addresses
        """
        redists = self.sc_gic.reg_base_per_redistributor.split(",")
        # make sure we have at least one redistributor
        assert len(redists) > 0 and "=" in redists[0]
        return [ int(r.split('=')[1], 16) for r in redists ]

    def get_addr_ranges(self):
        """ Return address ranges that should be served by this GIC """
        sc_gic = self.sc_gic
        gic_frame_size = 0x10000
        # Add range of distributor
        ranges = [AddrRange(sc_gic.reg_base, size=gic_frame_size)]
        # Add ranges of redistributors
        redist_frame_size = gic_frame_size * (4 if sc_gic.has_gicv4_1 else 2)
        ranges += [
            AddrRange(redist_base, size=redist_frame_size)
            for redist_base in self.get_redist_bases()
        ]
        # Add ranges of ITSs
        its_bases = [
            sc_gic.its0_base, sc_gic.its1_base, sc_gic.its2_base,
            sc_gic.its3_base
        ]
        ranges += [
            AddrRange(its_bases[i], size=2 * gic_frame_size)
            for i in xrange(sc_gic.its_count)
        ]

        return ranges

    def interruptCells(self, int_type, int_num, int_flag):
        """
        Interupt cells generation helper:
        Following specifications described in

        Documentation/devicetree/bindings/interrupt-controller/arm,gic-v3.txt
        """
        prop = self._state.interruptCells(0)
        assert len(prop) >= 3
        prop[0] = int_type
        prop[1] = int_num
        prop[2] = int_flag
        return prop

    def generateDeviceTree(self, state):
        sc_gic = self.sc_gic

        node = FdtNode("interrupt-controller")
        node.appendCompatible(["arm,gic-v3"])
        node.append(self._state.interruptCellsProperty())
        node.append(self._state.addrCellsProperty())
        node.append(self._state.sizeCellsProperty())
        node.append(FdtProperty("ranges"))
        node.append(FdtProperty("interrupt-controller"))

        redist_stride = 0x40000 if sc_gic.has_gicv4_1 else 0x20000
        node.append(FdtPropertyWords("redistributor-stride",
            state.sizeCells(redist_stride)))

        regs = (
            state.addrCells(sc_gic.reg_base) +
            state.sizeCells(0x10000) +
            state.addrCells(self.get_redist_bases()[0]) +
            state.sizeCells(0x2000000) )

        node.append(FdtPropertyWords("reg", regs))
        # Maintenance interrupt (PPI 25).
        node.append(FdtPropertyWords("interrupts",
            self.interruptCells(1, 9, 0xf04)))

        node.appendPhandle(self)

        # Generate the ITS device tree
        its_frame_size = 0x10000
        its_bases = [
            sc_gic.its0_base, sc_gic.its1_base, sc_gic.its2_base,
            sc_gic.its3_base
        ]
        for its_base in its_bases:
            its_node = self.generateBasicPioDeviceNode(state, "gic-its",
                                                       its_base,
                                                       2 * its_frame_size)
            its_node.appendCompatible(["arm,gic-v3-its"])
            its_node.append(FdtProperty("msi-controller"))
            its_node.append(FdtPropertyWords("#msi-cells", [1]))
            node.append(its_node)

        yield node
