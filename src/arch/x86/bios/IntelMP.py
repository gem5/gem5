# Copyright (c) 2008 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use of this software in source and binary forms,
# with or without modification, are permitted provided that the
# following conditions are met:
#
# The software must be used only for Non-Commercial Use which means any
# use which is NOT directed to receiving any direct monetary
# compensation for, or commercial advantage from such use.  Illustrative
# examples of non-commercial use are academic research, personal study,
# teaching, education and corporate research & development.
# Illustrative examples of commercial use are distributing products for
# commercial advantage and providing services using the software for
# commercial advantage.
#
# If you wish to use this software or functionality therein that may be
# covered by patents for commercial use, please contact:
#     Director of Intellectual Property Licensing
#     Office of Strategy and Technology
#     Hewlett-Packard Company
#     1501 Page Mill Road
#     Palo Alto, California  94304
#
# Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.  Redistributions
# in binary form must reproduce the above copyright notice, this list of
# conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.  Neither the name of
# the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.  No right of
# sublicense is granted herewith.  Derivatives of the software and
# output created using the software may be prepared, but only for
# Non-Commercial Uses.  Derivatives of the software may be shared with
# others provided: (i) the others agree to abide by the list of
# conditions herein which includes the Non-Commercial Use restrictions;
# and (ii) such Derivatives of the software include the above copyright
# notice to acknowledge the contribution from this software where
# applicable, this list of conditions and the disclaimer below.
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
# Authors: Gabe Black

from m5.params import *
from m5.SimObject import SimObject

class X86IntelMPFloatingPointer(SimObject):
    type = 'X86IntelMPFloatingPointer'
    cxx_class = 'X86ISA::IntelMP::FloatingPointer'

    # The minor revision of the spec to support. The major version is assumed
    # to be 1 in accordance with the spec.
    spec_rev = Param.UInt8(4, 'minor revision of the MP spec supported')
    # If no default configuration is used, set this to 0.
    default_config = Param.UInt8(0, 'which default configuration to use')
    imcr_present = Param.Bool(True,
            'whether the IMCR register is present in the APIC')

class X86IntelMPConfigTable(SimObject):
    type = 'X86IntelMPConfigTable'
    cxx_class = 'X86ISA::IntelMP::ConfigTable'

    spec_rev = Param.UInt8(4, 'minor revision of the MP spec supported')
    oem_id = Param.String("", 'system manufacturer')
    product_id = Param.String("", 'product family')
    oem_table_addr = Param.UInt32(0,
            'pointer to the optional oem configuration table')
    oem_table_size = Param.UInt16(0, 'size of the oem configuration table')
    local_apic = Param.UInt32(0xFEE00000, 'address of the local APIC')

    base_entries = VectorParam.X86IntelMPBaseConfigEntry([],
            'base configuration table entries')

    ext_entries = VectorParam.X86IntelMPExtConfigEntry([],
            'extended configuration table entries')

class X86IntelMPBaseConfigEntry(SimObject):
    type = 'X86IntelMPBaseConfigEntry'
    cxx_class = 'X86ISA::IntelMP::BaseConfigEntry'
    abstract = True

class X86IntelMPExtConfigEntry(SimObject):
    type = 'X86IntelMPExtConfigEntry'
    cxx_class = 'X86ISA::IntelMP::ExtConfigEntry'
    abstract = True

class X86IntelMPProcessor(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPProcessor'
    cxx_class = 'X86ISA::IntelMP::Processor'

    local_apic_id = Param.UInt8(0, 'local APIC id')
    local_apic_version = Param.UInt8(0,
            'bits 0-7 of the local APIC version register')
    enable = Param.Bool(True, 'if this processor is usable')
    bootstrap = Param.Bool(False, 'if this is the bootstrap processor')

    stepping = Param.UInt8(0)
    model = Param.UInt8(0)
    family = Param.UInt8(0)

    feature_flags = Param.UInt32(0, 'flags returned by the CPUID instruction')

class X86IntelMPBus(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPBus'
    cxx_class = 'X86ISA::IntelMP::Bus'

    bus_id = Param.UInt8(0, 'bus id assigned by the bios')
    bus_type = Param.String("", 'string that identify the bus type')
    # Legal values for bus_type are:
    #
    # "CBUS", "CBUSII", "EISA", "FUTURE", "INTERN", "ISA", "MBI", "MBII",
    # "MCA", "MPI", "MPSA", "NUBUS", "PCI", "PCMCIA", "TC", "VL", "VME",
    # "XPRESS"

class X86IntelMPIOAPIC(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPIOAPIC'
    cxx_class = 'X86ISA::IntelMP::IOAPIC'

    id = Param.UInt8(0, 'id of this APIC')
    version = Param.UInt8(0, 'bits 0-7 of the version register')

    enable = Param.Bool(True, 'if this APIC is usable')

    address = Param.UInt32(0xfec00000, 'address of this APIC')

class X86IntelMPInterruptType(Enum):
    map = {'INT' : 0,
           'NMI' : 1,
           'SMI' : 2,
           'ExtInt' : 3
    }

class X86IntelMPPolarity(Enum):
    map = {'ConformPolarity' : 0,
           'ActiveHigh' : 1,
           'ActiveLow' : 3
    }

class X86IntelMPTriggerMode(Enum):
    map = {'ConformTrigger' : 0,
           'EdgeTrigger' : 1,
           'LevelTrigger' : 3
    }

class X86IntelMPIOIntAssignment(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPIOIntAssignment'
    cxx_class = 'X86ISA::IntelMP::IOIntAssignment'

    interrupt_type = Param.X86IntelMPInterruptType('INT', 'type of interrupt')

    polarity = Param.X86IntelMPPolarity('ConformPolarity', 'polarity')
    trigger = Param.X86IntelMPTriggerMode('ConformTrigger', 'trigger mode')

    source_bus_id = Param.UInt8(0,
            'id of the bus from which the interrupt signal comes')
    source_bus_irq = Param.UInt8(0,
            'which interrupt signal from the source bus')

    dest_io_apic_id = Param.UInt8(0,
            'id of the IO APIC the interrupt is going to')
    dest_io_apic_intin = Param.UInt8(0,
            'the INTIN pin on the IO APIC the interrupt is connected to')

class X86IntelMPLocalIntAssignment(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPLocalIntAssignment'
    cxx_class = 'X86ISA::IntelMP::LocalIntAssignment'

    interrupt_type = Param.X86IntelMPInterruptType('INT', 'type of interrupt')

    polarity = Param.X86IntelMPPolarity('ConformPolarity', 'polarity')
    trigger = Param.X86IntelMPTriggerMode('ConformTrigger', 'trigger mode')

    source_bus_id = Param.UInt8(0,
            'id of the bus from which the interrupt signal comes')
    source_bus_irq = Param.UInt8(0,
            'which interrupt signal from the source bus')

    dest_local_apic_id = Param.UInt8(0,
            'id of the local APIC the interrupt is going to')
    dest_local_apic_intin = Param.UInt8(0,
            'the INTIN pin on the local APIC the interrupt is connected to')

class X86IntelMPAddressType(Enum):
    map = {"IOAddress" : 0,
           "MemoryAddress" : 1,
           "PrefetchAddress" : 2
    }

class X86IntelMPAddrSpaceMapping(X86IntelMPExtConfigEntry):
    type = 'X86IntelMPAddrSpaceMapping'
    cxx_class = 'X86ISA::IntelMP::AddrSpaceMapping'

    bus_id = Param.UInt8(0, 'id of the bus the address space is mapped to')
    address_type = Param.X86IntelMPAddressType('IOAddress',
            'address type used to access bus')
    address = Param.Addr(0, 'starting address of the mapping')
    length = Param.UInt64(0, 'length of mapping in bytes')

class X86IntelMPBusHierarchy(X86IntelMPExtConfigEntry):
    type = 'X86IntelMPBusHierarchy'
    cxx_class = 'X86ISA::IntelMP::BusHierarchy'

    bus_id = Param.UInt8(0, 'id of the bus being described')
    subtractive_decode = Param.Bool(False,
            'whether this bus contains all addresses not used by its children')
    parent_bus = Param.UInt8(0, 'bus id of this busses parent')

class X86IntelMPRangeList(Enum):
    map = {"ISACompatible" : 0,
           "VGACompatible" : 1
    }

class X86IntelMPCompatAddrSpaceMod(X86IntelMPExtConfigEntry):
    type = 'X86IntelMPCompatAddrSpaceMod'
    cxx_class = 'X86ISA::IntelMP::CompatAddrSpaceMod'

    bus_id = Param.UInt8(0, 'id of the bus being described')
    add = Param.Bool(False,
            'if the range should be added to the original mapping')
    range_list = Param.X86IntelMPRangeList('ISACompatible',
            'which predefined range of addresses to use')
