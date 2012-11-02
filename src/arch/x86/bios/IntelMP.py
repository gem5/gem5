# Copyright (c) 2008 The Hewlett-Packard Development Company
# All rights reserved.
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
# Authors: Gabe Black

from m5.params import *
from m5.SimObject import SimObject

class X86IntelMPFloatingPointer(SimObject):
    type = 'X86IntelMPFloatingPointer'
    cxx_class = 'X86ISA::IntelMP::FloatingPointer'
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

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

    def add_entry(self, entry):
        if isinstance(entry, X86IntelMPBaseConfigEntry):
            self.base_entries.append(entry)
        elif isinstance(entry, X86IntelMPExtConfigEntry):
            self.ext_entries.append(entry)
        else:
            panic("Don't know what type of Intel MP entry %s is." \
                    % entry.__class__.__name__)

class X86IntelMPBaseConfigEntry(SimObject):
    type = 'X86IntelMPBaseConfigEntry'
    cxx_class = 'X86ISA::IntelMP::BaseConfigEntry'
    cxx_header = 'arch/x86/bios/intelmp.hh'
    abstract = True

class X86IntelMPExtConfigEntry(SimObject):
    type = 'X86IntelMPExtConfigEntry'
    cxx_class = 'X86ISA::IntelMP::ExtConfigEntry'
    cxx_header = 'arch/x86/bios/intelmp.hh'
    abstract = True

class X86IntelMPProcessor(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPProcessor'
    cxx_class = 'X86ISA::IntelMP::Processor'
    cxx_header = 'arch/x86/bios/intelmp.hh'

    local_apic_id = Param.UInt8(0, 'local APIC id')
    local_apic_version = Param.UInt8(0,
            'bits 0-7 of the local APIC version register')
    enable = Param.Bool(True, 'if this processor is usable')
    bootstrap = Param.Bool(False, 'if this is the bootstrap processor')

    stepping = Param.UInt8(0, 'Processor stepping')
    model = Param.UInt8(0, 'Processor model')
    family = Param.UInt8(0, 'Processor family')

    feature_flags = Param.UInt32(0, 'flags returned by the CPUID instruction')

class X86IntelMPBus(X86IntelMPBaseConfigEntry):
    type = 'X86IntelMPBus'
    cxx_class = 'X86ISA::IntelMP::Bus'
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

    bus_id = Param.UInt8(0, 'id of the bus the address space is mapped to')
    address_type = Param.X86IntelMPAddressType('IOAddress',
            'address type used to access bus')
    address = Param.Addr(0, 'starting address of the mapping')
    length = Param.UInt64(0, 'length of mapping in bytes')

class X86IntelMPBusHierarchy(X86IntelMPExtConfigEntry):
    type = 'X86IntelMPBusHierarchy'
    cxx_class = 'X86ISA::IntelMP::BusHierarchy'
    cxx_header = 'arch/x86/bios/intelmp.hh'

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
    cxx_header = 'arch/x86/bios/intelmp.hh'

    bus_id = Param.UInt8(0, 'id of the bus being described')
    add = Param.Bool(False,
            'if the range should be added to the original mapping')
    range_list = Param.X86IntelMPRangeList('ISACompatible',
            'which predefined range of addresses to use')
