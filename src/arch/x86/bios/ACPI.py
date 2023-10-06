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
from m5.params import *
from m5.SimObject import SimObject


# ACPI description table header. Subclasses contain and handle the actual
# contents as appropriate for that type of table.
class X86ACPISysDescTable(SimObject):
    type = "X86ACPISysDescTable"
    cxx_class = "gem5::X86ISA::ACPI::SysDescTable"
    cxx_header = "arch/x86/bios/acpi.hh"
    abstract = True

    oem_id = Param.String("", "string identifying the oem")
    oem_table_id = Param.String("", "oem table ID")
    oem_revision = Param.UInt32(0, "oem revision number for the table")

    creator_id = Param.UInt32(0, "ID identifying the generator of the table")
    creator_revision = Param.UInt32(
        0, "revision number for the creator of the table"
    )


class X86ACPIRSDT(X86ACPISysDescTable):
    type = "X86ACPIRSDT"
    cxx_class = "gem5::X86ISA::ACPI::RSDT"
    cxx_header = "arch/x86/bios/acpi.hh"

    entries = VectorParam.X86ACPISysDescTable([], "system description tables")


class X86ACPIXSDT(X86ACPISysDescTable):
    type = "X86ACPIXSDT"
    cxx_class = "gem5::X86ISA::ACPI::XSDT"
    cxx_header = "arch/x86/bios/acpi.hh"

    entries = VectorParam.X86ACPISysDescTable([], "system description tables")


class X86ACPIMadtRecord(SimObject):
    type = "X86ACPIMadtRecord"
    cxx_class = "gem5::X86ISA::ACPI::MADT::Record"
    cxx_header = "arch/x86/bios/acpi.hh"
    abstract = True


class X86ACPIMadt(X86ACPISysDescTable):
    type = "X86ACPIMadt"
    cxx_class = "gem5::X86ISA::ACPI::MADT::MADT"
    cxx_header = "arch/x86/bios/acpi.hh"

    local_apic_address = Param.UInt32(0, "Address of the local apic")
    flags = Param.UInt32(0, "Flags")
    records = VectorParam.X86ACPIMadtRecord([], "Records in this MADT")


class X86ACPIMadtLAPIC(X86ACPIMadtRecord):
    type = "X86ACPIMadtLAPIC"
    cxx_header = "arch/x86/bios/acpi.hh"
    cxx_class = "gem5::X86ISA::ACPI::MADT::LAPIC"

    acpi_processor_id = Param.UInt8(0, "ACPI Processor ID")
    apic_id = Param.UInt8(0, "APIC ID")
    flags = Param.UInt32(0, "Flags")


class X86ACPIMadtIOAPIC(X86ACPIMadtRecord):
    type = "X86ACPIMadtIOAPIC"
    cxx_header = "arch/x86/bios/acpi.hh"
    cxx_class = "gem5::X86ISA::ACPI::MADT::IOAPIC"

    id = Param.UInt8(0, "I/O APIC ID")
    address = Param.Addr(0, "I/O APIC Address")
    int_base = Param.UInt32(0, "Global Interrupt Base")


class X86ACPIMadtIntSourceOverride(X86ACPIMadtRecord):
    type = "X86ACPIMadtIntSourceOverride"
    cxx_header = "arch/x86/bios/acpi.hh"
    cxx_class = "gem5::X86ISA::ACPI::MADT::IntSourceOverride"

    bus_source = Param.UInt8(0, "Bus Source")
    irq_source = Param.UInt8(0, "IRQ Source")
    sys_int = Param.UInt32(0, "Global System Interrupt")
    flags = Param.UInt16(0, "Flags")


class X86ACPIMadtNMI(X86ACPIMadtRecord):
    type = "X86ACPIMadtNMI"
    cxx_header = "arch/x86/bios/acpi.hh"
    cxx_class = "gem5::X86ISA::ACPI::MADT::NMI"

    acpi_processor_id = Param.UInt8(0, "ACPI Processor ID")
    flags = Param.UInt16(0, "Flags")
    lint_no = Param.UInt8(0, "LINT# (0 or 1)")


class X86ACPIMadtLAPICOverride(X86ACPIMadtRecord):
    type = "X86ACPIMadtLAPICOverride"
    cxx_header = "arch/x86/bios/acpi.hh"
    cxx_class = "gem5::X86ISA::ACPI::MADT::LAPICOverride"

    address = Param.Addr(0, "64-bit Physical Address of Local APIC")


# Root System Description Pointer Structure
class X86ACPIRSDP(SimObject):
    type = "X86ACPIRSDP"
    cxx_class = "gem5::X86ISA::ACPI::RSDP"
    cxx_header = "arch/x86/bios/acpi.hh"

    oem_id = Param.String("", "string identifying the oem")
    # Because 0 encodes ACPI 1.0, 2 encodes ACPI 3.0, the version implemented
    # here.
    revision = Param.UInt8(2, "revision of ACPI being used, zero indexed")

    rsdt = Param.X86ACPIRSDT(X86ACPIRSDT(), "root system description table")
    xsdt = Param.X86ACPIXSDT(
        X86ACPIXSDT(), "extended system description table"
    )
