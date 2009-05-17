/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/x86/bios/intelmp.hh"
#include "arch/x86/isa_traits.hh"
#include "base/misc.hh"
#include "base/types.hh"
#include "mem/port.hh"
#include "sim/byteswap.hh"

// Config entry types
#include "params/X86IntelMPBaseConfigEntry.hh"
#include "params/X86IntelMPExtConfigEntry.hh"

// General table structures
#include "params/X86IntelMPConfigTable.hh"
#include "params/X86IntelMPFloatingPointer.hh"

// Base entry types
#include "params/X86IntelMPBus.hh"
#include "params/X86IntelMPIOAPIC.hh"
#include "params/X86IntelMPIOIntAssignment.hh"
#include "params/X86IntelMPLocalIntAssignment.hh"
#include "params/X86IntelMPProcessor.hh"

// Extended entry types
#include "params/X86IntelMPAddrSpaceMapping.hh"
#include "params/X86IntelMPBusHierarchy.hh"
#include "params/X86IntelMPCompatAddrSpaceMod.hh"

using namespace std;

const char X86ISA::IntelMP::FloatingPointer::signature[] = "_MP_";

template<class T>
uint8_t
writeOutField(FunctionalPort * port, Addr addr, T val)
{
    T guestVal = X86ISA::htog(val);
    port->writeBlob(addr, (uint8_t *)(&guestVal), sizeof(T));

    uint8_t checkSum = 0;
    while(guestVal) {
        checkSum += guestVal;
        guestVal >>= 8;
    }
    return checkSum;
}

uint8_t
writeOutString(FunctionalPort * port, Addr addr, string str, int length)
{
    char cleanedString[length + 1];
    cleanedString[length] = 0;

    if (str.length() > length) {
        memcpy(cleanedString, str.c_str(), length);
        warn("Intel MP configuration table string \"%s\" "
                "will be truncated to \"%s\".\n", str, cleanedString);
    } else {
        memcpy(cleanedString, str.c_str(), str.length());
        memset(cleanedString + str.length(), 0, length - str.length());
    }
    port->writeBlob(addr, (uint8_t *)(&cleanedString), length);

    uint8_t checkSum = 0;
    for (int i = 0; i < length; i++)
        checkSum += cleanedString[i];

    return checkSum;
}

Addr
X86ISA::IntelMP::FloatingPointer::writeOut(FunctionalPort * port, Addr addr)
{
    // Make sure that either a config table is present or a default
    // configuration was found but not both.
    if (!tableAddr && !defaultConfig)
        fatal("Either an MP configuration table or a default configuration "
                "must be used.");
    if (tableAddr && defaultConfig)
        fatal("Both an MP configuration table and a default configuration "
                "were set.");

    uint8_t checkSum = 0;

    port->writeBlob(addr, (uint8_t *)signature, 4);
    for (int i = 0; i < 4; i++)
        checkSum += signature[i];

    checkSum += writeOutField(port, addr + 4, tableAddr);

    // The length of the structure in paragraphs, aka 16 byte chunks.
    uint8_t length = 1;
    port->writeBlob(addr + 8, &length, 1);
    checkSum += length;

    port->writeBlob(addr + 9, &specRev, 1);
    checkSum += specRev;

    port->writeBlob(addr + 11, &defaultConfig, 1);
    checkSum += defaultConfig;

    uint32_t features2_5 = imcrPresent ? (1 << 7) : 0;
    checkSum += writeOutField(port, addr + 12, features2_5);

    checkSum = -checkSum;
    port->writeBlob(addr + 10, &checkSum, 1);

    return 16;
}

X86ISA::IntelMP::FloatingPointer::FloatingPointer(Params * p) :
    SimObject(p), tableAddr(0), specRev(p->spec_rev),
    defaultConfig(p->default_config), imcrPresent(p->imcr_present)
{}

X86ISA::IntelMP::FloatingPointer *
X86IntelMPFloatingPointerParams::create()
{
    return new X86ISA::IntelMP::FloatingPointer(this);
}

Addr
X86ISA::IntelMP::BaseConfigEntry::writeOut(FunctionalPort * port,
        Addr addr, uint8_t &checkSum)
{
    port->writeBlob(addr, &type, 1);
    checkSum += type;
    return 1;
}

X86ISA::IntelMP::BaseConfigEntry::BaseConfigEntry(Params * p, uint8_t _type) :
    SimObject(p), type(_type)
{}

Addr
X86ISA::IntelMP::ExtConfigEntry::writeOut(FunctionalPort * port,
        Addr addr, uint8_t &checkSum)
{
    port->writeBlob(addr, &type, 1);
    checkSum += type;
    port->writeBlob(addr + 1, &length, 1);
    checkSum += length;
    return 1;
}

X86ISA::IntelMP::ExtConfigEntry::ExtConfigEntry(Params * p,
        uint8_t _type, uint8_t _length) :
    SimObject(p), type(_type), length(_length)
{}

const char X86ISA::IntelMP::ConfigTable::signature[] = "PCMP";

Addr
X86ISA::IntelMP::ConfigTable::writeOut(FunctionalPort * port, Addr addr)
{
    uint8_t checkSum = 0;

    port->writeBlob(addr, (uint8_t *)signature, 4);
    for (int i = 0; i < 4; i++)
        checkSum += signature[i];

    // Base table length goes here but will be calculated later.

    port->writeBlob(addr + 6, (uint8_t *)(&specRev), 1);
    checkSum += specRev;

    // The checksum goes here but is still being calculated.

    checkSum += writeOutString(port, addr + 8, oemID, 8);
    checkSum += writeOutString(port, addr + 16, productID, 12);

    checkSum += writeOutField(port, addr + 28, oemTableAddr);
    checkSum += writeOutField(port, addr + 32, oemTableSize);
    checkSum += writeOutField(port, addr + 34, (uint16_t)baseEntries.size());
    checkSum += writeOutField(port, addr + 36, localApic);

    uint8_t reserved = 0;
    port->writeBlob(addr + 43, &reserved, 1);
    checkSum += reserved;

    vector<BaseConfigEntry *>::iterator baseEnt;
    uint16_t offset = 44;
    for (baseEnt = baseEntries.begin();
            baseEnt != baseEntries.end(); baseEnt++) {
        offset += (*baseEnt)->writeOut(port, addr + offset, checkSum);
    }

    // We've found the end of the base table this point.
    checkSum += writeOutField(port, addr + 4, offset);

    vector<ExtConfigEntry *>::iterator extEnt;
    uint16_t extOffset = 0;
    uint8_t extCheckSum = 0;
    for (extEnt = extEntries.begin();
            extEnt != extEntries.end(); extEnt++) {
        extOffset += (*extEnt)->writeOut(port,
                addr + offset + extOffset, extCheckSum);
    }

    checkSum += writeOutField(port, addr + 40, extOffset);
    extCheckSum = -extCheckSum;
    checkSum += writeOutField(port, addr + 42, extCheckSum);

    // And now, we finally have the whole check sum completed.
    checkSum = -checkSum;
    writeOutField(port, addr + 7, checkSum);

    return offset + extOffset;
};

X86ISA::IntelMP::ConfigTable::ConfigTable(Params * p) : SimObject(p),
    specRev(p->spec_rev), oemID(p->oem_id), productID(p->product_id),
    oemTableAddr(p->oem_table_addr), oemTableSize(p->oem_table_size),
    localApic(p->local_apic),
    baseEntries(p->base_entries), extEntries(p->ext_entries)
{}

X86ISA::IntelMP::ConfigTable *
X86IntelMPConfigTableParams::create()
{
    return new X86ISA::IntelMP::ConfigTable(this);
}

Addr
X86ISA::IntelMP::Processor::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    BaseConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 1, localApicID);
    checkSum += writeOutField(port, addr + 2, localApicVersion);
    checkSum += writeOutField(port, addr + 3, cpuFlags);
    checkSum += writeOutField(port, addr + 4, cpuSignature);
    checkSum += writeOutField(port, addr + 8, featureFlags);

    uint32_t reserved = 0;
    port->writeBlob(addr + 12, (uint8_t *)(&reserved), 4);
    port->writeBlob(addr + 16, (uint8_t *)(&reserved), 4);
    return 20;
}

X86ISA::IntelMP::Processor::Processor(Params * p) : BaseConfigEntry(p, 0),
    localApicID(p->local_apic_id), localApicVersion(p->local_apic_version),
    cpuFlags(0), cpuSignature(0), featureFlags(p->feature_flags)
{
    if (p->enable)
        cpuFlags |= (1 << 0);
    if (p->bootstrap)
        cpuFlags |= (1 << 1);

    replaceBits(cpuSignature, 0, 3, p->stepping);
    replaceBits(cpuSignature, 4, 7, p->model);
    replaceBits(cpuSignature, 8, 11, p->family);
}

X86ISA::IntelMP::Processor *
X86IntelMPProcessorParams::create()
{
    return new X86ISA::IntelMP::Processor(this);
}

Addr
X86ISA::IntelMP::Bus::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    BaseConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 1, busID);
    checkSum += writeOutString(port, addr + 2, busType, 6);
    return 8;
}

X86ISA::IntelMP::Bus::Bus(Params * p) : BaseConfigEntry(p, 1),
    busID(p->bus_id), busType(p->bus_type)
{}

X86ISA::IntelMP::Bus *
X86IntelMPBusParams::create()
{
    return new X86ISA::IntelMP::Bus(this);
}

Addr
X86ISA::IntelMP::IOAPIC::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    BaseConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 1, id);
    checkSum += writeOutField(port, addr + 2, version);
    checkSum += writeOutField(port, addr + 3, flags);
    checkSum += writeOutField(port, addr + 4, address);
    return 8;
}

X86ISA::IntelMP::IOAPIC::IOAPIC(Params * p) : BaseConfigEntry(p, 2),
    id(p->id), version(p->version), flags(0), address(p->address)
{
    if (p->enable)
        flags |= 1;
}

X86ISA::IntelMP::IOAPIC *
X86IntelMPIOAPICParams::create()
{
    return new X86ISA::IntelMP::IOAPIC(this);
}

Addr
X86ISA::IntelMP::IntAssignment::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    BaseConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 1, interruptType);
    checkSum += writeOutField(port, addr + 2, flags);
    checkSum += writeOutField(port, addr + 4, sourceBusID);
    checkSum += writeOutField(port, addr + 5, sourceBusIRQ);
    checkSum += writeOutField(port, addr + 6, destApicID);
    checkSum += writeOutField(port, addr + 7, destApicIntIn);
    return 8;
}

X86ISA::IntelMP::IOIntAssignment::IOIntAssignment(Params * p) :
    IntAssignment(p, p->interrupt_type, p->polarity, p->trigger, 3,
            p->source_bus_id, p->source_bus_irq,
            p->dest_io_apic_id, p->dest_io_apic_intin)
{}

X86ISA::IntelMP::IOIntAssignment *
X86IntelMPIOIntAssignmentParams::create()
{
    return new X86ISA::IntelMP::IOIntAssignment(this);
}

X86ISA::IntelMP::LocalIntAssignment::LocalIntAssignment(Params * p) :
    IntAssignment(p, p->interrupt_type, p->polarity, p->trigger, 4,
            p->source_bus_id, p->source_bus_irq,
            p->dest_local_apic_id, p->dest_local_apic_intin)
{}

X86ISA::IntelMP::LocalIntAssignment *
X86IntelMPLocalIntAssignmentParams::create()
{
    return new X86ISA::IntelMP::LocalIntAssignment(this);
}

Addr
X86ISA::IntelMP::AddrSpaceMapping::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    ExtConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 2, busID);
    checkSum += writeOutField(port, addr + 3, addrType);
    checkSum += writeOutField(port, addr + 4, addr);
    checkSum += writeOutField(port, addr + 12, addrLength);
    return length;
}

X86ISA::IntelMP::AddrSpaceMapping::AddrSpaceMapping(Params * p) :
    ExtConfigEntry(p, 128, 20),
    busID(p->bus_id), addrType(p->address_type),
    addr(p->address), addrLength(p->length)
{}

X86ISA::IntelMP::AddrSpaceMapping *
X86IntelMPAddrSpaceMappingParams::create()
{
    return new X86ISA::IntelMP::AddrSpaceMapping(this);
}

Addr
X86ISA::IntelMP::BusHierarchy::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    ExtConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 2, busID);
    checkSum += writeOutField(port, addr + 3, info);
    checkSum += writeOutField(port, addr + 4, parentBus);

    uint32_t reserved = 0;
    port->writeBlob(addr + 5, (uint8_t *)(&reserved), 3);

    return length;
}

X86ISA::IntelMP::BusHierarchy::BusHierarchy(Params * p) :
    ExtConfigEntry(p, 129, 8),
    busID(p->bus_id), info(0), parentBus(p->parent_bus)
{
    if (p->subtractive_decode)
        info |= 1;
}

X86ISA::IntelMP::BusHierarchy *
X86IntelMPBusHierarchyParams::create()
{
    return new X86ISA::IntelMP::BusHierarchy(this);
}

Addr
X86ISA::IntelMP::CompatAddrSpaceMod::writeOut(
        FunctionalPort * port, Addr addr, uint8_t &checkSum)
{
    ExtConfigEntry::writeOut(port, addr, checkSum);
    checkSum += writeOutField(port, addr + 2, busID);
    checkSum += writeOutField(port, addr + 3, mod);
    checkSum += writeOutField(port, addr + 4, rangeList);
    return length;
}

X86ISA::IntelMP::CompatAddrSpaceMod::CompatAddrSpaceMod(Params * p) :
    ExtConfigEntry(p, 130, 8),
    busID(p->bus_id), mod(0), rangeList(p->range_list)
{
    if (p->add)
        mod |= 1;
}

X86ISA::IntelMP::CompatAddrSpaceMod *
X86IntelMPCompatAddrSpaceModParams::create()
{
    return new X86ISA::IntelMP::CompatAddrSpaceMod(this);
}
