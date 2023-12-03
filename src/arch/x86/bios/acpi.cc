/*
 * Copyright (c) 2008 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 */

#include "arch/x86/bios/acpi.hh"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "base/trace.hh"
#include "mem/port.hh"
#include "mem/port_proxy.hh"
#include "sim/byteswap.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace X86ISA
{

namespace ACPI
{

static void
fillCopy(void *dest, size_t dest_size, const void *src, size_t src_size)
{
    src_size = std::min(src_size, dest_size);
    const size_t zero_size = dest_size - src_size;

    uint8_t *pos = (uint8_t *)dest;

    std::memcpy(pos, src, src_size);
    pos += src_size;

    std::memset(pos, 0, zero_size);
}

static void
fillCopy(void *dest, size_t dest_size, const std::string &src)
{
    fillCopy(dest, dest_size, src.c_str(), src.length());
}

const char RSDP::signature[] = "RSD PTR ";

static uint8_t
apic_checksum(uint8_t *ptr, std::size_t size)
{
    uint8_t sum = 0;
    for (unsigned i = 0; i < size; ++i)
        sum += ptr[i];
    return 0x100 - sum;
}

Addr
LinearAllocator::alloc(std::size_t size, unsigned align)
{
    if (align) {
        unsigned offset = next % align;
        if (offset)
            next += (align - offset);
    }
    Addr chunk = next;
    next += size;
    assert(0 == end || next <= end);
    return chunk;
}

RSDP::RSDP(const Params &p) : SimObject(p), rsdt(p.rsdt), xsdt(p.xsdt) {}

Addr
RSDP::write(PortProxy &phys_proxy, Allocator &alloc) const
{
    std::vector<uint8_t> mem(sizeof(Mem));
    Addr addr = alloc.alloc(mem.size(), 16);

    Mem *data = (Mem *)mem.data();
    static_assert(sizeof(signature) - 1 == sizeof(data->signature),
                  "signature length mismatch");
    std::memcpy(data->signature, signature, sizeof(data->signature));
    fillCopy(data->oemID, sizeof(data->oemID), params().oem_id);
    data->revision = params().revision;
    data->length = mem.size();

    if (rsdt) {
        data->rsdtAddress = rsdt->write(phys_proxy, alloc);
        DPRINTF(ACPI, "Allocated RSDT @ %llx\n", data->rsdtAddress);
    }
    if (xsdt) {
        data->xsdtAddress = xsdt->write(phys_proxy, alloc);
        DPRINTF(ACPI, "Allocated XSDT @ %llx\n", data->xsdtAddress);
    }

    // checksum calculation
    data->checksum = apic_checksum(mem.data(), sizeof(MemR0));
    data->extendedChecksum = apic_checksum(mem.data(), mem.size());

    // write the whole thing
    phys_proxy.writeBlob(addr, mem.data(), mem.size());

    return addr;
}

Addr
SysDescTable::writeBuf(PortProxy &phys_proxy, Allocator &alloc,
                       std::vector<uint8_t> &mem) const
{
    // An empty SysDescTable doesn't make any sense, so assert that somebody
    // else allocated a large enough blob.
    assert(mem.size() >= sizeof(Mem));

    // Allocate a place to write this blob.
    Addr addr = alloc.alloc(mem.size());

    DPRINTF(ACPI, "Writing system description table [%llx - %llx]\n", addr,
            addr + mem.size());

    // Fill in the header.
    auto &p = params();
    Mem *header = (Mem *)mem.data();
    fillCopy(header->signature, sizeof(header->signature), signature);
    header->length = mem.size();
    header->revision = revision;
    fillCopy(header->oemID, sizeof(header->oemID), p.oem_id);
    fillCopy(header->oemTableID, sizeof(header->oemTableID), p.oem_table_id);
    header->oemRevision = p.oem_revision;
    header->creatorID = p.creator_id;
    header->creatorRevision = p.creator_revision;

    // Update checksum.
    header->checksum = apic_checksum(mem.data(), mem.size());

    // Write to memory.
    phys_proxy.writeBlob(addr, mem.data(), mem.size());

    return addr;
}

//// RSDT, XSDT
template <class T>
RXSDT<T>::RXSDT(const Params &p, const char *_signature, uint8_t _revision)
    : SysDescTable(p, _signature, _revision)
{}

template <class T>
Addr
RXSDT<T>::writeBuf(PortProxy &phys_proxy, Allocator &alloc,
                   std::vector<uint8_t> &mem) const
{
    // Since this table ends with a variably sized array, it can't be extended
    // by another table type.
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    auto base_size = mem.size();
    mem.resize(base_size + sizeof(Ptr) * entries.size());

    Ptr *ptr_array = reinterpret_cast<Ptr *>(mem.data() + base_size);
    DPRINTF(ACPI, "RXSDT: Writing %d entries (ptr size: %d)\n", entries.size(),
            sizeof(Ptr));
    for (const auto *entry : entries) {
        Addr entry_addr = entry->write(phys_proxy, alloc);
        fatal_if((entry_addr & mask(sizeof(Ptr) * 8)) != entry_addr,
                 "RXSDT: Entry address doesn't fit in pointer type.");
        DPRINTF(ACPI, "RXSDT: wrote entry @ %llx\n", entry_addr);
        *ptr_array++ = entry_addr;
    }

    return SysDescTable::writeBuf(phys_proxy, alloc, mem);
}

RSDT::RSDT(const Params &p) : RXSDT(p, "RSDT", 1) { entries = p.entries; }

XSDT::XSDT(const Params &p) : RXSDT(p, "XSDT", 1) { entries = p.entries; }

//// MADT
MADT::MADT::MADT(const Params &p)
    : SysDescTable(p, "APIC", 4), records(p.records)
{}

Addr
MADT::MADT::writeBuf(PortProxy &phys_proxy, Allocator &alloc,
                     std::vector<uint8_t> &mem) const
{
    // Since this table ends with a variably sized array, it can't be extended
    // by another table type.
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *header = reinterpret_cast<Mem *>(mem.data());
    header->localAPICAddress = params().local_apic_address;
    header->flags = params().flags;

    for (const auto &record : records) {
        auto entry = record->prepare();
        mem.insert(mem.end(), entry.begin(), entry.end());
    }

    DPRINTF(ACPI, "MADT: writing %d records (size: %d)\n", records.size(),
            mem.size());

    return SysDescTable::writeBuf(phys_proxy, alloc, mem);
}

void
MADT::Record::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.size() >= sizeof(Mem));
    DPRINTF(ACPI, "MADT: writing record type %d (size: %d)\n", type,
            mem.size());

    Mem *header = reinterpret_cast<Mem *>(mem.data());
    header->type = type;
    header->length = mem.size();
}

void
MADT::LAPIC::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *data = reinterpret_cast<Mem *>(mem.data());
    data->acpiProcessorId = params().acpi_processor_id;
    data->apicId = params().apic_id;
    data->flags = params().flags;

    Record::prepareBuf(mem);
}

void
MADT::IOAPIC::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *data = reinterpret_cast<Mem *>(mem.data());
    data->ioApicId = params().id;
    data->ioApicAddress = params().address;
    data->intBase = params().int_base;

    Record::prepareBuf(mem);
}

void
MADT::IntSourceOverride::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *data = reinterpret_cast<Mem *>(mem.data());
    data->busSource = params().bus_source;
    data->irqSource = params().irq_source;
    data->globalSystemInterrupt = params().sys_int;
    data->flags = params().flags;

    Record::prepareBuf(mem);
}

void
MADT::NMI::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *data = reinterpret_cast<Mem *>(mem.data());
    data->acpiProcessorId = params().acpi_processor_id;
    // The "flags" field is not properly aligned.
    memcpy(&data->flags, &params().flags, sizeof(data->flags));
    data->lintNo = params().lint_no;

    Record::prepareBuf(mem);
}

void
MADT::LAPICOverride::prepareBuf(std::vector<uint8_t> &mem) const
{
    assert(mem.empty());
    mem.resize(sizeof(Mem));

    Mem *data = reinterpret_cast<Mem *>(mem.data());
    data->localAPICAddress = params().address;

    Record::prepareBuf(mem);
}

} // namespace ACPI

} // namespace X86ISA
} // namespace gem5
