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

#ifndef __ARCH_X86_BIOS_ACPI_HH__
#define __ARCH_X86_BIOS_ACPI_HH__

#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"
#include "debug/ACPI.hh"
#include "params/X86ACPIDSDT.hh"
#include "params/X86ACPIFADT.hh"
#include "params/X86ACPIMadt.hh"
#include "params/X86ACPIMadtIOAPIC.hh"
#include "params/X86ACPIMadtIntSourceOverride.hh"
#include "params/X86ACPIMadtLAPIC.hh"
#include "params/X86ACPIMadtLAPICOverride.hh"
#include "params/X86ACPIMadtNMI.hh"
#include "params/X86ACPIMadtRecord.hh"
#include "params/X86ACPIRSDP.hh"
#include "params/X86ACPIRSDT.hh"
#include "params/X86ACPISysDescTable.hh"
#include "params/X86ACPIXSDT.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PortProxy;

namespace X86ISA
{

namespace ACPI
{

class RSDT;
class XSDT;

struct Allocator
{
    virtual Addr alloc(std::size_t size, unsigned align=1) = 0;
};
struct LinearAllocator : public Allocator
{
    LinearAllocator(Addr begin, Addr end=0) :
        next(begin),
        end(end)
    {}

    Addr alloc(std::size_t size, unsigned align) override;

  protected:
    Addr next;
    Addr const end;
};

class RSDP : public SimObject
{
  protected:
    PARAMS(X86ACPIRSDP);

    static const char signature[];

    struct GEM5_PACKED MemR0
    {
        // src: https://wiki.osdev.org/RSDP
        char signature[8] = {};
        uint8_t checksum = 0;
        char oemID[6] = {};
        uint8_t revision = 0;
        uint32_t rsdtAddress = 0;
    };
    static_assert(std::is_trivially_copyable_v<MemR0>,
            "Type not suitable for memcpy.");

    struct GEM5_PACKED Mem : public MemR0
    {
        // since version 2
        uint32_t length = 0;
        uint64_t xsdtAddress = 0;
        uint8_t extendedChecksum = 0;
        uint8_t _reserved[3] = {};
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy,");

    RSDT* rsdt;
    XSDT* xsdt;

  public:
    RSDP(const Params &p);

    Addr write(PortProxy& phys_proxy, Allocator& alloc) const;
};

// The Differentiated System Description Table. Its body is a block of AML
// bytecode interpreted by the guest OS (gem5 never interprets AML). The blob
// is a complete, self-contained ACPI table (header + AML) embedded at build
// time from src/arch/x86/bios/dsdt.aml, so it is written out verbatim rather
// than going through the SysDescTable header-generation path. It is reached by
// the guest only via the FADT, never from the RSDT/XSDT entry list.
class DSDT : public SimObject
{
  protected:
    PARAMS(X86ACPIDSDT);

  public:
    DSDT(const Params &p);

    Addr write(PortProxy &phys_proxy, Allocator &alloc) const;
};

class SysDescTable : public SimObject
{
  protected:
    PARAMS(X86ACPISysDescTable);

    struct GEM5_PACKED Mem
    {
        // src: https://wiki.osdev.org/RSDT
        char signature[4] = {};
        uint32_t length = 0;
        uint8_t revision = 0;
        uint8_t checksum = 0;
        char oemID[6] = {};
        char oemTableID[8] = {};
        uint32_t oemRevision = 0;
        uint32_t creatorID = 0;
        uint32_t creatorRevision = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    virtual Addr writeBuf(PortProxy& phys_proxy, Allocator& alloc,
            std::vector<uint8_t>& mem) const = 0;

    const char* signature;
    uint8_t revision;

    SysDescTable(const Params& p, const char* _signature, uint8_t _revision) :
        SimObject(p), signature(_signature), revision(_revision)
    {}

  public:
    Addr
    write(PortProxy& phys_proxy, Allocator& alloc) const
    {
        std::vector<uint8_t> mem;
        return writeBuf(phys_proxy, alloc, mem);
    }
};

template<class T>
class RXSDT : public SysDescTable
{
  protected:
    using Ptr = T;

    std::vector<SysDescTable *> entries;

    Addr writeBuf(PortProxy& phys_proxy, Allocator& alloc,
            std::vector<uint8_t>& mem) const override;

  protected:
    RXSDT(const Params& p, const char* _signature, uint8_t _revision);
};

class RSDT : public RXSDT<uint32_t>
{
  protected:
    PARAMS(X86ACPIRSDT);

  public:
    RSDT(const Params &p);
};

class XSDT : public RXSDT<uint64_t>
{
  protected:
    PARAMS(X86ACPIXSDT);

  public:
    XSDT(const Params &p);
};

// ACPI Generic Address Structure (GAS), used by several FADT fields.
struct GEM5_PACKED GenericAddress
{
    uint8_t spaceId = 0;
    uint8_t bitWidth = 0;
    uint8_t bitOffset = 0;
    uint8_t accessSize = 0;
    uint64_t address = 0;
};
static_assert(sizeof(GenericAddress) == 12, "Unexpected GAS size.");

// Fixed ACPI Description Table. gem5 writes this so the simulate OS finds
// the DSDT which is required to make sure it does not disable ACPI.
class FADT : public SysDescTable
{
  protected:
    PARAMS(X86ACPIFADT);

    // Replica of FADT table from Linux kernel:
    // https://elixir.bootlin.com/linux/v7.1/source/include/acpi/actbl.h#L199
    struct GEM5_PACKED Mem : public SysDescTable::Mem
    {
        uint32_t firmwareCtrl = 0;
        uint32_t dsdt = 0;
        uint8_t _reserved0 = 0;
        uint8_t preferredPmProfile = 0;
        uint16_t sciInt = 0;
        uint32_t smiCmd = 0;
        uint8_t acpiEnable = 0;
        uint8_t acpiDisable = 0;
        uint8_t s4BiosReq = 0;
        uint8_t pstateCnt = 0;
        uint32_t pm1aEvtBlk = 0;
        uint32_t pm1bEvtBlk = 0;
        uint32_t pm1aCntBlk = 0;
        uint32_t pm1bCntBlk = 0;
        uint32_t pm2CntBlk = 0;
        uint32_t pmTmrBlk = 0;
        uint32_t gpe0Blk = 0;
        uint32_t gpe1Blk = 0;
        uint8_t pm1EvtLen = 0;
        uint8_t pm1CntLen = 0;
        uint8_t pm2CntLen = 0;
        uint8_t pmTmrLen = 0;
        uint8_t gpe0BlkLen = 0;
        uint8_t gpe1BlkLen = 0;
        uint8_t gpe1Base = 0;
        uint8_t cstCnt = 0;
        uint16_t pLvl2Lat = 0;
        uint16_t pLvl3Lat = 0;
        uint16_t flushSize = 0;
        uint16_t flushStride = 0;
        uint8_t dutyOffset = 0;
        uint8_t dutyWidth = 0;
        uint8_t dayAlrm = 0;
        uint8_t monAlrm = 0;
        uint8_t century = 0;
        uint16_t iapcBootArch = 0;
        uint8_t _reserved1 = 0;
        uint32_t flags = 0;
        GenericAddress resetReg;
        uint8_t resetValue = 0;
        uint16_t armBootArch = 0;
        uint8_t minorRevision = 0;
        uint64_t xFirmwareCtrl = 0;
        uint64_t xDsdt = 0;
        GenericAddress xPm1aEvtBlk;
        GenericAddress xPm1bEvtBlk;
        GenericAddress xPm1aCntBlk;
        GenericAddress xPm1bCntBlk;
        GenericAddress xPm2CntBlk;
        GenericAddress xPmTmrBlk;
        GenericAddress xGpe0Blk;
        GenericAddress xGpe1Blk;
        GenericAddress sleepControlReg;
        GenericAddress sleepStatusReg;
        uint64_t hypervisorVendorId = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
                  "Type not suitable for memcpy.");

    DSDT *dsdt;
    bool hwReduced;

    Addr writeBuf(PortProxy &phys_proxy, Allocator &alloc,
                  std::vector<uint8_t> &mem) const override;

  public:
    FADT(const Params &p);
};

namespace MADT
{
class Record : public SimObject
{
  protected:
    PARAMS(X86ACPIMadtRecord);

    struct GEM5_PACKED Mem
    {
        uint8_t type = 0;
        uint8_t length = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    uint8_t type;

    virtual void prepareBuf(std::vector<uint8_t>& mem) const = 0;

  public:
    Record(const Params& p, uint8_t _type) : SimObject(p), type(_type) {}

    std::vector<uint8_t>
    prepare() const
    {
        std::vector<uint8_t> mem;
        prepareBuf(mem);
        return mem;
    }
};

class LAPIC : public Record
{
  protected:
    PARAMS(X86ACPIMadtLAPIC);

    struct GEM5_PACKED Mem : public Record::Mem
    {
        uint8_t acpiProcessorId = 0;
        uint8_t apicId = 0;
        uint32_t flags = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    void prepareBuf(std::vector<uint8_t>& mem) const override;

  public:
    LAPIC(const Params& p) : Record(p, 0) {}
};

class IOAPIC : public Record
{
  protected:
    PARAMS(X86ACPIMadtIOAPIC);

    struct GEM5_PACKED Mem : public Record::Mem
    {
        uint8_t ioApicId = 0;
        uint8_t _reserved = 0;
        uint32_t ioApicAddress = 0;
        uint32_t intBase = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    void prepareBuf(std::vector<uint8_t>& mem) const override;

  public:
    IOAPIC(const Params& p) : Record(p, 1) {}
};

class IntSourceOverride : public Record
{
  protected:
    PARAMS(X86ACPIMadtIntSourceOverride);

    struct GEM5_PACKED Mem : public Record::Mem
    {
        uint8_t busSource = 0;
        uint8_t irqSource = 0;
        uint32_t globalSystemInterrupt = 0;
        uint16_t flags = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    void prepareBuf(std::vector<uint8_t>& mem) const override;

  public:
    IntSourceOverride(const Params& p) : Record(p, 2) {}
};

class NMI : public Record
{
  protected:
    PARAMS(X86ACPIMadtNMI);

    struct GEM5_PACKED Mem : public Record::Mem
    {
        uint8_t acpiProcessorId = 0;
        uint16_t flags = 0;
        uint8_t lintNo = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    void prepareBuf(std::vector<uint8_t>& mem) const override;

  public:
    NMI(const Params& p) : Record(p, 3) {}
};

class LAPICOverride : public Record
{
  protected:
    PARAMS(X86ACPIMadtLAPICOverride);

    struct GEM5_PACKED Mem : public Record::Mem
    {
        uint16_t _reserved = 0;
        uint64_t localAPICAddress = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    void prepareBuf(std::vector<uint8_t>& mem) const override;

  public:
    LAPICOverride(const Params& p) : Record(p, 5) {}
};

class MADT : public SysDescTable
{
  protected:
    PARAMS(X86ACPIMadt);

    struct GEM5_PACKED Mem : public SysDescTable::Mem
    {
        uint32_t localAPICAddress = 0;
        uint32_t flags = 0;
    };
    static_assert(std::is_trivially_copyable_v<Mem>,
            "Type not suitable for memcpy.");

    std::vector<Record *> records;

    Addr writeBuf(PortProxy& phys_proxy, Allocator& alloc,
            std::vector<uint8_t>& mem) const override;

  public:
    MADT(const Params &p);
};

} // namespace MADT

} // namespace ACPI

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_BIOS_E820_HH__
