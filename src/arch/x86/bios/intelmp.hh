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

#ifndef __ARCH_X86_BIOS_INTELMP_HH__
#define __ARCH_X86_BIOS_INTELMP_HH__

#include <string>
#include <vector>

#include "base/bitfield.hh"
#include "base/compiler.hh"
#include "enums/X86IntelMPAddressType.hh"
#include "enums/X86IntelMPInterruptType.hh"
#include "enums/X86IntelMPPolarity.hh"
#include "enums/X86IntelMPRangeList.hh"
#include "enums/X86IntelMPTriggerMode.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class PortProxy;

// Config entry types
struct X86IntelMPBaseConfigEntryParams;
struct X86IntelMPExtConfigEntryParams;

// General table structures
struct X86IntelMPConfigTableParams;
struct X86IntelMPFloatingPointerParams;

// Base entry types
struct X86IntelMPBusParams;
struct X86IntelMPIOAPICParams;
struct X86IntelMPIOIntAssignmentParams;
struct X86IntelMPLocalIntAssignmentParams;
struct X86IntelMPProcessorParams;

// Extended entry types
struct X86IntelMPAddrSpaceMappingParams;
struct X86IntelMPBusHierarchyParams;
struct X86IntelMPCompatAddrSpaceModParams;

template<class T>
uint8_t writeOutField(PortProxy& proxy, Addr addr, T val);

uint8_t writeOutString(PortProxy& proxy, Addr addr, std::string str,
                       int length);

namespace X86ISA
{

namespace intelmp
{

class FloatingPointer : public SimObject
{
  protected:
    typedef X86IntelMPFloatingPointerParams Params;

    uint32_t tableAddr;
    uint8_t specRev;
    uint8_t defaultConfig;
    bool imcrPresent;

    static const char signature[];

  public:

    Addr writeOut(PortProxy& proxy, Addr addr);

    Addr getTableAddr()
    {
        return tableAddr;
    }

    void setTableAddr(Addr addr)
    {
        tableAddr = addr;
    }

    FloatingPointer(const Params &p);
};

class BaseConfigEntry : public SimObject
{
  protected:
    typedef X86IntelMPBaseConfigEntryParams Params;

    uint8_t type;

  public:

    virtual Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    BaseConfigEntry(const Params &p, uint8_t _type);
};

class ExtConfigEntry : public SimObject
{
  protected:
    typedef X86IntelMPExtConfigEntryParams Params;

    uint8_t type;
    uint8_t length;

  public:

    virtual Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    ExtConfigEntry(const Params &p, uint8_t _type, uint8_t _length);
};

class ConfigTable : public SimObject
{
  protected:
    typedef X86IntelMPConfigTableParams Params;

    static const char signature[];

    uint8_t specRev;
    std::string oemID;
    std::string productID;
    uint32_t oemTableAddr;
    uint16_t oemTableSize;
    uint32_t localApic;

    std::vector<BaseConfigEntry *> baseEntries;
    std::vector<ExtConfigEntry *> extEntries;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr);

    ConfigTable(const Params &p);
};

class Processor : public BaseConfigEntry
{
  protected:
    typedef X86IntelMPProcessorParams Params;

    uint8_t localApicID;
    uint8_t localApicVersion;
    uint8_t cpuFlags;
    uint32_t cpuSignature;
    uint32_t featureFlags;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    Processor(const Params &p);
};

class Bus : public BaseConfigEntry
{
  protected:
    typedef X86IntelMPBusParams Params;

    uint8_t busID;
    std::string busType;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    Bus(const Params &p);
};

class IOAPIC : public BaseConfigEntry
{
  protected:
    typedef X86IntelMPIOAPICParams Params;

    uint8_t id;
    uint8_t version;
    uint8_t flags;
    uint32_t address;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    IOAPIC(const Params &p);
};

class IntAssignment : public BaseConfigEntry
{
  protected:
    uint8_t interruptType;

    uint16_t flags;

    uint8_t sourceBusID;
    uint8_t sourceBusIRQ;

    uint8_t destApicID;
    uint8_t destApicIntIn;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    IntAssignment(const X86IntelMPBaseConfigEntryParams &p,
            enums::X86IntelMPInterruptType _interruptType,
            enums::X86IntelMPPolarity polarity,
            enums::X86IntelMPTriggerMode trigger,
            uint8_t _type,
            uint8_t _sourceBusID, uint8_t _sourceBusIRQ,
            uint8_t _destApicID, uint8_t _destApicIntIn) :
        BaseConfigEntry(p, _type),
        interruptType(_interruptType), flags(0),
        sourceBusID(_sourceBusID), sourceBusIRQ(_sourceBusIRQ),
        destApicID(_destApicID), destApicIntIn(_destApicIntIn)
    {
        replaceBits(flags, 1, 0, polarity);
        replaceBits(flags, 3, 2, trigger);
    }
};

class IOIntAssignment : public IntAssignment
{
  protected:
    typedef X86IntelMPIOIntAssignmentParams Params;

  public:
    IOIntAssignment(const Params &p);
};

class LocalIntAssignment : public IntAssignment
{
  protected:
    typedef X86IntelMPLocalIntAssignmentParams Params;

  public:
    LocalIntAssignment(const Params &p);
};

class AddrSpaceMapping : public ExtConfigEntry
{
  protected:
    typedef X86IntelMPAddrSpaceMappingParams Params;

    uint8_t busID;
    uint8_t addrType;
    uint64_t addr;
    uint64_t addrLength;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    AddrSpaceMapping(const Params &p);
};

class BusHierarchy : public ExtConfigEntry
{
  protected:
    typedef X86IntelMPBusHierarchyParams Params;

    uint8_t busID;
    uint8_t info;
    uint8_t parentBus;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    BusHierarchy(const Params &p);
};

class CompatAddrSpaceMod : public ExtConfigEntry
{
  protected:
    typedef X86IntelMPCompatAddrSpaceModParams Params;

    uint8_t busID;
    uint8_t mod;
    uint32_t rangeList;

  public:
    Addr writeOut(PortProxy& proxy, Addr addr, uint8_t &checkSum);

    CompatAddrSpaceMod(const Params &p);
};

} // namespace intelmp
} // namespace X86ISA
} // namespace gem5

#endif
