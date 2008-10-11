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

#ifndef __ARCH_X86_BIOS_INTELMP_HH__
#define __ARCH_X86_BIOS_INTELMP_HH__

#include <string>
#include <vector>

#include "base/bitfield.hh"
#include "sim/sim_object.hh"

#include "enums/X86IntelMPAddressType.hh"
#include "enums/X86IntelMPInterruptType.hh"
#include "enums/X86IntelMPPolarity.hh"
#include "enums/X86IntelMPRangeList.hh"
#include "enums/X86IntelMPTriggerMode.hh"

class FunctionalPort;

// Config entry types
class X86IntelMPBaseConfigEntryParams;
class X86IntelMPExtConfigEntryParams;

// General table structures
class X86IntelMPConfigTableParams;
class X86IntelMPFloatingPointerParams;

// Base entry types
class X86IntelMPBusParams;
class X86IntelMPIOAPICParams;
class X86IntelMPIOIntAssignmentParams;
class X86IntelMPLocalIntAssignmentParams;
class X86IntelMPProcessorParams;

// Extended entry types
class X86IntelMPAddrSpaceMappingParams;
class X86IntelMPBusHierarchyParams;
class X86IntelMPCompatAddrSpaceModParams;

namespace X86ISA
{

namespace IntelMP
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

    Addr writeOut(FunctionalPort * port, Addr addr);

    Addr getTableAddr()
    {
        return tableAddr;
    }

    void setTableAddr(Addr addr)
    {
        tableAddr = addr;
    }

    FloatingPointer(Params * p);
};

class BaseConfigEntry : public SimObject
{
  protected:
    typedef X86IntelMPBaseConfigEntryParams Params;

    uint8_t type;

  public:

    virtual Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    BaseConfigEntry(Params * p, uint8_t _type);
};

class ExtConfigEntry : public SimObject
{
  protected:
    typedef X86IntelMPExtConfigEntryParams Params;

    uint8_t type;
    uint8_t length;

  public:

    virtual Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    ExtConfigEntry(Params * p, uint8_t _type, uint8_t _length);
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
    Addr writeOut(FunctionalPort * port, Addr addr);

    ConfigTable(Params * p);
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
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    Processor(Params * p);
};

class Bus : public BaseConfigEntry
{
  protected:
    typedef X86IntelMPBusParams Params;

    uint8_t busID;
    std::string busType;

  public:
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    Bus(Params * p);
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
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    IOAPIC(Params * p);
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
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    IntAssignment(X86IntelMPBaseConfigEntryParams * p,
            Enums::X86IntelMPInterruptType _interruptType,
            Enums::X86IntelMPPolarity polarity,
            Enums::X86IntelMPTriggerMode trigger,
            uint8_t _type,
            uint8_t _sourceBusID, uint8_t _sourceBusIRQ,
            uint8_t _destApicID, uint8_t _destApicIntIn) :
        BaseConfigEntry(p, _type),
        interruptType(_interruptType), flags(0),
        sourceBusID(_sourceBusID), sourceBusIRQ(_sourceBusIRQ),
        destApicID(_destApicID), destApicIntIn(_destApicIntIn)
    {
        replaceBits(flags, 0, 1, polarity);
        replaceBits(flags, 2, 3, trigger);
    }
};

class IOIntAssignment : public IntAssignment
{
  protected:
    typedef X86IntelMPIOIntAssignmentParams Params;

  public:
    IOIntAssignment(Params * p);
};

class LocalIntAssignment : public IntAssignment
{
  protected:
    typedef X86IntelMPLocalIntAssignmentParams Params;

  public:
    LocalIntAssignment(Params * p);
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
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    AddrSpaceMapping(Params * p);
};

class BusHierarchy : public ExtConfigEntry
{
  protected:
    typedef X86IntelMPBusHierarchyParams Params;

    uint8_t busID;
    uint8_t info;
    uint8_t parentBus;

  public:
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    BusHierarchy(Params * p);
};

class CompatAddrSpaceMod : public ExtConfigEntry
{
  protected:
    typedef X86IntelMPCompatAddrSpaceModParams Params;

    uint8_t busID;
    uint8_t mod;
    uint32_t rangeList;

  public:
    Addr writeOut(FunctionalPort * port, Addr addr, uint8_t &checkSum);

    CompatAddrSpaceMod(Params * p);
};

} //IntelMP

} //X86ISA

#endif
