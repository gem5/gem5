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

#ifndef __ARCH_X86_BIOS_ACPI_HH__
#define __ARCH_X86_BIOS_ACPI_HH__

#include <string>
#include <vector>

#include "base/types.hh"
#include "sim/sim_object.hh"

class Port;

class X86ACPIRSDPParams;

class X86ACPISysDescTableParams;
class X86ACPIRSDTParams;
class X86ACPIXSDTParams;

namespace X86ISA
{

namespace ACPI
{

class RSDT;
class XSDT;
class SysDescTable;

class RSDP : public SimObject
{
  protected:
    typedef X86ACPIRSDPParams Params;

    static const char signature[];

    std::string oemID;
    uint8_t revision;

    RSDT * rsdt;
    XSDT * xsdt;

  public:
    RSDP(Params *p);
};

class SysDescTable : public SimObject
{
  protected:
    typedef X86ACPISysDescTableParams Params;

    const char * signature;
    uint8_t revision;

    std::string oemID;
    std::string oemTableID;
    uint32_t oemRevision;

    std::string creatorID;
    uint32_t creatorRevision;

  public:
    SysDescTable(Params *p, const char * _signature, uint8_t _revision);
};

class RSDT : public SysDescTable
{
  protected:
    typedef X86ACPIRSDTParams Params;

    std::vector<SysDescTable *> entries;

  public:
    RSDT(Params *p);
};

class XSDT : public SysDescTable
{
  protected:
    typedef X86ACPIXSDTParams Params;

    std::vector<SysDescTable *> entries;

  public:
    XSDT(Params *p);
};

} // namespace ACPI

} // namespace X86ISA

#endif // __ARCH_X86_BIOS_E820_HH__
