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

struct X86ACPIRSDPParams;

struct X86ACPISysDescTableParams;
struct X86ACPIRSDTParams;
struct X86ACPIXSDTParams;

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
