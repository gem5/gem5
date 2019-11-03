/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#ifndef __ARCH_X86_SYSTEM_HH__
#define __ARCH_X86_SYSTEM_HH__

#include <string>
#include <vector>

#include "arch/x86/regs/misc.hh"
#include "params/X86System.hh"
#include "sim/system.hh"

namespace X86ISA
{
    namespace SMBios
    {
        class SMBiosTable;
    }
    namespace IntelMP
    {
        class FloatingPointer;
        class ConfigTable;
    }

    void installSegDesc(ThreadContext *tc, SegmentRegIndex seg,
                        SegDescriptor desc, bool longmode);

    /* memory mappings for KVMCpu in SE mode */
    const uint64_t syscallCodeVirtAddr = 0xffff800000000000;
    const uint64_t GDTVirtAddr = 0xffff800000001000;
    const uint64_t IDTVirtAddr = 0xffff800000002000;
    const uint64_t TSSVirtAddr = 0xffff800000003000;
    const uint64_t TSSPhysAddr = 0x63000;
    const uint64_t ISTVirtAddr = 0xffff800000004000;
    const uint64_t PFHandlerVirtAddr = 0xffff800000005000;
    const uint64_t MMIORegionVirtAddr = 0xffffc90000000000;
    const uint64_t MMIORegionPhysAddr = 0xffff0000;
}

class X86System : public System
{
  public:
    typedef X86SystemParams Params;
    X86System(Params *p);
    ~X86System();

/**
 * Serialization stuff
 */
  public:

    void initState();

  protected:

    X86ISA::SMBios::SMBiosTable * smbiosTable;
    X86ISA::IntelMP::FloatingPointer * mpFloatingPointer;
    X86ISA::IntelMP::ConfigTable * mpConfigTable;
    X86ISA::ACPI::RSDP * rsdp;

    void writeOutSMBiosTable(Addr header,
            Addr &headerSize, Addr &tableSize, Addr table = 0);

    void writeOutMPTable(Addr fp,
            Addr &fpSize, Addr &tableSize, Addr table = 0);

    const Params *params() const { return (const Params *)_params; }

    virtual Addr fixFuncEventAddr(Addr addr)
    {
        // XXX This may eventually have to do something useful.
        return addr;
    }
};

#endif

