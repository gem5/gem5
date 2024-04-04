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
 */

#ifndef __ARCH_X86_X86TRAITS_HH__
#define __ARCH_X86_X86TRAITS_HH__

#include <cassert>

#include "base/types.hh"

namespace gem5
{

namespace X86ISA
{
    const int NumMicroIntRegs = 16;

    const int NumMMXRegs = 8;
    const int NumXMMRegs = 16;
    const int NumMicroFpRegs = 8;

    const int NumCRegs = 16;
    const int NumDRegs = 8;
    const int NumXCRegs = 1;

    const int NumSegments = 6;
    const int NumSysSegments = 4;

    const Addr IntAddrPrefixMask = 0xffffffff00000000ULL;
    const Addr IntAddrPrefixCPUID = 0x100000000ULL;
    const Addr IntAddrPrefixMSR = 0x200000000ULL;
    const Addr IntAddrPrefixIO = 0x300000000ULL;

    const Addr PhysAddrPrefixIO = 0x8000000000000000ULL;
    const Addr PhysAddrPrefixPciConfig = 0xC000000000000000ULL;
    const Addr PhysAddrPrefixLocalAPIC = 0x2000000000000000ULL;
    const Addr PhysAddrPrefixInterrupts = 0xA000000000000000ULL;
    // Each APIC gets two pages. One page is used for local apics to field
    // accesses from the CPU, and the other is for all APICs to communicate.
    const Addr PhysAddrAPICRangeSize = 1 << 12;

    // Put this in an unused part of the 16 bit IO port address space.
    const Addr PhysAddrIntA = 0x8000000100000000ULL;

    static inline Addr
    x86IOAddress(const uint32_t port)
    {
        return PhysAddrPrefixIO | port;
    }

    static inline Addr
    x86PciConfigAddress(const uint32_t addr)
    {
        return PhysAddrPrefixPciConfig | addr;
    }

    static inline Addr
    x86LocalAPICAddress(const uint8_t id, const uint16_t addr)
    {
        assert(addr < (1 << 12));
        return PhysAddrPrefixLocalAPIC | (id * (1 << 12)) | addr;
    }

    static inline Addr
    x86InterruptAddress(const uint8_t id, const uint16_t addr)
    {
        assert(addr < PhysAddrAPICRangeSize);
        return PhysAddrPrefixInterrupts | (id * PhysAddrAPICRangeSize) | addr;
    }

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_X86TRAITS_HH__
