/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * All rights reserved.
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#ifndef __ARCH_ALPHA_UTILITY_HH__
#define __ARCH_ALPHA_UTILITY_HH__

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/registers.hh"
#include "arch/alpha/types.hh"
#include "base/misc.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "arch/alpha/ev5.hh"

namespace AlphaISA {

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.advance();
    return retPC;
}

uint64_t getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

inline bool
inUserMode(ThreadContext *tc)
{
    return (tc->readMiscRegNoEffect(IPR_DTB_CM) & 0x18) != 0;
}

/**
 * Function to insure ISA semantics about 0 registers.
 * @param tc The thread context.
 */
template <class TC>
void zeroRegisters(TC *tc);

// Alpha IPR register accessors
inline bool PcPAL(Addr addr) { return addr & 0x3; }
inline void startupCPU(ThreadContext *tc, int cpuId)
{ tc->activate(); }

////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//

inline Addr PteAddr(Addr a) { return (a & PteMask) << PteShift; }

// User Virtual
inline bool IsUSeg(Addr a) { assert(USegBase == 0); return a <= USegEnd; }

// Kernel Direct Mapped
inline bool IsK0Seg(Addr a) { return K0SegBase <= a && a <= K0SegEnd; }
inline Addr K0Seg2Phys(Addr addr) { return addr & ~K0SegBase; }

// Kernel Virtual
inline bool IsK1Seg(Addr a) { return K1SegBase <= a && a <= K1SegEnd; }

inline Addr
TruncPage(Addr addr)
{ return addr & ~(PageBytes - 1); }

inline Addr
RoundPage(Addr addr)
{ return (addr + PageBytes - 1) & ~(PageBytes - 1); }

void initIPRs(ThreadContext *tc, int cpuId);
void initCPU(ThreadContext *tc, int cpuId);

void copyRegs(ThreadContext *src, ThreadContext *dest);

void copyMiscRegs(ThreadContext *src, ThreadContext *dest);

void skipFunction(ThreadContext *tc);

inline void
advancePC(PCState &pc, const StaticInstPtr &inst)
{
    pc.advance();
}

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return DTB_ASN_ASN(tc->readMiscRegNoEffect(IPR_DTB_ASN));
}

} // namespace AlphaISA

#endif // __ARCH_ALPHA_UTILITY_HH__
