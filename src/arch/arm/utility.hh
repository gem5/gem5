/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Korey Sewell
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_UTILITY_HH__
#define __ARCH_ARM_UTILITY_HH__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

namespace ArmISA {

inline PCState
buildRetPC(const PCState &curPC, const PCState &callPC)
{
    PCState retPC = callPC;
    retPC.uEnd();
    return retPC;
}

inline bool
testPredicate(uint32_t nz, uint32_t c, uint32_t v, ConditionCode code)
{
    bool n = (nz & 0x2);
    bool z = (nz & 0x1);

    switch (code)
    {
        case COND_EQ: return  z;
        case COND_NE: return !z;
        case COND_CS: return  c;
        case COND_CC: return !c;
        case COND_MI: return  n;
        case COND_PL: return !n;
        case COND_VS: return  v;
        case COND_VC: return !v;
        case COND_HI: return  (c && !z);
        case COND_LS: return !(c && !z);
        case COND_GE: return !(n ^ v);
        case COND_LT: return  (n ^ v);
        case COND_GT: return !(n ^ v || z);
        case COND_LE: return  (n ^ v || z);
        case COND_AL: return true;
        case COND_UC: return true;
        default:
            panic("Unhandled predicate condition: %d\n", code);
    }
}

/**
 * Function to insure ISA semantics about 0 registers.
 * @param tc The thread context.
 */
template <class TC>
void zeroRegisters(TC *tc);

inline void startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate(Cycles(0));
}

void copyRegs(ThreadContext *src, ThreadContext *dest);

static inline void
copyMiscRegs(ThreadContext *src, ThreadContext *dest)
{
    panic("Copy Misc. Regs Not Implemented Yet\n");
}

void initCPU(ThreadContext *tc, int cpuId);

static inline bool
inUserMode(CPSR cpsr)
{
    return cpsr.mode == MODE_USER;
}

static inline bool
inUserMode(ThreadContext *tc)
{
    return inUserMode(tc->readMiscRegNoEffect(MISCREG_CPSR));
}

static inline bool
inPrivilegedMode(CPSR cpsr)
{
    return !inUserMode(cpsr);
}

static inline bool
inPrivilegedMode(ThreadContext *tc)
{
    return !inUserMode(tc);
}

static inline bool
vfpEnabled(CPACR cpacr, CPSR cpsr)
{
    return cpacr.cp10 == 0x3 ||
        (cpacr.cp10 == 0x1 && inPrivilegedMode(cpsr));
}

static inline bool
vfpEnabled(CPACR cpacr, CPSR cpsr, FPEXC fpexc)
{
    if ((cpacr.cp11 == 0x3) ||
        ((cpacr.cp11 == 0x1) && inPrivilegedMode(cpsr)))
        return fpexc.en && vfpEnabled(cpacr, cpsr);
    else
        return fpexc.en && vfpEnabled(cpacr, cpsr) &&
            (cpacr.cp11 == cpacr.cp10);
}

static inline bool
neonEnabled(CPACR cpacr, CPSR cpsr, FPEXC fpexc)
{
    return !cpacr.asedis && vfpEnabled(cpacr, cpsr, fpexc);
}

uint64_t getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp);

void skipFunction(ThreadContext *tc);

inline void
advancePC(PCState &pc, const StaticInstPtr inst)
{
    inst->advancePC(pc);
}

Addr truncPage(Addr addr);
Addr roundPage(Addr addr);

inline uint64_t
getExecutingAsid(ThreadContext *tc)
{
    return tc->readMiscReg(MISCREG_CONTEXTIDR);
}

}

#endif
