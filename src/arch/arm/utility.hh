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

#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "base/hashmap.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"

namespace __hash_namespace {
    template<>
    struct hash<ArmISA::ExtMachInst> : public hash<uint32_t> {
        size_t operator()(const ArmISA::ExtMachInst &emi) const {
            return hash<uint32_t>::operator()((uint32_t)emi);
        };
    };
}

namespace ArmISA {

    inline bool
    testPredicate(CPSR cpsr, ConditionCode code)
    {
        switch (code)
        {
            case COND_EQ: return  cpsr.z;
            case COND_NE: return !cpsr.z;
            case COND_CS: return  cpsr.c;
            case COND_CC: return !cpsr.c;
            case COND_MI: return  cpsr.n;
            case COND_PL: return !cpsr.n;
            case COND_VS: return  cpsr.v;
            case COND_VC: return !cpsr.v;
            case COND_HI: return  (cpsr.c && !cpsr.z);
            case COND_LS: return !(cpsr.c && !cpsr.z);
            case COND_GE: return !(cpsr.n ^ cpsr.v);
            case COND_LT: return  (cpsr.n ^ cpsr.v);
            case COND_GT: return !(cpsr.n ^ cpsr.v || cpsr.z);
            case COND_LE: return  (cpsr.n ^ cpsr.v || cpsr.z);
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
        tc->activate(0);
    }

    template <class XC>
    Fault
    checkFpEnableFault(XC *xc)
    {
        return NoFault;
    }

    static inline void
    copyRegs(ThreadContext *src, ThreadContext *dest)
    {
        panic("Copy Regs Not Implemented Yet\n");
    }

    static inline void
    copyMiscRegs(ThreadContext *src, ThreadContext *dest)
    {
        panic("Copy Misc. Regs Not Implemented Yet\n");
    }

    void initCPU(ThreadContext *tc, int cpuId);
    
    static inline bool
    inUserMode(ThreadContext *tc)
    {
        return (tc->readMiscRegNoEffect(MISCREG_CPSR) & 0x1f) == MODE_USER;
    }

uint64_t getArgument(ThreadContext *tc, int number, bool fp);
    
Fault setCp15Register(uint32_t &Rd, int CRn, int opc1, int CRm, int opc2);
Fault readCp15Register(uint32_t &Rd, int CRn, int opc1, int CRm, int opc2);

};


#endif
