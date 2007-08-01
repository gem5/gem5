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
 * Authors: Gabe Black
 */

#ifndef __ARCH_SPARC_UTILITY_HH__
#define __ARCH_SPARC_UTILITY_HH__

#include "arch/sparc/faults.hh"
#include "arch/sparc/isa_traits.hh"
#include "arch/sparc/tlb.hh"
#include "base/misc.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"

namespace SparcISA
{


    uint64_t getArgument(ThreadContext *tc, int number, bool fp);

    static inline bool
    inUserMode(ThreadContext *tc)
    {
        return !(tc->readMiscRegNoEffect(MISCREG_PSTATE & (1 << 2)) ||
                tc->readMiscRegNoEffect(MISCREG_HPSTATE & (1 << 2)));
    }

    inline bool isCallerSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    inline bool isCalleeSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    inline bool isCallerSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    inline bool isCalleeSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    // Instruction address compression hooks
    inline Addr realPCToFetchPC(const Addr &addr)
    {
        return addr;
    }

    inline Addr fetchPCToRealPC(const Addr &addr)
    {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    inline size_t fetchInstSize()
    {
        return sizeof(MachInst);
    }

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param tc The thread context.
     */
    template <class TC>
    void zeroRegisters(TC *tc);

    inline void initCPU(ThreadContext *tc, int cpuId)
    {
        static Fault por = new PowerOnReset();
        if (cpuId == 0)
            por->invoke(tc);

    }

    inline void startupCPU(ThreadContext *tc, int cpuId)
    {
#if FULL_SYSTEM
        // Other CPUs will get activated by IPIs
        if (cpuId == 0)
            tc->activate(0);
#else
        tc->activate(0);
#endif
    }

} // namespace SparcISA

#endif
