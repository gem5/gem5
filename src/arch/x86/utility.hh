/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#ifndef __ARCH_X86_UTILITY_HH__
#define __ARCH_X86_UTILITY_HH__

#include "arch/x86/types.hh"
#include "base/hashmap.hh"
#include "base/misc.hh"
#include "cpu/thread_context.hh"
#include "sim/host.hh"

class ThreadContext;

namespace __hash_namespace {
    template<>
    struct hash<X86ISA::ExtMachInst> {
        size_t operator()(const X86ISA::ExtMachInst &emi) const {
            return (((uint64_t)emi.legacy << 56) |
                    ((uint64_t)emi.rex  << 48) |
                    ((uint64_t)emi.modRM << 40) |
                    ((uint64_t)emi.sib << 32) |
                    ((uint64_t)emi.opcode.num << 24) |
                    ((uint64_t)emi.opcode.prefixA << 16) |
                    ((uint64_t)emi.opcode.prefixB << 8) |
                    ((uint64_t)emi.opcode.op)) ^
                    emi.immediate ^ emi.displacement;
        };
    };
}

namespace X86ISA
{
    static inline bool
    inUserMode(ThreadContext *tc)
    {
        return false;
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
        panic("initCPU not implemented!\n");
    }

    inline void startupCPU(ThreadContext *tc, int cpuId)
    {
        tc->activate(0);
    }
};

#endif // __ARCH_X86_UTILITY_HH__
