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
 */

#ifndef __ARCH_ALPHA_UTILITY_HH__
#define __ARCH_ALPHA_UTILITY_HH__

#include "config/full_system.hh"
#include "arch/alpha/types.hh"
#include "arch/alpha/constants.hh"
#include "arch/alpha/regfile.hh"
#include "base/misc.hh"

namespace AlphaISA
{

    static inline ExtMachInst
    makeExtMI(MachInst inst, const uint64_t &pc) {
#if FULL_SYSTEM
        ExtMachInst ext_inst = inst;
        if (pc && 0x1)
            return ext_inst|=(static_cast<ExtMachInst>(pc & 0x1) << 32);
        else
            return ext_inst;
#else
        return ExtMachInst(inst);
#endif
    }

    static inline bool isCallerSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 1 && reg <= 8 || reg >= 22 && reg <= 25 || reg == 27);
    }

    static inline bool isCalleeSaveIntegerRegister(unsigned int reg) {
        panic("register classification not implemented");
        return (reg >= 9 && reg <= 15);
    }

    static inline bool isCallerSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline bool isCalleeSaveFloatRegister(unsigned int reg) {
        panic("register classification not implemented");
        return false;
    }

    static inline Addr alignAddress(const Addr &addr,
                                         unsigned int nbytes) {
        return (addr & ~(nbytes - 1));
    }

    // Instruction address compression hooks
    static inline Addr realPCToFetchPC(const Addr &addr) {
        return addr;
    }

    static inline Addr fetchPCToRealPC(const Addr &addr) {
        return addr;
    }

    // the size of "fetched" instructions (not necessarily the size
    // of real instructions for PISA)
    static inline size_t fetchInstSize() {
        return sizeof(MachInst);
    }

    static inline MachInst makeRegisterCopy(int dest, int src) {
        panic("makeRegisterCopy not implemented");
        return 0;
    }

    // Machine operations

    void saveMachineReg(AnyReg &savereg, const RegFile &reg_file,
                               int regnum);

    void restoreMachineReg(RegFile &regs, const AnyReg &reg,
                                  int regnum);

    /**
     * Function to insure ISA semantics about 0 registers.
     * @param xc The execution context.
     */
    template <class XC>
    void zeroRegisters(XC *xc);

#if FULL_SYSTEM
    // Alpha IPR register accessors
    static inline bool PcPAL(Addr addr) { return addr & 0x1; }

    ////////////////////////////////////////////////////////////////////////
    //
    //  Translation stuff
    //

    static inline Addr PteAddr(Addr a) { return (a & PteMask) << PteShift; }

    // User Virtual
    static inline bool IsUSeg(Addr a) { return USegBase <= a && a <= USegEnd; }

    // Kernel Direct Mapped
    extern inline bool IsK0Seg(Addr a) { return K0SegBase <= a && a <= K0SegEnd; }
    static inline Addr K0Seg2Phys(Addr addr) { return addr & ~K0SegBase; }

    // Kernel Virtual
    static inline bool IsK1Seg(Addr a) { return K1SegBase <= a && a <= K1SegEnd; }

    static inline Addr
    TruncPage(Addr addr)
    { return addr & ~(PageBytes - 1); }

    static inline Addr
    RoundPage(Addr addr)
    { return (addr + PageBytes - 1) & ~(PageBytes - 1); }

    void initCPU(ExecContext *xc, int cpuId);
    void initIPRs(ExecContext *xc, int cpuId);

    /**
     * Function to check for and process any interrupts.
     * @param xc The execution context.
     */
    template <class XC>
    void processInterrupts(XC *xc);
#endif

} // namespace AlphaISA

#endif
