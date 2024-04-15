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

#ifndef __ARCH_X86_INSTS_STATICINST_HH__
#define __ARCH_X86_INSTS_STATICINST_HH__

#include "arch/x86/pcstate.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/types.hh"
#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/X86.hh"

namespace gem5
{

namespace X86ISA
{

/**
 * Classes for register indices passed to instruction constructors. Using a
 * wrapper struct for these lets take advantage of the compiler's type
 * checking.
 */
struct GpRegIndex
{
    RegIndex index;

    explicit GpRegIndex(RegIndex idx) : index(idx) {}
};

struct FpRegIndex
{
    RegIndex index;

    explicit FpRegIndex(RegIndex idx) : index(idx) {}
};

struct CtrlRegIndex
{
    RegIndex index;

    explicit CtrlRegIndex(RegIndex idx) : index(idx) {}
};

struct CrRegIndex
{
    RegIndex index;

    explicit CrRegIndex(RegIndex idx) : index(idx) {}
};

struct DbgRegIndex
{
    RegIndex index;

    explicit DbgRegIndex(RegIndex idx) : index(idx) {}
};

struct SegRegIndex
{
    RegIndex index;

    explicit SegRegIndex(RegIndex idx) : index(idx) {}
};

/**
 * Base class for all X86 static instructions.
 */

class X86StaticInst : public StaticInst
{
  public:
    static void printMnemonic(std::ostream &os, const char *mnemonic);
    static void printMnemonic(std::ostream &os, const char *instMnemonic,
                              const char *mnemonic);
    static void printMem(std::ostream &os, uint8_t segment, uint8_t scale,
                         RegIndex index, RegIndex base, uint64_t disp,
                         uint8_t addressSize, bool rip);

    static void printSegment(std::ostream &os, int segment);

    static void printReg(std::ostream &os, RegId reg, int size);

  protected:
    using ExtMachInst = X86ISA::ExtMachInst;

  public:
    ExtMachInst machInst;

  protected:
    // Constructor.
    X86StaticInst(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : StaticInst(mnem, __opClass), machInst(_machInst)
    {}

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;

    static void divideStep(uint64_t divident, uint64_t divisor,
                           uint64_t &quotient, uint64_t &remainder);

    static inline uint64_t
    merge(uint64_t into, RegIndex index, uint64_t val, int size)
    {
        X86IntReg reg = into;
        if (index & IntFoldBit) {
            reg.H = val;
            return reg;
        }
        switch (size) {
        case 1:
            reg.L = val;
            break;
        case 2:
            reg.X = val;
            break;
        case 4:
            // XXX Check if this should be zeroed or sign extended
            reg = 0;
            reg.E = val;
            break;
        case 8:
            reg.R = val;
            break;
        default:
            panic("Tried to merge with unrecognized size %d.\n", size);
        }
        return reg;
    }

    static inline uint64_t
    pick(uint64_t from, RegIndex index, int size)
    {
        X86IntReg reg = from;
        DPRINTF(X86, "Picking with size %d\n", size);
        if (index & IntFoldBit)
            return reg.H;
        switch (size) {
        case 1:
            return reg.L;
        case 2:
            return reg.X;
        case 4:
            return reg.E;
        case 8:
            return reg.R;
        default:
            panic("Tried to pick with unrecognized size %d.\n", size);
        }
    }

    static inline int64_t
    signedPick(uint64_t from, RegIndex index, int size)
    {
        X86IntReg reg = from;
        DPRINTF(X86, "Picking with size %d\n", size);
        if (index & IntFoldBit)
            return reg.SH;
        switch (size) {
        case 1:
            return reg.SL;
        case 2:
            return reg.SX;
        case 4:
            return reg.SE;
        case 8:
            return reg.SR;
        default:
            panic("Tried to pick with unrecognized size %d.\n", size);
        }
    }

    void
    advancePC(PCStateBase &pcState) const override
    {
        pcState.as<PCState>().advance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        pc.advance();
        tc->pcState(pc);
    }

    std::unique_ptr<PCStateBase>
    buildRetPC(const PCStateBase &cur_pc,
               const PCStateBase &call_pc) const override
    {
        PCStateBase *ret_pc_ptr = call_pc.clone();
        ret_pc_ptr->as<PCState>().uEnd();
        return std::unique_ptr<PCStateBase>{ ret_pc_ptr };
    }
};

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_INSTS_STATICINST_HH__
