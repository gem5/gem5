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

#ifndef __ARCH_X86_INSTS_MICROOP_HH__
#define __ARCH_X86_INSTS_MICROOP_HH__

#include "arch/x86/insts/static_inst.hh"
#include "arch/x86/pcstate.hh"
#include "base/compiler.hh"

namespace gem5
{

namespace X86ISA
{

namespace condition_tests
{

enum CondTest
{
    True,
    NotFalse = True,
    ECF,
    EZF,
    SZnZF,
    MSTRZ,
    STRZ,
    MSTRC,
    STRZnEZF,
    OF,
    CF,
    ZF,
    CvZF,
    SF,
    PF,
    SxOF,
    SxOvZF,

    False,
    NotTrue = False,
    NotECF,
    NotEZF,
    NotSZnZF,
    NotMSTRZ,
    NotSTRZ,
    NotMSTRC,
    STRnZnEZF,
    NotOF,
    NotCF,
    NotZF,
    NotCvZF,
    NotSF,
    NotPF,
    NotSxOF,
    NotSxOvZF
};

}

// A class which is the base of all x86 micro ops. It provides a function to
// set necessary flags appropriately.
class X86MicroopBase : public X86StaticInst
{
  protected:
    const char *instMnem;
    uint8_t opSize;
    uint8_t addrSize;

    X86MicroopBase(ExtMachInst _machInst, const char *mnem,
                   const char *_instMnem, uint64_t setFlags, OpClass __opClass)
        : X86ISA::X86StaticInst(mnem, _machInst, __opClass),
          instMnem(_instMnem)
    {
        const int ChunkSize = sizeof(unsigned long);
        const int Chunks = sizeof(setFlags) / ChunkSize;

        // Since the bitset constructor can only handle unsigned long
        // sized chunks, feed it those one at a time while oring them in.
        for (int i = 0; i < Chunks; i++) {
            unsigned shift = i * ChunkSize * 8;
            flags |= (std::bitset<Num_Flags>(setFlags >> shift) << shift);
        }
    }

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override
    {
        std::stringstream ss;

        ccprintf(ss, "\t%s.%s", instMnem, mnemonic);

        return ss.str();
    }

    bool checkCondition(uint64_t flags, int condition) const;

    void
    advancePC(PCStateBase &pcState) const override
    {
        auto &xpc = pcState.as<PCState>();
        if (flags[IsLastMicroop])
            xpc.uEnd();
        else
            xpc.uAdvance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        if (flags[IsLastMicroop])
            pc.uEnd();
        else
            pc.uAdvance();
        tc->pcState(pc);
    }

    std::unique_ptr<PCStateBase>
    branchTarget(const PCStateBase &branch_pc) const override;

    // Explicitly import the otherwise hidden branchTarget.
    using StaticInst::branchTarget;
};

class MicroCondBase : public X86MicroopBase
{
  protected:
    uint8_t cc;

  public:
    MicroCondBase(ExtMachInst mach_inst, const char *mnem,
                  const char *inst_mnem, uint64_t set_flags, OpClass op_class,
                  uint8_t _cc)
        : X86MicroopBase(mach_inst, mnem, inst_mnem, set_flags, op_class),
          cc(_cc)
    {}
};

} // namespace X86ISA
} // namespace gem5

#endif //__ARCH_X86_INSTS_MICROOP_HH__
