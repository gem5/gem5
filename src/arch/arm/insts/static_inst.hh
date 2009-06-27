/* Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Stephen Hines
 */
#ifndef __ARCH_ARM_INSTS_STATICINST_HH__
#define __ARCH_ARM_INSTS_STATICINST_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"

namespace ArmISA
{
class ArmStaticInst : public StaticInst
{
  protected:
    int32_t shift_rm_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    int32_t shift_rm_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    bool shift_carry_imm(uint32_t base, uint32_t shamt,
                         uint32_t type, uint32_t cfval) const;
    bool shift_carry_rs(uint32_t base, uint32_t shamt,
                        uint32_t type, uint32_t cfval) const;

    bool arm_add_carry(int32_t result, int32_t lhs, int32_t rhs) const;
    bool arm_sub_carry(int32_t result, int32_t lhs, int32_t rhs) const;

    bool arm_add_overflow(int32_t result, int32_t lhs, int32_t rhs) const;
    bool arm_sub_overflow(int32_t result, int32_t lhs, int32_t rhs) const;

    // Constructor
    ArmStaticInst(const char *mnem, MachInst _machInst, OpClass __opClass)
        : StaticInst(mnem, _machInst, __opClass)
    {
    }

    /// Print a register name for disassembly given the unique
    /// dependence tag number (FP or int).
    void printReg(std::ostream &os, int reg) const;
    void printMnemonic(std::ostream &os,
                       const std::string &suffix = "",
                       bool withPred = true) const;

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};
}

#endif //__ARCH_ARM_INSTS_STATICINST_HH__
