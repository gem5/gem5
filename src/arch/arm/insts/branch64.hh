/*
 * Copyright (c) 2011-2013 ARM Limited
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
#ifndef __ARCH_ARM_INSTS_BRANCH64_HH__
#define __ARCH_ARM_INSTS_BRANCH64_HH__

#include "arch/arm/insts/static_inst.hh"

namespace ArmISA
{
// Branch to a target computed with an immediate
class BranchImm64 : public ArmStaticInst
{
  protected:
    int64_t imm;

  public:
    BranchImm64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                int64_t _imm) :
        ArmStaticInst(mnem, _machInst, __opClass), imm(_imm)
    {}

    ArmISA::PCState branchTarget(
            const ArmISA::PCState &branchPC) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Conditionally Branch to a target computed with an immediate
class BranchImmCond64 : public BranchImm64
{
  protected:
    ConditionCode condCode;

  public:
    BranchImmCond64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                    int64_t _imm, ConditionCode _condCode) :
        BranchImm64(mnem, _machInst, __opClass, _imm), condCode(_condCode)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Branch to a target computed with a register
class BranchReg64 : public ArmStaticInst
{
  protected:
    IntRegIndex op1;

  public:
    BranchReg64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _op1) :
        ArmStaticInst(mnem, _machInst, __opClass), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Ret instruction
class BranchRet64 : public BranchReg64
{
  public:
    BranchRet64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _op1) :
        BranchReg64(mnem, _machInst, __opClass, _op1)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Eret instruction
class BranchEret64 : public ArmStaticInst
{
  public:
    BranchEret64(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        ArmStaticInst(mnem, _machInst, __opClass)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Branch to a target computed with an immediate and a register
class BranchImmReg64 : public ArmStaticInst
{
  protected:
    int64_t imm;
    IntRegIndex op1;

  public:
    BranchImmReg64(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                   int64_t _imm, IntRegIndex _op1) :
        ArmStaticInst(mnem, _machInst, __opClass), imm(_imm), op1(_op1)
    {}

    ArmISA::PCState branchTarget(
            const ArmISA::PCState &branchPC) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

// Branch to a target computed with two immediates
class BranchImmImmReg64 : public ArmStaticInst
{
  protected:
    int64_t imm1;
    int64_t imm2;
    IntRegIndex op1;

  public:
    BranchImmImmReg64(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, int64_t _imm1, int64_t _imm2,
                      IntRegIndex _op1) :
        ArmStaticInst(mnem, _machInst, __opClass),
        imm1(_imm1), imm2(_imm2), op1(_op1)
    {}

    ArmISA::PCState branchTarget(
            const ArmISA::PCState &branchPC) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

}

#endif //__ARCH_ARM_INSTS_BRANCH_HH__
