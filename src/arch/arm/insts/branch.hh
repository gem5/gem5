/*
 * Copyright (c) 2010,2018 ARM Limited
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
 */

#ifndef __ARCH_ARM_INSTS_BRANCH_HH__
#define __ARCH_ARM_INSTS_BRANCH_HH__

#include "arch/arm/insts/pred_inst.hh"

namespace ArmISA
{
// Branch to a target computed with an immediate
class BranchImm : public PredOp
{
  protected:
    int32_t imm;

  public:
    BranchImm(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              int32_t _imm) :
        PredOp(mnem, _machInst, __opClass), imm(_imm)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const;
};

// Conditionally Branch to a target computed with an immediate
class BranchImmCond : public BranchImm
{
  public:
    BranchImmCond(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  int32_t _imm, ConditionCode _condCode) :
        BranchImm(mnem, _machInst, __opClass, _imm)
    {
        // Only update if this isn't part of an IT block
        if (!machInst.itstateMask)
            condCode = _condCode;
    }
};

// Branch to a target computed with a register
class BranchReg : public PredOp
{
  protected:
    IntRegIndex op1;

  public:
    BranchReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _op1) :
        PredOp(mnem, _machInst, __opClass), op1(_op1)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const;
};

// Conditionally Branch to a target computed with a register
class BranchRegCond : public BranchReg
{
  public:
    BranchRegCond(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  IntRegIndex _op1, ConditionCode _condCode) :
        BranchReg(mnem, _machInst, __opClass, _op1)
    {
        // Only update if this isn't part of an IT block
        if (!machInst.itstateMask)
            condCode = _condCode;
    }
};

// Branch to a target computed with two registers
class BranchRegReg : public PredOp
{
  protected:
    IntRegIndex op1;
    IntRegIndex op2;

  public:
    BranchRegReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _op1, IntRegIndex _op2) :
        PredOp(mnem, _machInst, __opClass), op1(_op1), op2(_op2)
    {}

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const;
};

// Branch to a target computed with an immediate and a register
class BranchImmReg : public PredOp
{
  protected:
    int32_t imm;
    IntRegIndex op1;

  public:
    BranchImmReg(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 int32_t _imm, IntRegIndex _op1) :
        PredOp(mnem, _machInst, __opClass), imm(_imm), op1(_op1)
    {}
};

}

#endif //__ARCH_ARM_INSTS_BRANCH_HH__
