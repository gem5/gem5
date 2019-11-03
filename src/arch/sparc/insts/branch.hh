/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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
 *          Steve Reinhardt
 */

#ifndef __ARCH_SPARC_INSTS_BRANCH_HH__
#define __ARCH_SPARC_INSTS_BRANCH_HH__

#include "arch/sparc/insts/static_inst.hh"

////////////////////////////////////////////////////////////////////
//
// Branch instructions
//

namespace SparcISA
{

/**
 * Base class for branch operations.
 */
class Branch : public SparcStaticInst
{
  protected:
    using SparcStaticInst::SparcStaticInst;

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;
};

/**
 * Base class for branch operations with an immediate displacement.
 */
class BranchDisp : public Branch
{
  protected:
    BranchDisp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               int32_t _disp) :
        Branch(mnem, _machInst, __opClass), disp(_disp)
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;

    int32_t disp;
};

/**
 * Base class for branches with n bit displacements.
 */
template<int bits>
class BranchNBits : public BranchDisp
{
  protected:
    // Constructor
    BranchNBits(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        BranchDisp(mnem, _machInst, __opClass,
                   sext<bits + 2>((_machInst & mask(bits)) << 2))
    {}
};

/**
 * Base class for 16bit split displacements.
 */
class BranchSplit : public BranchDisp
{
  protected:
    // Constructor
    BranchSplit(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        BranchDisp(mnem, _machInst, __opClass,
                   sext<18>((bits(_machInst, 21, 20) << 16) |
                            (bits(_machInst, 13, 0) << 2)))
    {}
};

/**
 * Base class for branches that use an immediate and a register to
 * compute their displacements.
 */
class BranchImm13 : public Branch
{
  protected:
    // Constructor
    BranchImm13(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        Branch(mnem, _machInst, __opClass),
        imm(sext<13>(bits(_machInst, 12, 0)))
    {}

    std::string generateDisassembly(
            Addr pc, const SymbolTable *symtab) const override;

    int32_t imm;
};

}

#endif // __ARCH_SPARC_INSTS_BRANCH_HH__
