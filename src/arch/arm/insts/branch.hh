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
#ifndef __ARCH_ARM_INSTS_BRANCH_HH__
#define __ARCH_ARM_INSTS_BRANCH_HH__

#include "arch/arm/insts/pred_inst.hh"

namespace ArmISA
{
/**
 * Base class for instructions whose disassembly is not purely a
 * function of the machine instruction (i.e., it depends on the
 * PC).  This class overrides the disassemble() method to check
 * the PC and symbol table values before re-using a cached
 * disassembly string.  This is necessary for branches and jumps,
 * where the disassembly string includes the target address (which
 * may depend on the PC and/or symbol table).
 */
class PCDependentDisassembly : public PredOp
{
  protected:
    /// Cached program counter from last disassembly
    mutable Addr cachedPC;

    /// Cached symbol table pointer from last disassembly
    mutable const SymbolTable *cachedSymtab;

    /// Constructor
    PCDependentDisassembly(const char *mnem, ExtMachInst _machInst,
                           OpClass __opClass)
        : PredOp(mnem, _machInst, __opClass),
          cachedPC(0), cachedSymtab(0)
    {
    }

    const std::string &
    disassemble(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for branches (PC-relative control transfers),
 * conditional or unconditional.
 */
class Branch : public PCDependentDisassembly
{
  protected:
    /// target address (signed) Displacement .
    int32_t disp;

    /// Constructor.
    Branch(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PCDependentDisassembly(mnem, _machInst, __opClass),
          disp(machInst.offset << 2)
    {
        //If Bit 26 is 1 then Sign Extend
        if ( (disp & 0x02000000) > 0  ) {
            disp |=  0xFC000000;
        }
    }

    Addr branchTarget(Addr branchPC) const;

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for branch and exchange instructions on the ARM
 */
class BranchExchange : public PredOp
{
  protected:
    /// Constructor
    BranchExchange(const char *mnem, ExtMachInst _machInst,
                           OpClass __opClass)
        : PredOp(mnem, _machInst, __opClass)
    {
    }

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};


/**
 * Base class for jumps (register-indirect control transfers).  In
 * the Arm ISA, these are always unconditional.
 */
class Jump : public PCDependentDisassembly
{
  protected:

    /// Displacement to target address (signed).
    int32_t disp;

    uint32_t target;

  public:
    /// Constructor
    Jump(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PCDependentDisassembly(mnem, _machInst, __opClass),
          disp(machInst.offset << 2)
    {
    }

    Addr branchTarget(ThreadContext *tc) const;

    std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};
}

#endif //__ARCH_ARM_INSTS_BRANCH_HH__
