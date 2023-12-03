/* Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#ifndef __ARCH_POWER_INSTS_BRANCH_HH__
#define __ARCH_POWER_INSTS_BRANCH_HH__

#include "arch/power/insts/static_inst.hh"

namespace gem5
{

namespace PowerISA
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
class PCDependentDisassembly : public PowerStaticInst
{
  protected:
    /// Cached program counter from last disassembly
    mutable Addr cachedPC;
    /// Cached symbol table pointer from last disassembly
    mutable const loader::SymbolTable *cachedSymtab;

    /// Constructor
    PCDependentDisassembly(const char *mnem, ExtMachInst _machInst,
                           OpClass __opClass)
        : PowerStaticInst(mnem, _machInst, __opClass),
          cachedPC(0),
          cachedSymtab(0)
    {}

    const std::string &disassemble(Addr pc,
                                   const loader::SymbolTable *symtab) const;
};

/**
 * Base class for unconditional, PC-relative or absolute address branches.
 */
class BranchOp : public PCDependentDisassembly
{
  protected:
    bool aa;
    bool lk;
    int64_t li;

    /// Constructor
    BranchOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : PCDependentDisassembly(mnem, _machInst, __opClass),
          aa(machInst.aa),
          lk(machInst.lk),
          li(sext<26>(machInst.li << 2))
    {}

    std::unique_ptr<PCStateBase>
    branchTarget(ThreadContext *tc) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

/**
 * Base class for conditional branches.
 */
class BranchCondOp : public PCDependentDisassembly
{
  protected:
    bool lk;
    uint8_t bi;
    uint8_t bo;

    /// Constructor
    BranchCondOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : PCDependentDisassembly(mnem, _machInst, __opClass),
          lk(machInst.lk),
          bi(machInst.bi),
          bo(machInst.bo)
    {}

    inline bool
    ctrOk(uint64_t &ctr) const
    {
        if (bits(bo, 2)) {
            return true;
        }

        ctr--;
        return !((ctr != 0) ^ (bits(bo, 1) == 0));
    }

    inline bool
    condOk(uint32_t cr) const
    {
        if (bits(bo, 4)) {
            return true;
        }

        return bits(cr >> (31 - bi), 0) == bits(bo >> 3, 0);
    }
};

/**
 * Base class for conditional, PC-relative or absolute address branches.
 */
class BranchDispCondOp : public BranchCondOp
{
  protected:
    bool aa;
    int64_t bd;

    /// Constructor
    BranchDispCondOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : BranchCondOp(mnem, _machInst, __opClass),
          aa(machInst.aa),
          bd(sext<16>(machInst.bd << 2))
    {}

    std::unique_ptr<PCStateBase>
    branchTarget(ThreadContext *tc) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

/**
 * Base class for conditional, register-based branches.
 */
class BranchRegCondOp : public BranchCondOp
{
  protected:
    /// TODO: Branch hints are currently ignored
    uint8_t bh;

    /// Constructor.
    BranchRegCondOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : BranchCondOp(mnem, _machInst, __opClass), bh(machInst.bh)
    {}

    std::unique_ptr<PCStateBase>
    branchTarget(ThreadContext *tc) const override;

    /// Explicitly import the otherwise hidden branchTarget
    using StaticInst::branchTarget;

    std::string
    generateDisassembly(Addr pc,
                        const loader::SymbolTable *symtab) const override;
};

} // namespace PowerISA
} // namespace gem5

#endif //__ARCH_POWER_INSTS_BRANCH_HH__
