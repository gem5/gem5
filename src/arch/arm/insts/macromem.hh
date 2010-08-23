/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Stephen Hines
 */
#ifndef __ARCH_ARM_MACROMEM_HH__
#define __ARCH_ARM_MACROMEM_HH__

#include "arch/arm/insts/pred_inst.hh"
#include "arch/arm/tlb.hh"

namespace ArmISA
{

static inline unsigned int
number_of_ones(int32_t val)
{
    uint32_t ones = 0;
    for (int i = 0; i < 32; i++ )
    {
        if ( val & (1<<i) )
            ones++;
    }
    return ones;
}

/**
 * Base class for Memory microops
 */
class MicroOp : public PredOp
{
  protected:
    MicroOp(const char *mnem, ExtMachInst machInst, OpClass __opClass)
            : PredOp(mnem, machInst, __opClass)
    {
    }

  public:
    void
    setDelayedCommit()
    {
        flags[IsDelayedCommit] = true;
    }
};

/**
 * Microops of the form IntRegA = IntRegB op Imm
 */
class MicroIntOp : public MicroOp
{
  protected:
    RegIndex ura, urb;
    uint8_t imm;

    MicroIntOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
               RegIndex _ura, RegIndex _urb, uint8_t _imm)
            : MicroOp(mnem, machInst, __opClass),
              ura(_ura), urb(_urb), imm(_imm)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Memory microops which use IntReg + Imm addressing
 */
class MicroMemOp : public MicroIntOp
{
  protected:
    bool up;
    unsigned memAccessFlags;

    MicroMemOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
               RegIndex _ura, RegIndex _urb, bool _up, uint8_t _imm)
            : MicroIntOp(mnem, machInst, __opClass, _ura, _urb, _imm),
              up(_up), memAccessFlags(TLB::MustBeOne | TLB::AlignWord)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for microcoded integer memory instructions.
 */
class MacroMemOp : public PredMacroOp
{
  protected:
    MacroMemOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
               IntRegIndex rn, bool index, bool up, bool user,
               bool writeback, bool load, uint32_t reglist);
};

/**
 * Base class for microcoded floating point memory instructions.
 */
class MacroVFPMemOp : public PredMacroOp
{
  protected:
    MacroVFPMemOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
                  IntRegIndex rn, RegIndex vd, bool single, bool up,
                  bool writeback, bool load, uint32_t offset);
};

}

#endif //__ARCH_ARM_INSTS_MACROMEM_HH__
