/*
 * Copyright (c) 2009 The University of Edinburgh
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
 * Authors: Timothy M. Jones
 */

#ifndef __ARCH_POWER_INSTS_INTEGER_HH__
#define __ARCH_POWER_INSTS_INTEGER_HH__

#include "arch/power/insts/static_inst.hh"
#include "base/bitfield.hh"
#include "base/cprintf.hh"

namespace PowerISA
{

/**
 * We provide a base class for integer operations and then inherit for
 * several other classes. These specialise for instructions using immediate
 * values and also rotate instructions. We also need to have versions that
 * consider the Rc and OE bits.
 */

/**
 * Base class for integer operations.
 */
class IntOp : public PowerStaticInst
{
  protected:

    bool rcSet;
    bool oeSet;

    // Needed for srawi only
    uint32_t sh;

    /// Constructor
    IntOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : PowerStaticInst(mnem, _machInst, __opClass),
        rcSet(false), oeSet(false)
    {
    }

    /* Compute the CR (condition register) field using signed comparison */
    inline uint32_t
    makeCRField(int32_t a, int32_t b, uint32_t xerSO) const
    {
        uint32_t c = xerSO;

        /* We've pre-shifted the immediate values here */
        if (a < b)      { c += 0x8; }
        else if (a > b) { c += 0x4; }
        else            { c += 0x2; }
        return c;
    }

    /* Compute the CR (condition register) field using unsigned comparison */
    inline uint32_t
    makeCRField(uint32_t a, uint32_t b, uint32_t xerSO) const
    {
        uint32_t c = xerSO;

        /* We've pre-shifted the immediate values here */
        if (a < b)      { c += 0x8; }
        else if (a > b) { c += 0x4; }
        else            { c += 0x2; }
        return c;
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};


/**
 * Class for integer immediate (signed and unsigned) operations.
 */
class IntImmOp : public IntOp
{
  protected:

    int32_t imm;
    uint32_t uimm;

    /// Constructor
    IntImmOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : IntOp(mnem, _machInst, __opClass),
        imm(sext<16>(machInst.si)),
        uimm(machInst.si)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};


/**
 * Class for integer operations with a shift.
 */
class IntShiftOp : public IntOp
{
  protected:

    uint32_t sh;

    /// Constructor
    IntShiftOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : IntOp(mnem, _machInst, __opClass),
        sh(machInst.sh)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};


/**
 * Class for integer rotate operations.
 */
class IntRotateOp : public IntShiftOp
{
  protected:

    uint32_t mb;
    uint32_t me;
    uint32_t fullMask;

    /// Constructor
    IntRotateOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : IntShiftOp(mnem, _machInst, __opClass),
        mb(machInst.mb),
        me(machInst.me)
    {
        if (me >= mb) {
            fullMask = mask(31 - mb, 31 - me);
        } else {
            fullMask = ~mask(31 - (me + 1), 31 - (mb - 1));
        }
    }

    uint32_t
    rotateValue(uint32_t rs, uint32_t shift) const
    {
        uint32_t n = shift & 31;
        return (rs << n) | (rs >> (32 - n));
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

} // namespace PowerISA

#endif //__ARCH_POWER_INSTS_INTEGER_HH__
