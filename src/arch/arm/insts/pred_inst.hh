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
#ifndef __ARCH_ARM_INSTS_PREDINST_HH__
#define __ARCH_ARM_INSTS_PREDINST_HH__

#include "arch/arm/insts/static_inst.hh"
#include "base/trace.hh"

namespace ArmISA
{
static inline uint32_t
rotate_imm(uint32_t immValue, int rotateValue)
{
    return ((immValue >> (rotateValue & 31)) |
            (immValue << (32 - (rotateValue & 31))));
}

/**
 * Base class for predicated integer operations.
 */
class PredOp : public ArmStaticInst
{
  protected:

    ConditionCode condCode;

    /// Constructor
    PredOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
           ArmStaticInst(mnem, _machInst, __opClass),
           condCode((ConditionCode)(unsigned)machInst.condCode)
    {
    }
};

/**
 * Base class for predicated immediate operations.
 */
class PredImmOp : public PredOp
{
    protected:

    uint32_t imm;
    uint32_t rotate;
    uint32_t rotated_imm;
    uint32_t rotated_carry;

    /// Constructor
    PredImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
              PredOp(mnem, _machInst, __opClass),
              imm(machInst.imm), rotate(machInst.rotate << 1),
              rotated_imm(0), rotated_carry(0)
    {
        rotated_imm = rotate_imm(imm, rotate);
        if (rotate != 0)
            rotated_carry = (rotated_imm >> 31) & 1;
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for predicated integer operations.
 */
class PredIntOp : public PredOp
{
    protected:

    uint32_t shift_size;
    uint32_t shift;

    /// Constructor
    PredIntOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
              PredOp(mnem, _machInst, __opClass),
              shift_size(machInst.shiftSize), shift(machInst.shift)
    {
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for predicated macro-operations.
 */
class PredMacroOp : public PredOp
{
    protected:

    uint32_t numMicroops;
    StaticInstPtr * microOps;

    /// Constructor
    PredMacroOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
                PredOp(mnem, _machInst, __opClass),
                numMicroops(0)
    {
        // We rely on the subclasses of this object to handle the
        // initialization of the micro-operations, since they are
        // all of variable length
        flags[IsMacroop] = true;
    }

    ~PredMacroOp()
    {
        if (numMicroops)
            delete [] microOps;
    }

    StaticInstPtr
    fetchMicroop(MicroPC microPC)
    {
        assert(microPC < numMicroops);
        return microOps[microPC];
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

/**
 * Base class for predicated micro-operations.
 */
class PredMicroop : public PredOp
{
    /// Constructor
    PredMicroop(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
                PredOp(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }
};
}

#endif //__ARCH_ARM_INSTS_PREDINST_HH__
