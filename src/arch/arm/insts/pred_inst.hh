/*
 * Copyright (c) 2010, 2012-2013 ARM Limited
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
#ifndef __ARCH_ARM_INSTS_PREDINST_HH__
#define __ARCH_ARM_INSTS_PREDINST_HH__

#include "arch/arm/insts/static_inst.hh"
#include "base/trace.hh"

namespace ArmISA
{
static inline uint32_t
rotate_imm(uint32_t immValue, uint32_t rotateValue)
{
    rotateValue &= 31;
    return rotateValue == 0 ? immValue :
        (immValue >> rotateValue) | (immValue << (32 - rotateValue));
}

static inline uint32_t
modified_imm(uint8_t ctrlImm, uint8_t dataImm)
{
    uint32_t bigData = dataImm;
    uint32_t bigCtrl = ctrlImm;
    if (bigCtrl < 4) {
        switch (bigCtrl) {
          case 0:
            return bigData;
          case 1:
            return bigData | (bigData << 16);
          case 2:
            return (bigData << 8) | (bigData << 24);
          case 3:
            return (bigData << 0) | (bigData << 8) |
                   (bigData << 16) | (bigData << 24);
        }
    }
    bigCtrl = (bigCtrl << 1) | ((bigData >> 7) & 0x1);
    bigData |= (1 << 7);
    return bigData << (32 - bigCtrl);
}

static inline uint64_t
simd_modified_imm(bool op, uint8_t cmode, uint8_t data, bool &immValid,
                  bool isAarch64 = false)
{
    uint64_t bigData = data;
    immValid = true;
    switch (cmode) {
      case 0x0:
      case 0x1:
        bigData = (bigData << 0) | (bigData << 32);
        break;
      case 0x2:
      case 0x3:
        bigData = (bigData << 8) | (bigData << 40);
        break;
      case 0x4:
      case 0x5:
        bigData = (bigData << 16) | (bigData << 48);
        break;
      case 0x6:
      case 0x7:
        bigData = (bigData << 24) | (bigData << 56);
        break;
      case 0x8:
      case 0x9:
        bigData = (bigData << 0) | (bigData << 16) |
                  (bigData << 32) | (bigData << 48);
        break;
      case 0xa:
      case 0xb:
        bigData = (bigData << 8) | (bigData << 24) |
                  (bigData << 40) | (bigData << 56);
        break;
      case 0xc:
        bigData = (0xffULL << 0) | (bigData << 8) |
                  (0xffULL << 32) | (bigData << 40);
        break;
      case 0xd:
        bigData = (0xffffULL << 0) | (bigData << 16) |
                  (0xffffULL << 32) | (bigData << 48);
        break;
      case 0xe:
        if (op) {
            bigData = 0;
            for (int i = 7; i >= 0; i--) {
                if (bits(data, i)) {
                    bigData |= (ULL(0xFF) << (i * 8));
                }
            }
        } else {
            bigData = (bigData << 0)  | (bigData << 8)  |
                      (bigData << 16) | (bigData << 24) |
                      (bigData << 32) | (bigData << 40) |
                      (bigData << 48) | (bigData << 56);
        }
        break;
      case 0xf:
        {
            uint64_t bVal = 0;
            if (!op) {
                bVal = bits(bigData, 6) ? (0x1F) : (0x20);
                bigData = (bits(bigData, 5, 0) << 19) |
                          (bVal << 25) | (bits(bigData, 7) << 31);
                bigData |= (bigData << 32);
                break;
            } else if (isAarch64) {
                bVal = bits(bigData, 6) ? (0x0FF) : (0x100);
                bigData = (bits(bigData, 5, 0) << 48) |
                          (bVal << 54) | (bits(bigData, 7) << 63);
                break;
            }
        }
        M5_FALLTHROUGH;
      default:
        immValid = false;
        break;
    }
    return bigData;
}

static inline uint64_t
vfp_modified_imm(uint8_t data, bool wide)
{
    uint64_t bigData = data;
    uint64_t repData;
    if (wide) {
        repData = bits(data, 6) ? 0xFF : 0;
        bigData = (bits(bigData, 5, 0) << 48) |
                  (repData << 54) | (bits(~bigData, 6) << 62) |
                  (bits(bigData, 7) << 63);
    } else {
        repData = bits(data, 6) ? 0x1F : 0;
        bigData = (bits(bigData, 5, 0) << 19) |
                  (repData << 25) | (bits(~bigData, 6) << 30) |
                  (bits(bigData, 7) << 31);
    }
    return bigData;
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
           ArmStaticInst(mnem, _machInst, __opClass)
    {
        if (machInst.aarch64)
            condCode = COND_UC;
        else if (machInst.itstateMask)
            condCode = (ConditionCode)(uint8_t)machInst.itstateCond;
        else
            condCode = (ConditionCode)(unsigned)machInst.condCode;
    }
};

/**
 * Base class for predicated immediate operations.
 */
class PredImmOp : public PredOp
{
    protected:

    uint32_t imm;
    uint32_t rotated_imm;
    uint32_t rotated_carry;
    uint32_t rotate;

    /// Constructor
    PredImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
              PredOp(mnem, _machInst, __opClass),
              imm(machInst.imm), rotated_imm(0), rotated_carry(0),
              rotate(machInst.rotate << 1)
    {
        rotated_imm = rotate_imm(imm, rotate);
        if (rotate != 0)
            rotated_carry = bits(rotated_imm, 31);
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

class DataImmOp : public PredOp
{
  protected:
    IntRegIndex dest, op1;
    uint32_t imm;
    // Whether the carry flag should be modified if that's an option for
    // this instruction.
    bool rotC;

    DataImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _op1, uint32_t _imm, bool _rotC) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), imm(_imm), rotC(_rotC)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class DataRegOp : public PredOp
{
  protected:
    IntRegIndex dest, op1, op2;
    int32_t shiftAmt;
    ArmShiftType shiftType;

    DataRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
              IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
              int32_t _shiftAmt, ArmShiftType _shiftType) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2),
        shiftAmt(_shiftAmt), shiftType(_shiftType)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class DataRegRegOp : public PredOp
{
  protected:
    IntRegIndex dest, op1, op2, shift;
    ArmShiftType shiftType;

    DataRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                 IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                 IntRegIndex _shift, ArmShiftType _shiftType) :
        PredOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), shift(_shift),
        shiftType(_shiftType)
    {}

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
                numMicroops(0), microOps(nullptr)
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
    fetchMicroop(MicroPC microPC) const
    {
        assert(microPC < numMicroops);
        return microOps[microPC];
    }

    Fault
    execute(ExecContext *, Trace::InstRecord *) const
    {
        panic("Execute method called when it shouldn't!");
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

    void
    advancePC(PCState &pcState) const
    {
        if (flags[IsLastMicroop])
            pcState.uEnd();
        else
            pcState.uAdvance();
    }
};
}

#endif //__ARCH_ARM_INSTS_PREDINST_HH__
