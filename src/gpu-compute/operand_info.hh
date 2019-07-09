/*
 * Copyright (c) 2019-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GPU_COMPUTE_OPERAND_INFO_HH__
#define __GPU_COMPUTE_OPERAND_INFO_HH__

#include "arch/gpu_registers.hh"
#include "base/flags.hh"
#include "config/the_gpu_isa.hh"

class OperandInfo
{
  public:
    OperandInfo() = delete;
    OperandInfo(int opSelectorVal, int size, bool src, bool scalar_reg,
                bool vector_reg, bool imm)
        : _opSelectorVal(opSelectorVal), _size(size)
    {
        if (src)
            flags.set(SRC);
        if (scalar_reg)
            flags.set(SCALAR_REG);
        if (vector_reg)
            flags.set(VECTOR_REG);
        if (imm)
            flags.set(IMMEDIATE);
        if (TheGpuISA::isVccReg(opSelectorVal))
            flags.set(VCC);
        if (TheGpuISA::isExecMask(opSelectorVal))
            flags.set(EXEC);
        if (TheGpuISA::isFlatScratchReg(opSelectorVal))
            flags.set(FLAT);
        if (TheGpuISA::isLiteral(opSelectorVal))
            flags.set(LITERAL);
        if (TheGpuISA::isConstVal(opSelectorVal))
            flags.set(CONSTANT);
        if (TheGpuISA::isPosConstVal(opSelectorVal))
            flags.set(POS_CONST);
    }

    int size() const { return _size; }
    int
    registerIndex(int numScalarRegs) const
    {
      // Some regs (i.e. VSRC, VDST) are explicitly declared as vectors
      // as opposed to checking if it's a vector through a function call, so
      // they don't have an offset applied and can be returned immediately
      if (isVectorReg() && _opSelectorVal < TheGpuISA::REG_VGPR_MIN)
        return _opSelectorVal;
      return TheGpuISA::opSelectorToRegIdx(_opSelectorVal, numScalarRegs);
    }
    bool isSrc() const { return flags.isSet(SRC); }
    bool isDst() const { return !flags.isSet(SRC); }
    bool isImm() const { return flags.isSet(IMMEDIATE); }
    bool isScalarReg() const { return flags.isSet(SCALAR_REG); }
    bool isVectorReg() const { return flags.isSet(VECTOR_REG); }
    bool isVcc() const { return flags.isSet(VCC); }
    bool isExec() const { return flags.isSet(EXEC); }
    bool isFlatScratch() const { return flags.isSet(FLAT); }

    typedef uint32_t FlagsType;
    typedef ::Flags<FlagsType> Flags;

  private:

    enum : FlagsType {
        // If the operand is a src or not
        SRC                 = 0x00000001,

        // If the operand is a scalar or not
        SCALAR_REG          = 0x00000002,

        // If the operand is a vector or not
        VECTOR_REG          = 0x00000004,

        // If the operand is an immediate or not
        IMMEDIATE           = 0x00000008,

        // If the operand is a VCC register
        VCC                 = 0x00000010,

        // If the operand is an EXEC register
        EXEC                = 0x00000020,

        // If the operand is a FLAT/SCRATCH register
        FLAT                = 0x00000040,

        // If the operand is a literal
        LITERAL             = 0x00000080,

        // If the operand is a constant value
        CONSTANT            = 0x00000100,

        // If the constant is positive or negative
        POS_CONST           = 0x00000200
    };

    Flags flags;

    /**
     * Index of the operand as used in registers.cc functions
     */
    const int _opSelectorVal;

    /**
     * Size of the operand in bytes
     */
    const int _size;
};

#endif // __GPU_COMPUTE_OPERAND_INFO_H__
