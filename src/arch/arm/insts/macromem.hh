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
 * Microops of the form IntRegA = IntRegB op Imm
 */
class MicroIntOp : public PredOp
{
  protected:
    RegIndex ura, urb;
    uint8_t imm;

    MicroIntOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
               RegIndex _ura, RegIndex _urb, uint8_t _imm)
            : PredOp(mnem, machInst, __opClass),
              ura(_ura), urb(_urb), imm(_imm)
    {
    }
};

/**
 * Memory microops which use IntReg + Imm addressing
 */
class MicroMemOp : public MicroIntOp
{
  protected:
    unsigned memAccessFlags;

    MicroMemOp(const char *mnem, ExtMachInst machInst, OpClass __opClass,
               RegIndex _ura, RegIndex _urb, uint8_t _imm)
            : MicroIntOp(mnem, machInst, __opClass, _ura, _urb, _imm),
              memAccessFlags(0)
    {
    }
};

/**
 * Arm Macro Memory operations like LDM/STM
 */
class ArmMacroMemoryOp : public PredMacroOp
{
  protected:
    /// Memory request flags.  See mem_req_base.hh.
    unsigned memAccessFlags;

    uint32_t reglist;
    uint32_t ones;

    ArmMacroMemoryOp(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass)
            : PredMacroOp(mnem, _machInst, __opClass), memAccessFlags(0),
              reglist(machInst.regList), ones(0)
    {
        ones = number_of_ones(reglist);
        numMicroops = ones + machInst.puswl.writeback + 1;
        // Remember that writeback adds a uop
        microOps = new StaticInstPtr[numMicroops];
    }
};
}

#endif //__ARCH_ARM_INSTS_MACROMEM_HH__
