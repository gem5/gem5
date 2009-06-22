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
 * Arm Macro Memory operations like LDM/STM
 */
class ArmMacroMemoryOp : public PredMacroOp
{
    protected:
    /// Memory request flags.  See mem_req_base.hh.
    unsigned memAccessFlags;
    /// Pointer to EAComp object.
    const StaticInstPtr eaCompPtr;
    /// Pointer to MemAcc object.
    const StaticInstPtr memAccPtr;

    uint32_t reglist;
    uint32_t ones;
    uint32_t puswl,
             prepost,
             up,
             psruser,
             writeback,
             loadop;

    ArmMacroMemoryOp(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass,
                     StaticInstPtr _eaCompPtr = nullStaticInstPtr,
                     StaticInstPtr _memAccPtr = nullStaticInstPtr)
            : PredMacroOp(mnem, _machInst, __opClass),
                          memAccessFlags(0),
                          eaCompPtr(_eaCompPtr), memAccPtr(_memAccPtr),
                          reglist(machInst.regList), ones(0),
                          puswl(machInst.puswl),
                          prepost(machInst.puswl.prepost),
                          up(machInst.puswl.up),
                          psruser(machInst.puswl.psruser),
                          writeback(machInst.puswl.writeback),
                          loadop(machInst.puswl.loadOp)
    {
        ones = number_of_ones(reglist);
        numMicroops = ones + writeback + 1;
        // Remember that writeback adds a uop
        microOps = new StaticInstPtr[numMicroops];
    }
};

/**
 * Arm Macro FPA operations to fix ldfd and stfd instructions
 */
class ArmMacroFPAOp : public PredMacroOp
{
    protected:
    uint32_t puswl,
             prepost,
             up,
             psruser,
             writeback,
             loadop;
    int32_t disp8;

    ArmMacroFPAOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PredMacroOp(mnem, _machInst, __opClass),
                      puswl(machInst.puswl),
                      prepost(machInst.puswl.prepost),
                      up(machInst.puswl.up),
                      psruser(machInst.puswl.psruser),
                      writeback(machInst.puswl.writeback),
                      loadop(machInst.puswl.loadOp),
                      disp8(machInst.immed7_0 << 2)
    {
        numMicroops = 3 + writeback;
        microOps = new StaticInstPtr[numMicroops];
    }
};

/**
 * Arm Macro FM operations to fix lfm and sfm
 */
class ArmMacroFMOp : public PredMacroOp
{
    protected:
    uint32_t punwl,
             prepost,
             up,
             n1bit,
             writeback,
             loadop,
             n0bit,
             count;
    int32_t disp8;

    ArmMacroFMOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass)
        : PredMacroOp(mnem, _machInst, __opClass),
                      punwl(machInst.punwl),
                      prepost(machInst.puswl.prepost),
                      up(machInst.puswl.up),
                      n1bit(machInst.opcode22),
                      writeback(machInst.puswl.writeback),
                      loadop(machInst.puswl.loadOp),
                      n0bit(machInst.opcode15),
                      disp8(machInst.immed7_0 << 2)
    {
        // Transfer 1-4 registers based on n1 and n0 bits (with 00 repr. 4)
        count = (n1bit << 1) | n0bit;
        if (count == 0)
            count = 4;
        numMicroops = (3*count) + writeback;
        microOps = new StaticInstPtr[numMicroops];
    }
};
}

#endif //__ARCH_ARM_INSTS_MACROMEM_HH__
