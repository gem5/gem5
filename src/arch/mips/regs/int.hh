/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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

#ifndef __ARCH_MIPS_REGS_INT_HH__
#define __ARCH_MIPS_REGS_INT_HH__

namespace gem5
{

namespace MipsISA
{

// Constants Related to the number of registers
const int NumIntArchRegs = 32;
const int NumIntSpecialRegs = 9;

const int MaxShadowRegSets = 16; // Maximum number of shadow register sets
const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs; //HI & LO Regs

enum MiscIntRegNums
{
   INTREG_LO = NumIntArchRegs,
   INTREG_DSP_LO0 = INTREG_LO,
   INTREG_HI,
   INTREG_DSP_HI0 = INTREG_HI,
   INTREG_DSP_ACX0,
   INTREG_DSP_LO1,
   INTREG_DSP_HI1,
   INTREG_DSP_ACX1,
   INTREG_DSP_LO2,
   INTREG_DSP_HI2,
   INTREG_DSP_ACX2,
   INTREG_DSP_LO3,
   INTREG_DSP_HI3,
   INTREG_DSP_ACX3,
   INTREG_DSP_CONTROL
};

// semantically meaningful register indices
const int SyscallSuccessReg = 7;
const int FirstArgumentReg = 4;
const int ReturnValueReg = 2;

const int StackPointerReg = 29;

const int SyscallPseudoReturnReg = 3;

} // namespace MipsISA
} // namespace gem5

#endif
