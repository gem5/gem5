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

#ifndef __ARCH_POWER_REGISTERS_HH__
#define __ARCH_POWER_REGISTERS_HH__

#include "arch/power/generated/max_inst_regs.hh"
#include "arch/power/miscregs.hh"

namespace PowerISA {

using PowerISAInst::MaxInstSrcRegs;
using PowerISAInst::MaxInstDestRegs;

// Power writes a misc register outside of the isa parser, so it can't
// be detected by it. Manually add it here.
const int MaxMiscDestRegs = PowerISAInst::MaxMiscDestRegs + 1;

typedef uint8_t RegIndex;

typedef uint64_t IntReg;

// Floating point register file entry type
typedef uint64_t FloatRegBits;
typedef double FloatReg;
typedef uint64_t MiscReg;

// dummy typedef since we don't have CC regs
typedef uint8_t CCReg;

// Constants Related to the number of registers
const int NumIntArchRegs = 32;

// CR, XER, LR, CTR, FPSCR, RSV, RSV-LEN, RSV-ADDR
// and zero register, which doesn't actually exist but needs a number
const int NumIntSpecialRegs = 9;
const int NumFloatArchRegs = 32;
const int NumFloatSpecialRegs = 0;
const int NumInternalProcRegs = 0;

const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;
const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;
const int NumCCRegs = 0;
const int NumMiscRegs = NUM_MISCREGS;

// Semantically meaningful register indices
const int ReturnValueReg = 3;
const int ArgumentReg0 = 3;
const int ArgumentReg1 = 4;
const int ArgumentReg2 = 5;
const int ArgumentReg3 = 6;
const int ArgumentReg4 = 7;
const int FramePointerReg = 31;
const int StackPointerReg = 1;

// There isn't one in Power, but we need to define one somewhere
const int ZeroReg = NumIntRegs - 1;

const int SyscallNumReg = 0;
const int SyscallPseudoReturnReg = 3;
const int SyscallSuccessReg = 3;

// These help enumerate all the registers for dependence tracking.
const int FP_Reg_Base = NumIntRegs;
const int CC_Reg_Base = FP_Reg_Base + NumFloatRegs;
const int Misc_Reg_Base = CC_Reg_Base + NumCCRegs; // NumCCRegs == 0
const int Max_Reg_Index = Misc_Reg_Base + NumMiscRegs;

typedef union {
    IntReg   intreg;
    FloatReg fpreg;
    MiscReg  ctrlreg;
} AnyReg;

enum MiscIntRegNums {
    INTREG_CR = NumIntArchRegs,
    INTREG_XER,
    INTREG_LR,
    INTREG_CTR,
    INTREG_FPSCR,
    INTREG_RSV,
    INTREG_RSV_LEN,
    INTREG_RSV_ADDR
};

} // namespace PowerISA

#endif // __ARCH_POWER_REGISTERS_HH__
