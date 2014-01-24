/*
 * Copyright (c) 2010-2011 ARM Limited
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

#ifndef __ARCH_ARM_REGISTERS_HH__
#define __ARCH_ARM_REGISTERS_HH__

#include "arch/arm/generated/max_inst_regs.hh"
#include "arch/arm/intregs.hh"
#include "arch/arm/miscregs.hh"

namespace ArmISA {


// For a predicated instruction, we need all the
// destination registers to also be sources
const int MaxInstSrcRegs = ArmISAInst::MaxInstDestRegs +
    ArmISAInst::MaxInstSrcRegs;
using ArmISAInst::MaxInstDestRegs;
using ArmISAInst::MaxMiscDestRegs;

typedef uint16_t  RegIndex;

typedef uint64_t IntReg;

// floating point register file entry type
typedef uint32_t FloatRegBits;
typedef float FloatReg;

// cop-0/cop-1 system control register
typedef uint64_t MiscReg;

// dummy typedef since we don't have CC regs
typedef uint8_t CCReg;

// Constants Related to the number of registers
const int NumIntArchRegs = NUM_ARCH_INTREGS;
// The number of single precision floating point registers
const int NumFloatV7ArchRegs  = 64;
const int NumFloatV8ArchRegs  = 128;
const int NumFloatSpecialRegs = 32;

const int NumIntRegs = NUM_INTREGS;
const int NumFloatRegs = NumFloatV8ArchRegs + NumFloatSpecialRegs;
const int NumCCRegs = 0;
const int NumMiscRegs = NUM_MISCREGS;

const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;

// semantically meaningful register indices
const int ReturnValueReg = 0;
const int ReturnValueReg1 = 1;
const int ReturnValueReg2 = 2;
const int NumArgumentRegs = 4;
const int NumArgumentRegs64 = 8;
const int ArgumentReg0 = 0;
const int ArgumentReg1 = 1;
const int ArgumentReg2 = 2;
const int ArgumentReg3 = 3;
const int FramePointerReg = 11;
const int StackPointerReg = INTREG_SP;
const int ReturnAddressReg = INTREG_LR;
const int PCReg = INTREG_PC;

const int ZeroReg = INTREG_ZERO;

const int SyscallNumReg = ReturnValueReg;
const int SyscallPseudoReturnReg = ReturnValueReg;
const int SyscallSuccessReg = ReturnValueReg;

// These help enumerate all the registers for dependence tracking.
const int FP_Reg_Base = NumIntRegs * (MODE_MAXMODE + 1);
const int CC_Reg_Base = FP_Reg_Base + NumFloatRegs;
const int Misc_Reg_Base = CC_Reg_Base + NumCCRegs; // NumCCRegs == 0
const int Max_Reg_Index = Misc_Reg_Base + NumMiscRegs;

typedef union {
    IntReg   intreg;
    FloatReg fpreg;
    MiscReg  ctrlreg;
} AnyReg;

} // namespace ArmISA

#endif
