/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#ifndef __ARCH_ALPHA_REGISTERS_HH__
#define __ARCH_ALPHA_REGISTERS_HH__

#include "arch/alpha/generated/max_inst_regs.hh"
#include "arch/alpha/ipr.hh"
#include "base/types.hh"

namespace AlphaISA {

using AlphaISAInst::MaxInstSrcRegs;
using AlphaISAInst::MaxInstDestRegs;

// Locked read/write flags are can't be detected by the ISA parser
const int MaxMiscDestRegs = AlphaISAInst::MaxMiscDestRegs + 1;

typedef uint8_t RegIndex;
typedef uint64_t IntReg;

// floating point register file entry type
typedef double FloatReg;
typedef uint64_t FloatRegBits;

// control register file contents
typedef uint64_t MiscReg;

// dummy typedef since we don't have CC regs
typedef uint8_t CCReg;

union AnyReg
{
    IntReg  intreg;
    FloatReg   fpreg;
    MiscReg ctrlreg;
};

enum MiscRegIndex
{
    MISCREG_FPCR = NumInternalProcRegs,
    MISCREG_UNIQ,
    MISCREG_LOCKFLAG,
    MISCREG_LOCKADDR,
    MISCREG_INTR,
    NUM_MISCREGS
};

// semantically meaningful register indices
const RegIndex ZeroReg = 31;     // architecturally meaningful
// the rest of these depend on the ABI
const RegIndex StackPointerReg = 30;
const RegIndex GlobalPointerReg = 29;
const RegIndex ProcedureValueReg = 27;
const RegIndex ReturnAddressReg = 26;
const RegIndex ReturnValueReg = 0;
const RegIndex FramePointerReg = 15;

const RegIndex SyscallNumReg = 0;
const RegIndex FirstArgumentReg = 16;
const RegIndex SyscallPseudoReturnReg = 20;
const RegIndex SyscallSuccessReg = 19;

const int NumIntArchRegs = 32;
const int NumPALShadowRegs = 8;
const int NumFloatArchRegs = 32;

const int NumIntRegs = NumIntArchRegs + NumPALShadowRegs;
const int NumFloatRegs = NumFloatArchRegs;
const int NumCCRegs = 0;
const int NumMiscRegs = NUM_MISCREGS;

const int TotalNumRegs =
    NumIntRegs + NumFloatRegs + NumMiscRegs;

// These enumerate all the registers for dependence tracking.
enum DependenceTags {
    // 0..31 are the integer regs 0..31
    // 32..63 are the FP regs 0..31, i.e. use (reg + FP_Reg_Base)
    FP_Reg_Base = NumIntRegs,
    CC_Reg_Base = FP_Reg_Base + NumFloatRegs,
    Misc_Reg_Base = CC_Reg_Base + NumCCRegs, // NumCCRegs == 0
    Max_Reg_Index = Misc_Reg_Base + NumMiscRegs + NumInternalProcRegs
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_REGFILE_HH__
