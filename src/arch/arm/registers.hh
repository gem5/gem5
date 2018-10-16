/*
 * Copyright (c) 2010-2011, 2014, 2016-2017 ARM Limited
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

#include "arch/arm/ccregs.hh"
#include "arch/arm/generated/max_inst_regs.hh"
#include "arch/arm/intregs.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/types.hh"
#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"

namespace ArmISA {


// For a predicated instruction, we need all the
// destination registers to also be sources
const int MaxInstSrcRegs = ArmISAInst::MaxInstDestRegs +
    ArmISAInst::MaxInstSrcRegs;
using ArmISAInst::MaxInstDestRegs;
using ArmISAInst::MaxMiscDestRegs;

// Number of VecElem per Vector Register, computed based on the vector length
constexpr unsigned NumVecElemPerVecReg = MaxSveVecLenInWords;

using VecElem = uint32_t;
using VecReg = ::VecRegT<VecElem, NumVecElemPerVecReg, false>;
using ConstVecReg = ::VecRegT<VecElem, NumVecElemPerVecReg, true>;
using VecRegContainer = VecReg::Container;

using VecPredReg = ::VecPredRegT<VecElem, NumVecElemPerVecReg,
                                 VecPredRegHasPackedRepr, false>;
using ConstVecPredReg = ::VecPredRegT<VecElem, NumVecElemPerVecReg,
                                      VecPredRegHasPackedRepr, true>;
using VecPredRegContainer = VecPredReg::Container;

// Constants Related to the number of registers
const int NumIntArchRegs = NUM_ARCH_INTREGS;
// The number of single precision floating point registers
const int NumFloatV7ArchRegs  = 64;
const int NumFloatV8ArchRegs  = 128;
const int NumFloatSpecialRegs = 32;
const int NumVecV7ArchRegs  = 64;
const int NumVecV8ArchRegs  = 32;
const int NumVecSpecialRegs = 8;

const int NumIntRegs = NUM_INTREGS;
const int NumFloatRegs = NumFloatV8ArchRegs + NumFloatSpecialRegs;
const int NumVecRegs = NumVecV8ArchRegs + NumVecSpecialRegs;
const int NumVecPredRegs = 17;  // P0-P15, FFR
const int PREDREG_FFR = 16;
const int NumCCRegs = NUM_CCREGS;
const int NumMiscRegs = NUM_MISCREGS;

#define ISA_HAS_CC_REGS

const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumVecRegs +
    NumVecPredRegs + NumMiscRegs;

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

} // namespace ArmISA

#endif
