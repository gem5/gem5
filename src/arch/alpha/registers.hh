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
#include "arch/generic/types.hh"
#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"
#include "base/types.hh"

namespace AlphaISA {

using AlphaISAInst::MaxInstSrcRegs;
using AlphaISAInst::MaxInstDestRegs;

// Locked read/write flags are can't be detected by the ISA parser
const int MaxMiscDestRegs = AlphaISAInst::MaxMiscDestRegs + 1;

// Not applicable to Alpha
using VecElem = ::DummyVecElem;
using VecReg = ::DummyVecReg;
using ConstVecReg = ::DummyConstVecReg;
using VecRegContainer = ::DummyVecRegContainer;
constexpr unsigned NumVecElemPerVecReg = ::DummyNumVecElemPerVecReg;
constexpr size_t VecRegSizeBytes = ::DummyVecRegSizeBytes;

// Not applicable to Alpha
using VecPredReg = ::DummyVecPredReg;
using ConstVecPredReg = ::DummyConstVecPredReg;
using VecPredRegContainer = ::DummyVecPredRegContainer;
constexpr size_t VecPredRegSizeBits = ::DummyVecPredRegSizeBits;
constexpr bool VecPredRegHasPackedRepr = ::DummyVecPredRegHasPackedRepr;

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
const int NumVecRegs = 1;  // Not applicable to Alpha
                           // (1 to prevent warnings)
const int NumVecPredRegs = 1;  // Not applicable to Alpha
                               // (1 to prevent warnings)
const int NumCCRegs = 0;
const int NumMiscRegs = NUM_MISCREGS;

const int TotalNumRegs =
    NumIntRegs + NumFloatRegs + NumMiscRegs;

} // namespace AlphaISA

#endif // __ARCH_ALPHA_REGFILE_HH__
