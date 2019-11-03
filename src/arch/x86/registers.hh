/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved.
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

#ifndef __ARCH_X86_REGISTERS_HH__
#define __ARCH_X86_REGISTERS_HH__

#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"
#include "arch/x86/generated/max_inst_regs.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/ccr.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/x86_traits.hh"

namespace X86ISA
{
using X86ISAInst::MaxInstSrcRegs;
using X86ISAInst::MaxInstDestRegs;
using X86ISAInst::MaxMiscDestRegs;
const int NumMiscRegs = NUM_MISCREGS;

const int NumIntArchRegs = NUM_INTREGS;
const int NumIntRegs = NumIntArchRegs + NumMicroIntRegs + NumImplicitIntRegs;
const int NumCCRegs = NUM_CCREGS;

#define ISA_HAS_CC_REGS

// Each 128 bit xmm register is broken into two effective 64 bit registers.
// Add 8 for the indices that are mapped over the fp stack
const int NumFloatRegs =
    NumMMXRegs + 2 * NumXMMRegs + NumMicroFpRegs + 8;

// These enumerate all the registers for dependence tracking.
enum DependenceTags {
    // FP_Reg_Base must be large enough to be bigger than any integer
    // register index which has the IntFoldBit (1 << 6) set.  To be safe
    // we just start at (1 << 7) == 128.
    FP_Reg_Base = 128,
    CC_Reg_Base = FP_Reg_Base + NumFloatRegs,
    Misc_Reg_Base = CC_Reg_Base + NumCCRegs,
    Max_Reg_Index = Misc_Reg_Base + NumMiscRegs
};

const int NumVecRegs = 1;  // Not applicable to x86
                           // (1 to prevent warnings)
const int NumVecPredRegs = 1;  // Not applicable to x86
                               // (1 to prevent warnings)

// semantically meaningful register indices
//There is no such register in X86
const int ZeroReg = NUM_INTREGS;
const int StackPointerReg = INTREG_RSP;
//X86 doesn't seem to have a link register
const int ReturnAddressReg = 0;
const int ReturnValueReg = INTREG_RAX;
const int FramePointerReg = INTREG_RBP;

// Some OS syscalls use a second register (rdx) to return a second
// value
const int SyscallPseudoReturnReg = INTREG_RDX;

// Not applicable to x86
using VecElem = ::DummyVecElem;
using VecReg = ::DummyVecReg;
using ConstVecReg = ::DummyConstVecReg;
using VecRegContainer = ::DummyVecRegContainer;
constexpr unsigned NumVecElemPerVecReg = ::DummyNumVecElemPerVecReg;
constexpr size_t VecRegSizeBytes = ::DummyVecRegSizeBytes;

// Not applicable to x86
using VecPredReg = ::DummyVecPredReg;
using ConstVecPredReg = ::DummyConstVecPredReg;
using VecPredRegContainer = ::DummyVecPredRegContainer;
constexpr size_t VecPredRegSizeBits = ::DummyVecPredRegSizeBits;
constexpr bool VecPredRegHasPackedRepr = ::DummyVecPredRegHasPackedRepr;

} // namespace X86ISA

#endif // __ARCH_X86_REGFILE_HH__
