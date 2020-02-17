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

#ifndef __ARCH_MIPS_REGISTERS_HH__
#define __ARCH_MIPS_REGISTERS_HH__

#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"
#include "arch/mips/generated/max_inst_regs.hh"
#include "base/logging.hh"
#include "base/types.hh"

class ThreadContext;

namespace MipsISA
{

using MipsISAInst::MaxInstSrcRegs;
using MipsISAInst::MaxInstDestRegs;
using MipsISAInst::MaxMiscDestRegs;

// Constants Related to the number of registers
const int NumIntArchRegs = 32;
const int NumIntSpecialRegs = 9;
const int NumFloatArchRegs = 32;
const int NumFloatSpecialRegs = 5;

const int MaxShadowRegSets = 16; // Maximum number of shadow register sets
const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;        //HI & LO Regs
const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;//
const int NumVecRegs = 1;  // Not applicable to MIPS
                           // (1 to prevent warnings)
const int NumVecPredRegs = 1;  // Not applicable to MIPS
                               // (1 to prevent warnings)
const int NumCCRegs = 0;

const uint32_t MIPS32_QNAN = 0x7fbfffff;
const uint64_t MIPS64_QNAN = ULL(0x7ff7ffffffffffff);

enum FPControlRegNums {
   FLOATREG_FIR = NumFloatArchRegs,
   FLOATREG_FCCR,
   FLOATREG_FEXR,
   FLOATREG_FENR,
   FLOATREG_FCSR
};

enum FCSRBits {
    Inexact = 1,
    Underflow,
    Overflow,
    DivideByZero,
    Invalid,
    Unimplemented
};

enum FCSRFields {
    Flag_Field = 1,
    Enable_Field = 6,
    Cause_Field = 11
};

enum MiscIntRegNums {
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
const int ZeroReg = 0;
const int AssemblerReg = 1;
const int SyscallSuccessReg = 7;
const int FirstArgumentReg = 4;
const int ReturnValueReg = 2;

const int KernelReg0 = 26;
const int KernelReg1 = 27;
const int GlobalPointerReg = 28;
const int StackPointerReg = 29;
const int FramePointerReg = 30;
const int ReturnAddressReg = 31;

const int SyscallPseudoReturnReg = 3;

// Enumerate names for 'Control' Registers in the CPU
// Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
// (Register Number-Register Select) Summary of Register
//------------------------------------------------------
// The first set of names classify the CP0 names as Register Banks
// for easy indexing when using the 'RD + SEL' index combination
// in CP0 instructions.
enum MiscRegIndex{
    MISCREG_INDEX = 0,       //Bank 0: 0 - 3
    MISCREG_MVP_CONTROL,
    MISCREG_MVP_CONF0,
    MISCREG_MVP_CONF1,

    MISCREG_CP0_RANDOM = 8,      //Bank 1: 8 - 15
    MISCREG_VPE_CONTROL,
    MISCREG_VPE_CONF0,
    MISCREG_VPE_CONF1,
    MISCREG_YQMASK,
    MISCREG_VPE_SCHEDULE,
    MISCREG_VPE_SCHEFBACK,
    MISCREG_VPE_OPT,

    MISCREG_ENTRYLO0 = 16,   //Bank 2: 16 - 23
    MISCREG_TC_STATUS,
    MISCREG_TC_BIND,
    MISCREG_TC_RESTART,
    MISCREG_TC_HALT,
    MISCREG_TC_CONTEXT,
    MISCREG_TC_SCHEDULE,
    MISCREG_TC_SCHEFBACK,

    MISCREG_ENTRYLO1 = 24,   // Bank 3: 24

    MISCREG_CONTEXT = 32,    // Bank 4: 32 - 33
    MISCREG_CONTEXT_CONFIG,

    MISCREG_PAGEMASK = 40, //Bank 5: 40 - 41
    MISCREG_PAGEGRAIN = 41,

    MISCREG_WIRED = 48,          //Bank 6:48-55
    MISCREG_SRS_CONF0,
    MISCREG_SRS_CONF1,
    MISCREG_SRS_CONF2,
    MISCREG_SRS_CONF3,
    MISCREG_SRS_CONF4,

    MISCREG_HWRENA = 56,         //Bank 7: 56-63

    MISCREG_BADVADDR = 64,       //Bank 8: 64-71

    MISCREG_COUNT = 72,          //Bank 9: 72-79

    MISCREG_ENTRYHI = 80,        //Bank 10: 80-87

    MISCREG_COMPARE = 88,        //Bank 11: 88-95

    MISCREG_STATUS = 96,         //Bank 12: 96-103
    MISCREG_INTCTL,
    MISCREG_SRSCTL,
    MISCREG_SRSMAP,

    MISCREG_CAUSE = 104,         //Bank 13: 104-111

    MISCREG_EPC = 112,           //Bank 14: 112-119

    MISCREG_PRID = 120,          //Bank 15: 120-127,
    MISCREG_EBASE,

    MISCREG_CONFIG = 128,        //Bank 16: 128-135
    MISCREG_CONFIG1,
    MISCREG_CONFIG2,
    MISCREG_CONFIG3,
    MISCREG_CONFIG4,
    MISCREG_CONFIG5,
    MISCREG_CONFIG6,
    MISCREG_CONFIG7,


    MISCREG_LLADDR = 136,        //Bank 17: 136-143

    MISCREG_WATCHLO0 = 144,      //Bank 18: 144-151
    MISCREG_WATCHLO1,
    MISCREG_WATCHLO2,
    MISCREG_WATCHLO3,
    MISCREG_WATCHLO4,
    MISCREG_WATCHLO5,
    MISCREG_WATCHLO6,
    MISCREG_WATCHLO7,

    MISCREG_WATCHHI0 = 152,     //Bank 19: 152-159
    MISCREG_WATCHHI1,
    MISCREG_WATCHHI2,
    MISCREG_WATCHHI3,
    MISCREG_WATCHHI4,
    MISCREG_WATCHHI5,
    MISCREG_WATCHHI6,
    MISCREG_WATCHHI7,

    MISCREG_XCCONTEXT64 = 160, //Bank 20: 160-167

                       //Bank 21: 168-175

                       //Bank 22: 176-183

    MISCREG_DEBUG = 184,       //Bank 23: 184-191
    MISCREG_TRACE_CONTROL1,
    MISCREG_TRACE_CONTROL2,
    MISCREG_USER_TRACE_DATA,
    MISCREG_TRACE_BPC,

    MISCREG_DEPC = 192,        //Bank 24: 192-199

    MISCREG_PERFCNT0 = 200,    //Bank 25: 200-207
    MISCREG_PERFCNT1,
    MISCREG_PERFCNT2,
    MISCREG_PERFCNT3,
    MISCREG_PERFCNT4,
    MISCREG_PERFCNT5,
    MISCREG_PERFCNT6,
    MISCREG_PERFCNT7,

    MISCREG_ERRCTL = 208,      //Bank 26: 208-215

    MISCREG_CACHEERR0 = 216,   //Bank 27: 216-223
    MISCREG_CACHEERR1,
    MISCREG_CACHEERR2,
    MISCREG_CACHEERR3,

    MISCREG_TAGLO0 = 224,      //Bank 28: 224-231
    MISCREG_DATALO1,
    MISCREG_TAGLO2,
    MISCREG_DATALO3,
    MISCREG_TAGLO4,
    MISCREG_DATALO5,
    MISCREG_TAGLO6,
    MISCREG_DATALO7,

    MISCREG_TAGHI0 = 232,      //Bank 29: 232-239
    MISCREG_DATAHI1,
    MISCREG_TAGHI2,
    MISCREG_DATAHI3,
    MISCREG_TAGHI4,
    MISCREG_DATAHI5,
    MISCREG_TAGHI6,
    MISCREG_DATAHI7,


    MISCREG_ERROR_EPC = 240,    //Bank 30: 240-247

    MISCREG_DESAVE = 248,       //Bank 31: 248-256

    MISCREG_LLFLAG = 257,
    MISCREG_TP_VALUE,

    MISCREG_NUMREGS
};

const int NumMiscRegs = MISCREG_NUMREGS;

const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;

// Not applicable to MIPS
using VecElem = ::DummyVecElem;
using VecReg = ::DummyVecReg;
using ConstVecReg = ::DummyConstVecReg;
using VecRegContainer = ::DummyVecRegContainer;
constexpr unsigned NumVecElemPerVecReg = ::DummyNumVecElemPerVecReg;
constexpr size_t VecRegSizeBytes = ::DummyVecRegSizeBytes;

// Not applicable to MIPS
using VecPredReg = ::DummyVecPredReg;
using ConstVecPredReg = ::DummyConstVecPredReg;
using VecPredRegContainer = ::DummyVecPredRegContainer;
constexpr size_t VecPredRegSizeBits = ::DummyVecPredRegSizeBits;
constexpr bool VecPredRegHasPackedRepr = ::DummyVecPredRegHasPackedRepr;

} // namespace MipsISA

#endif
