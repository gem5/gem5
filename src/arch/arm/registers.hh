/*
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

#include "arch/arm/max_inst_regs.hh"
#include "arch/arm/miscregs.hh"

namespace ArmISA {

using ArmISAInst::MaxInstSrcRegs;
using ArmISAInst::MaxInstDestRegs;

typedef uint8_t  RegIndex;

typedef uint64_t IntReg;

// floating point register file entry type
typedef uint32_t FloatRegBits;
typedef float FloatReg;

// cop-0/cop-1 system control register
typedef uint64_t MiscReg;

// Constants Related to the number of registers
const int NumIntArchRegs = 16;
const int NumIntSpecialRegs = 19;
const int NumFloatArchRegs = 16;
const int NumFloatSpecialRegs = 5;
const int NumInternalProcRegs = 0;

const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;
const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;

const int NumMiscRegs = NUM_MISCREGS;


// semantically meaningful register indices
const int ReturnValueReg = 0;
const int ReturnValueReg1 = 1;
const int ReturnValueReg2 = 2;
const int ArgumentReg0 = 0;
const int ArgumentReg1 = 1;
const int ArgumentReg2 = 2;
const int ArgumentReg3 = 3;
const int FramePointerReg = 11;
const int StackPointerReg = 13;
const int ReturnAddressReg = 14;
const int PCReg = 15;

const int ZeroReg = NumIntArchRegs;
const int AddrReg = ZeroReg + 1; // Used to generate address for uops

const int SyscallNumReg = ReturnValueReg;
const int SyscallPseudoReturnReg = ReturnValueReg;
const int SyscallSuccessReg = ReturnValueReg;

// These help enumerate all the registers for dependence tracking.
const int FP_Base_DepTag = NumIntRegs;
const int Ctrl_Base_DepTag = FP_Base_DepTag + NumFloatRegs;

typedef union {
    IntReg   intreg;
    FloatReg fpreg;
    MiscReg  ctrlreg;
} AnyReg;

enum FPControlRegNums {
   FIR = NumFloatArchRegs,
   FCCR,
   FEXR,
   FENR,
   FCSR
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
    zero_reg = NumIntArchRegs,
    addr_reg,

    rhi,
    rlo,

    r8_fiq,    /* FIQ mode register bank */
    r9_fiq,
    r10_fiq,
    r11_fiq,
    r12_fiq,

    r13_fiq,   /* FIQ mode SP and LR */
    r14_fiq,

    r13_irq,   /* IRQ mode SP and LR */
    r14_irq,

    r13_svc,   /* SVC mode SP and LR */
    r14_svc,

    r13_undef, /* UNDEF mode SP and LR */
    r14_undef,

    r13_abt,   /* ABT mode SP and LR */
    r14_abt
};

} // namespace ArmISA

#endif
