/*
 * Copyright (c) 2010-2014 ARM Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#include <cassert>

#ifndef __ARCH_ARM_REGS_INT_HH__
#define __ARCH_ARM_REGS_INT_HH__

#include "arch/arm/types.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "debug/IntRegs.hh"
#include "sim/core.hh"

namespace gem5
{

namespace ArmISA
{

BitUnion32(PackedIntReg)
    Bitfield<31, 16> uh1;
    Bitfield<15, 0> uh0;
    SignedBitfield<31, 16> sh1;
    SignedBitfield<15, 0> sh0;
    Bitfield<31, 0> uw;
    SignedBitfield<31, 0> sw;
EndBitUnion(PackedIntReg)

namespace int_reg
{

enum : RegIndex
{
    /* All the unique register indices. */
    _R0Idx,
    _R1Idx,
    _R2Idx,
    _R3Idx,
    _R4Idx,
    _R5Idx,
    _R6Idx,
    _R7Idx,
    _R8Idx,
    _R9Idx,
    _R10Idx,
    _R11Idx,
    _R12Idx,
    _R13Idx,
    _R14Idx,
    _R15Idx,

    _R13SvcIdx,
    _R14SvcIdx,

    _R13MonIdx,
    _R14MonIdx,

    _R13HypIdx,

    _R13AbtIdx,
    _R14AbtIdx,

    _R13UndIdx,
    _R14UndIdx,

    _R13IrqIdx,
    _R14IrqIdx,

    _R8FiqIdx,
    _R9FiqIdx,
    _R10FiqIdx,
    _R11FiqIdx,
    _R12FiqIdx,
    _R13FiqIdx,
    _R14FiqIdx,

    _ZeroIdx,
    _Ureg0Idx,
    _Ureg1Idx,
    _Ureg2Idx,

    _Sp0Idx,
    _Sp1Idx,
    _Sp2Idx,
    _Sp3Idx,

    NumRegs,
    _SpxIdx = NumRegs,

    NumArchRegs = 32,

    _X0Idx = 0,
    _X1Idx,
    _X2Idx,
    _X3Idx,
    _X4Idx,
    _X5Idx,
    _X6Idx,
    _X7Idx,
    _X8Idx,
    _X9Idx,
    _X10Idx,
    _X11Idx,
    _X12Idx,
    _X13Idx,
    _X14Idx,
    _X15Idx,
    _X16Idx,
    _X17Idx,
    _X18Idx,
    _X19Idx,
    _X20Idx,
    _X21Idx,
    _X22Idx,
    _X23Idx,
    _X24Idx,
    _X25Idx,
    _X26Idx,
    _X27Idx,
    _X28Idx,
    _X29Idx,
    _X30Idx,
    _X31Idx
};

} // namespace int_reg

class IntRegClassOps : public RegClassOps
{
    RegId flatten(const BaseISA &isa, const RegId &id) const override;
};

inline constexpr IntRegClassOps intRegClassOps;

inline constexpr RegClass intRegClass =
    RegClass(IntRegClass, IntRegClassName, int_reg::NumRegs, debug::IntRegs).
    ops(intRegClassOps).
    needsFlattening();

inline constexpr RegClass flatIntRegClass =
    RegClass(IntRegClass, IntRegClassName, int_reg::NumRegs, debug::IntRegs);

namespace int_reg
{

inline constexpr RegId
    /* All the unique register indices. */
    R0 = intRegClass[_R0Idx],
    R1 = intRegClass[_R1Idx],
    R2 = intRegClass[_R2Idx],
    R3 = intRegClass[_R3Idx],
    R4 = intRegClass[_R4Idx],
    R5 = intRegClass[_R5Idx],
    R6 = intRegClass[_R6Idx],
    R7 = intRegClass[_R7Idx],
    R8 = intRegClass[_R8Idx],
    R9 = intRegClass[_R9Idx],
    R10 = intRegClass[_R10Idx],
    R11 = intRegClass[_R11Idx],
    R12 = intRegClass[_R12Idx],
    R13 = intRegClass[_R13Idx],
    R14 = intRegClass[_R14Idx],
    R15 = intRegClass[_R15Idx],

    R13Svc = intRegClass[_R13SvcIdx],
    R14Svc = intRegClass[_R14SvcIdx],

    R13Mon = intRegClass[_R13MonIdx],
    R14Mon = intRegClass[_R14MonIdx],

    R13Hyp = intRegClass[_R13HypIdx],

    R13Abt = intRegClass[_R13AbtIdx],
    R14Abt = intRegClass[_R14AbtIdx],

    R13Und = intRegClass[_R13UndIdx],
    R14Und = intRegClass[_R14UndIdx],

    R13Irq = intRegClass[_R13IrqIdx],
    R14Irq = intRegClass[_R14IrqIdx],

    R8Fiq = intRegClass[_R8FiqIdx],
    R9Fiq = intRegClass[_R9FiqIdx],
    R10Fiq = intRegClass[_R10FiqIdx],
    R11Fiq = intRegClass[_R11FiqIdx],
    R12Fiq = intRegClass[_R12FiqIdx],
    R13Fiq = intRegClass[_R13FiqIdx],
    R14Fiq = intRegClass[_R14FiqIdx],

    Zero = intRegClass[_ZeroIdx],
    Ureg0 = intRegClass[_Ureg0Idx],
    Ureg1 = intRegClass[_Ureg1Idx],
    Ureg2 = intRegClass[_Ureg2Idx],

    Sp0 = intRegClass[_Sp0Idx],
    Sp1 = intRegClass[_Sp1Idx],
    Sp2 = intRegClass[_Sp2Idx],
    Sp3 = intRegClass[_Sp3Idx],

    Spx = intRegClass[_SpxIdx],

    X0 = intRegClass[_X0Idx],
    X1 = intRegClass[_X1Idx],
    X2 = intRegClass[_X2Idx],
    X3 = intRegClass[_X3Idx],
    X4 = intRegClass[_X4Idx],
    X5 = intRegClass[_X5Idx],
    X6 = intRegClass[_X6Idx],
    X7 = intRegClass[_X7Idx],
    X8 = intRegClass[_X8Idx],
    X9 = intRegClass[_X9Idx],
    X10 = intRegClass[_X10Idx],
    X11 = intRegClass[_X11Idx],
    X12 = intRegClass[_X12Idx],
    X13 = intRegClass[_X13Idx],
    X14 = intRegClass[_X14Idx],
    X15 = intRegClass[_X15Idx],
    X16 = intRegClass[_X16Idx],
    X17 = intRegClass[_X17Idx],
    X18 = intRegClass[_X18Idx],
    X19 = intRegClass[_X19Idx],
    X20 = intRegClass[_X20Idx],
    X21 = intRegClass[_X21Idx],
    X22 = intRegClass[_X22Idx],
    X23 = intRegClass[_X23Idx],
    X24 = intRegClass[_X24Idx],
    X25 = intRegClass[_X25Idx],
    X26 = intRegClass[_X26Idx],
    X27 = intRegClass[_X27Idx],
    X28 = intRegClass[_X28Idx],
    X29 = intRegClass[_X29Idx],
    X30 = intRegClass[_X30Idx],
    X31 = intRegClass[_X31Idx];

inline constexpr auto
    &Sp = R13,
    &Lr = R14,
    &Pc = R15,

    &SpSvc = R13Svc,
    &LRSvc = R14Svc,

    &SPMon = R13Mon,
    &LRMon = R14Mon,

    &SPHyp = R13Hyp,

    &SPAbt = R13Abt,
    &LRAbt = R14Abt,

    &SPUnd = R13Und,
    &LRUnd = R14Und,

    &SPIrq = R13Irq,
    &LRIrq = R14Irq,

    &SPFiq = R13Fiq,
    &LRFiq = R14Fiq,

    /* USR mode */
    &R0Usr = R0,
    &R1Usr = R1,
    &R2Usr = R2,
    &R3Usr = R3,
    &R4Usr = R4,
    &R5Usr = R5,
    &R6Usr = R6,
    &R7Usr = R7,
    &R8Usr = R8,
    &R9Usr = R9,
    &R10Usr = R10,
    &R11Usr = R11,
    &R12Usr = R12,
    &R13Usr = R13,
    &SPUsr = Sp,
    &R14Usr = R14,
    &LRUsr = Lr,
    &R15Usr = R15,
    &PcUsr = Pc,

    /* SVC mode */
    &R0Svc = R0,
    &R1Svc = R1,
    &R2Svc = R2,
    &R3Svc = R3,
    &R4Svc = R4,
    &R5Svc = R5,
    &R6Svc = R6,
    &R7Svc = R7,
    &R8Svc = R8,
    &R9Svc = R9,
    &R10Svc = R10,
    &R11Svc = R11,
    &R12Svc = R12,
    &PcSvc = Pc,
    &R15Svc = R15,

    /* MON mode */
    &R0Mon = R0,
    &R1Mon = R1,
    &R2Mon = R2,
    &R3Mon = R3,
    &R4Mon = R4,
    &R5Mon = R5,
    &R6Mon = R6,
    &R7Mon = R7,
    &R8Mon = R8,
    &R9Mon = R9,
    &R10Mon = R10,
    &R11Mon = R11,
    &R12Mon = R12,
    &PcMon = Pc,
    &R15Mon = R15,

    /* ABT mode */
    &R0Abt = R0,
    &R1Abt = R1,
    &R2Abt = R2,
    &R3Abt = R3,
    &R4Abt = R4,
    &R5Abt = R5,
    &R6Abt = R6,
    &R7Abt = R7,
    &R8Abt = R8,
    &R9Abt = R9,
    &R10Abt = R10,
    &R11Abt = R11,
    &R12Abt = R12,
    &PcAbt = Pc,
    &R15Abt = R15,

    /* HYP mode */
    &R0Hyp = R0,
    &R1Hyp = R1,
    &R2Hyp = R2,
    &R3Hyp = R3,
    &R4Hyp = R4,
    &R5Hyp = R5,
    &R6Hyp = R6,
    &R7Hyp = R7,
    &R8Hyp = R8,
    &R9Hyp = R9,
    &R10Hyp = R10,
    &R11Hyp = R11,
    &R12Hyp = R12,
    &LRHyp = Lr,
    &R14Hyp = R14,
    &PcHyp = Pc,
    &R15Hyp = R15,

    /* UND mode */
    &R0Und = R0,
    &R1Und = R1,
    &R2Und = R2,
    &R3Und = R3,
    &R4Und = R4,
    &R5Und = R5,
    &R6Und = R6,
    &R7Und = R7,
    &R8Und = R8,
    &R9Und = R9,
    &R10Und = R10,
    &R11Und = R11,
    &R12Und = R12,
    &PcUnd = Pc,
    &R15Und = R15,

    /* IRQ mode */
    &R0Irq = R0,
    &R1Irq = R1,
    &R2Irq = R2,
    &R3Irq = R3,
    &R4Irq = R4,
    &R5Irq = R5,
    &R6Irq = R6,
    &R7Irq = R7,
    &R8Irq = R8,
    &R9Irq = R9,
    &R10Irq = R10,
    &R11Irq = R11,
    &R12Irq = R12,
    &PcIrq = Pc,
    &R15Irq = R15,

    /* FIQ mode */
    &R0Fiq = R0,
    &R1Fiq = R1,
    &R2Fiq = R2,
    &R3Fiq = R3,
    &R4Fiq = R4,
    &R5Fiq = R5,
    &R6Fiq = R6,
    &R7Fiq = R7,
    &PcFiq = Pc,
    &R15Fiq = R15;

typedef const RegId RegMap[NumArchRegs];

const RegMap Reg64Map = {
    R0,     R1,     R2,     R3,     R4,     R5,     R6,     R7,
    R8Usr,  R9Usr,  R10Usr, R11Usr, R12Usr, R13Usr, R14Usr, R13Hyp,
    R14Irq, R13Irq, R14Svc, R13Svc, R14Abt, R13Abt, R14Und, R13Und,
    R8Fiq,  R9Fiq,  R10Fiq, R11Fiq, R12Fiq, R13Fiq, R14Fiq, Zero
};

static inline RegId
x(unsigned index)
{
    assert(index < NumArchRegs);
    return intRegClass[_X0Idx + index];
}

const RegMap RegUsrMap = {
    R0Usr,  R1Usr,  R2Usr,  R3Usr,  R4Usr,  R5Usr,  R6Usr,  R7Usr,
    R8Usr,  R9Usr,  R10Usr, R11Usr, R12Usr, R13Usr, R14Usr, R15Usr,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
usr(unsigned index)
{
    assert(index < NumArchRegs);
    return RegUsrMap[index];
}

const RegMap RegHypMap = {
    R0Hyp,  R1Hyp,  R2Hyp,  R3Hyp,  R4Hyp,  R5Hyp,  R6Hyp,  R7Hyp,
    R8Hyp,  R9Hyp,  R10Hyp, R11Hyp, R12Hyp, R13Hyp, R14Hyp, R15Hyp,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
hyp(unsigned index)
{
    assert(index < NumArchRegs);
    return RegHypMap[index];
}

const RegMap RegSvcMap = {
    R0Svc,  R1Svc,  R2Svc,  R3Svc,  R4Svc,  R5Svc,  R6Svc,  R7Svc,
    R8Svc,  R9Svc,  R10Svc, R11Svc, R12Svc, R13Svc, R14Svc, R15Svc,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
svc(unsigned index)
{
    assert(index < NumArchRegs);
    return RegSvcMap[index];
}

const RegMap RegMonMap = {
    R0Mon,  R1Mon,  R2Mon,  R3Mon,  R4Mon,  R5Mon,  R6Mon,  R7Mon,
    R8Mon,  R9Mon,  R10Mon, R11Mon, R12Mon, R13Mon, R14Mon, R15Mon,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
mon(unsigned index)
{
    assert(index < NumArchRegs);
    return RegMonMap[index];
}

const RegMap RegAbtMap = {
    R0Abt,  R1Abt,  R2Abt,  R3Abt,  R4Abt,  R5Abt,  R6Abt,  R7Abt,
    R8Abt,  R9Abt,  R10Abt, R11Abt, R12Abt, R13Abt, R14Abt, R15Abt,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
abt(unsigned index)
{
    assert(index < NumArchRegs);
    return RegAbtMap[index];
}

const RegMap RegUndMap = {
    R0Und,  R1Und,  R2Und,  R3Und,  R4Und,  R5Und,  R6Und,  R7Und,
    R8Und,  R9Und,  R10Und, R11Und, R12Und, R13Und, R14Und, R15Und,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
und(unsigned index)
{
    assert(index < NumArchRegs);
    return RegUndMap[index];
}

const RegMap RegIrqMap = {
    R0Irq,  R1Irq,  R2Irq,  R3Irq,  R4Irq,  R5Irq,  R6Irq,  R7Irq,
    R8Irq,  R9Irq,  R10Irq, R11Irq, R12Irq, R13Irq, R14Irq, R15Irq,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
irq(unsigned index)
{
    assert(index < NumArchRegs);
    return RegIrqMap[index];
}

const RegMap RegFiqMap = {
    R0Fiq,  R1Fiq,  R2Fiq,  R3Fiq,  R4Fiq,  R5Fiq,  R6Fiq,  R7Fiq,
    R8Fiq,  R9Fiq,  R10Fiq, R11Fiq, R12Fiq, R13Fiq, R14Fiq, R15Fiq,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,
    Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero,   Zero
};

static inline const RegId &
fiq(unsigned index)
{
    assert(index < NumArchRegs);
    return RegFiqMap[index];
}

static const unsigned regsPerMode = NumRegs;

static inline int
regInMode(OperatingMode mode, int reg)
{
    assert(reg < NumArchRegs);
    return mode * regsPerMode + reg;
}

} // namespace int_reg

static inline const RegId &
flattenIntRegModeIndex(int reg)
{
    int mode = reg / int_reg::regsPerMode;
    reg = reg % int_reg::regsPerMode;
    switch (mode) {
      case MODE_USER:
      case MODE_SYSTEM:
        return int_reg::usr(reg);
      case MODE_FIQ:
        return int_reg::fiq(reg);
      case MODE_IRQ:
        return int_reg::irq(reg);
      case MODE_SVC:
        return int_reg::svc(reg);
      case MODE_MON:
        return int_reg::mon(reg);
      case MODE_ABORT:
        return int_reg::abt(reg);
      case MODE_HYP:
        return int_reg::hyp(reg);
      case MODE_UNDEFINED:
        return int_reg::und(reg);
      default:
        panic("%d: Flattening into an unknown mode: reg:%#x mode:%#x\n",
                curTick(), reg, mode);
    }
}


static inline RegIndex
makeSP(RegIndex reg)
{
    if (reg == int_reg::X31)
        reg = int_reg::Spx;
    return reg;
}

static inline bool
couldBeSP(RegIndex reg)
{
    return (reg == int_reg::X31 || reg == int_reg::Spx);
}

static inline bool
isSP(RegIndex reg)
{
    return reg == int_reg::Spx;
}

static inline bool
couldBeZero(RegIndex reg)
{
    return (reg == int_reg::X31 || reg == int_reg::Zero);
}

static inline bool
isZero(RegIndex reg)
{
    return reg == int_reg::Zero;
}

static inline RegIndex
makeZero(RegIndex reg)
{
    if (reg == int_reg::X31)
        reg = int_reg::Zero;
    return reg;
}

// Semantically meaningful register indices
inline constexpr size_t NumArgumentRegs = 4;
inline constexpr size_t NumArgumentRegs64 = 8;
inline constexpr auto
    &ReturnValueReg = int_reg::X0,
    &ReturnValueReg1 = int_reg::X1,
    &ArgumentReg0 = int_reg::X0,
    &ArgumentReg1 = int_reg::X1,
    &ArgumentReg2 = int_reg::X2,
    &FramePointerReg = int_reg::X11,
    &StackPointerReg = int_reg::Sp,
    &ReturnAddressReg = int_reg::Lr,

    &SyscallNumReg = ReturnValueReg,
    &SyscallPseudoReturnReg = ReturnValueReg,
    &SyscallSuccessReg = ReturnValueReg;

} // namespace ArmISA
} // namespace gem5

#endif
