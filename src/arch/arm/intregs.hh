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
 *
 * Authors: Gabe Black
 */

#include <cassert>

#ifndef __ARCH_ARM_INTREGS_HH__
#define __ARCH_ARM_INTREGS_HH__

#include "arch/arm/types.hh"

namespace ArmISA
{

enum IntRegIndex
{
    /* All the unique register indices. */
    INTREG_R0,
    INTREG_R1,
    INTREG_R2,
    INTREG_R3,
    INTREG_R4,
    INTREG_R5,
    INTREG_R6,
    INTREG_R7,
    INTREG_R8,
    INTREG_R9,
    INTREG_R10,
    INTREG_R11,
    INTREG_R12,
    INTREG_R13,
    INTREG_SP = INTREG_R13,
    INTREG_R14,
    INTREG_LR = INTREG_R14,
    INTREG_R15,
    INTREG_PC = INTREG_R15,

    INTREG_R13_SVC,
    INTREG_SP_SVC = INTREG_R13_SVC,
    INTREG_R14_SVC,
    INTREG_LR_SVC = INTREG_R14_SVC,

    INTREG_R13_MON,
    INTREG_SP_MON = INTREG_R13_MON,
    INTREG_R14_MON,
    INTREG_LR_MON = INTREG_R14_MON,

    INTREG_R13_HYP,
    INTREG_SP_HYP = INTREG_R13_HYP,

    INTREG_R13_ABT,
    INTREG_SP_ABT = INTREG_R13_ABT,
    INTREG_R14_ABT,
    INTREG_LR_ABT = INTREG_R14_ABT,

    INTREG_R13_UND,
    INTREG_SP_UND = INTREG_R13_UND,
    INTREG_R14_UND,
    INTREG_LR_UND = INTREG_R14_UND,

    INTREG_R13_IRQ,
    INTREG_SP_IRQ = INTREG_R13_IRQ,
    INTREG_R14_IRQ,
    INTREG_LR_IRQ = INTREG_R14_IRQ,

    INTREG_R8_FIQ,
    INTREG_R9_FIQ,
    INTREG_R10_FIQ,
    INTREG_R11_FIQ,
    INTREG_R12_FIQ,
    INTREG_R13_FIQ,
    INTREG_SP_FIQ = INTREG_R13_FIQ,
    INTREG_R14_FIQ,
    INTREG_LR_FIQ = INTREG_R14_FIQ,

    INTREG_ZERO,
    INTREG_UREG0,
    INTREG_UREG1,
    INTREG_UREG2,
    INTREG_DUMMY, // Dummy reg used to throw away int reg results

    INTREG_SP0,
    INTREG_SP1,
    INTREG_SP2,
    INTREG_SP3,

    NUM_INTREGS,
    NUM_ARCH_INTREGS = 32,

    /* AArch64 registers */
    INTREG_X0 = 0,
    INTREG_X1,
    INTREG_X2,
    INTREG_X3,
    INTREG_X4,
    INTREG_X5,
    INTREG_X6,
    INTREG_X7,
    INTREG_X8,
    INTREG_X9,
    INTREG_X10,
    INTREG_X11,
    INTREG_X12,
    INTREG_X13,
    INTREG_X14,
    INTREG_X15,
    INTREG_X16,
    INTREG_X17,
    INTREG_X18,
    INTREG_X19,
    INTREG_X20,
    INTREG_X21,
    INTREG_X22,
    INTREG_X23,
    INTREG_X24,
    INTREG_X25,
    INTREG_X26,
    INTREG_X27,
    INTREG_X28,
    INTREG_X29,
    INTREG_X30,
    INTREG_X31,

    INTREG_SPX = NUM_INTREGS,

    /* All the aliased indexes. */

    /* USR mode */
    INTREG_R0_USR = INTREG_R0,
    INTREG_R1_USR = INTREG_R1,
    INTREG_R2_USR = INTREG_R2,
    INTREG_R3_USR = INTREG_R3,
    INTREG_R4_USR = INTREG_R4,
    INTREG_R5_USR = INTREG_R5,
    INTREG_R6_USR = INTREG_R6,
    INTREG_R7_USR = INTREG_R7,
    INTREG_R8_USR = INTREG_R8,
    INTREG_R9_USR = INTREG_R9,
    INTREG_R10_USR = INTREG_R10,
    INTREG_R11_USR = INTREG_R11,
    INTREG_R12_USR = INTREG_R12,
    INTREG_R13_USR = INTREG_R13,
    INTREG_SP_USR = INTREG_SP,
    INTREG_R14_USR = INTREG_R14,
    INTREG_LR_USR = INTREG_LR,
    INTREG_R15_USR = INTREG_R15,
    INTREG_PC_USR = INTREG_PC,

    /* SVC mode */
    INTREG_R0_SVC = INTREG_R0,
    INTREG_R1_SVC = INTREG_R1,
    INTREG_R2_SVC = INTREG_R2,
    INTREG_R3_SVC = INTREG_R3,
    INTREG_R4_SVC = INTREG_R4,
    INTREG_R5_SVC = INTREG_R5,
    INTREG_R6_SVC = INTREG_R6,
    INTREG_R7_SVC = INTREG_R7,
    INTREG_R8_SVC = INTREG_R8,
    INTREG_R9_SVC = INTREG_R9,
    INTREG_R10_SVC = INTREG_R10,
    INTREG_R11_SVC = INTREG_R11,
    INTREG_R12_SVC = INTREG_R12,
    INTREG_PC_SVC = INTREG_PC,
    INTREG_R15_SVC = INTREG_R15,

    /* MON mode */
    INTREG_R0_MON = INTREG_R0,
    INTREG_R1_MON = INTREG_R1,
    INTREG_R2_MON = INTREG_R2,
    INTREG_R3_MON = INTREG_R3,
    INTREG_R4_MON = INTREG_R4,
    INTREG_R5_MON = INTREG_R5,
    INTREG_R6_MON = INTREG_R6,
    INTREG_R7_MON = INTREG_R7,
    INTREG_R8_MON = INTREG_R8,
    INTREG_R9_MON = INTREG_R9,
    INTREG_R10_MON = INTREG_R10,
    INTREG_R11_MON = INTREG_R11,
    INTREG_R12_MON = INTREG_R12,
    INTREG_PC_MON = INTREG_PC,
    INTREG_R15_MON = INTREG_R15,

    /* ABT mode */
    INTREG_R0_ABT = INTREG_R0,
    INTREG_R1_ABT = INTREG_R1,
    INTREG_R2_ABT = INTREG_R2,
    INTREG_R3_ABT = INTREG_R3,
    INTREG_R4_ABT = INTREG_R4,
    INTREG_R5_ABT = INTREG_R5,
    INTREG_R6_ABT = INTREG_R6,
    INTREG_R7_ABT = INTREG_R7,
    INTREG_R8_ABT = INTREG_R8,
    INTREG_R9_ABT = INTREG_R9,
    INTREG_R10_ABT = INTREG_R10,
    INTREG_R11_ABT = INTREG_R11,
    INTREG_R12_ABT = INTREG_R12,
    INTREG_PC_ABT = INTREG_PC,
    INTREG_R15_ABT = INTREG_R15,

    /* HYP mode */
    INTREG_R0_HYP = INTREG_R0,
    INTREG_R1_HYP = INTREG_R1,
    INTREG_R2_HYP = INTREG_R2,
    INTREG_R3_HYP = INTREG_R3,
    INTREG_R4_HYP = INTREG_R4,
    INTREG_R5_HYP = INTREG_R5,
    INTREG_R6_HYP = INTREG_R6,
    INTREG_R7_HYP = INTREG_R7,
    INTREG_R8_HYP = INTREG_R8,
    INTREG_R9_HYP = INTREG_R9,
    INTREG_R10_HYP = INTREG_R10,
    INTREG_R11_HYP = INTREG_R11,
    INTREG_R12_HYP = INTREG_R12,
    INTREG_LR_HYP = INTREG_LR,
    INTREG_R14_HYP = INTREG_R14,
    INTREG_PC_HYP = INTREG_PC,
    INTREG_R15_HYP = INTREG_R15,

    /* UND mode */
    INTREG_R0_UND = INTREG_R0,
    INTREG_R1_UND = INTREG_R1,
    INTREG_R2_UND = INTREG_R2,
    INTREG_R3_UND = INTREG_R3,
    INTREG_R4_UND = INTREG_R4,
    INTREG_R5_UND = INTREG_R5,
    INTREG_R6_UND = INTREG_R6,
    INTREG_R7_UND = INTREG_R7,
    INTREG_R8_UND = INTREG_R8,
    INTREG_R9_UND = INTREG_R9,
    INTREG_R10_UND = INTREG_R10,
    INTREG_R11_UND = INTREG_R11,
    INTREG_R12_UND = INTREG_R12,
    INTREG_PC_UND = INTREG_PC,
    INTREG_R15_UND = INTREG_R15,

    /* IRQ mode */
    INTREG_R0_IRQ = INTREG_R0,
    INTREG_R1_IRQ = INTREG_R1,
    INTREG_R2_IRQ = INTREG_R2,
    INTREG_R3_IRQ = INTREG_R3,
    INTREG_R4_IRQ = INTREG_R4,
    INTREG_R5_IRQ = INTREG_R5,
    INTREG_R6_IRQ = INTREG_R6,
    INTREG_R7_IRQ = INTREG_R7,
    INTREG_R8_IRQ = INTREG_R8,
    INTREG_R9_IRQ = INTREG_R9,
    INTREG_R10_IRQ = INTREG_R10,
    INTREG_R11_IRQ = INTREG_R11,
    INTREG_R12_IRQ = INTREG_R12,
    INTREG_PC_IRQ = INTREG_PC,
    INTREG_R15_IRQ = INTREG_R15,

    /* FIQ mode */
    INTREG_R0_FIQ = INTREG_R0,
    INTREG_R1_FIQ = INTREG_R1,
    INTREG_R2_FIQ = INTREG_R2,
    INTREG_R3_FIQ = INTREG_R3,
    INTREG_R4_FIQ = INTREG_R4,
    INTREG_R5_FIQ = INTREG_R5,
    INTREG_R6_FIQ = INTREG_R6,
    INTREG_R7_FIQ = INTREG_R7,
    INTREG_PC_FIQ = INTREG_PC,
    INTREG_R15_FIQ = INTREG_R15
};

typedef IntRegIndex IntRegMap[NUM_ARCH_INTREGS];

const IntRegMap IntReg64Map = {
    INTREG_R0,      INTREG_R1,      INTREG_R2,      INTREG_R3,
    INTREG_R4,      INTREG_R5,      INTREG_R6,      INTREG_R7,
    INTREG_R8_USR,  INTREG_R9_USR,  INTREG_R10_USR, INTREG_R11_USR,
    INTREG_R12_USR, INTREG_R13_USR, INTREG_R14_USR, INTREG_R13_HYP,
    INTREG_R14_IRQ, INTREG_R13_IRQ, INTREG_R14_SVC, INTREG_R13_SVC,
    INTREG_R14_ABT, INTREG_R13_ABT, INTREG_R14_UND, INTREG_R13_UND,
    INTREG_R8_FIQ,  INTREG_R9_FIQ,  INTREG_R10_FIQ, INTREG_R11_FIQ,
    INTREG_R12_FIQ, INTREG_R13_FIQ, INTREG_R14_FIQ, INTREG_ZERO
};

const IntRegMap IntRegUsrMap = {
    INTREG_R0_USR,  INTREG_R1_USR,  INTREG_R2_USR,  INTREG_R3_USR,
    INTREG_R4_USR,  INTREG_R5_USR,  INTREG_R6_USR,  INTREG_R7_USR,
    INTREG_R8_USR,  INTREG_R9_USR,  INTREG_R10_USR, INTREG_R11_USR,
    INTREG_R12_USR, INTREG_R13_USR, INTREG_R14_USR, INTREG_R15_USR,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_USR(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegUsrMap[index];
}

const IntRegMap IntRegHypMap = {
    INTREG_R0_HYP,  INTREG_R1_HYP,  INTREG_R2_HYP,  INTREG_R3_HYP,
    INTREG_R4_HYP,  INTREG_R5_HYP,  INTREG_R6_HYP,  INTREG_R7_HYP,
    INTREG_R8_HYP,  INTREG_R9_HYP,  INTREG_R10_HYP, INTREG_R11_HYP,
    INTREG_R12_HYP, INTREG_R13_HYP, INTREG_R14_HYP, INTREG_R15_HYP,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_HYP(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegHypMap[index];
}

const IntRegMap IntRegSvcMap = {
    INTREG_R0_SVC,  INTREG_R1_SVC,  INTREG_R2_SVC,  INTREG_R3_SVC,
    INTREG_R4_SVC,  INTREG_R5_SVC,  INTREG_R6_SVC,  INTREG_R7_SVC,
    INTREG_R8_SVC,  INTREG_R9_SVC,  INTREG_R10_SVC, INTREG_R11_SVC,
    INTREG_R12_SVC, INTREG_R13_SVC, INTREG_R14_SVC, INTREG_R15_SVC,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_SVC(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegSvcMap[index];
}

const IntRegMap IntRegMonMap = {
    INTREG_R0_MON,  INTREG_R1_MON,  INTREG_R2_MON,  INTREG_R3_MON,
    INTREG_R4_MON,  INTREG_R5_MON,  INTREG_R6_MON,  INTREG_R7_MON,
    INTREG_R8_MON,  INTREG_R9_MON,  INTREG_R10_MON, INTREG_R11_MON,
    INTREG_R12_MON, INTREG_R13_MON, INTREG_R14_MON, INTREG_R15_MON,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_MON(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegMonMap[index];
}

const IntRegMap IntRegAbtMap = {
    INTREG_R0_ABT,  INTREG_R1_ABT,  INTREG_R2_ABT,  INTREG_R3_ABT,
    INTREG_R4_ABT,  INTREG_R5_ABT,  INTREG_R6_ABT,  INTREG_R7_ABT,
    INTREG_R8_ABT,  INTREG_R9_ABT,  INTREG_R10_ABT, INTREG_R11_ABT,
    INTREG_R12_ABT, INTREG_R13_ABT, INTREG_R14_ABT, INTREG_R15_ABT,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_ABT(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegAbtMap[index];
}

const IntRegMap IntRegUndMap = {
    INTREG_R0_UND,  INTREG_R1_UND,  INTREG_R2_UND,  INTREG_R3_UND,
    INTREG_R4_UND,  INTREG_R5_UND,  INTREG_R6_UND,  INTREG_R7_UND,
    INTREG_R8_UND,  INTREG_R9_UND,  INTREG_R10_UND, INTREG_R11_UND,
    INTREG_R12_UND, INTREG_R13_UND, INTREG_R14_UND, INTREG_R15_UND,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_UND(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegUndMap[index];
}

const IntRegMap IntRegIrqMap = {
    INTREG_R0_IRQ,  INTREG_R1_IRQ,  INTREG_R2_IRQ,  INTREG_R3_IRQ,
    INTREG_R4_IRQ,  INTREG_R5_IRQ,  INTREG_R6_IRQ,  INTREG_R7_IRQ,
    INTREG_R8_IRQ,  INTREG_R9_IRQ,  INTREG_R10_IRQ, INTREG_R11_IRQ,
    INTREG_R12_IRQ, INTREG_R13_IRQ, INTREG_R14_IRQ, INTREG_R15_IRQ,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_IRQ(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegIrqMap[index];
}

const IntRegMap IntRegFiqMap = {
    INTREG_R0_FIQ,  INTREG_R1_FIQ,  INTREG_R2_FIQ,  INTREG_R3_FIQ,
    INTREG_R4_FIQ,  INTREG_R5_FIQ,  INTREG_R6_FIQ,  INTREG_R7_FIQ,
    INTREG_R8_FIQ,  INTREG_R9_FIQ,  INTREG_R10_FIQ, INTREG_R11_FIQ,
    INTREG_R12_FIQ, INTREG_R13_FIQ, INTREG_R14_FIQ, INTREG_R15_FIQ,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,
    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO,    INTREG_ZERO
};

static inline IntRegIndex
INTREG_FIQ(unsigned index)
{
    assert(index < NUM_ARCH_INTREGS);
    return IntRegFiqMap[index];
}

static const unsigned intRegsPerMode = NUM_INTREGS;

static inline int
intRegInMode(OperatingMode mode, int reg)
{
    assert(reg < NUM_ARCH_INTREGS);
    return mode * intRegsPerMode + reg;
}

static inline int
flattenIntRegModeIndex(int reg)
{
    int mode = reg / intRegsPerMode;
    reg = reg % intRegsPerMode;
    switch (mode) {
      case MODE_USER:
      case MODE_SYSTEM:
        return INTREG_USR(reg);
      case MODE_FIQ:
        return INTREG_FIQ(reg);
      case MODE_IRQ:
        return INTREG_IRQ(reg);
      case MODE_SVC:
        return INTREG_SVC(reg);
      case MODE_MON:
        return INTREG_MON(reg);
      case MODE_ABORT:
        return INTREG_ABT(reg);
      case MODE_HYP:
        return INTREG_HYP(reg);
      case MODE_UNDEFINED:
        return INTREG_UND(reg);
      default:
        panic("%d: Flattening into an unknown mode: reg:%#x mode:%#x\n",
                curTick(), reg, mode);
    }
}


static inline IntRegIndex
makeSP(IntRegIndex reg)
{
    if (reg == INTREG_X31)
        reg = INTREG_SPX;
    return reg;
}

static inline IntRegIndex
makeZero(IntRegIndex reg)
{
    if (reg == INTREG_X31)
        reg = INTREG_ZERO;
    return reg;
}

static inline bool
isSP(IntRegIndex reg)
{
    return reg == INTREG_SPX;
}

}

#endif
