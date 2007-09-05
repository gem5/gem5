/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#ifndef __ARCH_X86_FLOATREGS_HH__
#define __ARCH_X86_FLOATREGS_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"

namespace X86ISA
{
    enum FloatRegIndex
    {
        // MMX/X87 registers
        FLOATREG_MMX_BASE,
        FLOATREG_FPR_BASE = FLOATREG_MMX_BASE,
        FLOATREG_MMX0 = FLOATREG_MMX_BASE,
        FLOATREG_MMX1,
        FLOATREG_MMX2,
        FLOATREG_MMX3,
        FLOATREG_MMX4,
        FLOATREG_MMX5,
        FLOATREG_MMX6,
        FLOATREG_MMX7,

        FLOATREG_FPR0 = FLOATREG_FPR_BASE,
        FLOATREG_FPR1,
        FLOATREG_FPR2,
        FLOATREG_FPR3,
        FLOATREG_FPR4,
        FLOATREG_FPR5,
        FLOATREG_FPR6,
        FLOATREG_FPR7,

        FLOATREG_XMM_BASE = FLOATREG_MMX_BASE + NumMMXRegs,
        FLOATREG_XMM0_LOW = FLOATREG_XMM_BASE,
        FLOATREG_XMM0_HIGH,
        FLOATREG_XMM1_LOW,
        FLOATREG_XMM1_HIGH,
        FLOATREG_XMM2_LOW,
        FLOATREG_XMM2_HIGH,
        FLOATREG_XMM3_LOW,
        FLOATREG_XMM3_HIGH,
        FLOATREG_XMM4_LOW,
        FLOATREG_XMM4_HIGH,
        FLOATREG_XMM5_LOW,
        FLOATREG_XMM5_HIGH,
        FLOATREG_XMM6_LOW,
        FLOATREG_XMM6_HIGH,
        FLOATREG_XMM7_LOW,
        FLOATREG_XMM7_HIGH,
        FLOATREG_XMM8_LOW,
        FLOATREG_XMM8_HIGH,
        FLOATREG_XMM9_LOW,
        FLOATREG_XMM9_HIGH,
        FLOATREG_XMM10_LOW,
        FLOATREG_XMM10_HIGH,
        FLOATREG_XMM11_LOW,
        FLOATREG_XMM11_HIGH,
        FLOATREG_XMM12_LOW,
        FLOATREG_XMM12_HIGH,
        FLOATREG_XMM13_LOW,
        FLOATREG_XMM13_HIGH,
        FLOATREG_XMM14_LOW,
        FLOATREG_XMM14_HIGH,
        FLOATREG_XMM15_LOW,
        FLOATREG_XMM15_HIGH,

        NUM_FLOATREGS = FLOATREG_XMM_BASE + 2 * NumXMMRegs
    };

    static inline FloatRegIndex
    FLOATREG_MMX(int index)
    {
        return (FloatRegIndex)(FLOATREG_MMX_BASE + index);
    }

    static inline FloatRegIndex
    FLOATREG_FPR(int index)
    {
        return (FloatRegIndex)(FLOATREG_FPR_BASE + index);
    }

    static inline FloatRegIndex
    FLOATREG_XMM_LOW(int index)
    {
        return (FloatRegIndex)(FLOATREG_XMM_BASE + 2 * index);
    }

    static inline FloatRegIndex
    FLOATREG_XMM_HIGH(int index)
    {
        return (FloatRegIndex)(FLOATREG_XMM_BASE + 2 * index + 1);
    }
};

#endif // __ARCH_X86_FLOATREGS_HH__
