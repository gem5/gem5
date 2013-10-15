/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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

#ifndef __ARCH_X86_INTREGS_HH__
#define __ARCH_X86_INTREGS_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "base/misc.hh"
#include "sim/core.hh"

namespace X86ISA
{
    BitUnion64(X86IntReg)
        Bitfield<63,0> R;
        SignedBitfield<63,0> SR;
        Bitfield<31,0> E;
        SignedBitfield<31,0> SE;
        Bitfield<15,0> X;
        SignedBitfield<15,0> SX;
        Bitfield<15,8> H;
        SignedBitfield<15,8> SH;
        Bitfield<7, 0> L;
        SignedBitfield<7, 0> SL;
    EndBitUnion(X86IntReg)

    enum IntRegIndex
    {
        INTREG_RAX,
        INTREG_EAX = INTREG_RAX,
        INTREG_AX = INTREG_RAX,
        INTREG_AL = INTREG_RAX,

        INTREG_RCX,
        INTREG_ECX = INTREG_RCX,
        INTREG_CX = INTREG_RCX,
        INTREG_CL = INTREG_RCX,

        INTREG_RDX,
        INTREG_EDX = INTREG_RDX,
        INTREG_DX = INTREG_RDX,
        INTREG_DL = INTREG_RDX,

        INTREG_RBX,
        INTREG_EBX = INTREG_RBX,
        INTREG_BX = INTREG_RBX,
        INTREG_BL = INTREG_RBX,

        INTREG_RSP,
        INTREG_ESP = INTREG_RSP,
        INTREG_SP = INTREG_RSP,
        INTREG_SPL = INTREG_RSP,
        INTREG_AH = INTREG_RSP,

        INTREG_RBP,
        INTREG_EBP = INTREG_RBP,
        INTREG_BP = INTREG_RBP,
        INTREG_BPL = INTREG_RBP,
        INTREG_CH = INTREG_RBP,

        INTREG_RSI,
        INTREG_ESI = INTREG_RSI,
        INTREG_SI = INTREG_RSI,
        INTREG_SIL = INTREG_RSI,
        INTREG_DH = INTREG_RSI,

        INTREG_RDI,
        INTREG_EDI = INTREG_RDI,
        INTREG_DI = INTREG_RDI,
        INTREG_DIL = INTREG_RDI,
        INTREG_BH = INTREG_RDI,

        INTREG_R8,
        INTREG_R8D = INTREG_R8,
        INTREG_R8W = INTREG_R8,
        INTREG_R8B = INTREG_R8,

        INTREG_R9,
        INTREG_R9D = INTREG_R9,
        INTREG_R9W = INTREG_R9,
        INTREG_R9B = INTREG_R9,

        INTREG_R10,
        INTREG_R10D = INTREG_R10,
        INTREG_R10W = INTREG_R10,
        INTREG_R10B = INTREG_R10,

        INTREG_R11,
        INTREG_R11D = INTREG_R11,
        INTREG_R11W = INTREG_R11,
        INTREG_R11B = INTREG_R11,

        INTREG_R12,
        INTREG_R12D = INTREG_R12,
        INTREG_R12W = INTREG_R12,
        INTREG_R12B = INTREG_R12,

        INTREG_R13,
        INTREG_R13D = INTREG_R13,
        INTREG_R13W = INTREG_R13,
        INTREG_R13B = INTREG_R13,

        INTREG_R14,
        INTREG_R14D = INTREG_R14,
        INTREG_R14W = INTREG_R14,
        INTREG_R14B = INTREG_R14,

        INTREG_R15,
        INTREG_R15D = INTREG_R15,
        INTREG_R15W = INTREG_R15,
        INTREG_R15B = INTREG_R15,

        NUM_INTREGS
    };

    // This needs to be large enough to miss all the other bits of an index.
    static const IntRegIndex IntFoldBit = (IntRegIndex)(1 << 6);

    inline static IntRegIndex
    INTREG_MICRO(int index)
    {
        return (IntRegIndex)(NUM_INTREGS + index);
    }

    inline static IntRegIndex
    INTREG_IMPLICIT(int index)
    {
        return (IntRegIndex)(NUM_INTREGS + NumMicroIntRegs + index);
    }

    inline static IntRegIndex
    INTREG_FOLDED(int index, int foldBit)
    {
        if ((index & 0x1C) == 4 && foldBit)
            index = (index - 4) | foldBit;
        return (IntRegIndex)index;
    }
}

#endif // __ARCH_X86_INTREGS_HH__
