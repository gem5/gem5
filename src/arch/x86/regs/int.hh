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
 */

#ifndef __ARCH_X86_INTREGS_HH__
#define __ARCH_X86_INTREGS_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "base/logging.hh"

namespace gem5
{

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

namespace int_reg
{

enum : RegIndex
{
    Rax,
    Eax = Rax,
    Ax = Rax,
    Al = Rax,

    Rcx,
    Ecx = Rcx,
    Cx = Rcx,
    Cl = Rcx,

    Rdx,
    Edx = Rdx,
    Dx = Rdx,
    Dl = Rdx,

    Rbx,
    Ebx = Rbx,
    Bx = Rbx,
    Bl = Rbx,

    Rsp,
    Esp = Rsp,
    Sp = Rsp,
    Spl = Rsp,
    Ah = Rsp,

    Rbp,
    Ebp = Rbp,
    Bp = Rbp,
    Bpl = Rbp,
    Ch = Rbp,

    Rsi,
    Esi = Rsi,
    Si = Rsi,
    Sil = Rsi,
    Dh = Rsi,

    Rdi,
    Edi = Rdi,
    Di = Rdi,
    Dil = Rdi,
    Bh = Rdi,

    R8,
    R8d = R8,
    R8w = R8,
    R8b = R8,

    R9,
    R9d = R9,
    R9w = R9,
    R9b = R9,

    R10,
    R10d = R10,
    R10w = R10,
    R10b = R10,

    R11,
    R11d = R11,
    R11w = R11,
    R11b = R11,

    R12,
    R12d = R12,
    R12w = R12,
    R12b = R12,

    R13,
    R13d = R13,
    R13w = R13,
    R13b = R13,

    R14,
    R14d = R14,
    R14w = R14,
    R14b = R14,

    R15,
    R15d = R15,
    R15w = R15,
    R15b = R15,

    NumArchRegs,

    MicroBegin = NumArchRegs,
    T0 = MicroBegin,
    MicroEnd = MicroBegin + NumMicroIntRegs,

    // The lower part of the result of multiplication.
    Prodlow,
    // The upper part of the result of multiplication.
    Prodhi,
    // The quotient from division.
    Quotient,
    // The remainder from division.
    Remainder,
    // The divisor for division.
    Divisor,
    // The register to use for shift doubles.
    Doublebits,

    NumRegs,
};

} // namespace int_reg

// This needs to be large enough to miss all the other bits of an index.
inline constexpr RegIndex IntFoldBit = 1 << 6;

inline static constexpr RegIndex
intRegMicro(int index)
{
    return int_reg::MicroBegin + index;
}

inline static constexpr RegIndex
intRegFolded(RegIndex index, RegIndex foldBit)
{
    if ((index & 0x1C) == 4 && foldBit)
        index = (index - 4) | foldBit;
    return index;
}

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_INTREGS_HH__
