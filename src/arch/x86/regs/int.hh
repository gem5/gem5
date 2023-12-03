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

#ifndef __ARCH_X86_REGS_INT_HH__
#define __ARCH_X86_REGS_INT_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "debug/IntRegs.hh"

namespace gem5
{

namespace X86ISA
{

BitUnion64(X86IntReg)
    Bitfield<63, 0> R;
    SignedBitfield<63, 0> SR;
    Bitfield<31, 0> E;
    SignedBitfield<31, 0> SE;
    Bitfield<15, 0> X;
    SignedBitfield<15, 0> SX;
    Bitfield<15, 8> H;
    SignedBitfield<15, 8> SH;
    Bitfield<7, 0> L;
    SignedBitfield<7, 0> SL;
EndBitUnion(X86IntReg)

namespace int_reg
{

enum : RegIndex
{
    _RaxIdx,
    _RcxIdx,
    _RdxIdx,
    _RbxIdx,
    _RspIdx,
    _RbpIdx,
    _RsiIdx,
    _RdiIdx,
    _R8Idx,
    _R9Idx,
    _R10Idx,
    _R11Idx,
    _R12Idx,
    _R13Idx,
    _R14Idx,
    _R15Idx,

    NumArchRegs,
    MicroBegin = NumArchRegs,
    _T0Idx = MicroBegin,
    MicroEnd = MicroBegin + NumMicroIntRegs,

    _ProdlowIdx,
    _ProdhiIdx,
    _QuotientIdx,
    _RemainderIdx,
    _DivisorIdx,
    _DoublebitsIdx,

    NumRegs
};

} // namespace int_reg

class FlatIntRegClassOps : public RegClassOps
{
    std::string regName(const RegId &id) const override;
};

inline constexpr FlatIntRegClassOps flatIntRegClassOps;

inline constexpr RegClass flatIntRegClass =
    RegClass(IntRegClass, IntRegClassName, int_reg::NumRegs, debug::IntRegs)
        .ops(flatIntRegClassOps);

class IntRegClassOps : public FlatIntRegClassOps
{
    RegId flatten(const BaseISA &isa, const RegId &id) const override;
};

inline constexpr IntRegClassOps intRegClassOps;

inline constexpr RegClass intRegClass =
    RegClass(IntRegClass, IntRegClassName, int_reg::NumRegs, debug::IntRegs)
        .ops(intRegClassOps)
        .needsFlattening();

namespace int_reg
{

inline constexpr RegId Rax = intRegClass[_RaxIdx], Rcx = intRegClass[_RcxIdx],
                       Rdx = intRegClass[_RdxIdx], Rbx = intRegClass[_RbxIdx],
                       Rsp = intRegClass[_RspIdx], Rbp = intRegClass[_RbpIdx],
                       Rsi = intRegClass[_RsiIdx], Rdi = intRegClass[_RdiIdx],
                       R8 = intRegClass[_R8Idx], R9 = intRegClass[_R9Idx],
                       R10 = intRegClass[_R10Idx], R11 = intRegClass[_R11Idx],
                       R12 = intRegClass[_R12Idx], R13 = intRegClass[_R13Idx],
                       R14 = intRegClass[_R14Idx], R15 = intRegClass[_R15Idx],
                       T0 = intRegClass[_T0Idx],
                       Prodlow = intRegClass[_ProdlowIdx],
                       Prodhi = intRegClass[_ProdhiIdx],
                       Quotient = intRegClass[_QuotientIdx],
                       Remainder = intRegClass[_RemainderIdx],
                       Divisor = intRegClass[_DivisorIdx],
                       Doublebits = intRegClass[_DoublebitsIdx];

// Aliases for other register sizes.
inline constexpr auto &Eax = Rax, &Ax = Rax, &Al = Rax, &Ecx = Rcx, &Cx = Rcx,
                      &Cl = Rcx, &Edx = Rdx, &Dx = Rdx, &Dl = Rdx, &Ebx = Rbx,
                      &Bx = Rbx, &Bl = Rbx, &Esp = Rsp, &Sp = Rsp, &Spl = Rsp,
                      &Ah = Rsp, &Ebp = Rbp, &Bp = Rbp, &Bpl = Rbp, &Ch = Rbp,
                      &Esi = Rsi, &Si = Rsi, &Sil = Rsi, &Dh = Rsi, &Edi = Rdi,
                      &Di = Rdi, &Dil = Rdi, &Bh = Rdi, &R8d = R8, &R8w = R8,
                      &R8b = R8, &R9d = R9, &R9w = R9, &R9b = R9, &R10d = R10,
                      &R10w = R10, &R10b = R10, &R11d = R11, &R11w = R11,
                      &R11b = R11, &R12d = R12, &R12w = R12, &R12b = R12,
                      &R13d = R13, &R13w = R13, &R13b = R13, &R14d = R14,
                      &R14w = R14, &R14b = R14, &R15d = R15, &R15w = R15,
                      &R15b = R15;

} // namespace int_reg

// This needs to be large enough to miss all the other bits of an index.
inline constexpr RegIndex IntFoldBit = 1 << 6;

inline static constexpr RegId
intRegMicro(int index)
{
    return intRegClass[int_reg::MicroBegin + index];
}

inline static constexpr RegId
intRegFolded(RegIndex index, RegIndex foldBit)
{
    if ((index & 0x1C) == 4 && foldBit)
        index = (index - 4) | foldBit;
    return intRegClass[index];
}

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_REGS_INT_HH__
