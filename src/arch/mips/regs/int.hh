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

#ifndef __ARCH_MIPS_REGS_INT_HH__
#define __ARCH_MIPS_REGS_INT_HH__

#include "cpu/reg_class.hh"

namespace gem5
{
namespace MipsISA
{

// Constants Related to the number of registers

const int MaxShadowRegSets = 16; // Maximum number of shadow register sets

namespace int_reg
{

enum : RegIndex
{
    _ZeroIdx = 0,

    _AtIdx = 1,

    _V0Idx = 2,
    _V1Idx = 3,

    _A0Idx = 4,
    _A1Idx = 5,
    _A2Idx = 6,
    _A3Idx = 7,

    _T0Idx = 8,
    _T1Idx = 9,
    _T2Idx = 10,
    _T3Idx = 11,
    _T4Idx = 12,
    _T5Idx = 13,
    _T6Idx = 14,
    _T7Idx = 15,

    _S0Idx = 16,
    _S1Idx = 17,
    _S2Idx = 18,
    _S3Idx = 19,
    _S4Idx = 20,
    _S5Idx = 21,
    _S6Idx = 22,
    _S7Idx = 23,

    _T8Idx = 24,
    _T9Idx = 25,

    _K0Idx = 26,
    _K1Idx = 27,

    _GpIdx = 28,

    _SpIdx = 29,

    _S8Idx = 30,
    _FpIdx = _S8Idx,

    _RaIdx = 31,

    NumArchRegs,

    _LoIdx = NumArchRegs,
    _DspLo0Idx = _LoIdx,
    _HiIdx,
    _DspHi0Idx = _HiIdx,
    _DspAcx0Idx,

    _DspLo1Idx,
    _DspHi1Idx,
    _DspAcx1Idx,

    _DspLo2Idx,
    _DspHi2Idx,
    _DspAcx2Idx,

    _DspLo3Idx,
    _DspHi3Idx,
    _DspAcx3Idx,

    _DspControlIdx,

    NumRegs
};

inline constexpr RegId
    // Zero register.
    Zero(IntRegClass, _ZeroIdx),

    // Assembly temporary.
    At(IntRegClass, _AtIdx),

    // Value returned by subroutine.
    V0(IntRegClass, _V0Idx),
    V1(IntRegClass, _V1Idx),

    // Arguments for subroutine.
    A0(IntRegClass, _A0Idx),
    A1(IntRegClass, _A1Idx),
    A2(IntRegClass, _A2Idx),
    A3(IntRegClass, _A3Idx),

    // Temporaries.
    T0(IntRegClass, _T0Idx),
    T1(IntRegClass, _T1Idx),
    T2(IntRegClass, _T2Idx),
    T3(IntRegClass, _T3Idx),
    T4(IntRegClass, _T4Idx),
    T5(IntRegClass, _T5Idx),
    T6(IntRegClass, _T6Idx),
    T7(IntRegClass, _T7Idx),
    T8(IntRegClass, _T8Idx),
    T9(IntRegClass, _T9Idx),

    // Subroutine registers.
    S0(IntRegClass, _S0Idx),
    S1(IntRegClass, _S1Idx),
    S2(IntRegClass, _S2Idx),
    S3(IntRegClass, _S3Idx),
    S4(IntRegClass, _S4Idx),
    S5(IntRegClass, _S5Idx),
    S6(IntRegClass, _S6Idx),
    S7(IntRegClass, _S7Idx),

    // For use in an interrupt/trap handler.
    K0(IntRegClass, _K0Idx),
    K1(IntRegClass, _K1Idx),

    // Global pointer.
    Gp(IntRegClass, _GpIdx),

    // Stack pointer.
    Sp(IntRegClass, _SpIdx),

    // Frame pointer.
    Fp(IntRegClass, _FpIdx),

    // Return address.
    Ra(IntRegClass, _RaIdx),

    DspLo0(IntRegClass, _DspLo0Idx),
    DspHi0(IntRegClass, _DspHi0Idx),
    DspAcx0(IntRegClass, _DspAcx0Idx),

    DspLo1(IntRegClass, _DspLo1Idx),
    DspHi1(IntRegClass, _DspHi1Idx),
    DspAcx1(IntRegClass, _DspAcx1Idx),

    DspLo2(IntRegClass, _DspLo2Idx),
    DspHi2(IntRegClass, _DspHi2Idx),
    DspAcx2(IntRegClass, _DspAcx2Idx),

    DspLo3(IntRegClass, _DspLo3Idx),
    DspHi3(IntRegClass, _DspHi3Idx),
    DspAcx3(IntRegClass, _DspAcx3Idx),

    DspControl(IntRegClass, _DspControlIdx);

// Register aliases.
inline constexpr auto
    &S8 = Fp,

    &Lo = DspLo0,
    &Hi = DspHi0,

    &SyscallSuccess = A3;

} // namespace int_reg
} // namespace MipsISA
} // namespace gem5

#endif
