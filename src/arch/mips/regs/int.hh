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
#include "debug/IntRegs.hh"

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

} // namespace int_reg

inline constexpr RegClass intRegClass(IntRegClass, IntRegClassName,
        int_reg::NumRegs, debug::IntRegs);

namespace int_reg
{

inline constexpr RegId
    // Zero register.
    Zero = intRegClass[_ZeroIdx],

    // Assembly temporary.
    At = intRegClass[_AtIdx],

    // Value returned by subroutine.
    V0 = intRegClass[_V0Idx],
    V1 = intRegClass[_V1Idx],

    // Arguments for subroutine.
    A0 = intRegClass[_A0Idx],
    A1 = intRegClass[_A1Idx],
    A2 = intRegClass[_A2Idx],
    A3 = intRegClass[_A3Idx],

    // Temporaries.
    T0 = intRegClass[_T0Idx],
    T1 = intRegClass[_T1Idx],
    T2 = intRegClass[_T2Idx],
    T3 = intRegClass[_T3Idx],
    T4 = intRegClass[_T4Idx],
    T5 = intRegClass[_T5Idx],
    T6 = intRegClass[_T6Idx],
    T7 = intRegClass[_T7Idx],
    T8 = intRegClass[_T8Idx],
    T9 = intRegClass[_T9Idx],

    // Subroutine registers.
    S0 = intRegClass[_S0Idx],
    S1 = intRegClass[_S1Idx],
    S2 = intRegClass[_S2Idx],
    S3 = intRegClass[_S3Idx],
    S4 = intRegClass[_S4Idx],
    S5 = intRegClass[_S5Idx],
    S6 = intRegClass[_S6Idx],
    S7 = intRegClass[_S7Idx],

    // For use in an interrupt/trap handler.
    K0 = intRegClass[_K0Idx],
    K1 = intRegClass[_K1Idx],

    // Global pointer.
    Gp = intRegClass[_GpIdx],

    // Stack pointer.
    Sp = intRegClass[_SpIdx],

    // Frame pointer.
    Fp = intRegClass[_FpIdx],

    // Return address.
    Ra = intRegClass[_RaIdx],

    DspLo0 = intRegClass[_DspLo0Idx],
    DspHi0 = intRegClass[_DspHi0Idx],
    DspAcx0 = intRegClass[_DspAcx0Idx],

    DspLo1 = intRegClass[_DspLo1Idx],
    DspHi1 = intRegClass[_DspHi1Idx],
    DspAcx1 = intRegClass[_DspAcx1Idx],

    DspLo2 = intRegClass[_DspLo2Idx],
    DspHi2 = intRegClass[_DspHi2Idx],
    DspAcx2 = intRegClass[_DspAcx2Idx],

    DspLo3 = intRegClass[_DspLo3Idx],
    DspHi3 = intRegClass[_DspHi3Idx],
    DspAcx3 = intRegClass[_DspAcx3Idx],

    DspControl = intRegClass[_DspControlIdx];

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
