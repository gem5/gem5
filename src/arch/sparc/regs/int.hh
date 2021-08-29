/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_REGS_INT_HH__
#define __ARCH_SPARC_REGS_INT_HH__

#include "arch/sparc/sparc_traits.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

namespace SparcISA
{

namespace int_reg
{

// semantically meaningful register indices
enum {
    _G0Idx, _G1Idx, _G2Idx, _G3Idx, _G4Idx, _G5Idx, _G6Idx, _G7Idx,
    _O0Idx, _O1Idx, _O2Idx, _O3Idx, _O4Idx, _O5Idx, _O6Idx, _O7Idx,
    _L0Idx, _L1Idx, _L2Idx, _L3Idx, _L4Idx, _L5Idx, _L6Idx, _L7Idx,
    _I0Idx, _I1Idx, _I2Idx, _I3Idx, _I4Idx, _I5Idx, _I6Idx, _I7Idx,

    NumArchRegs,

    _Ureg0Idx = NumArchRegs,
    _YIdx,
    _CcrIdx,
    _CansaveIdx,
    _CanrestoreIdx,
    _CleanwinIdx,
    _OtherwinIdx,
    _WstateIdx,
    _GsrIdx,

    NumMicroRegs = _GsrIdx - _Ureg0Idx + 1
};

inline constexpr RegId
    // Globals
    G0(IntRegClass, _G0Idx),
    G1(IntRegClass, _G1Idx),
    G2(IntRegClass, _G2Idx),
    G3(IntRegClass, _G3Idx),
    G4(IntRegClass, _G4Idx),
    G5(IntRegClass, _G5Idx),
    G6(IntRegClass, _G6Idx),
    G7(IntRegClass, _G7Idx),

    // Outputs
    O0(IntRegClass, _O0Idx),
    O1(IntRegClass, _O1Idx),
    O2(IntRegClass, _O2Idx),
    O3(IntRegClass, _O3Idx),
    O4(IntRegClass, _O4Idx),
    O5(IntRegClass, _O5Idx),
    O6(IntRegClass, _O6Idx),
    O7(IntRegClass, _O7Idx),

    // Locals
    L0(IntRegClass, _L0Idx),
    L1(IntRegClass, _L1Idx),
    L2(IntRegClass, _L2Idx),
    L3(IntRegClass, _L3Idx),
    L4(IntRegClass, _L4Idx),
    L5(IntRegClass, _L5Idx),
    L6(IntRegClass, _L6Idx),
    L7(IntRegClass, _L7Idx),

    // Inputs
    I0(IntRegClass, _I0Idx),
    I1(IntRegClass, _I1Idx),
    I2(IntRegClass, _I2Idx),
    I3(IntRegClass, _I3Idx),
    I4(IntRegClass, _I4Idx),
    I5(IntRegClass, _I5Idx),
    I6(IntRegClass, _I6Idx),
    I7(IntRegClass, _I7Idx),

    Ureg0(IntRegClass, _Ureg0Idx),
    Y(IntRegClass, _YIdx),
    Ccr(IntRegClass, _CcrIdx),
    Cansave(IntRegClass, _CansaveIdx),
    Canrestore(IntRegClass, _CanrestoreIdx),
    Cleanwin(IntRegClass, _CleanwinIdx),
    Otherwin(IntRegClass, _OtherwinIdx),
    Wstate(IntRegClass, _WstateIdx),
    Gsr(IntRegClass, _GsrIdx);

inline constexpr RegId
g(int index)
{
    return RegId(IntRegClass, G0 + index);
}

inline constexpr RegId
o(int index)
{
    return RegId(IntRegClass, O0 + index);
}

inline constexpr RegId
l(int index)
{
    return RegId(IntRegClass, L0 + index);
}

inline constexpr RegId
i(int index)
{
    return RegId(IntRegClass, I0 + index);
}

const int NumRegs = (MaxGL + 1) * 8 + NWindows * 16 + NumMicroRegs;

} // namespace int_reg

// the rest of these depend on the ABI
inline constexpr auto
    &ReturnAddressReg = int_reg::I7, // post call, precall is 15
    &ReturnValueReg = int_reg::O0, // Post return, 24 is pre-return.
    &StackPointerReg = int_reg::O6,
    &FramePointerReg = int_reg::I6,

    // Some OS syscall use a second register to return a second value
    &SyscallPseudoReturnReg = int_reg::O1;

} // namespace SparcISA
} // namespace gem5

#endif
