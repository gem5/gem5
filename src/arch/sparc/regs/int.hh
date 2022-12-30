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
#include "debug/IntRegs.hh"

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

const int NumRegs = (MaxGL + 1) * 8 + NWindows * 16 + NumMicroRegs;

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
    // Globals
    G0 = intRegClass[_G0Idx],
    G1 = intRegClass[_G1Idx],
    G2 = intRegClass[_G2Idx],
    G3 = intRegClass[_G3Idx],
    G4 = intRegClass[_G4Idx],
    G5 = intRegClass[_G5Idx],
    G6 = intRegClass[_G6Idx],
    G7 = intRegClass[_G7Idx],

    // Outputs
    O0 = intRegClass[_O0Idx],
    O1 = intRegClass[_O1Idx],
    O2 = intRegClass[_O2Idx],
    O3 = intRegClass[_O3Idx],
    O4 = intRegClass[_O4Idx],
    O5 = intRegClass[_O5Idx],
    O6 = intRegClass[_O6Idx],
    O7 = intRegClass[_O7Idx],

    // Locals
    L0 = intRegClass[_L0Idx],
    L1 = intRegClass[_L1Idx],
    L2 = intRegClass[_L2Idx],
    L3 = intRegClass[_L3Idx],
    L4 = intRegClass[_L4Idx],
    L5 = intRegClass[_L5Idx],
    L6 = intRegClass[_L6Idx],
    L7 = intRegClass[_L7Idx],

    // Inputs
    I0 = intRegClass[_I0Idx],
    I1 = intRegClass[_I1Idx],
    I2 = intRegClass[_I2Idx],
    I3 = intRegClass[_I3Idx],
    I4 = intRegClass[_I4Idx],
    I5 = intRegClass[_I5Idx],
    I6 = intRegClass[_I6Idx],
    I7 = intRegClass[_I7Idx],

    Ureg0 = intRegClass[_Ureg0Idx],
    Y = intRegClass[_YIdx],
    Ccr = intRegClass[_CcrIdx],
    Cansave = intRegClass[_CansaveIdx],
    Canrestore = intRegClass[_CanrestoreIdx],
    Cleanwin = intRegClass[_CleanwinIdx],
    Otherwin = intRegClass[_OtherwinIdx],
    Wstate = intRegClass[_WstateIdx],
    Gsr = intRegClass[_GsrIdx];

inline constexpr RegId
g(int index)
{
    return intRegClass[G0 + index];
}

inline constexpr RegId
o(int index)
{
    return intRegClass[O0 + index];
}

inline constexpr RegId
l(int index)
{
    return intRegClass[L0 + index];
}

inline constexpr RegId
i(int index)
{
    return intRegClass[I0 + index];
}

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
