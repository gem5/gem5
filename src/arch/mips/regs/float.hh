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

#ifndef __ARCH_MIPS_REGS_FLOAT_HH__
#define __ARCH_MIPS_REGS_FLOAT_HH__

#include <cstdint>

#include "cpu/reg_class.hh"
#include "debug/FloatRegs.hh"

namespace gem5
{
namespace MipsISA
{
namespace float_reg
{

enum : RegIndex
{
    _F0Idx,
    _F1Idx,
    _F2Idx,
    _F3Idx,
    _F4Idx,
    _F5Idx,
    _F6Idx,
    _F7Idx,
    _F8Idx,
    _F9Idx,
    _F10Idx,
    _F11Idx,
    _F12Idx,
    _F13Idx,
    _F14Idx,
    _F15Idx,
    _F16Idx,
    _F17Idx,
    _F18Idx,
    _F19Idx,
    _F20Idx,
    _F21Idx,
    _F22Idx,
    _F23Idx,
    _F24Idx,
    _F25Idx,
    _F26Idx,
    _F27Idx,
    _F28Idx,
    _F29Idx,
    _F30Idx,
    _F31Idx,
    NumArchRegs,

    _FirIdx = NumArchRegs,
    _FccrIdx,
    _FexrIdx,
    _FenrIdx,
    _FcsrIdx,

    NumRegs,
};

} // namespace float_reg

inline constexpr RegClass floatRegClass(FloatRegClass, FloatRegClassName,
        float_reg::NumRegs, debug::FloatRegs);

namespace float_reg
{

inline constexpr RegId
    F0 = floatRegClass[_F0Idx],
    F1 = floatRegClass[_F1Idx],
    F2 = floatRegClass[_F2Idx],
    F3 = floatRegClass[_F3Idx],
    F4 = floatRegClass[_F4Idx],
    F5 = floatRegClass[_F5Idx],
    F6 = floatRegClass[_F6Idx],
    F7 = floatRegClass[_F7Idx],
    F8 = floatRegClass[_F8Idx],
    F9 = floatRegClass[_F9Idx],
    F10 = floatRegClass[_F10Idx],
    F11 = floatRegClass[_F11Idx],
    F12 = floatRegClass[_F12Idx],
    F13 = floatRegClass[_F13Idx],
    F14 = floatRegClass[_F14Idx],
    F15 = floatRegClass[_F15Idx],
    F16 = floatRegClass[_F16Idx],
    F17 = floatRegClass[_F17Idx],
    F18 = floatRegClass[_F18Idx],
    F19 = floatRegClass[_F19Idx],
    F20 = floatRegClass[_F20Idx],
    F21 = floatRegClass[_F21Idx],
    F22 = floatRegClass[_F22Idx],
    F23 = floatRegClass[_F23Idx],
    F24 = floatRegClass[_F24Idx],
    F25 = floatRegClass[_F25Idx],
    F26 = floatRegClass[_F26Idx],
    F27 = floatRegClass[_F27Idx],
    F28 = floatRegClass[_F28Idx],
    F29 = floatRegClass[_F29Idx],
    F30 = floatRegClass[_F30Idx],
    F31 = floatRegClass[_F31Idx],

    Fir = floatRegClass[_FirIdx],
    Fccr = floatRegClass[_FccrIdx],
    Fexr = floatRegClass[_FexrIdx],
    Fenr = floatRegClass[_FenrIdx],
    Fcsr = floatRegClass[_FcsrIdx];

} // namespace float_reg

enum FCSRBits
{
    Inexact = 1,
    Underflow,
    Overflow,
    DivideByZero,
    Invalid,
    Unimplemented
};

enum FCSRFields
{
    Flag_Field = 1,
    Enable_Field = 6,
    Cause_Field = 11
};

const uint32_t MIPS32_QNAN = 0x7fbfffff;
const uint64_t MIPS64_QNAN = 0x7ff7ffffffffffffULL;

} // namespace MipsISA
} // namespace gem5

#endif
