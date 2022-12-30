/*
 * Copyright (c) 2009 The University of Edinburgh
 * Copyright (c) 2021 IBM Corporation
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

#ifndef __ARCH_POWER_REGS_INT_HH__
#define __ARCH_POWER_REGS_INT_HH__

#include "cpu/reg_class.hh"
#include "debug/IntRegs.hh"

namespace gem5
{

namespace PowerISA
{

namespace int_reg
{

enum : RegIndex
{
    _R0Idx,
    _R1Idx,
    _R2Idx,
    _R3Idx,
    _R4Idx,
    _R5Idx,
    _R6Idx,
    _R7Idx,
    _R8Idx,
    _R9Idx,
    _R10Idx,
    _R11Idx,
    _R12Idx,
    _R13Idx,
    _R14Idx,
    _R15Idx,
    _R16Idx,
    _R17Idx,
    _R18Idx,
    _R19Idx,
    _R20Idx,
    _R21Idx,
    _R22Idx,
    _R23Idx,
    _R24Idx,
    _R25Idx,
    _R26Idx,
    _R27Idx,
    _R28Idx,
    _R29Idx,
    _R30Idx,
    _R31Idx,

    NumArchRegs,

    _CrIdx = NumArchRegs,
    _XerIdx,
    _LrIdx,
    _CtrIdx,
    _TarIdx,
    _FpscrIdx,
    _MsrIdx,
    _RsvIdx,
    _RsvLenIdx,
    _RsvAddrIdx,

    NumRegs
};

} // namespace int_reg

inline constexpr RegClass intRegClass(IntRegClass, IntRegClassName,
        int_reg::NumRegs, debug::IntRegs);

namespace int_reg
{

inline constexpr RegId
    R0 = intRegClass[_R0Idx],
    R1 = intRegClass[_R1Idx],
    R2 = intRegClass[_R2Idx],
    R3 = intRegClass[_R3Idx],
    R4 = intRegClass[_R4Idx],
    R5 = intRegClass[_R5Idx],
    R6 = intRegClass[_R6Idx],
    R7 = intRegClass[_R7Idx],
    R8 = intRegClass[_R8Idx],
    R9 = intRegClass[_R9Idx],
    R10 = intRegClass[_R10Idx],
    R11 = intRegClass[_R11Idx],
    R12 = intRegClass[_R12Idx],
    R13 = intRegClass[_R13Idx],
    R14 = intRegClass[_R14Idx],
    R15 = intRegClass[_R15Idx],
    R16 = intRegClass[_R16Idx],
    R17 = intRegClass[_R17Idx],
    R18 = intRegClass[_R18Idx],
    R19 = intRegClass[_R19Idx],
    R20 = intRegClass[_R20Idx],
    R21 = intRegClass[_R21Idx],
    R22 = intRegClass[_R22Idx],
    R23 = intRegClass[_R23Idx],
    R24 = intRegClass[_R24Idx],
    R25 = intRegClass[_R25Idx],
    R26 = intRegClass[_R26Idx],
    R27 = intRegClass[_R27Idx],
    R28 = intRegClass[_R28Idx],
    R29 = intRegClass[_R29Idx],
    R30 = intRegClass[_R30Idx],
    R31 = intRegClass[_R31Idx],

    Cr = intRegClass[_CrIdx],
    Xer = intRegClass[_XerIdx],
    Lr = intRegClass[_LrIdx],
    Ctr = intRegClass[_CtrIdx],
    Tar = intRegClass[_TarIdx],
    Fpscr = intRegClass[_FpscrIdx],
    Msr = intRegClass[_MsrIdx],
    Rsv = intRegClass[_RsvIdx],
    RsvLen = intRegClass[_RsvLenIdx],
    RsvAddr = intRegClass[_RsvAddrIdx];

} // namespace int_reg

// Semantically meaningful register indices
inline constexpr auto
    &ReturnValueReg = int_reg::R3,
    &ArgumentReg0 = int_reg::R3,
    &ArgumentReg1 = int_reg::R4,
    &ArgumentReg2 = int_reg::R5,
    &ArgumentReg3 = int_reg::R6,
    &ArgumentReg4 = int_reg::R7,
    &ArgumentReg5 = int_reg::R8,
    &StackPointerReg = int_reg::R1,
    &TOCPointerReg = int_reg::R2,
    &ThreadPointerReg = int_reg::R13;

} // namespace PowerISA
} // namespace gem5

#endif // __ARCH_POWER_REGS_INT_HH__
