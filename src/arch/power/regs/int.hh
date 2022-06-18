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

inline constexpr RegId
    R0(IntRegClass, _R0Idx),
    R1(IntRegClass, _R1Idx),
    R2(IntRegClass, _R2Idx),
    R3(IntRegClass, _R3Idx),
    R4(IntRegClass, _R4Idx),
    R5(IntRegClass, _R5Idx),
    R6(IntRegClass, _R6Idx),
    R7(IntRegClass, _R7Idx),
    R8(IntRegClass, _R8Idx),
    R9(IntRegClass, _R9Idx),
    R10(IntRegClass, _R10Idx),
    R11(IntRegClass, _R11Idx),
    R12(IntRegClass, _R12Idx),
    R13(IntRegClass, _R13Idx),
    R14(IntRegClass, _R14Idx),
    R15(IntRegClass, _R15Idx),
    R16(IntRegClass, _R16Idx),
    R17(IntRegClass, _R17Idx),
    R18(IntRegClass, _R18Idx),
    R19(IntRegClass, _R19Idx),
    R20(IntRegClass, _R20Idx),
    R21(IntRegClass, _R21Idx),
    R22(IntRegClass, _R22Idx),
    R23(IntRegClass, _R23Idx),
    R24(IntRegClass, _R24Idx),
    R25(IntRegClass, _R25Idx),
    R26(IntRegClass, _R26Idx),
    R27(IntRegClass, _R27Idx),
    R28(IntRegClass, _R28Idx),
    R29(IntRegClass, _R29Idx),
    R30(IntRegClass, _R30Idx),
    R31(IntRegClass, _R31Idx),

    Cr(IntRegClass, _CrIdx),
    Xer(IntRegClass, _XerIdx),
    Lr(IntRegClass, _LrIdx),
    Ctr(IntRegClass, _CtrIdx),
    Tar(IntRegClass, _TarIdx),
    Fpscr(IntRegClass, _FpscrIdx),
    Msr(IntRegClass, _MsrIdx),
    Rsv(IntRegClass, _RsvIdx),
    RsvLen(IntRegClass, _RsvLenIdx),
    RsvAddr(IntRegClass, _RsvAddrIdx);

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
