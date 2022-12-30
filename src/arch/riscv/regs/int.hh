/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
 * Copyright (c) 2019 Yifei Liu
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2021 StreamComputing Corp
 * All rights reserved
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
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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

#ifndef __ARCH_RISCV_REGS_INT_HH__
#define __ARCH_RISCV_REGS_INT_HH__

#include <string>
#include <vector>

#include "cpu/reg_class.hh"
#include "debug/IntRegs.hh"

namespace gem5
{

namespace RiscvISA
{

namespace int_reg
{

enum : RegIndex
{
    _ZeroIdx, _RaIdx, _SpIdx,  _GpIdx,
    _TpIdx,   _T0Idx, _T1Idx,  _T2Idx,
    _S0Idx,   _S1Idx, _A0Idx,  _A1Idx,
    _A2Idx,   _A3Idx, _A4Idx,  _A5Idx,
    _A6Idx,   _A7Idx, _S2Idx,  _S3Idx,
    _S4Idx,   _S5Idx, _S6Idx,  _S7Idx,
    _S8Idx,   _S9Idx, _S10Idx, _S11Idx,
    _T3Idx,   _T4Idx, _T5Idx,  _T6Idx,

    NumArchRegs,

    _Ureg0Idx = NumArchRegs,

    NumRegs
};

} // namespace int_reg

inline constexpr RegClass intRegClass(IntRegClass, IntRegClassName,
        int_reg::NumRegs, debug::IntRegs);

namespace int_reg
{

inline constexpr RegId
    Zero = intRegClass[_ZeroIdx],
    Ra = intRegClass[_RaIdx],
    Sp = intRegClass[_SpIdx],
    Gp = intRegClass[_GpIdx],
    Tp = intRegClass[_TpIdx],
    T0 = intRegClass[_T0Idx],
    T1 = intRegClass[_T1Idx],
    T2 = intRegClass[_T2Idx],
    S0 = intRegClass[_S0Idx],
    S1 = intRegClass[_S1Idx],
    A0 = intRegClass[_A0Idx],
    A1 = intRegClass[_A1Idx],
    A2 = intRegClass[_A2Idx],
    A3 = intRegClass[_A3Idx],
    A4 = intRegClass[_A4Idx],
    A5 = intRegClass[_A5Idx],
    A6 = intRegClass[_A6Idx],
    A7 = intRegClass[_A7Idx],
    S2 = intRegClass[_S2Idx],
    S3 = intRegClass[_S3Idx],
    S4 = intRegClass[_S4Idx],
    S5 = intRegClass[_S5Idx],
    S6 = intRegClass[_S6Idx],
    S7 = intRegClass[_S7Idx],
    S8 = intRegClass[_S8Idx],
    S9 = intRegClass[_S9Idx],
    S10 = intRegClass[_S10Idx],
    S11 = intRegClass[_S11Idx],
    T3 = intRegClass[_T3Idx],
    T4 = intRegClass[_T4Idx],
    T5 = intRegClass[_T5Idx],
    T6 = intRegClass[_T6Idx],
    Ureg0 = intRegClass[_Ureg0Idx];

const std::vector<std::string> RegNames = {
    "zero", "ra", "sp", "gp",
    "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1",
    "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3",
    "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11",
    "t3", "t4", "t5", "t6"
};

} // namespace int_reg

// Semantically meaningful register indices
inline constexpr auto
    &ReturnAddrReg = int_reg::Ra,
    &StackPointerReg = int_reg::Sp,
    &ThreadPointerReg = int_reg::Tp,
    &ReturnValueReg = int_reg::A0,
    &AMOTempReg = int_reg::Ureg0,
    &SyscallNumReg = int_reg::A7;

inline constexpr RegId ArgumentRegs[] = {
    int_reg::A0, int_reg::A1, int_reg::A2, int_reg::A3,
    int_reg::A4, int_reg::A5, int_reg::A6, int_reg::A7
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_INT_HH__
