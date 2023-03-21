/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
 * Copyright (c) 2018 TU Dresden
 * Copyright (c) 2020 Barkhausen Institut
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
 * Copyright (c) 2016-2017 The University of Virginia
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

#ifndef __ARCH_RISCV_UTILITY_HH__
#define __ARCH_RISCV_UTILITY_HH__

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "enums/RiscvType.hh"
#include "rvk.hh"

namespace gem5
{

namespace RiscvISA
{

template<typename T> inline bool
isquietnan(T val)
{
    return false;
}

template<> inline bool
isquietnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00400000);
}

template<> inline bool
isquietnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0008000000000000ULL);
}

template<typename T> inline bool
issignalingnan(T val)
{
    return false;
}

template<> inline bool
issignalingnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00200000);
}

template<> inline bool
issignalingnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0004000000000000ULL);
}

inline std::string
registerName(RegId reg)
{
    if (reg.is(IntRegClass)) {
        if (reg.index() >= int_reg::NumArchRegs) {
            /*
             * This should only happen if a instruction is being speculatively
             * executed along a not-taken branch, and if that instruction's
             * width was incorrectly predecoded (i.e., it was predecoded as a
             * full instruction rather than a compressed one or vice versa).
             * It also should only happen if a debug flag is on that prints
             * disassembly information, so rather than panic the incorrect
             * value is printed for debugging help.
             */
            std::stringstream str;
            str << "?? (x" << reg.index() << ')';
            return str.str();
        }
        return int_reg::RegNames[reg.index()];
    } else if (reg.is(FloatRegClass)) {
        if (reg.index() >= float_reg::NumRegs) {
            std::stringstream str;
            str << "?? (f" << reg.index() << ')';
            return str.str();
        }
        return float_reg::RegNames[reg.index()];
    } else if (reg.is(VecRegClass)) {
        if (reg.index() >= NumVecRegs) {
            std::stringstream str;
            str << "?? (v" << reg.index() << ')';
            return str.str();
        }
        return VecRegNames[reg.index()];
    } else  {
        /* It must be an InvalidRegClass, in RISC-V we should treat it as a
         * zero register for the disassembler to work correctly.
         */
        return int_reg::RegNames[reg.index()];
    }
}

inline uint32_t
mulhu_32(uint32_t rs1, uint32_t rs2)
{
    return ((uint64_t)rs1 * rs2) >> 32;
}

inline uint64_t
mulhu_64(uint64_t rs1, uint64_t rs2)
{
    uint64_t rs1_lo = (uint32_t)rs1;
    uint64_t rs1_hi = rs1 >> 32;
    uint64_t rs2_lo = (uint32_t)rs2;
    uint64_t rs2_hi = rs2 >> 32;

    uint64_t hi = rs1_hi * rs2_hi;
    uint64_t mid1 = rs1_hi * rs2_lo;
    uint64_t mid2 = rs1_lo * rs2_hi;
    uint64_t lo = rs1_lo * rs2_lo;
    uint64_t carry = ((uint64_t)(uint32_t)mid1
            + (uint64_t)(uint32_t)mid2
            + (lo >> 32)) >> 32;

    return hi + (mid1 >> 32) + (mid2 >> 32) + carry;
}

inline int32_t
mulh_32(int32_t rs1, int32_t rs2)
{
    return ((int64_t)rs1 * rs2) >> 32;
}

inline int64_t
mulh_64(int64_t rs1, int64_t rs2)
{
    bool negate = (rs1 < 0) != (rs2 < 0);
    uint64_t res = mulhu_64(std::abs(rs1), std::abs(rs2));
    return negate ? ~res + (rs1 * rs2 == 0 ? 1 : 0) : res;
}

inline int32_t
mulhsu_32(int32_t rs1, uint32_t rs2)
{
    return ((int64_t)rs1 * rs2) >> 32;
}

inline int64_t
mulhsu_64(int64_t rs1, uint64_t rs2)
{
    bool negate = rs1 < 0;
    uint64_t res = mulhu_64(std::abs(rs1), rs2);
    return negate ? ~res + (rs1 * rs2 == 0 ? 1 : 0) : res;
}

template<typename T> inline T
div(T rs1, T rs2)
{
    constexpr T kRsMin = std::numeric_limits<T>::min();
    if (rs2 == 0) {
        return -1;
    } else if (rs1 == kRsMin && rs2 == -1) {
        return kRsMin;
    } else {
        return rs1 / rs2;
    }
}

template<typename T> inline T
divu(T rs1, T rs2)
{
    if (rs2 == 0) {
        return std::numeric_limits<T>::max();
    } else {
        return rs1 / rs2;
    }
}

template<typename T> inline T
rem(T rs1, T rs2)
{
    constexpr T kRsMin = std::numeric_limits<T>::min();
    if (rs2 == 0) {
        return rs1;
    } else if (rs1 == kRsMin && rs2 == -1) {
        return 0;
    } else {
        return rs1 % rs2;
    }
}

template<typename T> inline T
remu(T rs1, T rs2)
{
    return (rs2 == 0) ? rs1 : rs1 % rs2;
}

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_UTILITY_HH__
