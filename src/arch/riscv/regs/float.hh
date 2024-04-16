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

#ifndef __ARCH_RISCV_REGS_FLOAT_HH__
#define __ARCH_RISCV_REGS_FLOAT_HH__

#include <softfloat.h>
#include <specialize.h>

#include <cstdint>
#include <string>
#include <vector>

#include "base/bitfield.hh"
#include "cpu/reg_class.hh"
#include "debug/FloatRegs.hh"

namespace gem5
{

namespace RiscvISA
{

/* Conversion functions for working with softfloat. */

// Generic floating point value type.
using freg_t = float64_t;

// Extract a 16 bit float packed into a 64 bit value.
static constexpr uint16_t
unboxF16(uint64_t v)
{
    // The upper 48 bits should all be ones.
    if (bits(v, 63, 16) == mask(48))
        return bits(v, 15, 0);
    else
        return defaultNaNF16UI;
}

// Extract a 32 bit float packed into a 64 bit value.
static constexpr uint32_t
unboxF32(uint64_t v)
{
    // The upper 32 bits should all be ones.
    if (bits(v, 63, 32) == mask(32))
        return bits(v, 31, 0);
    else
        return defaultNaNF32UI;
}

/* clang-format off */
static constexpr uint64_t boxF16(uint16_t v) { return mask(63, 16) | v; }
static constexpr uint64_t boxF32(uint32_t v) { return mask(63, 32) | v; }

// Create fixed size floats from raw bytes or generic floating point values.
static constexpr float16_t f16(uint16_t v) { return {v}; }
static constexpr float32_t f32(uint32_t v) { return {v}; }
static constexpr float64_t f64(uint64_t v) { return {v}; }
static constexpr float16_t f16(freg_t r) { return {unboxF16(r.v)}; }
static constexpr float32_t f32(freg_t r) { return {unboxF32(r.v)}; }
static constexpr float64_t f64(freg_t r) { return r; }

// Create generic floating point values from fixed size floats.
static constexpr freg_t freg(float16_t f) { return {boxF16(f.v)}; }
static constexpr freg_t freg(float32_t f) { return {boxF32(f.v)}; }
static constexpr freg_t freg(float64_t f) { return f; }
static constexpr freg_t freg(uint_fast64_t f) { return {f}; }

/* clang-format on */

namespace float_reg
{

enum : RegIndex
{
    _Ft0Idx,
    _Ft1Idx,
    _Ft2Idx,
    _Ft3Idx,
    _Ft4Idx,
    _Ft5Idx,
    _Ft6Idx,
    _Ft7Idx,

    _Fs0Idx,
    _Fs1Idx,

    _Fa0Idx,
    _Fa1Idx,
    _Fa2Idx,
    _Fa3Idx,
    _Fa4Idx,
    _Fa5Idx,
    _Fa6Idx,
    _Fa7Idx,

    _Fs2Idx,
    _Fs3Idx,
    _Fs4Idx,
    _Fs5Idx,
    _Fs6Idx,
    _Fs7Idx,
    _Fs8Idx,
    _Fs9Idx,
    _Fs10Idx,
    _Fs11Idx,

    _Ft8Idx,
    _Ft9Idx,
    _Ft10Idx,
    _Ft11Idx,

    NumRegs
};

} // namespace float_reg

inline constexpr RegClass floatRegClass(FloatRegClass, FloatRegClassName,
                                        float_reg::NumRegs, debug::FloatRegs);

namespace float_reg
{

/* clang-format off */
inline constexpr RegId
    Ft0 = floatRegClass[_Ft0Idx],
    Ft1 = floatRegClass[_Ft1Idx],
    Ft2 = floatRegClass[_Ft2Idx],
    Ft3 = floatRegClass[_Ft3Idx],
    Ft4 = floatRegClass[_Ft4Idx],
    Ft5 = floatRegClass[_Ft5Idx],
    Ft6 = floatRegClass[_Ft6Idx],
    Ft7 = floatRegClass[_Ft7Idx],

    Fs0 = floatRegClass[_Fs0Idx],
    Fs1 = floatRegClass[_Fs1Idx],

    Fa0 = floatRegClass[_Fa0Idx],
    Fa1 = floatRegClass[_Fa1Idx],
    Fa2 = floatRegClass[_Fa2Idx],
    Fa3 = floatRegClass[_Fa3Idx],
    Fa4 = floatRegClass[_Fa4Idx],
    Fa5 = floatRegClass[_Fa5Idx],
    Fa6 = floatRegClass[_Fa6Idx],
    Fa7 = floatRegClass[_Fa7Idx],

    Fs2 = floatRegClass[_Fs2Idx],
    Fs3 = floatRegClass[_Fs3Idx],
    Fs4 = floatRegClass[_Fs4Idx],
    Fs5 = floatRegClass[_Fs5Idx],
    Fs6 = floatRegClass[_Fs6Idx],
    Fs7 = floatRegClass[_Fs7Idx],
    Fs8 = floatRegClass[_Fs8Idx],
    Fs9 = floatRegClass[_Fs9Idx],
    Fs10 = floatRegClass[_Fs10Idx],
    Fs11 = floatRegClass[_Fs11Idx],

    Ft8 = floatRegClass[_Ft8Idx],
    Ft9 = floatRegClass[_Ft9Idx],
    Ft10 = floatRegClass[_Ft10Idx],
    Ft11 = floatRegClass[_Ft11Idx];

const std::vector<std::string> RegNames = {
    "ft0", "ft1", "ft2", "ft3",
    "ft4", "ft5", "ft6", "ft7",
    "fs0", "fs1", "fa0", "fa1",
    "fa2", "fa3", "fa4", "fa5",
    "fa6", "fa7", "fs2", "fs3",
    "fs4", "fs5", "fs6", "fs7",
    "fs8", "fs9", "fs10", "fs11",
    "ft8", "ft9", "ft10", "ft11"
};
/* clang-format on */

} // namespace float_reg

inline float32_t
fsgnj32(float32_t a, float32_t b, bool n, bool x)
{
    if (n)
        b.v = ~b.v;
    else if (x)
        b.v = a.v ^ b.v;
    return f32(insertBits(b.v, 30, 0, a.v));
}

inline float64_t
fsgnj64(float64_t a, float64_t b, bool n, bool x)
{
    if (n)
        b.v = ~b.v;
    else if (x)
        b.v = a.v ^ b.v;
    return f64(insertBits(b.v, 62, 0, a.v));
}

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_FLOAT_HH__
