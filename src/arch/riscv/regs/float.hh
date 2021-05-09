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

namespace gem5
{

namespace RiscvISA
{

/* Conversion functions for working with softfloat. */

// Generic floating point value type.
using freg_t = float64_t;

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

static constexpr uint64_t boxF32(uint32_t v) { return mask(63, 32) | v; }

// Create fixed size floats from raw bytes or generic floating point values.
static constexpr float32_t f32(uint32_t v) { return {v}; }
static constexpr float64_t f64(uint64_t v) { return {v}; }
static constexpr float32_t f32(freg_t r) { return {unboxF32(r.v)}; }
static constexpr float64_t f64(freg_t r) { return r; }

// Create generic floating point values from fixed size floats.
static constexpr freg_t freg(float32_t f) { return {boxF32(f.v)}; }
static constexpr freg_t freg(float64_t f) { return f; }
static constexpr freg_t freg(uint_fast16_t f) { return {f}; }

const int NumFloatRegs = 32;

const std::vector<std::string> FloatRegNames = {
    "ft0", "ft1", "ft2", "ft3",
    "ft4", "ft5", "ft6", "ft7",
    "fs0", "fs1", "fa0", "fa1",
    "fa2", "fa3", "fa4", "fa5",
    "fa6", "fa7", "fs2", "fs3",
    "fs4", "fs5", "fs6", "fs7",
    "fs8", "fs9", "fs10", "fs11",
    "ft8", "ft9", "ft10", "ft11"
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_FLOAT_HH__
