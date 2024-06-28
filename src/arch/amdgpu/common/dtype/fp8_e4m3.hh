/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_FP8_E4M3_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_FP8_E4M3_HH__

#include <cassert>

namespace gem5
{

namespace AMDGPU
{

typedef union
{
    enum bitSizes
    {
        ebits = 4,
        mbits = 3,
        sbits = 1,
        zbits = 24,
        bias = 7,

        inf = (0x7f << zbits),
        nan = (0xff << zbits),
        max = (0x7f << zbits)
    };

    uint32_t storage;
    struct
    {
        unsigned zero : zbits;
        unsigned mant : mbits;
        unsigned exp  : ebits;
        unsigned sign : sbits;
    };
} fp8_e4m3_info;
static_assert(sizeof(fp8_e4m3_info) == 4);

} // namespace AMDGPU

} // namespace gem5


// std library cmath definitions
namespace std
{

// Inf not defined
constexpr bool isinf(gem5::AMDGPU::fp8_e4m3_info a) { return false; }

constexpr bool isnan(gem5::AMDGPU::fp8_e4m3_info a)
{
    return a.exp == 0xF && a.mant == 0x7;
}

constexpr bool isnormal(gem5::AMDGPU::fp8_e4m3_info a)
{
    return !(a.exp == 0 && a.mant != 0);
}


template<>
class numeric_limits<gem5::AMDGPU::fp8_e4m3_info>
{
  public:
    static constexpr bool has_quiet_NaN = true;
    static gem5::AMDGPU::fp8_e4m3_info quiet_NaN()
    {
        assert(has_quiet_NaN);
        gem5::AMDGPU::fp8_e4m3_info tmp;
        tmp.storage = gem5::AMDGPU::fp8_e4m3_info::nan;
        return tmp;
    }

    static constexpr bool has_infinity = false;
    static gem5::AMDGPU::fp8_e4m3_info infinity()
    {
        assert(has_infinity);
        gem5::AMDGPU::fp8_e4m3_info tmp;
        tmp.storage = gem5::AMDGPU::fp8_e4m3_info::inf;
        return tmp;
    }

    static gem5::AMDGPU::fp8_e4m3_info max()
    {
        gem5::AMDGPU::fp8_e4m3_info tmp;
        tmp.storage = gem5::AMDGPU::fp8_e4m3_info::max;
        return tmp;
    }
};

} // namespace std

#endif // __ARCH_AMDGPU_COMMON_DTYPE_FP8_E4M3_HH__
