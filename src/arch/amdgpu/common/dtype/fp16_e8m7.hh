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

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_FP16_E8M7_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_FP16_E8M7_HH__

#include <cassert>

namespace gem5
{

namespace AMDGPU
{

union fp16_e8m7_info
{
    static constexpr size_t ebits = 8;
    static constexpr size_t mbits = 7;
    static constexpr size_t sbits = 1;
    static constexpr size_t zbits = 16;
    static constexpr size_t bias = 127;

    static constexpr uint32_t inf = 0x7f800000;
    static constexpr uint32_t nan = 0x7f810000;
    static constexpr uint32_t max = 0x7f7f0000;

    uint32_t storage;
    struct
    {
        unsigned zero : zbits;
        unsigned mant : mbits;
        unsigned exp : ebits;
        unsigned sign : sbits;
    };
};
static_assert(sizeof(fp16_e8m7_info) == 4);

} // namespace AMDGPU

} // namespace gem5

// std library cmath definitions
namespace std
{

constexpr bool
isinf(gem5::AMDGPU::fp16_e8m7_info a)
{
    return a.exp == 0xFF && a.mant == 0;
}

constexpr bool
isnan(gem5::AMDGPU::fp16_e8m7_info a)
{
    return a.exp == 0xFF && a.mant != 0;
}

constexpr bool
isnormal(gem5::AMDGPU::fp16_e8m7_info a)
{
    return !(a.exp == 0 && a.mant != 0);
}

template <> class numeric_limits<gem5::AMDGPU::fp16_e8m7_info>
{
  public:
    static constexpr bool has_quiet_NaN = true;
    static gem5::AMDGPU::fp16_e8m7_info
    quiet_NaN()
    {
        assert(has_quiet_NaN);
        gem5::AMDGPU::fp16_e8m7_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e8m7_info::nan;
        return tmp;
    }

    static constexpr bool has_infinity = true;
    static gem5::AMDGPU::fp16_e8m7_info
    infinity()
    {
        assert(has_infinity);
        gem5::AMDGPU::fp16_e8m7_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e8m7_info::inf;
        return tmp;
    }

    static gem5::AMDGPU::fp16_e8m7_info
    max()
    {
        gem5::AMDGPU::fp16_e8m7_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e8m7_info::max;
        return tmp;
    }
};

} // namespace std

#endif // __ARCH_AMDGPU_COMMON_DTYPE_FP16_E8M7_HH__
