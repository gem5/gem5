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

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_FP16_E5M10_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_FP16_E5M10_HH__

#include <cassert>

namespace gem5
{

namespace AMDGPU
{

typedef union
{
    enum bitSizes
    {
        ebits = 5,
        mbits = 10,
        sbits = 1,
        zbits = 16,
        bias = 15,

        inf = 0x7c000000,
        nan = 0x7c100000,
        max = 0x7bff0000
    };

    uint32_t storage;
    struct
    {
        unsigned zero : zbits;
        unsigned mant : mbits;
        unsigned exp  : ebits;
        unsigned sign : sbits;
    };
} fp16_e5m10_info;
static_assert(sizeof(fp16_e5m10_info) == 4);

} // namespace AMDGPU

} // namespace gem5


// std library cmath definitions
namespace std
{

constexpr bool isinf(gem5::AMDGPU::fp16_e5m10_info a)
{
    return a.exp == 0x1F && a.mant == 0;
}

constexpr bool isnan(gem5::AMDGPU::fp16_e5m10_info a)
{
    return a.exp == 0x1F && a.mant != 0;
}

constexpr bool isnormal(gem5::AMDGPU::fp16_e5m10_info a)
{
    return !(a.exp == 0 && a.mant != 0);
}

template<>
class numeric_limits<gem5::AMDGPU::fp16_e5m10_info>
{
  public:
    static constexpr bool has_quiet_NaN = true;
    static gem5::AMDGPU::fp16_e5m10_info quiet_NaN()
    {
        assert(has_quiet_NaN);
        gem5::AMDGPU::fp16_e5m10_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e5m10_info::nan;
        return tmp;
    }

    static constexpr bool has_infinity = true;
    static gem5::AMDGPU::fp16_e5m10_info infinity()
    {
        assert(has_infinity);
        gem5::AMDGPU::fp16_e5m10_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e5m10_info::inf;
        return tmp;
    }

    static gem5::AMDGPU::fp16_e5m10_info max()
    {
        gem5::AMDGPU::fp16_e5m10_info tmp;
        tmp.storage = gem5::AMDGPU::fp16_e5m10_info::max;
        return tmp;
    }
};

} // namespace std

#endif // __ARCH_AMDGPU_COMMON_DTYPE_FP16_E5M10_HH__
