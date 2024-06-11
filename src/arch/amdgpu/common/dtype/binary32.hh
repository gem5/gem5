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

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_BINARY32_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_BINARY32_HH__

namespace gem5
{

namespace AMDGPU
{

// Same as IEEE 754 binary 32 - Microscaling types are converted to/from
// this format by default. For now as there do not seem to be any MI300
// instructions operating directly on the types (i.e., they all cast to FP32
// first and then perform arithmetic operations).
typedef union binary32_u
{
    enum bitSizes
    {
        ebits = 8,
        mbits = 23,
        sbits = 1,
        bias = 127,

        inf = 0x7f800000,
        nan = 0x7f800100,
        max = 0x7f7fffff
    };

    uint32_t storage;
    float    fp32;
    struct
    {
        unsigned mant : 23;
        unsigned exp  : 8;
        unsigned sign : 1;
    };

    // To help with stdlib functions with T = float.
    operator float() const
    {
        return fp32;
    }
} binary32;
static_assert(sizeof(binary32) == 4);

} // namespace AMDGPU

} // namespace gem5

namespace std
{

template<>
class numeric_limits<gem5::AMDGPU::binary32>
{
  public:
    static constexpr bool has_quiet_NaN = true;
    static gem5::AMDGPU::binary32 quiet_NaN()
    {
        gem5::AMDGPU::binary32 tmp;
        tmp.fp32 = std::numeric_limits<float>::quiet_NaN();
        return tmp;
    }

    static constexpr bool has_infinity = true;
    static gem5::AMDGPU::binary32 infinity()
    {
        gem5::AMDGPU::binary32 tmp;
        tmp.fp32 = std::numeric_limits<float>::infinity();
        return tmp;
    }

    static gem5::AMDGPU::binary32 max()
    {
        gem5::AMDGPU::binary32 tmp;
        tmp.fp32 = std::numeric_limits<float>::max();
        return tmp;
    }
};

} // namespace std

#endif // __ARCH_AMDGPU_COMMON_DTYPE_BINARY32_HH__
