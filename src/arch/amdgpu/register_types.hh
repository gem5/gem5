/*
 * Copyright (c) 2026 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_AMDGPU_REGISTER_TYPES_HH__
#define __ARCH_AMDGPU_REGISTER_TYPES_HH__

#include <cstdint>

namespace gem5
{

namespace AMDGPU
{

// Typedefs for the various sizes/types of scalar regs
typedef uint8_t ScalarRegU8;
typedef int8_t ScalarRegI8;
typedef uint16_t ScalarRegU16;
typedef int16_t ScalarRegI16;
typedef uint32_t ScalarRegU32;
typedef int32_t ScalarRegI32;
typedef float ScalarRegF32;
typedef uint64_t ScalarRegU64;
typedef int64_t ScalarRegI64;
typedef double ScalarRegF64;

// Typedefs for the various sizes/types of vector reg elements
typedef uint8_t VecElemU8;
typedef int8_t VecElemI8;
typedef uint16_t VecElemU16;
typedef int16_t VecElemI16;
typedef uint32_t VecElemU32;
typedef int32_t VecElemI32;
typedef float VecElemF32;
typedef uint64_t VecElemU64;
typedef int64_t VecElemI64;
typedef double VecElemF64;

} // namespace AMDGPU
} // namespace gem5

#endif // __ARCH_AMDGPU_REGISTER_TYPES_HH__
