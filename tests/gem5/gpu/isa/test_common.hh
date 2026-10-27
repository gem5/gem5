/*
 * Copyright (c) 2026 Boyana Norris
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

#ifndef GEM5_GPU_ISA_TEST_COMMON_HH
#define GEM5_GPU_ISA_TEST_COMMON_HH

#include <hip/hip_runtime.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

inline void
checkHip(hipError_t status, const char *operation)
{
    if (status != hipSuccess) {
        std::fprintf(stderr, "%s: %s\n", operation, hipGetErrorString(status));
        std::exit(1);
    }
}

#define HIP_CHECK(operation) checkHip(operation, #operation)

inline void
requireMi300x()
{
    hipDeviceProp_t properties;
    HIP_CHECK(hipGetDeviceProperties(&properties, 0));
    if (properties.warpSize != 64 ||
        std::strncmp(properties.gcnArchName, "gfx942", 6) != 0) {
        std::fprintf(stderr,
                     "Expected gfx942 with 64-lane wavefronts, got %s\n",
                     properties.gcnArchName);
        std::exit(1);
    }
}

inline void
expectBits(const char *test, unsigned lane, uint32_t got, uint32_t expected,
           unsigned &checks, unsigned &errors)
{
    ++checks;
    if (got != expected && errors++ < 20) {
        std::fprintf(stderr, "%s lane %u: got %08x, expected %08x\n", test,
                     lane, got, expected);
    }
}

inline int
finish(const char *test, unsigned checks, unsigned errors)
{
    if (errors) {
        std::fprintf(stderr, "GPU ISA %s failed: %u / %u checks\n", test,
                     errors, checks);
        return 1;
    }
    std::printf("GPU ISA %s passed: %u checks\n", test, checks);
    return 0;
}

#endif
