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

#include <gtest/gtest.h>

#include "arch/amdgpu/common/dtype/mxfp_types.hh"

template<typename T>
bool test_raw_mxfp(T raw_mxfp, int bits)
{
    float tmp = float(raw_mxfp);
    T from_float(tmp);

    // Simply check that casting to float and back yields the same bit values.
    // Exclude inf/NaN as those have multiple values in some MXFP types.
    if (raw_mxfp.data != from_float.data &&
        !std::isnan(tmp) && !std::isinf(tmp)) {
        return false;
    }

    return true;
}

template<typename T>
int test_type(int bits)
{
    T raw_mxfp;
    int errors = 0;

    int max_val = 1 << bits;
    for (int val = 0; val < max_val; ++val) {
        // Raw data is aligned to MSb in MXFP types. Shift into place.
        raw_mxfp.data = val << (32 - bits);
        if (!test_raw_mxfp(raw_mxfp, bits)) {
            errors++;
        }
    }

    return errors;
}

TEST(MxfpTest, MxBf16Test)
{
    using T = gem5::AMDGPU::mxbfloat16;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
}

TEST(MxfpTest, MxFp16Test)
{
    using T = gem5::AMDGPU::mxfloat16;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
}

TEST(MxfpTest, MxBf8Test)
{
    using T = gem5::AMDGPU::mxbfloat8;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
}

TEST(MxfpTest, MxFp8Test)
{
    using T = gem5::AMDGPU::mxfloat8;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
}
