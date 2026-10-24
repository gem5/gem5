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

template <typename T>
bool
test_raw_mxfp(T raw_mxfp, int bits)
{
    float tmp = float(raw_mxfp);
    T from_float(tmp);

    // Simply check that casting to float and back yields the same bit values.
    // Exclude inf/NaN as those have multiple values in some MXFP types.
    if (raw_mxfp.data != from_float.data && !std::isnan(tmp) &&
        !std::isinf(tmp)) {
        return false;
    }

    return true;
}

template <typename T>
int
test_type(int bits)
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

// Checks rounding to nearest, ties to even, from float: values between two
// neighbouring T values convert to the nearer one and midpoints to the one
// with the even encoding. Values past the largest finite T value convert
// like an infinity.
template <typename T>
int
test_rounding()
{
    using FMT = decltype(T().getFmt());
    const int shift = 32 - T::size();
    const uint32_t sign = 1 << (T::size() - 1);

    // Largest finite encoding, without the sign bit.
    uint32_t max_bits = sign - 1;
    if (std::numeric_limits<FMT>::has_infinity) {
        max_bits -= 1 << FMT::mbits;
    } else if (std::numeric_limits<FMT>::has_quiet_NaN) {
        max_bits -= 1;
    }
    const uint32_t overflow =
        (std::numeric_limits<FMT>::has_infinity ? FMT::inf : FMT::max) >>
        shift;

    auto value = [](uint32_t bits) { return float(T(bits)); };
    auto convert = [shift](float f) { return T(f).data >> shift; };
    const float inf = std::numeric_limits<float>::infinity();

    int errors = 0;
    for (uint32_t bits = 0; bits <= max_bits; ++bits) {
        float lo = value(bits);
        float step =
            bits < max_bits ? value(bits + 1) - lo : lo - value(bits - 1);
        float mid = lo + step / 2;
        uint32_t up = bits < max_bits ? bits + 1 : overflow;
        uint32_t even = (bits & 1) ? up : bits;

        errors += convert(std::nextafter(mid, 0.0f)) != bits;
        errors += convert(mid) != even;
        errors += convert(-mid) != (even | sign);
        errors += convert(std::nextafter(mid, inf)) != up;
    }
    errors += convert(std::numeric_limits<float>::max()) != overflow;
    errors += convert(std::numeric_limits<float>::denorm_min()) != 0;

    return errors;
}

TEST(MxfpTest, MxBf16Test)
{
    using T = gem5::AMDGPU::mxbfloat16;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxFp16Test)
{
    using T = gem5::AMDGPU::mxfloat16;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxBf8Test)
{
    using T = gem5::AMDGPU::mxbfloat8;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxFp8Test)
{
    using T = gem5::AMDGPU::mxfloat8;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxBf6Test)
{
    using T = gem5::AMDGPU::mxbf6;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxFp6Test)
{
    using T = gem5::AMDGPU::mxfp6;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}

TEST(MxfpTest, MxFp4Test)
{
    using T = gem5::AMDGPU::mxfp4;

    int errors = test_type<T>(T::size());

    EXPECT_EQ(errors, 0);
    EXPECT_EQ(test_rounding<T>(), 0);
}
