/*
 * Copyright 2020 Google, Inc.
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

#include <gtest/gtest.h>

#include "arch/arm/aapcs64.hh"

using namespace gem5;

TEST(Aapcs64, IsAapcs64ShortVector)
{
    using Scalar = uint64_t;
    using TooShort = uint8_t[2];
    using TooLong = uint16_t[32];
    using TooLongFloat = double[4];
    using EightLong = uint32_t[2];
    using SixteenLong = uint64_t[2];
    using EightLongFloat = float[2];
    using SixteenLongFloat = double[2];

    EXPECT_FALSE(guest_abi::IsAapcs64ShortVectorV<Scalar>);
    EXPECT_FALSE(guest_abi::IsAapcs64ShortVectorV<TooShort>);
    EXPECT_FALSE(guest_abi::IsAapcs64ShortVectorV<TooLong>);
    EXPECT_FALSE(guest_abi::IsAapcs64ShortVectorV<TooLongFloat>);
    EXPECT_FALSE(guest_abi::IsAapcs64ShortVectorV<void>);

    EXPECT_TRUE(guest_abi::IsAapcs64ShortVectorV<EightLong>);
    EXPECT_TRUE(guest_abi::IsAapcs64ShortVectorV<SixteenLong>);
    EXPECT_TRUE(guest_abi::IsAapcs64ShortVectorV<EightLongFloat>);
    EXPECT_TRUE(guest_abi::IsAapcs64ShortVectorV<SixteenLongFloat>);
}

TEST(Aapcs64, IsAapcs64Hfa)
{
    // Accept floating point arrays with up to 4 members.
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<float[1]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<float[2]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<float[3]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<float[4]>);

    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<double[1]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<double[2]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<double[3]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HfaV<double[4]>);

    // Too many members.
    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<float[5]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<double[5]>);

    // Wrong type of members, or not arrays.
    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<int32_t[3]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<float>);

    struct Struct
    {
    };

    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<Struct>);
    EXPECT_FALSE(guest_abi::IsAapcs64HfaV<void>);
}

TEST(Aapcs64, IsAapcs64Hva)
{
    using SvaInt = uint32_t[2];
    using SvaTiny = uint8_t[16];
    using SvaFloat = float[2];

    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaInt[3]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaInt[4]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<SvaInt[5]>);

    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaFloat[3]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaFloat[4]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<SvaFloat[5]>);

    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaTiny[3]>);
    EXPECT_TRUE(guest_abi::IsAapcs64HvaV<SvaTiny[4]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<SvaTiny[5]>);

    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<uint64_t>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<uint64_t[1]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<SvaTiny>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<void>);
    EXPECT_FALSE(guest_abi::IsAapcs64HvaV<float>);
}

TEST(Aapcs64, IsAapcs64Hxa)
{
    using SvaInt = uint32_t[2];

    EXPECT_TRUE(guest_abi::IsAapcs64HxaV<SvaInt[4]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HxaV<SvaInt[5]>);

    EXPECT_TRUE(guest_abi::IsAapcs64HxaV<float[4]>);
    EXPECT_FALSE(guest_abi::IsAapcs64HxaV<float[5]>);

    EXPECT_FALSE(guest_abi::IsAapcs64HxaV<SvaInt>);
    EXPECT_FALSE(guest_abi::IsAapcs64HxaV<uint64_t>);
    EXPECT_FALSE(guest_abi::IsAapcs64HxaV<void>);
}
