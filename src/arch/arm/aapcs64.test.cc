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

    EXPECT_FALSE(GuestABI::IsAapcs64ShortVector<Scalar>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64ShortVector<TooShort>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64ShortVector<TooLong>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64ShortVector<TooLongFloat>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64ShortVector<void>::value);

    EXPECT_TRUE(GuestABI::IsAapcs64ShortVector<EightLong>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64ShortVector<SixteenLong>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64ShortVector<EightLongFloat>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64ShortVector<SixteenLongFloat>::value);
}

TEST(Aapcs64, IsAapcs64Hfa)
{
    // Accept floating point arrays with up to 4 members.
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<float[1]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<float[2]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<float[3]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<float[4]>::value);

    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<double[1]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<double[2]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<double[3]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hfa<double[4]>::value);

    // Too many members.
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<float[5]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<double[5]>::value);

    // Wrong type of members, or not arrays.
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<int32_t[3]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<float>::value);
    struct Struct {};
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<Struct>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hfa<void>::value);
}

TEST(Aapcs64, IsAapcs64Hva)
{
    using SvaInt = uint32_t[2];
    using SvaTiny = uint8_t[16];
    using SvaFloat = float[2];

    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaInt[3]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaInt[4]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<SvaInt[5]>::value);

    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaFloat[3]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaFloat[4]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<SvaFloat[5]>::value);

    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaTiny[3]>::value);
    EXPECT_TRUE(GuestABI::IsAapcs64Hva<SvaTiny[4]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<SvaTiny[5]>::value);

    EXPECT_FALSE(GuestABI::IsAapcs64Hva<uint64_t>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<uint64_t[1]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<SvaTiny>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<void>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hva<float>::value);
}

TEST(Aapcs64, IsAapcs64Hxa)
{
    using SvaInt = uint32_t[2];

    EXPECT_TRUE(GuestABI::IsAapcs64Hxa<SvaInt[4]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hxa<SvaInt[5]>::value);

    EXPECT_TRUE(GuestABI::IsAapcs64Hxa<float[4]>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hxa<float[5]>::value);

    EXPECT_FALSE(GuestABI::IsAapcs64Hxa<SvaInt>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hxa<uint64_t>::value);
    EXPECT_FALSE(GuestABI::IsAapcs64Hxa<void>::value);
}
