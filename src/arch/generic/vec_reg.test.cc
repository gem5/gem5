/*
 * Copyright (c) 2021 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "arch/generic/vec_reg.hh"

using namespace gem5;

TEST(VecReg, Size)
{
    {
        // Minimum size
        VecRegContainer<1> vec;
        ASSERT_EQ(1, vec.size());
    }

    {
        // Medium size
        constexpr size_t size = MaxVecRegLenInBytes / 2;
        VecRegContainer<size> vec;
        ASSERT_EQ(size, vec.size());
    }

    {
        // Maximum size
        VecRegContainer<MaxVecRegLenInBytes> vec;
        ASSERT_EQ(MaxVecRegLenInBytes, vec.size());
    }
}

TEST(VecReg, Zero)
{
    constexpr size_t size = 16;
    VecRegContainer<size> vec;
    auto *vec_ptr = vec.as<uint8_t>();

    // Initializing with non-zero value
    for (auto idx = 0; idx < size; idx++) {
        vec_ptr[idx] = 0xAA;
    }

    // zeroing the vector
    vec.zero();

    // checking if every vector element is set to zero
    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(vec_ptr[idx], 0);
    }
}

class TwoDifferentVecRegs : public testing::Test
{
  protected:
    VecRegContainer<16> vec1;
    VecRegContainer<16> vec2;
    uint8_t *vec1_ptr;
    uint8_t *vec2_ptr;

    void
    SetUp() override
    {
        vec1_ptr = vec1.as<uint8_t>();
        vec2_ptr = vec2.as<uint8_t>();

        // Initializing with non-zero value vector1
        for (auto idx = 0; idx < vec1.size(); idx++) {
            vec1_ptr[idx] = 0xAA;
        }

        // Initializing with zero value vector2
        for (auto idx = 0; idx < vec2.size(); idx++) {
            vec2_ptr[idx] = 0;
        }
    }
};

// Testing operator=
TEST_F(TwoDifferentVecRegs, Assignment)
{
    // Copying the vector
    vec2 = vec1;

    // Checking if vector2 elements are 0xAA
    for (auto idx = 0; idx < vec2.size(); idx++) {
        ASSERT_EQ(vec2_ptr[idx], 0xAA);
    }
}

// Testing operator==
TEST_F(TwoDifferentVecRegs, Equality)
{
    // Equality check
    ASSERT_TRUE(vec1 == vec1);
    ASSERT_TRUE(vec2 == vec2);
    ASSERT_FALSE(vec1 == vec2);
}

// Testing operator!=
TEST_F(TwoDifferentVecRegs, Inequality)
{
    // Inequality check
    ASSERT_FALSE(vec1 != vec1);
    ASSERT_FALSE(vec2 != vec2);
    ASSERT_TRUE(vec1 != vec2);
}

// Testing operator<<
TEST_F(TwoDifferentVecRegs, Printing)
{
    {
        std::ostringstream stream;
        stream << vec1;
        ASSERT_EQ(stream.str(), "[aaaaaaaa_aaaaaaaa_aaaaaaaa_aaaaaaaa]");
    }

    {
        std::ostringstream stream;
        stream << vec2;
        ASSERT_EQ(stream.str(), "[00000000_00000000_00000000_00000000]");
    }
}

// Testing ParseParam
TEST_F(TwoDifferentVecRegs, ParseParam)
{
    ParseParam<decltype(vec1)> parser;

    parser.parse("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb", vec1);
    parser.parse("cccccccccccccccccccccccccccccccc", vec2);

    for (auto idx = 0; idx < 2; idx++) {
        ASSERT_EQ(vec1_ptr[idx], 0xbb);
        ASSERT_EQ(vec2_ptr[idx], 0xcc);
    }
}

// Testing ShowParam
TEST_F(TwoDifferentVecRegs, ShowParam)
{
    ShowParam<decltype(vec1)> parser;

    {
        std::stringstream ss;
        parser.show(ss, vec1);
        ASSERT_EQ(ss.str(), "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    }

    {
        std::stringstream ss;
        parser.show(ss, vec2);
        ASSERT_EQ(ss.str(), "00000000000000000000000000000000");
    }
}
