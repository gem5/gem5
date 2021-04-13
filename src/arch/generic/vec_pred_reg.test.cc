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

#include "arch/generic/vec_pred_reg.hh"
#include "base/str.hh"

using namespace gem5;

TEST(VecPredReg, reset)
{
    constexpr size_t size = 4;
    VecPredRegContainer<size, false> vec;

    vec.reset();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_FALSE(vec[idx]);
    }
}

TEST(VecPredReg, set)
{
    constexpr size_t size = 4;
    VecPredRegContainer<size, false> vec;

    vec.set();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_TRUE(vec[idx]);
    }
}

template <bool T>
class TwoDifferentVecPredRegsBase : public testing::Test
{
  protected:
    static constexpr ssize_t size = 4;
    VecPredRegContainer<size, T> pred1;
    VecPredRegContainer<size, T> pred2;

    void
    SetUp() override
    {
        // Initializing with:
        // 0,1,0,1
        for (auto idx = 0; idx < size; idx++) {
            pred1[idx] = (idx % 2);
        }

        // Initializing with:
        // 1,0,1,0
        for (auto idx = 0; idx < size; idx++) {
            pred2[idx] = !(idx % 2);
        }
    }
};

using TwoDifferentVecPredRegs = TwoDifferentVecPredRegsBase<false>;
using TwoPackedDifferentVecPredRegs = TwoDifferentVecPredRegsBase<true>;

// Testing operator=
TEST_F(TwoDifferentVecPredRegs, Assignment)
{
    pred2 = pred1;

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred2[idx], idx % 2);
    }
}

// Testing operator==
TEST_F(TwoDifferentVecPredRegs, Equality)
{
    // Equality check
    ASSERT_TRUE(pred1 == pred1);
    ASSERT_TRUE(pred2 == pred2);
    ASSERT_FALSE(pred1 == pred2);
}

// Testing operator!=
TEST_F(TwoDifferentVecPredRegs, Inequality)
{
    // Inequality check
    ASSERT_FALSE(pred1 != pred1);
    ASSERT_FALSE(pred2 != pred2);
    ASSERT_TRUE(pred1 != pred2);
}

// Testing operator<<
TEST_F(TwoDifferentVecPredRegs, Printing)
{
    {
        std::ostringstream stream;
        stream << pred1;
        ASSERT_EQ(stream.str(), "[0 1 0 1]");
    }

    {
        std::ostringstream stream;
        stream << pred2;
        ASSERT_EQ(stream.str(), "[1 0 1 0]");
    }
}

// Testing ParseParam
TEST_F(TwoDifferentVecPredRegs, ParseParam)
{
    ParseParam<decltype(pred1)> parser;
    parser.parse("1111", pred1);
    parser.parse("0000", pred2);

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred1[idx], 1);
        ASSERT_EQ(pred2[idx], 0);
    }
}

// Testing ShowParam
TEST_F(TwoDifferentVecPredRegs, ShowParam)
{
    ShowParam<decltype(pred1)> parser;

    {
        std::stringstream ss;
        parser.show(ss, pred1);
        ASSERT_EQ(ss.str(), "0101");
    }

    {
        std::stringstream ss;
        parser.show(ss, pred2);
        ASSERT_EQ(ss.str(), "1010");
    }
}

// Testing VecPredReg view as uint8_t
// pred1 is 0101
//     -> pred1_view[0] = false
//     -> pred1_view[1] = true
//     -> pred1_view[2] = false
//     -> pred1_view[3] = true
// pred2 is 1010
//     -> pred2_view[0] = true
//     -> pred2_view[1] = false
//     -> pred2_view[2] = true
//     -> pred2_view[3] = false
TEST_F(TwoDifferentVecPredRegs, View8bit)
{
    auto pred1_view = pred1.as<uint8_t>();
    auto pred2_view = pred2.as<uint8_t>();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred1_view[idx], idx % 2);
        ASSERT_EQ(pred2_view[idx], !(idx % 2));
    }
}

// Testing VecPredReg view as uint16_t
// pred1 is 0101
//     -> pred1_view[0] = false
//     -> pred1_view[1] = false
// pred2 is 1010
//     -> pred2_view[0] = true
//     -> pred2_view[1] = true
TEST_F(TwoDifferentVecPredRegs, View16bit)
{
    auto pred1_view = pred1.as<uint16_t>();
    auto pred2_view = pred2.as<uint16_t>();

    for (auto idx = 0; idx < size / sizeof(uint16_t); idx++) {
        ASSERT_FALSE(pred1_view[idx]);
        ASSERT_TRUE(pred2_view[idx]);
    }
}

// Testing VecPredReg view as uint32_t
// pred1 is 0101
//     -> pred1_view[0] = false
// pred2 is 1010
//     -> pred2_view[0] = true
TEST_F(TwoDifferentVecPredRegs, View32bit)
{
    auto pred1_view = pred1.as<uint32_t>();
    auto pred2_view = pred2.as<uint32_t>();

    ASSERT_FALSE(pred1_view[0]);
    ASSERT_TRUE(pred2_view[0]);
}

// Testing VecPredReg view as uint8_t
// pred1 is 0101
//     -> pred1_view[0] = false
//     -> pred1_view[1] = true
//     -> pred1_view[2] = false
//     -> pred1_view[3] = true
// pred2 is 1010
//     -> pred2_view[0] = true
//     -> pred2_view[1] = false
//     -> pred2_view[2] = true
//     -> pred2_view[3] = false
TEST_F(TwoPackedDifferentVecPredRegs, View8bit)
{
    auto pred1_view = pred1.as<uint8_t>();
    auto pred2_view = pred2.as<uint8_t>();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred1_view[idx], idx % 2);
        ASSERT_EQ(pred2_view[idx], !(idx % 2));
    }
}

// Testing VecPredReg view as uint16_t
// pred1 is 0101
//     -> pred1_view[0] = false
//     -> pred1_view[1] = true
//     -> pred1_view[2] = false
//     -> pred1_view[3] = true
// pred2 is 1010
//     -> pred2_view[0] = true
//     -> pred2_view[1] = false
//     -> pred2_view[2] = true
//     -> pred2_view[3] = false
TEST_F(TwoPackedDifferentVecPredRegs, View16bit)
{
    auto pred1_view = pred1.as<uint16_t>();
    auto pred2_view = pred2.as<uint16_t>();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred1_view[idx], idx % 2);
        ASSERT_EQ(pred2_view[idx], !(idx % 2));
    }
}

// Testing VecPredReg view as uint32_t
// pred1 is 0101
//     -> pred1_view[0] = false
//     -> pred1_view[1] = true
//     -> pred1_view[2] = false
//     -> pred1_view[3] = true
// pred2 is 1010
//     -> pred2_view[0] = true
//     -> pred2_view[1] = false
//     -> pred2_view[2] = true
//     -> pred2_view[3] = false
TEST_F(TwoPackedDifferentVecPredRegs, View32bit)
{
    auto pred1_view = pred1.as<uint32_t>();
    auto pred2_view = pred2.as<uint32_t>();

    for (auto idx = 0; idx < size; idx++) {
        ASSERT_EQ(pred1_view[idx], idx % 2);
        ASSERT_EQ(pred2_view[idx], !(idx % 2));
    }
}
