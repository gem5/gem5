/*
 * Copyright (c) 2021 ARM Limited
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
 * Copyright (c) 2019 The Regents of the University of California
 * All rights reserved
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
#include <tuple>

#include "base/intmath.hh"

using namespace gem5;

TEST(IntmathTest, isPowerOf2)
{
    EXPECT_TRUE(isPowerOf2(1));
    EXPECT_TRUE(isPowerOf2(32));
    EXPECT_TRUE(isPowerOf2(65536));
    EXPECT_TRUE(isPowerOf2(131072));
    EXPECT_TRUE(isPowerOf2(262144));
    EXPECT_FALSE(isPowerOf2(0));
    EXPECT_FALSE(isPowerOf2(36));
    EXPECT_FALSE(isPowerOf2(2521));
    EXPECT_FALSE(isPowerOf2(1679616));
}

TEST(IntmathTest, floorLog2)
{
    EXPECT_EQ(0, floorLog2(1));
    EXPECT_EQ(4, floorLog2(16));
    EXPECT_EQ(4, floorLog2(31));
    EXPECT_EQ(5, floorLog2(36));
    EXPECT_EQ(8, floorLog2(436));
    EXPECT_EQ(16, floorLog2(65537));
    EXPECT_EQ(20, floorLog2(1783592));
    EXPECT_EQ(41, floorLog2(2821109907456));

    // Test unsigned integers of various sizes.
    EXPECT_EQ(0, floorLog2((uint8_t)1));
    EXPECT_EQ(0, floorLog2((uint16_t)1));
    EXPECT_EQ(0, floorLog2((uint32_t)1));
    EXPECT_EQ(0, floorLog2((uint64_t)1));

    // Test signed integers of various sizes.
    EXPECT_EQ(0, floorLog2((int8_t)1));
    EXPECT_EQ(0, floorLog2((int16_t)1));
    EXPECT_EQ(0, floorLog2((int32_t)1));
    EXPECT_EQ(0, floorLog2((int64_t)1));
}

/* The IntmathDeathTest floorLog2 test is dependent on an assert being
 * triggered. We therefore only run this test for .debug and .opt (where
 * `TRACING_ON == 1`).
 */
#if TRACING_ON
TEST(IntmathDeathTest, floorLog2)
{
    // Verify a non-positive input triggers an assert.
    EXPECT_DEATH_IF_SUPPORTED(floorLog2(0), "x > 0");
}
#endif

TEST(IntmathTest, ceilLog2)
{
    EXPECT_EQ(0, ceilLog2(1));
    EXPECT_EQ(4, ceilLog2(16));
    EXPECT_EQ(5, ceilLog2(31));
    EXPECT_EQ(6, ceilLog2(36));
    EXPECT_EQ(9, ceilLog2(436));
    EXPECT_EQ(17, ceilLog2(65537));
    EXPECT_EQ(21, ceilLog2(1783592));
    EXPECT_EQ(42, ceilLog2(2821109907456));
}


TEST(IntmathTest, divCeil)
{
    EXPECT_EQ(5, divCeil(55, 13));
    EXPECT_EQ(90, divCeil(7922, 89));
    EXPECT_EQ(4, divCeil(4800, 1442));
    EXPECT_EQ(4, divCeil(75, 24));
    EXPECT_EQ(46, divCeil(451, 10));
}

TEST(IntmathTest, mulUnsignedNarrow)
{
    uint8_t a = 0xff;
    uint8_t b = 0x02;
    uint8_t hi;
    uint8_t low;
    mulUnsigned<uint8_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x1);
    EXPECT_EQ(low, 0xfe);

    a = 14;
    b = 9;
    mulUnsigned<uint8_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0x7e);

    a = 0;
    b = 0x55;
    mulUnsigned<uint8_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, mulSignedNarrow)
{
    int8_t a = -0x80;
    int8_t b = -0x7f;
    int8_t hi;
    int8_t low;
    mulSigned<int8_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x3f);
    EXPECT_EQ(low, -0x80);

    a = 14;
    b = -9;
    mulSigned<int8_t>(hi, low, a, b);
    EXPECT_EQ(hi, -0x01);
    EXPECT_EQ(low, -0x7e);

    a = 0;
    b = -0x55;
    mulSigned<int8_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, mulUnsignedMid)
{
    uint32_t a = 0xffffffffULL;
    uint32_t b = 0x00000002ULL;
    uint32_t hi;
    uint32_t low;
    mulUnsigned<uint32_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x1);
    EXPECT_EQ(low, 0xfffffffe);

    a = 68026386;
    b = 5152;
    mulUnsigned<uint32_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x51);
    EXPECT_EQ(low, 0x99c16a40);

    a = 0;
    b = 0x55555555;
    mulUnsigned<uint32_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, mulSignedMid)
{
    int32_t a = -0x80000000;
    int32_t b = -0x7fffffff;
    int32_t hi;
    int32_t low;
    mulSigned<int32_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x3fffffff);
    EXPECT_EQ(low, -0x80000000);

    a = -68026386;
    b = 5152;
    mulSigned<int32_t>(hi, low, a, b);
    EXPECT_EQ(hi, -0x52);
    EXPECT_EQ(low, -0x99c16a40);

    a = 0;
    b = -0x55555555;
    mulSigned<int32_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, mulUnsignedWide)
{
    uint64_t a = 0xffffffffffffffffULL;
    uint64_t b = 0x0000000000000002ULL;
    uint64_t hi;
    uint64_t low;
    mulUnsigned<uint64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x1);
    EXPECT_EQ(low, 0xfffffffffffffffe);

    hi = 0;
    low = 0;
    mulUnsignedManual<uint64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x1);
    EXPECT_EQ(low, 0xfffffffffffffffe);

    hi = 0;
    low = 0;
    std::tie(hi, low) = mulUnsigned<uint64_t>(a, b);
    EXPECT_EQ(hi, 0x1);
    EXPECT_EQ(low, 0xfffffffffffffffe);

    a = 0;
    b = 0x5555555555555555;
    mulUnsigned<uint64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, mulSignedWide)
{
    int64_t a = -0x8000000000000000;
    int64_t b = -0x7fffffffffffffff;
    int64_t hi;
    int64_t low;
    mulSigned<int64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x3fffffffffffffff);
    EXPECT_EQ(low, -0x8000000000000000);

    hi = 0;
    low = 0;
    mulSignedManual<int64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0x3fffffffffffffff);
    EXPECT_EQ(low, -0x8000000000000000);

    hi = 0;
    low = 0;
    std::tie(hi, low) = mulSigned<int64_t>(a, b);
    EXPECT_EQ(hi, 0x3fffffffffffffff);
    EXPECT_EQ(low, -0x8000000000000000);

    a = 0;
    b = -0x5555555555555555;
    mulSigned<int64_t>(hi, low, a, b);
    EXPECT_EQ(hi, 0);
    EXPECT_EQ(low, 0);
}

TEST(IntmathTest, roundUp)
{
    EXPECT_EQ(4104, roundUp(4101, 4));
    EXPECT_EQ(4112, roundUp(4105, 8));
    EXPECT_EQ(4112, roundUp(4101, 16));
    EXPECT_EQ(8192, roundUp(7991, 256));
}

TEST(IntmathTest, roundDown)
{
    EXPECT_EQ(4100, roundDown(4101, 4));
    EXPECT_EQ(4104, roundDown(4105, 8));
    EXPECT_EQ(4096, roundDown(4101, 16));
    EXPECT_EQ(7936, roundDown(7991, 256));
}

/** This is testing if log2i actually works.
 * at every iteration value is multiplied by 2 (left shift) and expected
 * is incremented by one. This until value reaches becomes negative (by
 * left shifting) which is when expected points to the MSB
 */
TEST(IntmathTest, Log2i)
{
    int expected = 0;
    for (int value = 1; value > 0; expected++, value <<= 1) {
        EXPECT_EQ(expected, log2i(value));
    }

    // Just as a sanity check for expected to point to the MSB
    EXPECT_EQ(expected, sizeof(int) * 8 - 1);
}

/** This is testing the assertions: what if invalid arguments are
 * provided to log2i:
 *
 * 1) value = 0
 * 2) value < 0
 * 3) value is not a power of 2
 */
TEST(IntmathDeathTest, Log2iDeath)
{

#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif

    // 1) value = 0
    EXPECT_DEATH({
        const int value = 0;
        log2i(value);
    }, "value > 0.*failed");

    // 2) value < 0
    EXPECT_DEATH({
        const int value = -1;
        log2i(value);
    }, "value > 0.*failed");

    // 3) value is not a power of 2
    EXPECT_DEATH({
        const int value = 5;
        log2i(value);
    }, "isPowerOf2");
}
