/*
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

#include "base/intmath.hh"

TEST(IntmathTest, isPowerOf2)
{
    EXPECT_TRUE(isPowerOf2(1));
    EXPECT_TRUE(isPowerOf2(65536));
    EXPECT_TRUE(isPowerOf2(131072));
    EXPECT_TRUE(isPowerOf2(262144));
    EXPECT_FALSE(isPowerOf2(0));
    EXPECT_FALSE(isPowerOf2(2521));
    EXPECT_FALSE(isPowerOf2(1679616));
}

TEST(IntmathTest, power)
{
    EXPECT_EQ(65536, power(2, 16));
    EXPECT_EQ(9765625, power(5, 10));
    EXPECT_EQ(43046721, power(power(3, 4), 4));
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
