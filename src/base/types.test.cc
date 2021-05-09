/*
 * Copyright (c) 2019 The Regents of the University of California
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

#include <sstream>

#include "base/types.hh"

using namespace gem5;

/*
 * The following test the Cycles class. Cycles is a wrapper for uint64_t.
 * It overloads most commonly used operators.
 */
TEST(CyclesTest, NoCycles)
{
    Cycles cycles;
    EXPECT_EQ(0, cycles);
}

TEST(CyclesTest, PrefixIncrement)
{
    Cycles cycles(0);
    EXPECT_EQ(1, ++cycles);
    EXPECT_EQ(2, ++cycles);
    EXPECT_EQ(2, cycles);
}


TEST(CyclesTest, PrefixDecrement)
{
    Cycles cycles(10);
    EXPECT_EQ(9, --cycles);
    EXPECT_EQ(8, --cycles);
    EXPECT_EQ(8, cycles);
}

TEST(CyclesTest, InPlaceAddition)
{
    Cycles cycles(12);
    Cycles to_add(5);
    cycles += to_add;
    EXPECT_EQ(17, cycles);
}

TEST(CyclesTest, GreaterThanLessThan)
{
    Cycles one_cycle(1);
    Cycles two_cycles(2);
    EXPECT_TRUE(two_cycles > one_cycle);
    EXPECT_TRUE(one_cycle < two_cycles);
}

TEST(CyclesTest, AddCycles)
{
    Cycles cycles_1(10);
    Cycles cycles_2(15);
    Cycles added = cycles_1 + cycles_2;
    EXPECT_EQ(25, added);
}

TEST(CyclesTest, SubtractCycles)
{
    Cycles cycles_1(25);
    Cycles cycles_2(1);
    Cycles subtracted = cycles_1 - cycles_2;
    EXPECT_EQ(24, subtracted);
}

TEST(CyclesTest, ShiftRight)
{
    Cycles cycles(1ULL << 40);
    Cycles cycles_shifted = cycles >> 5;
    EXPECT_EQ((1ULL << 35), cycles_shifted);
}

TEST(CyclesTest, ShiftLeft)
{
    Cycles cycles(1ULL << 40);
    Cycles cycles_shifted = cycles << 20;
    EXPECT_EQ((1ULL << 60), cycles_shifted);
}

TEST(CyclesTest, OutStream)
{
    Cycles cycles(56);
    std::ostringstream ss;
    ss << "The number of cycles is: " << cycles << std::endl;
    EXPECT_EQ("The number of cycles is: 56\n", ss.str());
}

/*
 * MicroPCRomBit is a constant. This simple test verifies it has not changed.
 * The following MicroPC tests rely heavily on this constant.
 */
TEST(MicroPCTest, CheckMicroPCRomBit)
{
    EXPECT_EQ((1 << 15), MicroPCRomBit);
}

TEST(MicroPCTest, RomMicroPCTest)
{
    EXPECT_EQ(MicroPCRomBit + (1 << 8), romMicroPC(MicroPCRomBit + (1 << 8)));
}

TEST(MicroPCTest, NormalMicroPCTest)
{
    EXPECT_EQ((1 << 8), normalMicroPC((1 << 8) + MicroPCRomBit));
}

TEST(MicroPCTest, IsRomMicroPCTest)
{
    EXPECT_TRUE(isRomMicroPC(MicroPCRomBit + (1 << 8)));
}

TEST(MicroPCTest, IsNotRomMicroPCTest)
{
    EXPECT_FALSE(isRomMicroPC((1 << 8)));
}

/*
 * Both the "floatToBits32" and "floatToBits64" functions use the standard
 * union approach to carry out type punning. These checks are simple regression
 * tests.
 */
TEST(TypesTest, FloatToBits32)
{
    EXPECT_EQ(0x3e828f5c, floatToBits32(0.255));
}

TEST(TypesTest, floatToBits64)
{
    EXPECT_EQ(0x3fd067dfe32a0664, floatToBits64(0.25634));
}

/*
 * "floatToBits(double val)" and "floatToBits(float val)" are simple overloads
 * for "floatToBits64" and "floatToBits32" respectively. Ergo, these tests
 * check this is the case.
 */
TEST(TypesTest, floatsToBitsDoubleInput)
{
    double val = 0.84023;
    EXPECT_EQ(floatToBits64(val), floatToBits(val));
}

TEST(TypesTest, floatsToBitsFloatInput)
{
    float val = 0.04567;
    EXPECT_EQ(floatToBits32(val), floatToBits(val));
}
