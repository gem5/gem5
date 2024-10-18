/*
 * Copyright (c) 2024 CNRS
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

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <unordered_map>

#include "base/gtest/logging.hh"
#include "base/random.hh"

using namespace gem5;

/**
 * Checking that default construction uses the default
 * seed as specified by the standard
 */
TEST(RandomCtor, UInt64DefaultConstruct)
{
    // Init seed was default seed 5489
    Random dut;
    // First uint64_t corresponds to the default seed
    ASSERT_EQ(dut.random<uint64_t>(), 14514284786278117030llu);
}

TEST(RandomCtor, DoubleDefaultConstruct)
{
    // Init seed was default seed 5489
    Random dut;
    // First double corresponds to the default seed
    // 14514284786278117030llu / std::numeric_limits<uint64_t>::max()
    ASSERT_EQ(dut.random<double>(),
      0.7868209548678020137657540544751100242137908935546875d);
}

TEST(RandomCtor, FloatDefaultConstruct)
{
    // Init seed was default seed 5489
    Random dut;
    // First float corresponds to the default seed
    // 14514284786278117030llu / std::numeric_limits<uint64_t>::max()
    ASSERT_EQ(dut.random<float>(), 0.786820948123931884765625f);
}


/**
 * Checking that default construction uses the default
 * see as specified by the standard
 */
TEST(RandomCtor, ConstructUserSpecifiedSeed)
{
    Random dut{42};
    ASSERT_EQ(dut.random<uint64_t>(), 13930160852258120406llu);
}

/**
 * Test that reseeding after construction with
 * an explicit seed works
 */
TEST(RandomCtor, ConstructThenReseed)
{
    Random dut{};
    ASSERT_EQ(dut.random<uint64_t>(), 14514284786278117030llu);
    dut.init(42);
    ASSERT_EQ(dut.random<uint64_t>(), 13930160852258120406llu);
}

/**
 * This tests check that range generation
 * works if min equals max
 */
TEST(RandomRange, MinEqualsMax)
{
    Random dut;

    for (int i = 0; i < 10; i++) {
        ASSERT_EQ(dut.random<int>(0, 0), 0);
        ASSERT_EQ(dut.random<uint64_t>(1, 1), 1);
        ASSERT_EQ(dut.random<int>(-1, -1), -1);
    }
}

constexpr int loopCount = 30000;

/**
 * Helper function for subsequent range tests,
 * returns true if a random number was produced
 * 32% <= freq <= 34%, otherwise false.
 * Assumes we are testing ranges that span 3
 * elements.
 */
bool withinFreqRange(int count)
{
  double freq = ((double) count / loopCount);
  return freq >= 0.32 && freq <= 0.34;
}

/**
 * This tests check that range generation
 * does generate all possible values within
 * a range
 */
TEST(RandomRange, Coverage)
{
    Random dut;

    // Count occurences if we want to check
    // frequencies in the future
    std::unordered_map<int, int> values;

    for (int i = 0; i < loopCount; i++) {
        values[dut.random<int>(4,6)]++;
    }

    ASSERT_EQ(values.count(4), 1);
    ASSERT_EQ(values.count(5), 1);
    ASSERT_EQ(values.count(6), 1);
    ASSERT_EQ(values.size(), 3);
    ASSERT_EQ(withinFreqRange(values[4]), true);
    ASSERT_EQ(withinFreqRange(values[5]), true);
    ASSERT_EQ(withinFreqRange(values[6]), true);
    values.clear();

    for (int i = 0; i < loopCount; i++) {
        values[dut.random<int>(-1,1)]++;
    }

    ASSERT_EQ(values.count(-1), 1);
    ASSERT_EQ(values.count(0), 1);
    ASSERT_EQ(values.count(1), 1);
    ASSERT_EQ(values.size(), 3);
    ASSERT_EQ(withinFreqRange(values[-1]), true);
    ASSERT_EQ(withinFreqRange(values[0]), true);
    ASSERT_EQ(withinFreqRange(values[1]), true);

    values.clear();

    for (int i = 0; i < loopCount; i++) {
        values[dut.random<int>(-6,-4)]++;
    }

    ASSERT_EQ(values.count(-6), 1);
    ASSERT_EQ(values.count(-5), 1);
    ASSERT_EQ(values.count(-4), 1);
    ASSERT_EQ(values.size(), 3);
    ASSERT_EQ(withinFreqRange(values[-6]), true);
    ASSERT_EQ(withinFreqRange(values[-5]), true);
    ASSERT_EQ(withinFreqRange(values[-4]), true);
}

/** Test that the range provided for random
 *  number generation is valid
 */
TEST(RandomDeathTest, InvalidRange)
{
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif
    Random dut;
    ASSERT_DEATH(dut.random<int>(4, 2), "");
}
