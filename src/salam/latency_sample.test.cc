/*
 * Copyright (c) 2026 Akanksha Chaudhari
 * All rights reserved.
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

#include "base/random.hh"
#include "salam/latency_sample.hh"

using gem5::Random;
using gem5::Tick;

TEST(SalamLatencySample, OneDrawPerSample)
{
    Random::RandomPtr rng = Random::genRandom(1234);
    Random::RandomPtr reference = Random::genRandom(1234);
    for (int i = 0; i < 100; i++) {
        Tick expected = 2000 + reference->random<Tick>(0, 5000);
        EXPECT_EQ(SALAM::sampleLatency(*rng, 2000, 5000), expected) << i;
    }
}

TEST(SalamLatencySample, ZeroVariationDrawsNothing)
{
    Random::RandomPtr rng = Random::genRandom(99);
    Random::RandomPtr reference = Random::genRandom(99);
    for (int i = 0; i < 10; i++) {
        EXPECT_EQ(SALAM::sampleLatency(*rng, 2000, 0), 2000u);
    }
    EXPECT_EQ(SALAM::sampleLatency(*rng, 0, 5000),
              reference->random<Tick>(0, 5000));
}

TEST(SalamLatencySample, InclusiveBounds)
{
    Random::RandomPtr rng = Random::genRandom(7);
    bool low = false;
    bool high = false;
    for (int i = 0; i < 2000; i++) {
        Tick latency = SALAM::sampleLatency(*rng, 100, 3);
        EXPECT_GE(latency, 100u);
        EXPECT_LE(latency, 103u);
        low = low || latency == 100;
        high = high || latency == 103;
    }
    EXPECT_TRUE(low);
    EXPECT_TRUE(high);
}

TEST(SalamLatencySample, SeparateGeneratorsDoNotAdvanceEachOther)
{
    Random::RandomPtr a = Random::genRandom(5);
    Random::RandomPtr b = Random::genRandom(5);
    std::vector<Tick> first;
    for (int i = 0; i < 20; i++) {
        first.push_back(SALAM::sampleLatency(*a, 0, 1000));
    }
    // Drawing from a did not move b: b repeats a's sequence.
    for (int i = 0; i < 20; i++) {
        EXPECT_EQ(SALAM::sampleLatency(*b, 0, 1000), first[i]) << i;
    }
}

// The default-seeded generator a scratchpad starts with; the SALAM
// access_cycles_var tests (2 cycles plus up to 5) expect these
// successive extra cycles.
TEST(SalamLatencySample, DefaultSeedSequence)
{
    Random::RandomPtr rng = Random::genRandom();
    std::vector<Tick> extras;
    for (int i = 0; i < 4; i++) {
        extras.push_back(SALAM::sampleLatency(*rng, 2, 5) - 2);
    }
    EXPECT_EQ(extras, (std::vector<Tick>{4, 0, 2, 4}));
}
