/*
 * Copyright (c) 2026 Barcelona Supercomputing Center
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "base/gtest/logging.hh"
#include "cpu/o3/lsq_occupancy.hh"
#include "cpu/o3/lsq_segments.hh"
#include "cpu/op_class.hh"
#include "params/ClampOccupancy.hh"
#include "params/ConstantOccupancy.hh"
#include "params/LineCrossOccupancy.hh"
#include "params/MaxOccupancy.hh"
#include "params/PerBlockOccupancy.hh"
#include "params/PerCacheLineOccupancy.hh"
#include "params/PerChunkOccupancy.hh"
#include "params/PerSegmentOccupancy.hh"
#include "params/SumOccupancy.hh"

using namespace gem5;
using namespace gem5::o3;

namespace
{

/** Returns params of a policy, without its own parameters set. */
template <class Params>
Params
policyParams(const char *name)
{
    Params params;
    params.name = name;
    params.eventq_index = 0;
    return params;
}

/** Returns a segment whose bytes are all enabled. */
MemSegment
segment(Addr vaddr, unsigned size)
{
    return {vaddr, size, size};
}

/**
 * Fixture holding the segments of an access and some policies to combine.
 * Params are kept alive as long as the objects built from them.
 */
class LSQOccupancyTest : public ::testing::Test
{
  protected:
    std::vector<MemSegment> segments;

    ConstantOccupancyParams twoParams =
        policyParams<ConstantOccupancyParams>("two");
    ConstantOccupancyParams fiveParams =
        policyParams<ConstantOccupancyParams>("five");
    PerCacheLineOccupancyParams lineParams =
        policyParams<PerCacheLineOccupancyParams>("line");
    std::unique_ptr<ConstantOccupancy> two;
    std::unique_ptr<ConstantOccupancy> five;
    std::unique_ptr<PerCacheLineOccupancy> line;

    LSQOccupancyTest()
    {
        twoParams.entries = 2;
        two = std::make_unique<ConstantOccupancy>(twoParams);
        fiveParams.entries = 5;
        five = std::make_unique<ConstantOccupancy>(fiveParams);
        lineParams.coalesce = true;
        line = std::make_unique<PerCacheLineOccupancy>(lineParams);
    }

    /** Returns the access of the segments, with 64-byte cache lines. */
    MemAccessInfo
    info() const
    {
        return {
            .segments = segments,
            .lineSize = 64,
            .opClass = MemReadOp,
            .isLoad = true,
            .isAtomic = false,
            .flags = {},
        };
    }
};

} // anonymous namespace

TEST_F(LSQOccupancyTest, Constant)
{
    segments = {segment(0x0, 8), segment(0x100, 8)};
    EXPECT_EQ(two->occupiedEntries(info()), 2);
}

TEST_F(LSQOccupancyTest, PerSegment)
{
    auto params = policyParams<PerSegmentOccupancyParams>("segment");
    PerSegmentOccupancy policy(params);

    segments = {segment(0x0, 8)};
    EXPECT_EQ(policy.occupiedEntries(info()), 1);
    segments = {segment(0x0, 8), segment(0x0, 8), segment(0x100, 4)};
    EXPECT_EQ(policy.occupiedEntries(info()), 3);
}

TEST_F(LSQOccupancyTest, PerBlock)
{
    auto params = policyParams<PerBlockOccupancyParams>("block");
    params.blockSize = 16;
    params.coalesce = true;
    PerBlockOccupancy coalesced(params);
    params.coalesce = false;
    PerBlockOccupancy separate(params);

    // Ends exactly on a block boundary.
    segments = {segment(0x10, 16)};
    EXPECT_EQ(coalesced.occupiedEntries(info()), 1);
    // Blocks 0 and 0-1 overlap in block 0.
    segments = {segment(0x0, 16), segment(0x8, 16)};
    EXPECT_EQ(coalesced.occupiedEntries(info()), 2);
    EXPECT_EQ(separate.occupiedEntries(info()), 3);
}

TEST_F(LSQOccupancyTest, PerCacheLine)
{
    auto params = policyParams<PerCacheLineOccupancyParams>("line");
    params.coalesce = false;
    PerCacheLineOccupancy separate(params);

    // Uses the line size of the access.
    segments = {segment(0x3c, 8)};
    EXPECT_EQ(line->occupiedEntries(info()), 2);
    segments = {segment(0x0, 64), segment(0x20, 64)};
    EXPECT_EQ(line->occupiedEntries(info()), 2);
    EXPECT_EQ(separate.occupiedEntries(info()), 3);
}

TEST_F(LSQOccupancyTest, PerChunk)
{
    auto params = policyParams<PerChunkOccupancyParams>("chunk");
    params.chunkSize = 16;
    params.coalesce = true;
    PerChunkOccupancy coalesced(params);
    params.coalesce = false;
    PerChunkOccupancy separate(params);

    // 4 and 8 enabled bytes, spread over two lines.
    segments = {{0x0, 16, 4}, {0x7c, 8, 8}};
    EXPECT_EQ(coalesced.occupiedEntries(info()), 1);
    EXPECT_EQ(separate.occupiedEntries(info()), 2);
}

TEST_F(LSQOccupancyTest, LineCross)
{
    auto params = policyParams<LineCrossOccupancyParams>("cross");
    params.entries = 1;
    params.extraEntries = 2;
    LineCrossOccupancy policy(params);

    segments = {segment(0x0, 64)};
    EXPECT_EQ(policy.occupiedEntries(info()), 1);
    segments = {segment(0x3c, 8), segment(0x80, 8), segment(0xfc, 8)};
    EXPECT_EQ(policy.occupiedEntries(info()), 5);
}

TEST_F(LSQOccupancyTest, Max)
{
    auto params = policyParams<MaxOccupancyParams>("max");
    params.policies = {two.get(), line.get()};
    MaxOccupancy policy(params);

    segments = {segment(0x0, 8)};
    EXPECT_EQ(policy.occupiedEntries(info()), 2);
    segments = {segment(0x0, 192)};
    EXPECT_EQ(policy.occupiedEntries(info()), 3);
}

TEST_F(LSQOccupancyTest, Sum)
{
    auto params = policyParams<SumOccupancyParams>("sum");
    params.policies = {two.get(), line.get(), five.get()};
    SumOccupancy policy(params);

    segments = {segment(0x3c, 8)};
    EXPECT_EQ(policy.occupiedEntries(info()), 9);
}

TEST_F(LSQOccupancyTest, Clamp)
{
    auto params = policyParams<ClampOccupancyParams>("clamp");
    params.policy = line.get();
    params.minEntries = 2;
    params.maxEntries = 3;
    ClampOccupancy policy(params);

    // 1, 2, 3 and 4 cache lines.
    segments = {segment(0x0, 64)};
    EXPECT_EQ(policy.occupiedEntries(info()), 2);
    segments = {segment(0x0, 128)};
    EXPECT_EQ(policy.occupiedEntries(info()), 2);
    segments = {segment(0x0, 192)};
    EXPECT_EQ(policy.occupiedEntries(info()), 3);
    segments = {segment(0x0, 256)};
    EXPECT_EQ(policy.occupiedEntries(info()), 3);
}

TEST_F(LSQOccupancyTest, ClampToSingleValue)
{
    auto params = policyParams<ClampOccupancyParams>("clamp");
    params.policy = five.get();
    params.minEntries = 4;
    params.maxEntries = 4;
    ClampOccupancy policy(params);

    EXPECT_EQ(policy.occupiedEntries(info()), 4);
}

TEST_F(LSQOccupancyTest, Composition)
{
    // max(2, clamp(sum(line, 5), 0, 6)).
    auto sum_params = policyParams<SumOccupancyParams>("sum");
    sum_params.policies = {line.get(), five.get()};
    SumOccupancy sum(sum_params);
    auto clamp_params = policyParams<ClampOccupancyParams>("clamp");
    clamp_params.policy = &sum;
    clamp_params.minEntries = 0;
    clamp_params.maxEntries = 6;
    ClampOccupancy clamp(clamp_params);
    auto max_params = policyParams<MaxOccupancyParams>("max");
    max_params.policies = {two.get(), &clamp};
    MaxOccupancy policy(max_params);

    // The sum is 2 + 5, clamped to 6.
    segments = {segment(0x0, 128)};
    EXPECT_EQ(policy.occupiedEntries(info()), 6);
}

TEST(LSQOccupancyConfigTest, PerBlockZeroBlockSize)
{
    auto params = policyParams<PerBlockOccupancyParams>("block");
    params.blockSize = 0;
    params.coalesce = true;
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<PerBlockOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("blockSize must not be zero"));
}

TEST(LSQOccupancyConfigTest, PerChunkZeroChunkSize)
{
    auto params = policyParams<PerChunkOccupancyParams>("chunk");
    params.chunkSize = 0;
    params.coalesce = true;
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<PerChunkOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("chunkSize must not be zero"));
}

TEST(LSQOccupancyConfigTest, EmptyMax)
{
    auto params = policyParams<MaxOccupancyParams>("max");
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<MaxOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("policies must not be empty"));
}

TEST(LSQOccupancyConfigTest, EmptySum)
{
    auto params = policyParams<SumOccupancyParams>("sum");
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<SumOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("policies must not be empty"));
}

TEST(LSQOccupancyConfigTest, ClampWithoutPolicy)
{
    auto params = policyParams<ClampOccupancyParams>("clamp");
    params.policy = nullptr;
    params.minEntries = 0;
    params.maxEntries = 1;
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<ClampOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("policy must be set"));
}

TEST(LSQOccupancyConfigTest, ClampMinAboveMax)
{
    auto policy_params = policyParams<ConstantOccupancyParams>("one");
    policy_params.entries = 1;
    ConstantOccupancy one(policy_params);

    auto params = policyParams<ClampOccupancyParams>("clamp");
    params.policy = &one;
    params.minEntries = 3;
    params.maxEntries = 2;
    gtestLogOutput.str("");
    EXPECT_THROW(std::make_unique<ClampOccupancy>(params), GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("minEntries (3) must not exceed "
                                     "maxEntries (2)"));
}
