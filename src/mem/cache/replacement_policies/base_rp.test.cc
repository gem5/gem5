/**
 * Copyright (c) 2026 Hiruna Vishwamith
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

/**
 * @file
 * Tests for the profiling statistics gathered by the common replacement
 * policy interface (gem5::replacement_policy::Base). LRU is used as a
 * concrete policy, as the counters live in the base class and are therefore
 * shared by every policy.
 */

#include <gtest/gtest.h>

#include <memory>

#include "mem/cache/replacement_policies/lru_rp.hh"
#include "params/LRURP.hh"
#include "sim/cur_tick.hh"
#include "sim/eventq.hh"

using namespace gem5;

namespace
{

// LRU calls curTick(), so a current event queue must exist for the policy to
// be exercised. See the fixture below.
EventQueue eventQueue("BaseRPStatsTest Event Queue");

/**
 * LRU subclass that exposes the base class' protected profiling counters so
 * that the Non-Virtual Interface wrappers can be observed from the tests.
 */
class TestableLRU : public replacement_policy::LRU
{
  public:
    using LRU::LRU;

    double
    victimizations() const
    { return stats.victimizations.value(); }
    double
    hits() const
    { return stats.hits.value(); }
    double
    insertions() const
    { return stats.insertions.value(); }
    double
    invalidations() const
    { return stats.invalidations.value(); }
};

std::shared_ptr<TestableLRU>
makeReplacementPolicy()
{
    LRURPParams params;
    params.eventq_index = 0;
    return std::make_shared<TestableLRU>(params);
}

/// Fixture that makes an event queue current so the policy can call curTick().
class BaseRPStatsTest : public ::testing::Test
{
  protected:
    BaseRPStatsTest() { curEventQueue(&eventQueue); }
};

} // namespace

/// A freshly built policy must not have recorded any activity.
TEST_F(BaseRPStatsTest, CountersStartAtZero)
{
    const auto rp = makeReplacementPolicy();
    EXPECT_EQ(rp->victimizations(), 0);
    EXPECT_EQ(rp->hits(), 0);
    EXPECT_EQ(rp->insertions(), 0);
    EXPECT_EQ(rp->invalidations(), 0);
}

/// Each public interface call must increment exactly one counter, exactly
/// once, regardless of the concrete policy.
TEST_F(BaseRPStatsTest, CountersTrackInterfaceCalls)
{
    const auto rp = makeReplacementPolicy();
    const auto data = rp->instantiateEntry();

    rp->reset(data);
    rp->reset(data);
    rp->reset(data);
    EXPECT_EQ(rp->insertions(), 3);

    rp->touch(data);
    rp->touch(data);
    EXPECT_EQ(rp->hits(), 2);

    rp->invalidate(data);
    EXPECT_EQ(rp->invalidations(), 1);

    // The other counters must be unaffected by unrelated operations.
    EXPECT_EQ(rp->victimizations(), 0);
}

/// getVictim must bump the victimization counter once per call.
TEST_F(BaseRPStatsTest, VictimizationsAreCounted)
{
    const auto rp = makeReplacementPolicy();

    ReplaceableEntry entry;
    entry.replacementData = rp->instantiateEntry();
    ReplacementCandidates candidates;
    candidates.push_back(&entry);

    rp->getVictim(candidates);
    rp->getVictim(candidates);
    EXPECT_EQ(rp->victimizations(), 2);

    // Selecting a victim is not an insertion, hit or invalidation.
    EXPECT_EQ(rp->hits(), 0);
    EXPECT_EQ(rp->insertions(), 0);
    EXPECT_EQ(rp->invalidations(), 0);
}
