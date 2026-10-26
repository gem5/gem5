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

#include <map>
#include <vector>

#include "salam/HWModeling/fu_pool.hh"

using gem5::MaxTick;
using gem5::Tick;
using SALAM::FUPool;

namespace
{

constexpr uint32_t Unit = 1;

FUPool
makePool(uint32_t instances, Tick interval)
{
    FUPool pool;
    pool.configure({{Unit, {instances, interval}}});
    return pool;
}

// Issue n operations of the given latency to Unit as early as the pool
// allows. On each tick, due completions are released before the
// admissions they enable. Returns the issue tick of each operation.
std::vector<Tick>
schedule(FUPool &pool, unsigned n, Tick latency)
{
    std::vector<Tick> issued;
    std::multimap<Tick, FUPool::Token> pending;
    for (Tick now = 0; issued.size() < n; now++) {
        EXPECT_LT(now, 1000u) << "no progress";
        if (now >= 1000) {
            break;
        }
        auto due = pending.equal_range(now);
        for (auto it = due.first; it != due.second; ++it) {
            pool.release(it->second);
        }
        pending.erase(due.first, due.second);
        while (issued.size() < n && pool.canAccept(Unit, now)) {
            FUPool::Token token = pool.acquire(Unit, now);
            issued.push_back(now);
            if (latency == 0) {
                pool.release(token);
            } else {
                pending.emplace(now + latency, token);
            }
        }
    }
    for (auto &entry : pending) {
        pool.release(entry.second);
    }
    return issued;
}

} // anonymous namespace

TEST(SalamFUPool, OneUnpipelinedInstanceWaitsForCompletion)
{
    FUPool pool = makePool(1, 0);
    EXPECT_EQ(schedule(pool, 3, 1), (std::vector<Tick>{0, 1, 2}));
    EXPECT_EQ(pool.inFlight(), 0u);
}

TEST(SalamFUPool, TwoInstancesAdmitTwoAtOnce)
{
    FUPool pool = makePool(2, 0);
    EXPECT_EQ(schedule(pool, 3, 1), (std::vector<Tick>{0, 0, 1}));
}

TEST(SalamFUPool, IntervalEqualToLatency)
{
    FUPool pool = makePool(1, 5);
    EXPECT_EQ(schedule(pool, 3, 5), (std::vector<Tick>{0, 5, 10}));
}

TEST(SalamFUPool, IntervalShorterThanLatencyOverlaps)
{
    FUPool pool = makePool(1, 1);
    EXPECT_EQ(schedule(pool, 3, 5), (std::vector<Tick>{0, 1, 2}));
}

TEST(SalamFUPool, IntervalLongerThanLatencyWaitsWithNothingInFlight)
{
    FUPool pool = makePool(1, 5);
    FUPool::Token token = pool.acquire(Unit, 0);
    pool.release(token); // completes at tick 1
    EXPECT_EQ(pool.inFlight(), 0u);
    EXPECT_FALSE(pool.canAccept(Unit, 2));
    // Waiting is not a stall: the next admission is at a known tick.
    EXPECT_EQ(pool.nextAdmission(Unit, 2), 5u);
    EXPECT_TRUE(pool.canAccept(Unit, 5));

    FUPool fresh = makePool(1, 5);
    EXPECT_EQ(schedule(fresh, 3, 1), (std::vector<Tick>{0, 5, 10}));
}

TEST(SalamFUPool, ZeroLatencyIsAdmittedAndReleasedOnce)
{
    FUPool pool = makePool(1, 0);
    EXPECT_EQ(schedule(pool, 3, 0), (std::vector<Tick>{0, 0, 0}));
    EXPECT_EQ(pool.inFlight(), 0u);
}

TEST(SalamFUPool, UnpipelinedBusyUnitNeedsARelease)
{
    FUPool pool = makePool(1, 0);
    FUPool::Token token = pool.acquire(Unit, 0);
    EXPECT_EQ(pool.nextAdmission(Unit, 3), MaxTick);
    pool.release(token);
    EXPECT_EQ(pool.nextAdmission(Unit, 3), 3u);
}

TEST(SalamFUPool, PipelinedOperationsShareABusyInstance)
{
    FUPool pool = makePool(1, 1);
    FUPool::Token a = pool.acquire(Unit, 0);
    FUPool::Token b = pool.acquire(Unit, 1);
    EXPECT_EQ(pool.inFlight(Unit), 2u);
    EXPECT_EQ(pool.busyInstances(Unit), 1u);
    pool.release(a);
    EXPECT_EQ(pool.busyInstances(Unit), 1u);
    pool.release(b);
    EXPECT_EQ(pool.busyInstances(Unit), 0u);
}

TEST(SalamFUPool, DoubleReleasePanics)
{
    FUPool pool = makePool(1, 0);
    FUPool::Token token = pool.acquire(Unit, 0);
    pool.release(token);
    ASSERT_ANY_THROW(pool.release(token));
    ASSERT_ANY_THROW(pool.release(FUPool::Token{Unit, 0, 999}));
}

TEST(SalamFUPool, ZeroInstancesAreNotUnlimited)
{
    FUPool pool = makePool(0, 0);
    EXPECT_EQ(pool.instances(Unit), 0u);
    EXPECT_FALSE(pool.canAccept(Unit, 0));
    EXPECT_EQ(pool.nextAdmission(Unit, 0), MaxTick);
    ASSERT_ANY_THROW(pool.acquire(Unit, 0));
}

TEST(SalamFUPool, UnknownUnitPanics)
{
    FUPool pool = makePool(1, 0);
    EXPECT_FALSE(pool.hasUnit(7));
    ASSERT_ANY_THROW(pool.canAccept(7, 0));
    ASSERT_ANY_THROW(pool.acquire(7, 0));
}

TEST(SalamFUPool, Reconfiguration)
{
    FUPool pool = makePool(1, 5);
    FUPool::Token token = pool.acquire(Unit, 0);
    ASSERT_ANY_THROW(pool.configure({{Unit, {2, 0}}}));
    pool.release(token);
    pool.configure({{Unit, {2, 0}}});
    EXPECT_EQ(pool.instances(Unit), 2u);
    // The new configuration starts without the old interval's wait.
    EXPECT_EQ(schedule(pool, 3, 1), (std::vector<Tick>{0, 0, 1}));
}
