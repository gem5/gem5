/*
 * Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
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

#include "salam/HWModeling/salam_power_model.hh"

using SALAM::PPA::Ledger;
using SALAM::PPA::Quantity;

namespace
{
constexpr double TicksPerSecond = 1e12;
// Absolute joule tolerance: far below the smallest (1 pJ) test event.
constexpr double EnergyTolerance = 1e-24;
constexpr double PowerTolerance = 1e-18;

Ledger
example()
{
    Ledger ledger;
    ledger.physical["adder"] = {2, {10, true, false}, {2e-6, true, false}};
    ledger.reset(0);
    ledger.begin(0);
    return ledger;
}
} // anonymous namespace

TEST(SalamPowerModel, IndependentlyCalculatedAreaEnergyAndPower)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 3);
    ledger.finish(5000000);
    auto result = ledger.result(ledger.invocation, 9000000, TicksPerSecond);
    EXPECT_DOUBLE_EQ(result.area, 20);
    EXPECT_NEAR(result.dynamic, 15e-12, EnergyTolerance);
    EXPECT_NEAR(result.leakage, 20e-12, EnergyTolerance);
    EXPECT_NEAR(result.total(), 35e-12, EnergyTolerance);
    EXPECT_NEAR(result.power(), 7e-6, PowerTolerance);
    EXPECT_TRUE(result.areaComplete);
    EXPECT_TRUE(result.dynamicComplete);
    EXPECT_TRUE(result.leakageComplete);
}

TEST(SalamPowerModel, StallsIncreaseOnlyLeakageEnergy)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 3);
    auto shortRun = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    auto stalled = ledger.result(ledger.invocation, 10000000, TicksPerSecond);
    EXPECT_DOUBLE_EQ(shortRun.area, stalled.area);
    EXPECT_DOUBLE_EQ(shortRun.dynamic, stalled.dynamic);
    EXPECT_NEAR(stalled.leakage, 40e-12, EnergyTolerance);
    EXPECT_NEAR(stalled.power(), 5.5e-6, PowerTolerance);
}

TEST(SalamPowerModel, ZeroDurationPreservesFiniteEnergyWithoutPower)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 1);
    ledger.finish(0);
    auto result = ledger.result(ledger.invocation, 5000, TicksPerSecond);
    EXPECT_FALSE(result.powerAvailable());
    EXPECT_TRUE(std::isnan(result.power()));
    EXPECT_DOUBLE_EQ(result.dynamic, 5e-12);
    EXPECT_DOUBLE_EQ(result.leakage, 0);
    EXPECT_DOUBLE_EQ(result.area, 20);
}

TEST(SalamPowerModel, ResetDuringActivityPreservesInvocationAndInventory)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 1);
    ledger.observe(&Ledger::Scope::operandReadBits, 32);
    ledger.reset(2000000);
    ledger.event({5e-12, true, false}, 2);
    ledger.observe(&Ledger::Scope::operandReadBits, 16);
    ledger.finish(5000000);
    auto invocation =
        ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    auto window = ledger.result(ledger.window, 5000000, TicksPerSecond);
    EXPECT_NEAR(invocation.total(), 35e-12, EnergyTolerance);
    EXPECT_NEAR(window.dynamic, 10e-12, EnergyTolerance);
    EXPECT_NEAR(window.leakage, 12e-12, EnergyTolerance);
    EXPECT_DOUBLE_EQ(window.area, 20);
    EXPECT_EQ(ledger.invocation.operandReadBits, 48);
    EXPECT_EQ(ledger.window.operandReadBits, 16);
}

TEST(SalamPowerModel, RepeatedDumpsAndFinalizationAreIdempotent)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 3);
    ledger.finish(5000000);
    auto first = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    for (int i = 0; i < 5; ++i) {
        ledger.finish(9000000);
        auto next = ledger.result(ledger.invocation, 9000000, TicksPerSecond);
        EXPECT_DOUBLE_EQ(first.total(), next.total());
        EXPECT_DOUBLE_EQ(first.duration, next.duration);
        EXPECT_DOUBLE_EQ(first.power(), next.power());
    }
}

TEST(SalamPowerModel, RepeatedInvocationsDoNotGrowHardware)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 3);
    ledger.finish(5000000);
    ledger.begin(10000000);
    ledger.event({5e-12, true, false}, 2);
    ledger.finish(15000000);
    auto invocation =
        ledger.result(ledger.invocation, 15000000, TicksPerSecond);
    auto window = ledger.result(ledger.window, 15000000, TicksPerSecond);
    EXPECT_DOUBLE_EQ(invocation.area, 20);
    EXPECT_NEAR(invocation.dynamic, 10e-12, EnergyTolerance);
    EXPECT_NEAR(window.dynamic, 25e-12, EnergyTolerance);
    // Leakage covers the common device window, including the idle gap.
    EXPECT_NEAR(window.leakage, 60e-12, EnergyTolerance);
}

TEST(SalamPowerModel, MissingZeroAndProxyRemainDistinct)
{
    auto ledger = example();
    ledger.event({0, true, false}, 4);
    auto zero = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_TRUE(zero.dynamicComplete);
    EXPECT_FALSE(zero.approximate);
    ledger.event({0, false, false}, 2);
    auto missing = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_FALSE(missing.dynamicComplete);
    EXPECT_EQ(ledger.invocation.missingEvents, 2);
    ledger.event({5e-12, true, true}, 1);
    auto proxy = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_TRUE(proxy.approximate);
    EXPECT_FALSE(proxy.dynamicComplete);
    EXPECT_DOUBLE_EQ(proxy.dynamic, 5e-12);
}

TEST(SalamPowerModel, UnusedAutoUnitHasNoPhysicalContribution)
{
    auto ledger = example();
    ledger.physical["unused"] = {0, {}, {}};
    auto result = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_DOUBLE_EQ(result.area, 20);
    EXPECT_TRUE(result.areaComplete);
    EXPECT_TRUE(result.leakageComplete);
}

TEST(SalamPowerModel, IdlePositiveLimitStillHasAreaAndLeakage)
{
    auto ledger = example();
    ledger.physical["idle"] = {3, {7, true, false}, {1e-6, true, false}};
    auto result = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_DOUBLE_EQ(result.area, 41);
    EXPECT_NEAR(result.leakage, 35e-12, EnergyTolerance);
    EXPECT_DOUBLE_EQ(result.dynamic, 0);
}

TEST(SalamPowerModel, IndependentCoverageForAreaEnergyAndLeakage)
{
    auto ledger = example();
    ledger.physical["partial"] = {1, {7, true, false}, {}};
    ledger.activityCoverage = false;
    auto result = ledger.result(ledger.invocation, 5000000, TicksPerSecond);
    EXPECT_TRUE(result.areaComplete);
    EXPECT_FALSE(result.leakageComplete);
    EXPECT_FALSE(result.dynamicComplete);
    EXPECT_DOUBLE_EQ(result.area, 27);
    EXPECT_NEAR(result.leakage, 20e-12, EnergyTolerance);
}

TEST(SalamPowerModel, MemoryWordsAreCountedWithoutByteRoundingLoss)
{
    Ledger ledger;
    ledger.reset(0);
    // A 3-byte access beginning at byte 3 touches two 4-byte words.
    ledger.observe(&Ledger::Scope::readWords, 2);
    ledger.event({7e-12, true, false}, 2);
    // A masked write touches one word and requires one read-modify-write.
    ledger.observe(&Ledger::Scope::readWords, 1);
    ledger.observe(&Ledger::Scope::writeWords, 1);
    ledger.event({7e-12, true, false}, 1);
    ledger.event({11e-12, true, false}, 1);
    auto result = ledger.result(ledger.window, 1000, TicksPerSecond);
    EXPECT_EQ(ledger.window.readWords, 3);
    EXPECT_EQ(ledger.window.writeWords, 1);
    EXPECT_NEAR(result.dynamic, 32e-12, EnergyTolerance);
}

TEST(SalamPowerModel, DeviceActivityOutsideInvocationIsKeptSeparate)
{
    auto ledger = example();
    ledger.event({5e-12, true, false}, 1);
    ledger.finish(1000);
    ledger.event({9e-12, true, false}, 1);
    auto invocation = ledger.result(ledger.invocation, 2000, TicksPerSecond);
    auto window = ledger.result(ledger.window, 2000, TicksPerSecond);
    EXPECT_NEAR(invocation.dynamic, 5e-12, EnergyTolerance);
    EXPECT_NEAR(window.dynamic, 14e-12, EnergyTolerance);
    EXPECT_DOUBLE_EQ(invocation.duration, 1e-9);
    EXPECT_DOUBLE_EQ(window.duration, 2e-9);
}

TEST(SalamPowerModel, UnknownStaticStoragePreservesKnownSubtotals)
{
    Ledger ledger;
    ledger.physical["known"] = {2, {10, true, false}, {2e-6, true, false}};
    ledger.staticCoverage = false;
    ledger.reset(0);
    ledger.begin(0);
    ledger.event({3e-12, true, false}, 2);
    auto result = ledger.result(ledger.window, 5, 1e6);
    EXPECT_FALSE(result.areaComplete);
    EXPECT_FALSE(result.leakageComplete);
    EXPECT_TRUE(result.dynamicComplete);
    EXPECT_DOUBLE_EQ(result.area, 20);
    EXPECT_NEAR(result.dynamic, 6e-12, EnergyTolerance);
    EXPECT_NEAR(result.leakage, 20e-12, EnergyTolerance);
    ledger.reset(5);
    result = ledger.result(ledger.window, 10, 1e6);
    EXPECT_FALSE(result.areaComplete);
    EXPECT_FALSE(result.leakageComplete);
    EXPECT_DOUBLE_EQ(result.area, 20);
    EXPECT_DOUBLE_EQ(result.dynamic, 0);
    EXPECT_NEAR(result.leakage, 20e-12, EnergyTolerance);
}

TEST(SalamPowerModel, UnknownTimingDoesNotBecomeZeroTiming)
{
    SALAM::PPA::TimingConstraints timing;
    const auto result = timing.check(0, 0, 1e-12);
    EXPECT_FALSE(result.available);
    EXPECT_FALSE(result.compatible());
}

TEST(SalamPowerModel, NonpipelinedTimingUsesActualOperationLatency)
{
    SALAM::PPA::TimingConstraints timing{true, 2e-9, 2e-9, false};
    EXPECT_TRUE(timing.check(2e-9, 0, 1e-12).compatible());
    const auto short_latency = timing.check(1e-9, 0, 1e-12);
    EXPECT_FALSE(short_latency.latency);
    EXPECT_FALSE(short_latency.interval);
    EXPECT_TRUE(short_latency.pipeline);
    EXPECT_FALSE(timing.check(2e-9, 2e-9, 1e-12).pipeline);
}

TEST(SalamPowerModel, PipelinedTimingChecksLatencyAndInitiationSeparately)
{
    SALAM::PPA::TimingConstraints timing{true, 3e-9, 1e-9, true};
    EXPECT_TRUE(timing.check(3e-9, 1e-9, 1e-12).compatible());
    EXPECT_FALSE(timing.check(2e-9, 1e-9, 1e-12).latency);
    EXPECT_FALSE(timing.check(3e-9, 0.5e-9, 1e-12).interval);
    EXPECT_FALSE(timing.check(3e-9, 0, 1e-12).pipeline);
}

TEST(SalamPowerModel, ZeroLatencyUsesTheComputePeriodForAdmission)
{
    SALAM::PPA::TimingConstraints timing{true, 0, 2e-9, false};
    EXPECT_TRUE(timing.check(0, 0, 2e-9).compatible());
    EXPECT_FALSE(timing.check(0, 0, 1e-9).interval);
    // Admission pacing does not invent a nonzero completion latency.
    timing.minimumLatency = 1e-9;
    const auto result = timing.check(0, 0, 2e-9);
    EXPECT_TRUE(result.interval);
    EXPECT_FALSE(result.latency);
    EXPECT_TRUE(result.pipeline);
}

TEST(SalamPowerModel, ComputePeriodDoesNotChangePipelineOrganization)
{
    SALAM::PPA::TimingConstraints timing{true, 0, 2e-9, true};
    EXPECT_TRUE(timing.check(0, 1e-9, 2e-9).compatible());
    EXPECT_FALSE(timing.check(0, 0, 2e-9).pipeline);
}

TEST(SalamPowerModel, UnknownStorageAccessHasNoInventedWords)
{
    Ledger ledger;
    ledger.reset(0);
    ledger.begin(0);
    ledger.unknownStorageAccess("memory:unknown_word_width:write");
    EXPECT_EQ(ledger.window.missingEvents, 1);
    EXPECT_EQ(ledger.window.readWords, 0);
    EXPECT_EQ(ledger.window.writeWords, 0);
    EXPECT_FALSE(ledger.window.wordCountsAvailable);
    EXPECT_FALSE(
        ledger.result(ledger.window, 10, TicksPerSecond).dynamicComplete);
    EXPECT_DOUBLE_EQ(ledger.window.dynamic, 0);
}

TEST(SalamPowerModel, MissingAccessReasonsFollowWindowAndInvocation)
{
    Ledger ledger;
    ledger.reset(0);
    ledger.begin(0);
    ledger.unknownStorageAccess("memory:unsupported_partial:write");
    ledger.reset(10);
    EXPECT_TRUE(ledger.window.wordCountsAvailable);
    EXPECT_EQ(ledger.window.missingEvents, 0);
    EXPECT_TRUE(ledger.window.missingEventKeys.empty());
    EXPECT_TRUE(
        ledger.result(ledger.window, 10, TicksPerSecond).dynamicComplete);
    EXPECT_FALSE(ledger.invocation.wordCountsAvailable);
    EXPECT_EQ(ledger.invocation.missingEvents, 1);
    EXPECT_EQ(ledger.invocation.missingEventKeys.count(
                  "memory:unsupported_partial:write"),
              1);
    ledger.event({2e-12, true, false}, 1);
    EXPECT_EQ(ledger.window.missingEvents, 0);
    ledger.finish(20);
    ledger.begin(30);
    EXPECT_TRUE(ledger.invocation.wordCountsAvailable);
    EXPECT_TRUE(ledger.invocation.missingEventKeys.empty());
}
