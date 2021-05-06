/*
 * Copyright (c) 2021 Daniel R. Carvalho
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

#include "base/stats/units.hh"

TEST(StatsUnitsTest, Cycle)
{
    Stats::units::Cycle *unit = Stats::units::Cycle::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Cycle::toString());
}

TEST(StatsUnitsTest, Tick)
{
    Stats::units::Tick *unit = Stats::units::Tick::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Tick::toString());
}

TEST(StatsUnitsTest, Second)
{
    Stats::units::Second *unit = Stats::units::Second::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Second::toString());
}

TEST(StatsUnitsTest, Bit)
{
    Stats::units::Bit *unit = Stats::units::Bit::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Bit::toString());
}

TEST(StatsUnitsTest, Byte)
{
    Stats::units::Byte *unit = Stats::units::Byte::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Byte::toString());
}

TEST(StatsUnitsTest, Watt)
{
    Stats::units::Watt *unit = Stats::units::Watt::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Watt::toString());
}

TEST(StatsUnitsTest, Joule)
{
    Stats::units::Joule *unit = Stats::units::Joule::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Joule::toString());
}

TEST(StatsUnitsTest, Volt)
{
    Stats::units::Volt *unit = Stats::units::Volt::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Volt::toString());
}

TEST(StatsUnitsTest, DegreeCelsius)
{
    Stats::units::DegreeCelsius *unit = Stats::units::DegreeCelsius::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::DegreeCelsius::toString());
}

TEST(StatsUnitsTest, Count)
{
    Stats::units::Count *unit = Stats::units::Count::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Count::toString());
}

TEST(StatsUnitsTest, Rate1)
{
    Stats::units::Rate<Stats::units::Count, Stats::units::Count> *unit =
        Stats::units::Rate<Stats::units::Count, Stats::units::Count>::get();
    ASSERT_EQ(unit->getUnitString(), "(Count/Count)");
    ASSERT_EQ(unit->getUnitString(), (Stats::units::Rate<Stats::units::Count,
        Stats::units::Count>::toString()));
}

TEST(StatsUnitsTest, Rate2)
{
    Stats::units::Rate<Stats::units::Tick, Stats::units::Second> *unit =
        Stats::units::Rate<Stats::units::Tick, Stats::units::Second>::get();
    ASSERT_EQ(unit->getUnitString(), "(Tick/Second)");
    ASSERT_EQ(unit->getUnitString(), (Stats::units::Rate<Stats::units::Tick,
        Stats::units::Second>::toString()));
}

TEST(StatsUnitsTest, RateOfRates)
{
    typedef Stats::units::Rate<Stats::units::Bit, Stats::units::Second>
        BitPerSecond;
    typedef Stats::units::Rate<Stats::units::Count, Stats::units::Cycle>
        CountPerCycle;
    Stats::units::Rate<BitPerSecond, CountPerCycle> *unit =
        Stats::units::Rate<BitPerSecond, CountPerCycle>::get();
    ASSERT_EQ(unit->getUnitString(), "((Bit/Second)/(Count/Cycle))");
    ASSERT_EQ(unit->getUnitString(),
        (Stats::units::Rate<BitPerSecond, CountPerCycle>::toString()));
}

TEST(StatsUnitsTest, Ratio)
{
    Stats::units::Ratio *unit = Stats::units::Ratio::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Ratio::toString());
}

TEST(StatsUnitsTest, Unspecified)
{
    Stats::units::Unspecified *unit = Stats::units::Unspecified::get();
    ASSERT_EQ(unit->getUnitString(), Stats::units::Unspecified::toString());
}
