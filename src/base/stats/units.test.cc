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
    Stats::Units::Cycle *unit = UNIT_CYCLE;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Cycle::toString());
}

TEST(StatsUnitsTest, Tick)
{
    Stats::Units::Tick *unit = UNIT_TICK;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Tick::toString());
}

TEST(StatsUnitsTest, Second)
{
    Stats::Units::Second *unit = UNIT_SECOND;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Second::toString());
}

TEST(StatsUnitsTest, Bit)
{
    Stats::Units::Bit *unit = UNIT_BIT;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Bit::toString());
}

TEST(StatsUnitsTest, Byte)
{
    Stats::Units::Byte *unit = UNIT_BYTE;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Byte::toString());
}

TEST(StatsUnitsTest, Watt)
{
    Stats::Units::Watt *unit = UNIT_WATT;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Watt::toString());
}

TEST(StatsUnitsTest, Joule)
{
    Stats::Units::Joule *unit = UNIT_JOULE;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Joule::toString());
}

TEST(StatsUnitsTest, Volt)
{
    Stats::Units::Volt *unit = UNIT_VOLT;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Volt::toString());
}

TEST(StatsUnitsTest, DegreeCelsius)
{
    Stats::Units::DegreeCelsius *unit = UNIT_CELSIUS;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::DegreeCelsius::toString());
}

TEST(StatsUnitsTest, Count)
{
    Stats::Units::Count *unit = UNIT_COUNT;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Count::toString());
}

TEST(StatsUnitsTest, Rate1)
{
    Stats::Units::Rate<Stats::Units::Count, Stats::Units::Count> *unit =
        UNIT_RATE(Stats::Units::Count, Stats::Units::Count);
    ASSERT_EQ(unit->getUnitString(), "(Count/Count)");
    ASSERT_EQ(unit->getUnitString(), (Stats::Units::Rate<Stats::Units::Count,
        Stats::Units::Count>::toString()));
}

TEST(StatsUnitsTest, Rate2)
{
    Stats::Units::Rate<Stats::Units::Tick, Stats::Units::Second> *unit =
        UNIT_RATE(Stats::Units::Tick, Stats::Units::Second);
    ASSERT_EQ(unit->getUnitString(), "(Tick/Second)");
    ASSERT_EQ(unit->getUnitString(), (Stats::Units::Rate<Stats::Units::Tick,
        Stats::Units::Second>::toString()));
}

TEST(StatsUnitsTest, RateOfRates)
{
    typedef Stats::Units::Rate<Stats::Units::Bit, Stats::Units::Second>
        BitPerSecond;
    typedef Stats::Units::Rate<Stats::Units::Count, Stats::Units::Cycle>
        CountPerCycle;
    Stats::Units::Rate<BitPerSecond, CountPerCycle> *unit =
        UNIT_RATE(BitPerSecond, CountPerCycle);
    ASSERT_EQ(unit->getUnitString(), "((Bit/Second)/(Count/Cycle))");
    ASSERT_EQ(unit->getUnitString(),
        (Stats::Units::Rate<BitPerSecond, CountPerCycle>::toString()));
}

TEST(StatsUnitsTest, Ratio)
{
    Stats::Units::Ratio *unit = UNIT_RATIO;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Ratio::toString());
}

TEST(StatsUnitsTest, Unspecified)
{
    Stats::Units::Unspecified *unit = UNIT_UNSPECIFIED;
    ASSERT_EQ(unit->getUnitString(), Stats::Units::Unspecified::toString());
}
