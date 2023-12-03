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

using namespace gem5;

TEST(StatsUnitsTest, Cycle)
{
    statistics::units::Cycle *unit = statistics::units::Cycle::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Cycle::toString());
}

TEST(StatsUnitsTest, Tick)
{
    statistics::units::Tick *unit = statistics::units::Tick::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Tick::toString());
}

TEST(StatsUnitsTest, Second)
{
    statistics::units::Second *unit = statistics::units::Second::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Second::toString());
}

TEST(StatsUnitsTest, Bit)
{
    statistics::units::Bit *unit = statistics::units::Bit::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Bit::toString());
}

TEST(StatsUnitsTest, Byte)
{
    statistics::units::Byte *unit = statistics::units::Byte::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Byte::toString());
}

TEST(StatsUnitsTest, Watt)
{
    statistics::units::Watt *unit = statistics::units::Watt::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Watt::toString());
}

TEST(StatsUnitsTest, Joule)
{
    statistics::units::Joule *unit = statistics::units::Joule::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Joule::toString());
}

TEST(StatsUnitsTest, Volt)
{
    statistics::units::Volt *unit = statistics::units::Volt::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Volt::toString());
}

TEST(StatsUnitsTest, DegreeCelsius)
{
    statistics::units::DegreeCelsius *unit =
        statistics::units::DegreeCelsius::get();
    ASSERT_EQ(unit->getUnitString(),
              statistics::units::DegreeCelsius::toString());
}

TEST(StatsUnitsTest, Count)
{
    statistics::units::Count *unit = statistics::units::Count::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Count::toString());
}

TEST(StatsUnitsTest, Rate1)
{
    statistics::units::Rate<statistics::units::Count, statistics::units::Count>
        *unit = statistics::units::Rate<statistics::units::Count,
                                        statistics::units::Count>::get();
    ASSERT_EQ(unit->getUnitString(), "(Count/Count)");
    ASSERT_EQ(unit->getUnitString(),
              (statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::toString()));
}

TEST(StatsUnitsTest, Rate2)
{
    statistics::units::Rate<statistics::units::Tick, statistics::units::Second>
        *unit = statistics::units::Rate<statistics::units::Tick,
                                        statistics::units::Second>::get();
    ASSERT_EQ(unit->getUnitString(), "(Tick/Second)");
    ASSERT_EQ(
        unit->getUnitString(),
        (statistics::units::Rate<statistics::units::Tick,
                                 statistics::units::Second>::toString()));
}

TEST(StatsUnitsTest, RateOfRates)
{
    typedef statistics::units::Rate<statistics::units::Bit,
                                    statistics::units::Second>
        BitPerSecond;
    typedef statistics::units::Rate<statistics::units::Count,
                                    statistics::units::Cycle>
        CountPerCycle;
    statistics::units::Rate<BitPerSecond, CountPerCycle> *unit =
        statistics::units::Rate<BitPerSecond, CountPerCycle>::get();
    ASSERT_EQ(unit->getUnitString(), "((Bit/Second)/(Count/Cycle))");
    ASSERT_EQ(
        unit->getUnitString(),
        (statistics::units::Rate<BitPerSecond, CountPerCycle>::toString()));
}

TEST(StatsUnitsTest, Ratio)
{
    statistics::units::Ratio *unit = statistics::units::Ratio::get();
    ASSERT_EQ(unit->getUnitString(), statistics::units::Ratio::toString());
}

TEST(StatsUnitsTest, Unspecified)
{
    statistics::units::Unspecified *unit =
        statistics::units::Unspecified::get();
    ASSERT_EQ(unit->getUnitString(),
              statistics::units::Unspecified::toString());
}
