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

#include "base/stats/info.hh"

using namespace gem5;

class TestInfo : public statistics::Info
{
  public:
    using statistics::Info::Info;

    int value = 0;

    bool check() const override { return true; }
    void prepare() override {}
    void reset() override { value = 0; }
    bool zero() const override { return false; }
    void visit(statistics::Output &visitor) override {}
};

/** Test that a name is properly assigned under the new style. */
TEST(StatsInfoTest, Name)
{
    TestInfo info;
    info.setName("InfoName");
    ASSERT_EQ(info.name, "InfoName");
}

/** Test that the first character accepts alpha and underscore. */
TEST(StatsInfoTest, NameFirstCharacter)
{
    TestInfo info;
    info.setName("X");
    ASSERT_EQ(info.name, "X");

    TestInfo info2;
    info2.setName("a");
    ASSERT_EQ(info2.name, "a");

    TestInfo info3;
    info3.setName("_");
    ASSERT_EQ(info3.name, "_");

    TestInfo info4;
    info4.setName("X.Y");
    ASSERT_EQ(info4.name, "X.Y");

    TestInfo info5;
    info5.setName("a.b._.d.e");
    ASSERT_EQ(info5.name, "a.b._.d.e");
}

/** Test that the non-first characters accept alpha-numeric and underscore. */
TEST(StatsInfoTest, NameOtherCharacters)
{
    TestInfo info;
    info.setName("abCde");
    ASSERT_EQ(info.name, "abCde");

    TestInfo info2;
    info2.setName("ab_de");
    ASSERT_EQ(info2.name, "ab_de");

    TestInfo info3;
    info3.setName("ab9de");
    ASSERT_EQ(info3.name, "ab9de");

    TestInfo info4;
    info4.setName("a_bCD12_ef");
    ASSERT_EQ(info4.name, "a_bCD12_ef");

    TestInfo info5;
    info5.setName("a_._bC.D12_ef");
    ASSERT_EQ(info5.name, "a_._bC.D12_ef");
}

/**
 * Test that a name can be duplicated within different groups. Duplicating
 * is not a problem that must be solved by Info in this case. The stats name
 * resolver does that.
 */
TEST(StatsInfoTest, NameDuplicate)
{
    TestInfo info;
    std::string name = "InfoNameDuplicate";
    info.setName(name);
    EXPECT_EQ(info.name, name);

    TestInfo info2;
    info2.setName(name);
    ASSERT_EQ(info2.name, name);
}

/** Test changing the separator. */
TEST(StatsInfoTest, Separator)
{
    TestInfo info;

    // Get current separator
    auto separator = info.separatorString;

    // Do the test with EXPECT_ so that we can restore the separator later
    info.setSeparator(",-=-,");
    EXPECT_EQ(statistics::Info::separatorString, ",-=-,");

    // Restore separator
    info.setSeparator(separator);
}

/** Test changing the less operation, which is applied on the name. */
TEST(StatsInfoTest, Less)
{
    TestInfo info;
    info.setName("Less1");
    TestInfo info2;
    info2.setName("Less2");

    ASSERT_TRUE(statistics::Info::less(&info, &info2));
    ASSERT_FALSE(statistics::Info::less(&info2, &info));
}

/** Test that checking Info after setting the init flag succeeds. */
TEST(StatsInfoTest, BaseCheckInit)
{
    TestInfo info;
    info.flags.set(statistics::init);
    ASSERT_TRUE(info.baseCheck());
}

/** Test that checking Info for display after setting the name succeeds. */
TEST(StatsInfoTest, BaseCheckDisplay)
{
    TestInfo info;
    info.setName("BaseCheckDisplay");
    info.flags.set(statistics::init | statistics::display);
    ASSERT_TRUE(info.baseCheck());
}

/**
 * Test changing the less operation, which is applied on the name (sub-groups).
 */
TEST(StatsInfoTest, LessSub)
{
    TestInfo info;
    info.setName("Less.Sub2.a");
    TestInfo info2;
    info2.setName("Less.Sub1.b");

    ASSERT_TRUE(statistics::Info::less(&info2, &info));
    ASSERT_FALSE(statistics::Info::less(&info, &info2));
}

/** Test that a name cannot be empty. */
TEST(StatsInfoDeathTest, NameEmpty)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName(""));
}

/** Test that a sub-group's name cannot be empty. */
TEST(StatsInfoDeathTest, NameSubEmpty)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName(".a"));
}

/** Test that a sub-group's name cannot be empty. */
TEST(StatsInfoDeathTest, NameSubEmpty2)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("A."));
}

/** Test that a sub-group's name cannot be empty. */
TEST(StatsInfoDeathTest, NameSubEmpty3)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("a.b..c"));
}

/** Test that the first character does not accept numbers. */
TEST(StatsInfoDeathTest, NameFirstCharacterNumber)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("1"));
}

/** Test that the first character does not accept numbers (sub-group). */
TEST(StatsInfoDeathTest, NameFirstCharacterNumberSub)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("A.1"));
}

/** Test that the first character does not accept special characters. */
TEST(StatsInfoDeathTest, NameFirstCharacterSpecial)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("!"));
}

/**
 * Test that the first character does not accept special characters
 * (sub-group).
 */
TEST(StatsInfoDeathTest, NameFirstCharacterSpecialSub)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("A.!"));
}

/** Test that the non-first characters do not accept special characters. */
TEST(StatsInfoDeathTest, NameOtherCharacterSpecial)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.setName("ab!de"));
}

/** Test that checking Info without setting the init flag fails. */
TEST(StatsInfoDeathTest, BaseCheckNoInit)
{
    TestInfo info;
    ASSERT_ANY_THROW(info.baseCheck());
}

/** Test that checking Info for display without setting the name fails. */
TEST(StatsInfoDeathTest, BaseCheckDisplayNoName)
{
    TestInfo info;
    info.flags.set(statistics::init | statistics::display);
    ASSERT_ANY_THROW(info.baseCheck());
}
