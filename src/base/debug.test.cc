/*
 * Copyright (c) 2020 Daniel R. Carvalho
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

#include "base/debug.hh"
#include "base/gtest/logging.hh"

using namespace gem5;

/** Test assignment of names and descriptions. */
TEST(DebugFlagTest, NameDesc)
{
    debug::SimpleFlag flag_a("FlagNameDescTestKidA", "Kid A");
    EXPECT_EQ("FlagNameDescTestKidA", flag_a.name());
    EXPECT_EQ("Kid A", flag_a.desc());

    debug::SimpleFlag flag_b("FlagNameDescTestKidB", "Kid B");
    EXPECT_EQ("FlagNameDescTestKidB", flag_b.name());
    EXPECT_EQ("Kid B", flag_b.desc());

    debug::CompoundFlag compound_flag("FlagNameDescTest", "Compound Flag",
        {&flag_a, &flag_b});
    EXPECT_EQ("FlagNameDescTest", compound_flag.name());
    EXPECT_EQ("Compound Flag", compound_flag.desc());
}

/** Test that names are unique. */
TEST(DebugFlagDeathTest, UniqueNames)
{
    debug::SimpleFlag flag("FlagUniqueNamesTest", "A");
    gtestLogOutput.str("");
    EXPECT_ANY_THROW(debug::SimpleFlag("FlagUniqueNamesTest", "B"));
    const std::string expected = "panic: panic condition !result.second "
        "occurred: Flag FlagUniqueNamesTest already defined!\n";
    std::string actual = gtestLogOutput.str();
    EXPECT_EQ(expected, actual);
}

/** Test format attribute. */
TEST(DebugFlagTest, IsFormat)
{
    debug::SimpleFlag flag_a("FlagIsFormatTestA", "", true);
    EXPECT_TRUE(flag_a.isFormat());
    debug::SimpleFlag flag_b("FlagIsFormatTestB", "", false);
    EXPECT_FALSE(flag_b.isFormat());
    debug::SimpleFlag flag_c("FlagIsFormatTestC", "");
    EXPECT_FALSE(flag_c.isFormat());
}

/** Test enabling and disabling simple flags, as well as the global enabler. */
TEST(DebugSimpleFlagTest, Enabled)
{
    debug::Flag::globalDisable();
    debug::SimpleFlag flag("SimpleFlagEnabledTest", "");

    // By default flags are initialized disabled
    ASSERT_FALSE(flag.tracing());

    // Flags must be globally enabled before individual flags are enabled
    flag.enable();
    ASSERT_FALSE(flag.tracing());
    debug::Flag::globalEnable();
    ASSERT_TRUE(!TRACING_ON || flag.tracing());

    // Verify that the global enabler works
    debug::Flag::globalDisable();
    ASSERT_FALSE(flag.tracing());
    debug::Flag::globalEnable();
    ASSERT_TRUE(!TRACING_ON || flag.tracing());

    // Test disabling the flag with global enabled
    flag.disable();
    ASSERT_FALSE(flag.tracing());
}

/**
 * Tests that manipulate the enablement status of the compound flag to change
 * the corresponding status of the kids.
 */
TEST(DebugCompoundFlagTest, Enabled)
{
    debug::Flag::globalDisable();
    debug::SimpleFlag flag_a("CompoundFlagEnabledTestKidA", "");
    debug::SimpleFlag flag_b("CompoundFlagEnabledTestKidB", "");
    debug::CompoundFlag flag("CompoundFlagEnabledTest", "",
        {&flag_a, &flag_b});

    // By default flags are initialized disabled
    ASSERT_FALSE(flag.tracing());

    // Flags must be globally enabled before individual flags are enabled
    flag.enable();
    ASSERT_FALSE(flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag.tracing());
    debug::Flag::globalEnable();
    for (auto &kid : flag.kids()) {
        ASSERT_TRUE(!TRACING_ON || kid->tracing());
    }
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());

    // Test disabling the flag with global enabled
    flag.disable();
    for (auto &kid : flag.kids()) {
        ASSERT_FALSE(kid->tracing());
    }
    ASSERT_FALSE(flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag.tracing());
}

/** Test that the conversion operator matches the enablement status. */
TEST(DebugFlagTest, ConversionOperator)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag("FlagConversionOperatorTest", "");

    ASSERT_EQ(flag, flag.tracing());
    flag.enable();
    ASSERT_EQ(flag, flag.tracing());
    flag.disable();
}

/**
 * Tests that manipulate the kids to change the enablement status of the
 * compound flag.
 */
TEST(DebugCompoundFlagTest, EnabledKids)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag_a("CompoundFlagEnabledKidsTestKidA", "");
    debug::SimpleFlag flag_b("CompoundFlagEnabledKidsTestKidB", "");
    debug::CompoundFlag flag("CompoundFlagEnabledKidsTest", "",
        {&flag_a, &flag_b});

    // Test enabling only flag A
    ASSERT_FALSE(flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag.tracing());
    flag_a.enable();
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag.tracing());

    // Test that enabling both flags enables the compound flag
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag.tracing());
    flag_b.enable();
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());

    // Test that disabling one of the flags disables the compound flag
    flag_a.disable();
    ASSERT_FALSE(flag_a.tracing());
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());
    ASSERT_FALSE(flag.tracing());
}

/** Search for existent and non-existent flags. */
TEST(DebugFlagTest, FindFlag)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag_a("FlagFindFlagTestA", "");
    debug::SimpleFlag flag_b("FlagFindFlagTestB", "");

    // Enable the found flags and verify that the original flags are
    // enabled too
    debug::Flag *flag;
    EXPECT_TRUE(flag = debug::findFlag("FlagFindFlagTestA"));
    ASSERT_FALSE(flag_a.tracing());
    flag->enable();
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    EXPECT_TRUE(flag = debug::findFlag("FlagFindFlagTestB"));
    ASSERT_FALSE(flag_b.tracing());
    flag->enable();
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());

    // Search for a non-existent flag
    EXPECT_FALSE(debug::findFlag("FlagFindFlagTestC"));
}

/** Test changing flag enabled. */
TEST(DebugFlagTest, ChangeFlag)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag_a("FlagChangeFlagTestA", "");
    debug::SimpleFlag flag_b("FlagChangeFlagTestB", "");

    // Enable the found flags and verify that the original flags are
    // enabled too
    ASSERT_FALSE(flag_a.tracing());
    EXPECT_TRUE(debug::changeFlag("FlagChangeFlagTestA", true));
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    EXPECT_TRUE(debug::changeFlag("FlagChangeFlagTestA", false));
    ASSERT_FALSE(flag_a.tracing());

    // Disable and enable a flag
    ASSERT_FALSE(flag_b.tracing());
    EXPECT_TRUE(debug::changeFlag("FlagChangeFlagTestB", false));
    ASSERT_FALSE(flag_b.tracing());
    EXPECT_TRUE(debug::changeFlag("FlagChangeFlagTestB", true));
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());

    // Change a non-existent flag
    ASSERT_FALSE(debug::changeFlag("FlagChangeFlagTestC", true));
}

/** Test changing flag enabled with aux functions. */
TEST(DebugFlagTest, SetClearDebugFlag)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag_a("FlagSetClearDebugFlagTestA", "");
    debug::SimpleFlag flag_b("FlagSetClearDebugFlagTestB", "");

    // Enable and disable a flag
    ASSERT_FALSE(flag_a.tracing());
    setDebugFlag("FlagSetClearDebugFlagTestA");
    ASSERT_TRUE(!TRACING_ON || flag_a.tracing());
    clearDebugFlag("FlagSetClearDebugFlagTestA");
    ASSERT_FALSE(flag_a.tracing());

    // Disable and enable a flag
    ASSERT_FALSE(flag_b.tracing());
    clearDebugFlag("FlagSetClearDebugFlagTestB");
    ASSERT_FALSE(flag_b.tracing());
    setDebugFlag("FlagSetClearDebugFlagTestB");
    ASSERT_TRUE(!TRACING_ON || flag_b.tracing());

    // Change a non-existent flag
    setDebugFlag("FlagSetClearDebugFlagTestC");
    clearDebugFlag("FlagSetClearDebugFlagTestC");
}

/** Test dumping no enabled debug flags. */
TEST(DebugFlagTest, NoDumpDebugFlags)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag("FlagDumpDebugFlagTest", "");

    // Verify that the names of the enabled flags are printed
    gtestLogOutput.str("");
    dumpDebugFlags();
    std::string output = gtestLogOutput.str();
    EXPECT_EQ(output, "");
    ASSERT_FALSE(flag.tracing());
}

/** Test dumping enabled debug flags with a larger set of flags. */
TEST(DebugFlagTest, DumpDebugFlags)
{
    debug::Flag::globalEnable();
    debug::SimpleFlag flag_a("FlagDumpDebugFlagTestA", "");
    debug::SimpleFlag flag_b("FlagDumpDebugFlagTestB", "");
    debug::SimpleFlag flag_c("FlagDumpDebugFlagTestC", "");
    debug::SimpleFlag flag_d("FlagDumpDebugFlagTestD", "");
    debug::SimpleFlag flag_e("FlagDumpDebugFlagTestE", "");
    debug::CompoundFlag compound_flag_a("CompoundFlagDumpDebugFlagTestA", "",
        {&flag_d});
    debug::CompoundFlag compound_flag_b("CompoundFlagDumpDebugFlagTestB", "",
        {&flag_e});

    // Enable a few flags
    ASSERT_FALSE(flag_a.tracing());
    ASSERT_FALSE(flag_b.tracing());
    ASSERT_FALSE(flag_c.tracing());
    ASSERT_FALSE(flag_d.tracing());
    ASSERT_FALSE(flag_e.tracing());
    flag_a.enable();
    flag_c.enable();
    compound_flag_b.enable();

    // Verify that the names of the enabled flags are printed if TRACING_ON.
    if (TRACING_ON) {
        std::ostringstream os;
        dumpDebugFlags(os);
        std::string output = os.str();
        EXPECT_EQ(output, "FlagDumpDebugFlagTestA\nFlagDumpDebugFlagTestC\n" \
            "FlagDumpDebugFlagTestE\n");
    }
}
