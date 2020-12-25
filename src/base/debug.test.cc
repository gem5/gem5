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

/** Test assignment of names and descriptions. */
TEST(DebugFlagTest, NameDesc)
{
    Debug::SimpleFlag flag_a("FlagNameDescTestKidA", "Kid A");
    EXPECT_EQ("FlagNameDescTestKidA", flag_a.name());
    EXPECT_EQ("Kid A", flag_a.desc());

    Debug::SimpleFlag flag_b("FlagNameDescTestKidB", "Kid B");
    EXPECT_EQ("FlagNameDescTestKidB", flag_b.name());
    EXPECT_EQ("Kid B", flag_b.desc());

    Debug::CompoundFlag compound_flag("FlagNameDescTest", "Compound Flag",
        {&flag_a, &flag_b});
    EXPECT_EQ("FlagNameDescTest", compound_flag.name());
    EXPECT_EQ("Compound Flag", compound_flag.desc());
}

/** Test that names are unique. */
TEST(DebugFlagDeathTest, UniqueNames)
{
    Debug::SimpleFlag flag("FlagUniqueNamesTest", "A");
    testing::internal::CaptureStderr();
    EXPECT_ANY_THROW(Debug::SimpleFlag("FlagUniqueNamesTest", "B"));
    const std::string expected = "panic: panic condition !result.second "
        "occurred: Flag FlagUniqueNamesTest already defined!\n";
    std::string actual = testing::internal::GetCapturedStderr().substr();
    actual = actual.substr(actual.find(":", actual.find(":") + 1) + 2);
    EXPECT_EQ(expected, actual);
}

/** Test enabling and disabling simple flags, as well as the global enabler. */
TEST(DebugSimpleFlagTest, Enabled)
{
    Debug::Flag::globalDisable();
    Debug::SimpleFlag flag("SimpleFlagEnabledTest", "");

    // By default flags are initialized disabled
    ASSERT_FALSE(flag.enabled());

    // Flags must be globally enabled before individual flags are enabled
    flag.enable();
    ASSERT_FALSE(flag.enabled());
    Debug::Flag::globalEnable();
    ASSERT_TRUE(flag.enabled());

    // Verify that the global enabler works
    Debug::Flag::globalDisable();
    ASSERT_FALSE(flag.enabled());
    Debug::Flag::globalEnable();
    ASSERT_TRUE(flag.enabled());

    // Test disabling the flag with global enabled
    flag.disable();
    ASSERT_FALSE(flag.enabled());
}

/**
 * Tests that manipulate the enablement status of the compound flag to change
 * the corresponding status of the kids.
 */
TEST(DebugCompoundFlagTest, Enabled)
{
    Debug::Flag::globalDisable();
    Debug::SimpleFlag flag_a("CompoundFlagEnabledTestKidA", "");
    Debug::SimpleFlag flag_b("CompoundFlagEnabledTestKidB", "");
    Debug::CompoundFlag flag("CompoundFlagEnabledTest", "",
        {&flag_a, &flag_b});

    // By default flags are initialized disabled
    ASSERT_FALSE(flag.enabled());

    // Flags must be globally enabled before individual flags are enabled
    flag.enable();
    ASSERT_FALSE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());
    Debug::Flag::globalEnable();
    for (auto &kid : flag.kids()) {
        ASSERT_TRUE(kid->enabled());
    }
    ASSERT_TRUE(flag_a.enabled());
    ASSERT_TRUE(flag_b.enabled());
    ASSERT_TRUE(flag.enabled());

    // Test disabling the flag with global enabled
    flag.disable();
    for (auto &kid : flag.kids()) {
        ASSERT_FALSE(kid->enabled());
    }
    ASSERT_FALSE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());
}

/** Test that the conversion operator matches the enablement status. */
TEST(DebugFlagTest, ConversionOperator)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag("FlagConversionOperatorTest", "");

    ASSERT_EQ(flag, flag.enabled());
    flag.enable();
    ASSERT_EQ(flag, flag.enabled());
    flag.disable();
}

/**
 * Tests that manipulate the kids to change the enablement status of the
 * compound flag.
 */
TEST(DebugCompoundFlagTest, EnabledKids)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag_a("CompoundFlagEnabledKidsTestKidA", "");
    Debug::SimpleFlag flag_b("CompoundFlagEnabledKidsTestKidB", "");
    Debug::CompoundFlag flag("CompoundFlagEnabledKidsTest", "",
        {&flag_a, &flag_b});

    // Test enabling only flag A
    ASSERT_FALSE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());
    flag_a.enable();
    ASSERT_TRUE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());

    // Test that enabling both flags enables the compound flag
    ASSERT_TRUE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());
    flag_b.enable();
    ASSERT_TRUE(flag_a.enabled());
    ASSERT_TRUE(flag_b.enabled());
    ASSERT_TRUE(flag.enabled());

    // Test that disabling one of the flags disables the compound flag
    flag_a.disable();
    ASSERT_FALSE(flag_a.enabled());
    ASSERT_TRUE(flag_b.enabled());
    ASSERT_FALSE(flag.enabled());
}

/** Search for existent and non-existent flags. */
TEST(DebugFlagTest, FindFlag)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag_a("FlagFindFlagTestA", "");
    Debug::SimpleFlag flag_b("FlagFindFlagTestB", "");

    // Enable the found flags and verify that the original flags are
    // enabled too
    Debug::Flag *flag;
    EXPECT_TRUE(flag = Debug::findFlag("FlagFindFlagTestA"));
    ASSERT_FALSE(flag_a.enabled());
    flag->enable();
    ASSERT_TRUE(flag_a.enabled());
    EXPECT_TRUE(flag = Debug::findFlag("FlagFindFlagTestB"));
    ASSERT_FALSE(flag_b.enabled());
    flag->enable();
    ASSERT_TRUE(flag_b.enabled());

    // Search for a non-existent flag
    EXPECT_FALSE(Debug::findFlag("FlagFindFlagTestC"));
}

/** Test changing flag enabled. */
TEST(DebugFlagTest, ChangeFlag)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag_a("FlagChangeFlagTestA", "");
    Debug::SimpleFlag flag_b("FlagChangeFlagTestB", "");

    // Enable the found flags and verify that the original flags are
    // enabled too
    ASSERT_FALSE(flag_a.enabled());
    EXPECT_TRUE(Debug::changeFlag("FlagChangeFlagTestA", true));
    ASSERT_TRUE(flag_a.enabled());
    EXPECT_TRUE(Debug::changeFlag("FlagChangeFlagTestA", false));
    ASSERT_FALSE(flag_a.enabled());

    // Disable and enable a flag
    ASSERT_FALSE(flag_b.enabled());
    EXPECT_TRUE(Debug::changeFlag("FlagChangeFlagTestB", false));
    ASSERT_FALSE(flag_b.enabled());
    EXPECT_TRUE(Debug::changeFlag("FlagChangeFlagTestB", true));
    ASSERT_TRUE(flag_b.enabled());

    // Change a non-existent flag
    ASSERT_FALSE(Debug::changeFlag("FlagChangeFlagTestC", true));
}

/** Test changing flag enabled with aux functions. */
TEST(DebugFlagTest, SetClearDebugFlag)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag_a("FlagSetClearDebugFlagTestA", "");
    Debug::SimpleFlag flag_b("FlagSetClearDebugFlagTestB", "");

    // Enable and disable a flag
    ASSERT_FALSE(flag_a.enabled());
    setDebugFlag("FlagSetClearDebugFlagTestA");
    ASSERT_TRUE(flag_a.enabled());
    clearDebugFlag("FlagSetClearDebugFlagTestA");
    ASSERT_FALSE(flag_a.enabled());

    // Disable and enable a flag
    ASSERT_FALSE(flag_b.enabled());
    clearDebugFlag("FlagSetClearDebugFlagTestB");
    ASSERT_FALSE(flag_b.enabled());
    setDebugFlag("FlagSetClearDebugFlagTestB");
    ASSERT_TRUE(flag_b.enabled());

    // Change a non-existent flag
    setDebugFlag("FlagSetClearDebugFlagTestC");
    clearDebugFlag("FlagSetClearDebugFlagTestC");
}

/** Test dumping no enabled debug flags. */
TEST(DebugFlagTest, NoDumpDebugFlags)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag("FlagDumpDebugFlagTest", "");

    // Verify that the names of the enabled flags are printed
    testing::internal::CaptureStdout();
    dumpDebugFlags();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "");
    ASSERT_FALSE(flag.enabled());
}

/** Test dumping enabled debug flags with a larger set of flags. */
TEST(DebugFlagTest, DumpDebugFlags)
{
    Debug::Flag::globalEnable();
    Debug::SimpleFlag flag_a("FlagDumpDebugFlagTestA", "");
    Debug::SimpleFlag flag_b("FlagDumpDebugFlagTestB", "");
    Debug::SimpleFlag flag_c("FlagDumpDebugFlagTestC", "");
    Debug::SimpleFlag flag_d("FlagDumpDebugFlagTestD", "");
    Debug::SimpleFlag flag_e("FlagDumpDebugFlagTestE", "");
    Debug::CompoundFlag compound_flag_a("CompoundFlagDumpDebugFlagTestA", "",
        {&flag_d});
    Debug::CompoundFlag compound_flag_b("CompoundFlagDumpDebugFlagTestB", "",
        {&flag_e});

    // Enable a few flags
    ASSERT_FALSE(flag_a.enabled());
    ASSERT_FALSE(flag_b.enabled());
    ASSERT_FALSE(flag_c.enabled());
    ASSERT_FALSE(flag_d.enabled());
    ASSERT_FALSE(flag_e.enabled());
    flag_a.enable();
    flag_c.enable();
    compound_flag_b.enable();

    // Verify that the names of the enabled flags are printed
    testing::internal::CaptureStdout();
    dumpDebugFlags();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "FlagDumpDebugFlagTestA\nFlagDumpDebugFlagTestC\n" \
        "FlagDumpDebugFlagTestE\n");
}
