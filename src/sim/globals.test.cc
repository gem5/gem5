/*
 * Copyright 2022 Daniel R. Carvalho
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
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <string>

#include "base/gtest/cur_tick_fake.hh"
#include "base/gtest/logging.hh"
#include "base/gtest/serialization_fixture.hh"
#include "sim/globals.hh"

// The version tags are declared as extern
namespace gem5
{
std::set<std::string> version_tags;
} // namespace gem5

using namespace gem5;

// Use the tick handled to manipulate the current tick
GTestTickHandler tickHandler;

using GlobalsSerializationFixture = SerializationFixture;
using GlobalsSerializationFixtureDeathTest = GlobalsSerializationFixture;

/** Test serialization. */
TEST_F(GlobalsSerializationFixture, Serialization)
{
    Globals globals;
    tickHandler.setCurTick(1234);
    version_tags = { "first-tag", "second-tag", "third-tag", "fourth-tag" };

    // Serialization
    std::ofstream cp(getCptPath());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");
    globals.serialize(cp);

    // The checkpoint must be flushed, otherwise the file may not be up-
    // to-date and the assertions below will fail
    cp.close();

    // Verify the output
    std::ifstream is(getCptPath());
    assert(is.good());
    std::string str = std::string(std::istreambuf_iterator<char>(is),
                                  std::istreambuf_iterator<char>());
    ASSERT_THAT(
        str, ::testing::StrEq(
                 "\n[Section1]\ncurTick=1234\n"
                 "version_tags=first-tag fourth-tag second-tag third-tag\n"));
}

/** Test unserialization. */
TEST_F(GlobalsSerializationFixture, Unserialization)
{
    version_tags = { "first-tag-un", "second-tag-un", "third-tag-un",
                     "fourth-tag-un" };
    simulateSerialization(
        "\n[Section1]\ncurTick=1111\nversion_tags="
        "first-tag-un second-tag-un third-tag-un fourth-tag-un\n");

    Globals globals;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    gtestLogOutput.str("");
    globals.unserialize(cp);
    ASSERT_THAT(gtestLogOutput.str(), ::testing::StrEq(""));
    ASSERT_EQ(globals.unserializedCurTick, 1111);
}

/**
 * Test that unserialization fails when there are no version tags in the
 * checkpoint.
 */
TEST_F(GlobalsSerializationFixture, UnserializationCptNoVersionTags)
{
    version_tags = {};
    simulateSerialization("\n[Section1]\ncurTick=2222\n");

    // Unserialization
    Globals globals;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    gtestLogOutput.str("");
    globals.unserialize(cp);
    ASSERT_THAT(
        gtestLogOutput.str(),
        ::testing::HasSubstr("Checkpoint uses an old versioning scheme."));
    ASSERT_EQ(globals.unserializedCurTick, 2222);
}

/** Test that a warning is thrown when the cpt misses any of gem5's tags. */
TEST_F(GlobalsSerializationFixture, UnserializationCptMissingVersionTags)
{
    version_tags = { "first-tag-un", "second-tag-un", "third-tag-un",
                     "fourth-tag-un" };
    simulateSerialization("\n[Section1]\ncurTick=3333\n"
                          "version_tags=second-tag-un fourth-tag-un\n");

    Globals globals;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    gtestLogOutput.str("");
    globals.unserialize(cp);
    ASSERT_THAT(
        gtestLogOutput.str(),
        ::testing::HasSubstr("warn:   first-tag-un\nwarn:   third-tag-un\n"));
    ASSERT_EQ(globals.unserializedCurTick, 3333);
}

/** Test that a warning is thrown when gem5 misses any of the cpt's tags. */
TEST_F(GlobalsSerializationFixture, UnserializationGem5MissingVersionTags)
{
    version_tags = { "first-tag-un", "second-tag-un", "third-tag-un" };
    simulateSerialization(
        "\n[Section1]\ncurTick=4444\nversion_tags="
        "first-tag-un second-tag-un third-tag-un fourth-tag-un\n");

    Globals globals;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");

    gtestLogOutput.str("");
    globals.unserialize(cp);
    ASSERT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("warn:   fourth-tag-un\n"));
    ASSERT_EQ(globals.unserializedCurTick, 4444);
}

/**
 * Test that unserialization fails when there are is no cur tick in the
 * checkpoint.
 */
TEST_F(GlobalsSerializationFixtureDeathTest, UnserializationCptNoCurTick)
{
    simulateSerialization("\n[Section1]\n");

    Globals globals;
    CheckpointIn cp(getDirName());
    Serializable::ScopedCheckpointSection scs(cp, "Section1");
    ASSERT_ANY_THROW(globals.unserialize(cp));
}
