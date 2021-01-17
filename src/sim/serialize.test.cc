/*
 * Copyright 2021 Daniel R. Carvalho
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
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <cstdint>
#include <deque>
#include <fstream>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "base/compiler.hh"
#include "base/gtest/cur_tick_fake.hh"
#include "base/gtest/logging.hh"
#include "base/gtest/serialization_fixture.hh"
#include "sim/serialize.hh"

using namespace gem5;

// Instantiate the mock class to have a valid curTick of 0
GTestTickHandler tickHandler;

/** @return Whether the given dir exists and is accessible. */
bool
dirExists(std::string dir)
{
    struct stat info;
    return (stat(dir.c_str(), &info) == 0) && (info.st_mode & S_IFDIR);
}

using SerializeFixture = SerializationFixture;

class CheckpointInFixture : public SerializationFixture
{
  public:
    std::unique_ptr<CheckpointIn> cpt;

    using SerializeFixture::SerializeFixture;

    void
    SetUp() override
    {
        SerializeFixture::SetUp();

        std::ofstream file(getCptPath());
        assert(file.good());
        file << R"cpt_file(
[General]
    Test1=BARasdf
    Test2=bar

[Junk]
Test3=yo
Test4=mama

[Foo]
Foo1=89
Foo2=384

[General]
Test3=89

[Junk]
Test4+=mia
)cpt_file";
        file.close();
        cpt = std::make_unique<CheckpointIn>(getDirName());
    }

    void
    TearDown() override
    {
        std::remove((getCptPath()).c_str());
        SerializeFixture::TearDown();
    }
};

/**
 * A fixture to handle checkpoint in and out variables, as well as the
 * testing of the temporary directory.
 */
class SerializableFixture : public SerializeFixture
{
  public:
    std::unique_ptr<CheckpointIn> cpt_in;
    std::unique_ptr<std::ofstream> cpt_out;

    using SerializeFixture::SerializeFixture;

    void
    SetUp() override
    {
        SerializeFixture::SetUp();

        cpt_out = std::make_unique<std::ofstream>(getCptPath());
        assert(cpt_out->good());
        cpt_in = std::make_unique<CheckpointIn>(getDirName());
    }

    void
    TearDown() override
    {
        std::remove((getCptPath()).c_str());
        SerializeFixture::TearDown();
    }
};

using SerializeFixtureDeathTest = SerializeFixture;
using CheckpointInFixtureDeathTest = CheckpointInFixture;
using SerializableFixtureDeathTest = SerializableFixture;

/** Tests that when setting a checkpoint dir it always ends with a slash. */
TEST(CheckpointInTest, SetGetDirSlash)
{
    ASSERT_EQ(CheckpointIn::setDir(""), "/");
    ASSERT_EQ(CheckpointIn::dir(), "/");
    ASSERT_EQ(CheckpointIn::setDir("/"), "/");
    ASSERT_EQ(CheckpointIn::dir(), "/");
    ASSERT_EQ(CheckpointIn::setDir("test_cpt_dir"), "test_cpt_dir/");
    ASSERT_EQ(CheckpointIn::dir(), "test_cpt_dir/");
    ASSERT_EQ(CheckpointIn::setDir("test_cpt_dir_2/"), "test_cpt_dir_2/");
    ASSERT_EQ(CheckpointIn::dir(), "test_cpt_dir_2/");
}

/**
 * Tests that when the dir name has a "%d" curTick is added. It may also
 * work with other formats, but it is not intended.
 */
TEST(CheckpointInTest, SetGetDirTick)
{
    ASSERT_EQ(CheckpointIn::setDir("tick%d"), "tick0/");
    ASSERT_EQ(CheckpointIn::dir(), "tick0/");
    ASSERT_EQ(CheckpointIn::setDir("ti%dck"), "ti0ck/");
    ASSERT_EQ(CheckpointIn::dir(), "ti0ck/");
}

/**
 * Test constructor failure by requesting the creation of a checkpoint in
 * a non-existent dir.
 */
TEST_F(SerializeFixtureDeathTest, ConstructorFailure)
{
    // Assign another cpt dir to make sure that it is changed when a new
    // CheckpointIn instance is created
    CheckpointIn::setDir("test");

    // Make sure file does not exist, so that the constructor will fail
    std::ifstream file(getCptPath());
    assert(!file.good());
    ASSERT_ANY_THROW(CheckpointIn cpt(getDirName()));
}

/**
 * Test constructor success by requesting the creation of a checkpoint
 * in a specific valid dir. The static cpt dir is change on instantiation.
 */
TEST_F(SerializeFixture, ConstructorSuccess)
{
    // Assign another cpt dir to make sure that it is changed when a new
    // CheckpointIn instance is created
    CheckpointIn::setDir("test");

    // Create the file before creating the cpt to make sure it exists
    std::ofstream file(getCptPath());
    assert(file.good());
    file.close();
    CheckpointIn cpt(getDirName());

    // When a new CheckpointIn instance is created the static cpt dir changes
    EXPECT_EQ(CheckpointIn::dir(), getDirName());
}

/**
 * Test that changing the static cpt dir does not change the name of the
 * cpt dir of previously created instances.
 */
TEST_F(CheckpointInFixtureDeathTest, GetCptDir)
{
    ASSERT_ANY_THROW(CheckpointIn("/random_dir_name"));
}

/** Test finding sections. */
TEST_F(CheckpointInFixture, FindSections)
{
    // Successful searches
    ASSERT_TRUE(cpt->sectionExists("General"));
    ASSERT_TRUE(cpt->sectionExists("Junk"));
    ASSERT_TRUE(cpt->sectionExists("Foo"));

    // Failed searches
    ASSERT_FALSE(cpt->sectionExists("Junk2"));
    ASSERT_FALSE(cpt->sectionExists("Test1"));
}

/** Test finding entries. */
TEST_F(CheckpointInFixture, FindEntries)
{
    // Successful searches
    ASSERT_TRUE(cpt->entryExists("General", "Test2"));
    ASSERT_TRUE(cpt->entryExists("Junk", "Test3"));
    ASSERT_TRUE(cpt->entryExists("Junk", "Test4"));
    ASSERT_TRUE(cpt->entryExists("General", "Test1"));
    ASSERT_TRUE(cpt->entryExists("General", "Test3"));

    // Failed searches
    ASSERT_FALSE(cpt->entryExists("Junk2", "test3"));
    ASSERT_FALSE(cpt->entryExists("Junk", "test4"));
}

/** Test extracting the values of entries. */
TEST_F(CheckpointInFixture, ExtractEntries)
{
    std::string value;

    // Successful searches
    ASSERT_TRUE(cpt->find("General", "Test2", value));
    ASSERT_EQ(value, "bar");
    ASSERT_TRUE(cpt->find("Junk", "Test3", value));
    ASSERT_EQ(value, "yo");
    ASSERT_TRUE(cpt->find("Junk", "Test4", value));
    ASSERT_EQ(value, "mama mia");
    ASSERT_TRUE(cpt->find("General", "Test1", value));
    ASSERT_EQ(value, "BARasdf");
    ASSERT_TRUE(cpt->find("General", "Test3", value));
    ASSERT_EQ(value, "89");

    // Failed searches
    ASSERT_FALSE(cpt->find("Junk2", "test3", value));
    ASSERT_FALSE(cpt->find("Junk", "test4", value));
}

/**
 * Test that paths are increased and decreased according to the scope that
 * its SCS was created in (using CheckpointIn).
 */
TEST_F(CheckpointInFixture, SCSCptInPathScoped)
{
    Serializable::ScopedCheckpointSection scs(*(cpt.get()), "Section1");
    ASSERT_EQ(Serializable::currentSection(), "Section1");

    {
        Serializable::ScopedCheckpointSection scs_2(*(cpt.get()), "Section2");
        ASSERT_EQ(Serializable::currentSection(), "Section1.Section2");

        Serializable::ScopedCheckpointSection scs_3(*(cpt.get()), "Section3");
        ASSERT_EQ(Serializable::currentSection(),
            "Section1.Section2.Section3");
    }

    Serializable::ScopedCheckpointSection scs_4(*(cpt.get()), "Section4");
    ASSERT_EQ(Serializable::currentSection(), "Section1.Section4");
}

/**
 * Test that paths are increased and decreased according to the scope that
 * its SCS was created in (using CheckpointOut).
 */
TEST_F(SerializeFixture, SCSCptOutPathScoped)
{
    std::ofstream cpt(getCptPath());

    Serializable::ScopedCheckpointSection scs(cpt, "Section1");
    ASSERT_EQ(Serializable::currentSection(), "Section1");

    {
        Serializable::ScopedCheckpointSection scs_2(cpt, "Section2");
        ASSERT_EQ(Serializable::currentSection(), "Section1.Section2");

        Serializable::ScopedCheckpointSection scs_3(cpt, "Section3");
        ASSERT_EQ(Serializable::currentSection(),
            "Section1.Section2.Section3");
    }

    Serializable::ScopedCheckpointSection scs_4(cpt, "Section4");
    ASSERT_EQ(Serializable::currentSection(), "Section1.Section4");
}

/**
 * Make sure that a SCS that uses a CheckpointIn does not change the
 * checkpoint file's contents.
 */
TEST_F(SerializeFixture, SCSNoChangeCptIn)
{
    {
        // Create empty cpt file
        std::ofstream os(getCptPath());
        assert(os.good());
        os.close();
    }

    CheckpointIn cpt(getDirName());

    std::ifstream is(getCptPath());
    assert(is.good());
    ASSERT_EQ(is.peek(), std::ifstream::traits_type::eof());

    Serializable::ScopedCheckpointSection scs(cpt, "Section1");
    ASSERT_EQ(is.peek(), std::ifstream::traits_type::eof());
    ASSERT_EQ(Serializable::currentSection(), "Section1");
}

/** @return The ostream as a std::string. */
std::string
getString(std::istream &is)
{
    auto buf = is.rdbuf();
    std::ostringstream oss;
    oss << buf;
    return oss.str();
}

/**
 * Flushes the checkpoint and reads its contents.
 *
 * @param cpt The checkpoint to be flushed.
 * @param filename The name of the file to be read.
 * @return The contents in the filename, as a string.
 */
std::string
getContents(std::ofstream &cpt, std::string filename)
{
    // The checkpoint must be flushed, otherwise the file may not be up-
    // to-date and the assertions below will fail
    cpt.flush();

    std::ifstream is(filename);
    assert(is.good());
    return std::string(std::istreambuf_iterator<char>(is),
        std::istreambuf_iterator<char>());
}

/**
 * Make sure that a SCS that uses a CheckpointOut changes the checkpoint
 * file's contents (single section).
 */
TEST_F(SerializableFixture, SCSChangeCptOutSingle)
{
    Serializable::ScopedCheckpointSection scs(*cpt_out, "Section1");
    ASSERT_EQ(getContents(*cpt_out, getCptPath()),
        "\n[Section1]\n");
}

/**
 * Make sure that a SCS that uses a CheckpointOut changes the checkpoint
 * file's contents (multiple sections).
 */
TEST_F(SerializableFixture, SCSChangeCptOutMultiple)
{
    std::string expected = "";
    Serializable::ScopedCheckpointSection scs(*cpt_out, "Section1");
    expected += "\n[Section1]\n";
    ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

    {
        Serializable::ScopedCheckpointSection scs_2(*cpt_out, "Section2");
        expected += "\n[Section1.Section2]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        Serializable::ScopedCheckpointSection scs_3(*cpt_out, "Section3");
        expected += "\n[Section1.Section2.Section3]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
    }

    Serializable::ScopedCheckpointSection scs_4(*cpt_out, "Section4");
    expected += "\n[Section1.Section4]\n";
    ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
}

/**
 * Make sure that a SCS that uses a CheckpointOut changes the checkpoint
 * file's contents (large test).
 */
TEST_F(SerializableFixture, SCSChangeCptOutLarge)
{
    std::string expected = "";
    Serializable::ScopedCheckpointSection scs(*cpt_out, "Section1");
    expected += "\n[Section1]\n";
    ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

    {
        Serializable::ScopedCheckpointSection scs_2(*cpt_out, "Section2");
        expected += "\n[Section1.Section2]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        {
            Serializable::ScopedCheckpointSection scs_3(*cpt_out, "Section3");
            expected += "\n[Section1.Section2.Section3]\n";
            ASSERT_EQ(getContents(*cpt_out, getCptPath()),
            expected);
        }

        Serializable::ScopedCheckpointSection scs_4(*cpt_out, "Section4");
        expected += "\n[Section1.Section2.Section4]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
    }

    Serializable::ScopedCheckpointSection scs_5(*cpt_out, "Section5");
    expected += "\n[Section1.Section5]\n";
    ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

    {
        Serializable::ScopedCheckpointSection scs_6(*cpt_out, "Section6");
        expected += "\n[Section1.Section5.Section6]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        Serializable::ScopedCheckpointSection scs_7(*cpt_out, "Section7");
        expected += "\n[Section1.Section5.Section6.Section7]\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
    }

    Serializable::ScopedCheckpointSection scs_8(*cpt_out, "Section8");
    expected += "\n[Section1.Section5.Section8]\n";
    ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
}

/** Test failure to create dir. Try to create a dir in a non-existent dir. */
TEST(SerializableDeathTest, GenerateCptOutFail)
{
    std::ofstream cpt;
    const std::string dir_name =
        SerializeFixture::generateTempDirName();
    ASSERT_FALSE(dirExists(dir_name));

    ASSERT_ANY_THROW(Serializable::generateCheckpointOut(
        dir_name + "/b/a/n/a/n/a/", cpt));
}

/** Test successful CheckpointOut generation with non-existent dir. */
TEST_F(SerializeFixture, GenerateCptOut)
{
    // The fixture will auto-create the dir. Remove it.
    assert(rmdir(getDirName().c_str()) == 0);

    std::ofstream cpt;
    Serializable::generateCheckpointOut(getDirName(), cpt);
    ASSERT_TRUE(dirExists(getDirName()));

    // Make sure the checkpoint was properly created. Using EXPECT to
    // force removing the directory at the end
    EXPECT_NE(getContents(cpt, getCptPath()).find(
        "## checkpoint generated: "), std::string::npos);
}

/** Test successful CheckpointOut generation with existing dir. */
TEST_F(SerializeFixture, GenerateCptOutExistent)
{
    assert(dirExists(getDirName()));

    // Create the checkpoint and make sure it has been properly created by
    // deleting it and making sure the function was successful
    std::ofstream cpt;
    Serializable::generateCheckpointOut(getDirName(), cpt);
    EXPECT_TRUE(remove((getCptPath()).c_str()) == 0);
}

class SerializableType : public Serializable
{
  private:
    mutable bool _serialized = false;
    bool _unserialized = false;

  public:
    SerializableType() : Serializable() {}

    void serialize(CheckpointOut &cp) const override { _serialized = true; }
    void unserialize(CheckpointIn &cp) override { _unserialized = true; }

    /**
     * Checks if serialize() has been called and then marks it as not called.
     *
     * @return True if serialize() has been called.
     */
    bool
    checkAndResetSerialized()
    {
        const bool serialized = _serialized;
        _serialized = false;
        return serialized;
    }

    /**
     * Checks if unserialize() has been called and then marks it as not called.
     *
     * @return True if unserialize() has been called.
     */
    bool
    checkAndResetUnserialized()
    {
        const bool unserialized = _unserialized;
        _unserialized = false;
        return unserialized;
    }
};

/**
 * Test section serialization and unserialization for an object without
 * serializable contents. Since how (un)serialize() works is independent of
 * the Serializable class, we just make sure that the respective functions
 * are called when calling (un)serializeSection().
 */
TEST_F(SerializableFixture, SectionSerializationSimple)
{
    SerializableType serializable;

    // Serialization
    {
        serializable.serializeSection(*cpt_out, "Section1");
        ASSERT_EQ(getContents(*cpt_out, getCptPath()),
            "\n[Section1]\n");
        ASSERT_TRUE(serializable.checkAndResetSerialized());
        ASSERT_FALSE(serializable.checkAndResetUnserialized());

        serializable.serializeSection(*cpt_out, "Section2");
        ASSERT_EQ(getContents(*cpt_out, getCptPath()),
            "\n[Section1]\n\n[Section2]\n");
        ASSERT_TRUE(serializable.checkAndResetSerialized());
        ASSERT_FALSE(serializable.checkAndResetUnserialized());

        serializable.serializeSection(*cpt_out, "Section3");
        ASSERT_EQ(getContents(*cpt_out, getCptPath()),
            "\n[Section1]\n\n[Section2]\n\n[Section3]\n");
        ASSERT_TRUE(serializable.checkAndResetSerialized());
        ASSERT_FALSE(serializable.checkAndResetUnserialized());
    }

    // Unserialization. Since the object has no serializable contents,
    // just make sure it does not throw for existent and non-existent
    // sections
    {
        ASSERT_NO_THROW(serializable.unserializeSection(*cpt_in, "Section1"));
        ASSERT_FALSE(serializable.checkAndResetSerialized());
        ASSERT_TRUE(serializable.checkAndResetUnserialized());

        ASSERT_NO_THROW(serializable.unserializeSection(*cpt_in, "Section2"));
        ASSERT_FALSE(serializable.checkAndResetSerialized());
        ASSERT_TRUE(serializable.checkAndResetUnserialized());

        ASSERT_NO_THROW(serializable.unserializeSection(*cpt_in, "Section3"));
        ASSERT_FALSE(serializable.checkAndResetSerialized());
        ASSERT_TRUE(serializable.checkAndResetUnserialized());

        ASSERT_NO_THROW(serializable.unserializeSection(*cpt_in, "Section4"));
        ASSERT_FALSE(serializable.checkAndResetSerialized());
        ASSERT_TRUE(serializable.checkAndResetUnserialized());
    }
}

/**
 * Test that paramIn called on a param that does not exist triggers an error.
 */
TEST_F(SerializableFixtureDeathTest, ParamIn)
{
    int unserialized_integer;
    Serializable::ScopedCheckpointSection scs(*cpt_in, "Section1");
    ASSERT_ANY_THROW(paramIn(*cpt_in, "Param1", unserialized_integer));
}

/**
 * Test serialization followed by unserialization, using ParamInImpl and
 * ParamOut.
 */
TEST_F(SerializableFixture, ParamOutIn)
{
    const int integer = 5;
    const double real = 3.7;
    const bool boolean = true;
    const std::string str = "string test";
    const char character = 'c';

    // Serialization
    {
        Serializable::ScopedCheckpointSection scs(*cpt_out, "Section1");
        std::string expected = "\n[Section1]\n";

        paramOut(*cpt_out, "Param1", integer);
        expected += "Param1=5\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        paramOut(*cpt_out, "Param2", real);
        expected += "Param2=3.7\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        paramOut(*cpt_out, "Param3", boolean);
        expected += "Param3=true\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        paramOut(*cpt_out, "Param4", str);
        expected += "Param4=string test\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        paramOut(*cpt_out, "Param5", character);
        expected += "Param5=99\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integer;
        double unserialized_real;
        bool unserialized_boolean;
        std::string unserialized_str;
        char unserialized_character;

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        paramIn(cpt, "Param1", unserialized_integer);
        ASSERT_EQ(integer, unserialized_integer);

        paramIn(cpt, "Param2", unserialized_real);
        ASSERT_EQ(real, unserialized_real);

        paramIn(cpt, "Param3", unserialized_boolean);
        ASSERT_EQ(boolean, unserialized_boolean);

        paramIn(cpt, "Param4", unserialized_str);
        ASSERT_EQ(str, unserialized_str);

        paramIn(cpt, "Param5", unserialized_character);
        ASSERT_EQ(character, unserialized_character);
    }
}

/**
 * Test serialization followed by unserialization, using ParamInImpl and
 * ParamOut, when multiple sections exist.
 */
TEST_F(SerializableFixture, ParamOutInMultipleSections)
{
    const int integer = 5;
    const double real = 3.7;
    const bool boolean = true;
    const std::string str = "string test";
    const char character = 'c';

    // Serialization
    {
        Serializable::ScopedCheckpointSection scs(*cpt_out, "Section1");
        std::string expected = "\n[Section1]\n";

        paramOut(*cpt_out, "Param1", integer);
        expected += "Param1=5\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        paramOut(*cpt_out, "Param2", real);
        expected += "Param2=3.7\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        {
            Serializable::ScopedCheckpointSection scs_2(*cpt_out, "Section2");
            expected += "\n[Section1.Section2]\n";

            paramOut(*cpt_out, "Param3", boolean);
            expected += "Param3=true\n";
            ASSERT_EQ(getContents(*cpt_out, getCptPath()),
                expected);
        }

        // Possibly unexpected behavior: Since scs_2 has gone out of scope
        // the user may expect that we'd go back to Section1; however, this
        // is not the case, and Param4 is added to Section1.Section2
        paramOut(*cpt_out, "Param4", str);
        expected += "Param4=string test\n";
        ASSERT_EQ(getContents(*cpt_out, getCptPath()), expected);

        {
            Serializable::ScopedCheckpointSection scs_3(*cpt_out, "Section3");
            expected += "\n[Section1.Section3]\n";

            Serializable::ScopedCheckpointSection scs_4(*cpt_out, "Section4");
            expected += "\n[Section1.Section3.Section4]\n";

            paramOut(*cpt_out, "Param5", character);
            expected += "Param5=99\n";
            ASSERT_EQ(getContents(*cpt_out, getCptPath()),
                expected);
        }
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integer;
        double unserialized_real;
        bool unserialized_boolean;
        std::string unserialized_str;
        char unserialized_character;

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        paramIn(cpt, "Param1", unserialized_integer);
        ASSERT_EQ(integer, unserialized_integer);

        paramIn(cpt, "Param2", unserialized_real);
        ASSERT_EQ(real, unserialized_real);

        {
            Serializable::ScopedCheckpointSection scs_2(cpt, "Section2");

            paramIn(cpt, "Param3", unserialized_boolean);
            ASSERT_EQ(boolean, unserialized_boolean);

            // Due to the reason mentioned above on serialization, this param
            // must be extracted within the scope of Section 2
            paramIn(cpt, "Param4", unserialized_str);
            ASSERT_EQ(str, unserialized_str);
        }

        {
            Serializable::ScopedCheckpointSection scs_3(cpt, "Section3");
            Serializable::ScopedCheckpointSection scs_4(cpt, "Section4");

            paramIn(cpt, "Param5", unserialized_character);
            ASSERT_EQ(character, unserialized_character);
        }
    }
}

/** Test optional parameters. */
TEST_F(SerializeFixture, OptParamOutIn)
{
    const double real = 3.7;
    const std::string str = "string test";

    // Serialization
    {
        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        paramOut(cpt, "Param2", real);
        paramOut(cpt, "Param4", str);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integer;
        double unserialized_real;
        bool unserialized_boolean;
        std::string unserialized_str;

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        // Optional without warning
        gtestLogOutput.str("");
        ASSERT_FALSE(optParamIn(cpt, "Param1", unserialized_integer, false));
        ASSERT_EQ(gtestLogOutput.str(), "");

        gtestLogOutput.str("");
        ASSERT_TRUE(optParamIn(cpt, "Param2", unserialized_real, false));
        ASSERT_EQ(real, unserialized_real);
        ASSERT_EQ(gtestLogOutput.str(), "");

        // Optional with default request for warning
        gtestLogOutput.str("");
        ASSERT_FALSE(optParamIn(cpt, "Param3", unserialized_boolean));
        ASSERT_THAT(gtestLogOutput.str(),
            ::testing::HasSubstr("warn: optional parameter Section1:Param3 "
            "not present\n"));

        gtestLogOutput.str("");
        ASSERT_TRUE(optParamIn(cpt, "Param4", unserialized_str));
        ASSERT_EQ(str, unserialized_str);
        ASSERT_EQ(gtestLogOutput.str(), "");

        // Explicit request for warning
        gtestLogOutput.str("");
        ASSERT_FALSE(optParamIn(cpt, "Param5", unserialized_boolean, true));
        ASSERT_THAT(gtestLogOutput.str(),
            ::testing::HasSubstr("warn: optional parameter Section1:Param5 "
            "not present\n"));
    }
}

/** Check for death when there is no section and paramIn is requested. */
TEST_F(SerializableFixtureDeathTest, NoSectionParamIn)
{
#ifndef NDEBUG
    GTEST_SKIP() << "Skipping as assertions are "
        "stripped out of fast builds";
#endif

    std::ofstream of(getCptPath());
    CheckpointIn cpt(getDirName());
    int unserialized_integer;
    ASSERT_DEATH(paramIn(cpt, "Param1", unserialized_integer), "");
}

/** Test arrayParamOut and arrayParamIn. */
TEST_F(SerializeFixture, ArrayParamOutIn)
{
    const int integer[] = {5, 10, 15};
    std::array<double, 4> real = {0.1, 1.345, 892.72, 1e+10};
    std::list<bool> boolean = {true, false};
    std::vector<std::string> str = {"a", "string", "test"};
    std::set<uint64_t> uint64 = {12751928501, 13, 111111};
    std::deque<uint8_t> uint8 = {17, 42, 255};

    // Serialization
    {
        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        std::string expected = "\n[Section1]\n";

        arrayParamOut(cpt, "Param1", integer);
        expected += "Param1=5 10 15\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        arrayParamOut(cpt, "Param2", real);
        expected += "Param2=0.1 1.345 892.72 1e+10\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        arrayParamOut(cpt, "Param3", boolean);
        expected += "Param3=true false\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        arrayParamOut(cpt, "Param4", str);
        expected += "Param4=a string test\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        arrayParamOut(cpt, "Param5", uint64);
        expected += "Param5=13 111111 12751928501\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        arrayParamOut(cpt, "Param6", uint8);
        expected += "Param6=17 42 255\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integer[3];
        std::array<double, 4> unserialized_real;
        std::list<bool> unserialized_boolean;
        std::vector<std::string> unserialized_str;
        std::set<uint64_t> unserialized_uint64;
        std::deque<uint8_t> unserialized_uint8;

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        arrayParamIn(cpt, "Param1", unserialized_integer, 3);
        ASSERT_THAT(unserialized_integer, testing::ElementsAre(5, 10, 15));

        arrayParamIn(cpt, "Param2", unserialized_real.data(),
            unserialized_real.size());
        ASSERT_EQ(real, unserialized_real);

        arrayParamIn(cpt, "Param3", unserialized_boolean);
        ASSERT_EQ(boolean, unserialized_boolean);

        arrayParamIn(cpt, "Param4", unserialized_str);
        ASSERT_EQ(str, unserialized_str);

        arrayParamIn(cpt, "Param5", unserialized_uint64);
        ASSERT_EQ(uint64, unserialized_uint64);

        arrayParamIn(cpt, "Param6", unserialized_uint8);
        ASSERT_EQ(uint8, unserialized_uint8);
    }
}

/**
 * Test arrayParamOut and arrayParamIn for strings with spaces.
 * @todo This is broken because spaces are delimiters between array
 * entries; so, strings containing spaces are seen as multiple entries
*/
TEST_F(SerializeFixture, DISABLED_ArrayParamOutInSpacedStrings)
{
    std::vector<std::string> str = {"a string test", "for", "array param out"};

    // Serialization
    {
        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        std::string expected = "\n[Section1]\n";

        arrayParamOut(cpt, "Param1", str);
        expected += "Param1=string test for array param out\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        std::vector<std::string> unserialized_str;

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        arrayParamIn(cpt, "Param1", unserialized_str);
        ASSERT_EQ(str, unserialized_str);
    }
}

/**
 * Test that using arrayParamIn with sizes smaller than the container's
 * throws an exception.
 */
TEST_F(SerializeFixtureDeathTest, ArrayParamOutInSmaller)
{
    // Serialization
    {
        const int integer[] = {5, 10, 15};

        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        std::string expected = "\n[Section1]\n";

        arrayParamOut(cpt, "Param1", integer);
        expected += "Param1=5 10 15\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integer[1];

        Serializable::ScopedCheckpointSection scs(cpt, "Section1");

        ASSERT_ANY_THROW(arrayParamIn(cpt, "Param1", unserialized_integer, 2));
    }
}

/** Test mappingParamOut and mappingParamIn with all keys. */
TEST_F(SerializeFixture, MappingParamOutIn)
{
    const int integers[] = {10, 32, 100};
    std::array<double, 4> reals = {0.1, 1.345, 892.72, 1e+10};
    const char* const names_ints[] = {"ten", "thirty-two", "one hundred"};
    const char* const names_reals[] = {"first", "second", "third", "fourth"};

    // Serialization
    {
        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        std::string expected = "\n[Section1]\n";

        mappingParamOut(cpt, "Integers", names_ints, integers, 3);
        expected += "\n[Section1.Integers]\nten=10\nthirty-two=32\n"
            "one hundred=100\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        mappingParamOut(cpt, "Reals", names_reals, reals.data(), reals.size());
        expected += "\n[Section1.Reals]\nfirst=0.1\nsecond=1.345\n"
            "third=892.72\nfourth=1e+10\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cpt(getDirName());

        int unserialized_integers[3];
        std::array<double, 4> unserialized_reals;

        mappingParamIn(cpt, "Section1.Integers", names_ints,
            unserialized_integers, 3);
        ASSERT_THAT(unserialized_integers, testing::ElementsAre(10, 32, 100));

        mappingParamIn(cpt, "Section1.Reals", names_reals,
            unserialized_reals.data(), unserialized_reals.size());
        ASSERT_EQ(unserialized_reals, reals);
    }
}

/** Test that missing keys are ignored on mappingParamIn. */
TEST_F(SerializeFixture, MappingParamOutInMissing)
{
    const int integers[] = {10, 32, 100};
    std::array<double, 4> reals = {0.1, 1.345, 892.72, 1e+10};

    // Serialization
    {
        const char* const names_ints[] = {"ten", "thirty-two", "one hundred"};
        const char* const names_reals[] =
            {"first", "second", "third", "fourth"};

        std::ofstream cpt(getCptPath());
        Serializable::ScopedCheckpointSection scs(cpt, "Section1");
        std::string expected = "\n[Section1]\n";

        mappingParamOut(cpt, "Integers", names_ints, integers, 3);
        expected += "\n[Section1.Integers]\nten=10\nthirty-two=32\n"
            "one hundred=100\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);

        mappingParamOut(cpt, "Reals", names_reals, reals.data(), reals.size());
        expected += "\n[Section1.Reals]\nfirst=0.1\nsecond=1.345\n"
            "third=892.72\nfourth=1e+10\n";
        ASSERT_EQ(getContents(cpt, getCptPath()), expected);
    }

    // Unserialization
    {
        const char* const names_ints[] = {"one hundred"};
        const char* const names_reals[] = {"first", "third"};
        std::array<double, 2> expected_reals = {0.1, 892.72};

        CheckpointIn cpt(getDirName());

        std::string err;
        int unserialized_integers[1];
        std::array<double, 2> unserialized_reals;

        gtestLogOutput.str("");
        mappingParamIn(cpt, "Section1.Integers", names_ints,
            unserialized_integers, 1);
        ASSERT_THAT(unserialized_integers, testing::ElementsAre(100));
        err = gtestLogOutput.str();
        ASSERT_THAT(err, ::testing::HasSubstr("warn: unknown entry found in "
            "checkpoint: Section1.Integers thirty-two 32\n"));
        ASSERT_THAT(err, ::testing::HasSubstr("warn: unknown entry found in "
            "checkpoint: Section1.Integers ten 10\n"));

        gtestLogOutput.str("");
        mappingParamIn(cpt, "Section1.Reals", names_reals,
            unserialized_reals.data(), unserialized_reals.size());
        ASSERT_EQ(unserialized_reals, expected_reals);
        err = gtestLogOutput.str();
        ASSERT_THAT(err, ::testing::HasSubstr("warn: unknown entry found in "
            "checkpoint: Section1.Reals fourth 1e+10\n"));
        ASSERT_THAT(err, ::testing::HasSubstr("warn: unknown entry found in "
            "checkpoint: Section1.Reals second 1.345\n"));
    }
}

/**
 * Test serialization followed by unserialization, using SERIALIZE_SCALAR and
 * UNSERIALIZE_SCALAR.
 */
TEST_F(SerializeFixture, SerializeScalar)
{
    const int expected_integer = 5;
    const double expected_real = 3.7;
    const bool expected_boolean = true;
    const std::string expected_str = "string test";

    // Serialization
    {
        const int integer = expected_integer;
        const double real = expected_real;
        const bool boolean = expected_boolean;
        const std::string str = expected_str;

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        std::string expected = "\n[Section1]\n";

        SERIALIZE_SCALAR(integer);
        expected += "integer=5\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_SCALAR(real);
        expected += "real=3.7\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_SCALAR(boolean);
        expected += "boolean=true\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_SCALAR(str);
        expected += "str=string test\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        int integer;
        double real;
        bool boolean;
        std::string str;

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        UNSERIALIZE_SCALAR(integer);
        ASSERT_EQ(integer, expected_integer);

        UNSERIALIZE_SCALAR(real);
        ASSERT_EQ(real, expected_real);

        UNSERIALIZE_SCALAR(boolean);
        ASSERT_EQ(boolean, expected_boolean);

        UNSERIALIZE_SCALAR(str);
        ASSERT_EQ(str, expected_str);
    }
}

/** Test optional parameters with UNSERIALIZE_OPT_SCALAR. */
TEST_F(SerializeFixture, UnserializeOptScalar)
{
    const double expected_real = 3.7;
    const std::string expected_str = "string test";

    // Serialization
    {
        const double real = expected_real;
        const std::string str = expected_str;

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        SERIALIZE_SCALAR(real);
        SERIALIZE_SCALAR(str);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        int integer;
        double real;
        bool boolean;
        std::string str;

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        // Optional without warning
        ASSERT_FALSE(UNSERIALIZE_OPT_SCALAR(integer));

        ASSERT_TRUE(UNSERIALIZE_OPT_SCALAR(real));
        ASSERT_EQ(real, expected_real);

        // Optional with default request for warning
        ASSERT_FALSE(UNSERIALIZE_OPT_SCALAR(boolean));

        ASSERT_TRUE(UNSERIALIZE_OPT_SCALAR(str));
        ASSERT_EQ(str, expected_str);
    }
}

/**
 * Test serialization followed by unserialization, using SERIALIZE_ENUM and
 * UNSERIALIZE_ENUM.
 */
TEST_F(SerializeFixture, SerializeEnum)
{
    enum Number
    {
        ZERO,
        TEN=10,
        THIRTY_TWO=32
    };
    const Number expected_val = ZERO;
    const Number expected_val_2 = THIRTY_TWO;

    // Serialization
    {
        const Number zero = expected_val;
        const Number thirty_two = expected_val_2;

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        std::string expected = "\n[Section1]\n";

        SERIALIZE_ENUM(zero);
        expected += "zero=0\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_ENUM(thirty_two);
        expected += "thirty_two=32\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        Number zero;
        Number thirty_two;

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        UNSERIALIZE_ENUM(zero);
        ASSERT_EQ(zero, expected_val);

        UNSERIALIZE_ENUM(thirty_two);
        ASSERT_EQ(thirty_two, expected_val_2);
    }
}

/** Test SERIALIZE_ARRAY and UNSERIALIZE_ARRAY. */
TEST_F(SerializeFixture, SerializeArray)
{
    std::array<double, 4> expected_real = {0.1, 1.345, 892.72, 1e+10};

    // Serialization
    {
        const int integer[] = {5, 10, 15};
        const double *real = expected_real.data();

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        std::string expected = "\n[Section1]\n";

        SERIALIZE_ARRAY(integer, 3);
        expected += "integer=5 10 15\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_ARRAY(real, expected_real.size());
        expected += "real=0.1 1.345 892.72 1e+10\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        int integer[3];
        std::array<double, 4> real_array;
        double *real = real_array.data();

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        UNSERIALIZE_ARRAY(integer, 3);
        ASSERT_THAT(integer, testing::ElementsAre(5, 10, 15));

        UNSERIALIZE_ARRAY(real, expected_real.size());
        ASSERT_EQ(real_array, expected_real);
    }
}

/** Test SERIALIZE_CONTAINER and UNSERIALIZE_CONTAINER. */
TEST_F(SerializeFixture, SerializeContainer)
{
    std::list<bool> expected_boolean = {true, false};
    std::vector<std::string> expected_str = {"a", "string", "test"};
    std::set<uint64_t> expected_uint64 = {12751928501, 13, 111111};
    std::deque<uint8_t> expected_uint8 = {17, 42, 255};

    // Serialization
    {
        const std::list<bool> boolean = expected_boolean;
        const std::vector<std::string> str = expected_str;
        const std::set<uint64_t> uint64 = expected_uint64;
        const std::deque<uint8_t> uint8 = expected_uint8;

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        std::string expected = "\n[Section1]\n";

        SERIALIZE_CONTAINER(boolean);
        expected += "boolean=true false\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_CONTAINER(str);
        expected += "str=a string test\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_CONTAINER(uint64);
        expected += "uint64=13 111111 12751928501\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_CONTAINER(uint8);
        expected += "uint8=17 42 255\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        std::list<bool> boolean;
        std::vector<std::string> str;
        std::set<uint64_t> uint64;
        std::deque<uint8_t> uint8;

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        UNSERIALIZE_CONTAINER(boolean);
        ASSERT_EQ(boolean, expected_boolean);

        UNSERIALIZE_CONTAINER(str);
        ASSERT_EQ(str, expected_str);

        UNSERIALIZE_CONTAINER(uint64);
        ASSERT_EQ(uint64, expected_uint64);

        UNSERIALIZE_CONTAINER(uint8);
        ASSERT_EQ(uint8, expected_uint8);
    }
}

/** Test SERIALIZE_MAPPING and UNSERIALIZE_MAPPING with all keys. */
TEST_F(SerializeFixture, SerializeMapping)
{
    const int expected_integers[] = {10, 32, 100};
    std::array<double, 4> expected_reals = {0.1, 1.345, 892.72, 1e+10};
    const char* const names_ints[] = {"ten", "thirty-two", "one hundred"};
    const char* const names_reals[] = {"first", "second", "third", "fourth"};

    // Serialization
    {
        const int *integers = expected_integers;
        const double *reals = expected_reals.data();

        std::ofstream cp(getCptPath());
        Serializable::ScopedCheckpointSection scs(cp, "Section1");
        std::string expected = "\n[Section1]\n";

        SERIALIZE_MAPPING(integers, names_ints, 3);
        expected += "\n[Section1.integers]\nten=10\nthirty-two=32\n"
            "one hundred=100\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);

        SERIALIZE_MAPPING(reals, names_reals, expected_reals.size());
        expected += "\n[Section1.reals]\nfirst=0.1\nsecond=1.345\n"
            "third=892.72\nfourth=1e+10\n";
        ASSERT_EQ(getContents(cp, getCptPath()), expected);
    }

    // Unserialization
    {
        CheckpointIn cp(getDirName());

        int integers[3];
        double reals[4];

        Serializable::ScopedCheckpointSection scs(cp, "Section1");

        UNSERIALIZE_MAPPING(integers, names_ints, 3);
        ASSERT_THAT(integers, testing::ElementsAre(10, 32, 100));

        UNSERIALIZE_MAPPING(reals, names_reals, expected_reals.size());
        ASSERT_THAT(reals, testing::ElementsAre(0.1, 1.345, 892.72, 1e+10));
    }
}
