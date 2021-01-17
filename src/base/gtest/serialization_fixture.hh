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

#include <gtest/gtest.h>

#include <cassert>
#include <cerrno>
#include <fstream>
#include <string>

#include "base/compiler.hh"
#include "sim/serialize.hh"

namespace gem5
{

/**
 * Fixture class that handles temporary directory creation. These temporary
 * directories are used by the tests in this file, in order to avoid that
 * a failed test will not remove its directory, causing future runs to fail.
 * This has been tailored for checkpoints, so it expects that the directory
 * may contain a cpt file on removal.
 *
 * @todo Ideally the checkpoints should not be necessarily using files, and
 *       stringstreams would be used instead to avoid overhead in the tests.
 */
class SerializationFixture : public ::testing::Test
{
  private:
    /**
     * Temporary directory names are generated based on this number, which
     * is updated every time the generator function is called.
     */
    static unsigned dirNumber;

    /** The name of the temporary directory. */
    std::string dirName;

  public:
    using ::testing::Test::Test;

    /** Generate a temporary directory name. */
    static std::string
    generateTempDirName()
    {
        return "/tmp/temp_dir_test" + std::to_string(dirNumber++) + "/";
    }

    /** Get the name of the directory we have created on SetUp. */
    std::string getDirName() const { return dirName; }

    /** Get the path to the checkpoint file. */
    std::string
    getCptPath() const
    {
        return getDirName() + std::string(CheckpointIn::baseFilename);
    }

    /**
     * Create a cpt file with the contents specified by the string. This
     * function should be used when testing unserialization, since it
     * simulates a previous serialization.
     */
    void
    simulateSerialization(std::string contents) const
    {
        std::ofstream cp(getCptPath());
        cp << contents;
        cp.close();
    }

    void
    SetUp() override
    {
        // Create the directory
        dirName = generateTempDirName();
        M5_VAR_USED int success = mkdir(dirName.c_str(), 0775);
        assert(!(success == -1 && errno != EEXIST));
    }

    void
    TearDown() override
    {
        // There may be a cpt file inside, so try to remove it; otherwise,
        // rmdir does not work
        std::remove(getCptPath().c_str());
        // Remove the directory we created on SetUp
        M5_VAR_USED int success = rmdir(dirName.c_str());
        assert(success == 0);
    }
};
unsigned SerializationFixture::dirNumber = 0;

} // anonymous namespace
