/*
 * Copyright (c) 2018 ARM Limited
 * All rights reserved
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 */

#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "base/inifile.hh"

using namespace std;

namespace {

std::istringstream iniFile(R"ini_file(
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
)ini_file");

};

TEST(Initest, MatchFound)
{
    IniFile simConfigDB;
    simConfigDB.load(iniFile);

    std::string value;

    auto ret = simConfigDB.find("General", "Test2", value);
    ASSERT_TRUE(ret);
    ASSERT_STREQ(value.c_str(), "bar");

    ret = simConfigDB.find("Junk", "Test3", value);
    ASSERT_TRUE(ret);
    ASSERT_STREQ(value.c_str(), "yo");

    ret = simConfigDB.find("Junk", "Test4", value);
    ASSERT_TRUE(ret);
    ASSERT_STREQ(value.c_str(), "mama mia");

    ret = simConfigDB.find("General", "Test1", value);
    ASSERT_TRUE(ret);
    ASSERT_STREQ(value.c_str(), "BARasdf");

    ret = simConfigDB.find("General", "Test3", value);
    ASSERT_TRUE(ret);
    ASSERT_STREQ(value.c_str(), "89");
}

TEST(Initest, MatchNotFound)
{
    IniFile simConfigDB;
    simConfigDB.load(iniFile);

    std::string value;

    auto ret = simConfigDB.find("Junk2", "test3", value);
    ASSERT_FALSE(ret);

    ret = simConfigDB.find("Junk", "test4", value);
    ASSERT_FALSE(ret);
}
