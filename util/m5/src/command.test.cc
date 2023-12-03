/*
 * Copyright 2020 Google Inc.
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

// For EXPECT_THAT and HasSubstr
#include <gmock/gmock.h>

#include "args.hh"
#include "command.hh"

bool ran_test1 = false;

// A dummy class so we can make a reference to pass around.
class DispatchTable
{
};

DispatchTable dt;

bool
do_test1(const DispatchTable &dt, Args &args)
{
    ran_test1 = true;
    return true;
}

bool ran_test2 = false;

bool
do_test2(const DispatchTable &dt, Args &args)
{
    ran_test2 = true;
    return true;
}

TEST(CommandTest, OneCommandNoArgs)
{
    Command test1("test1", 0, 0, do_test1, "");

    // Try to run the command with an extra argument, expecting it to fail.
    EXPECT_FALSE(ran_test1);
    Args args1({ "test1", "extra" });
    EXPECT_FALSE(Command::run(dt, args1));
    EXPECT_FALSE(ran_test1);

    // Try to run with an unrecognized command.
    Args args2({ "bad_command" });
    EXPECT_FALSE(Command::run(dt, args2));
    EXPECT_FALSE(ran_test1);

    // Run with the right name and number of arguments.
    Args args3({ "test1" });
    EXPECT_TRUE(Command::run(dt, args3));
    EXPECT_TRUE(ran_test1);
    ran_test1 = false;

    // Try with no command at all.
    Args args4({});
    EXPECT_FALSE(Command::run(dt, args4));
    EXPECT_FALSE(ran_test1);
}

TEST(CommandTest, OneCommandSomeArgs)
{
    Command test1("test1", 2, 3, do_test1, "");

    // Too few arguments.
    EXPECT_FALSE(ran_test1);
    Args args1({ "test1" });
    EXPECT_FALSE(Command::run(dt, args1));
    EXPECT_FALSE(ran_test1);

    // Too many arguments.
    Args args2({ "test1", "arg1", "arg2", "arg3", "arg4" });
    EXPECT_FALSE(Command::run(dt, args2));
    EXPECT_FALSE(ran_test1);

    // Just enough arguments.
    Args args3({ "test1", "arg1", "arg2" });
    EXPECT_TRUE(Command::run(dt, args3));
    EXPECT_TRUE(ran_test1);
    ran_test1 = false;

    // Almost too many arguments.
    Args args4({ "test1", "arg1", "arg2", "arg3" });
    EXPECT_TRUE(Command::run(dt, args4));
    EXPECT_TRUE(ran_test1);
    ran_test1 = false;
}

TEST(CommandTest, TwoCommands)
{
    Command test1("test1", 0, 0, do_test1, "");
    Command test2("test2", 1, 1, do_test2, "");

    // Try a bad command name.
    Args args1({ "bad_command" });
    EXPECT_FALSE(Command::run(dt, args1));
    EXPECT_FALSE(ran_test1);
    EXPECT_FALSE(ran_test2);

    // Try the right command with the wrong number of arguments.
    Args args2({ "test1", "arg1" });
    EXPECT_FALSE(Command::run(dt, args2));
    EXPECT_FALSE(ran_test1);
    EXPECT_FALSE(ran_test2);

    // Run the first command.
    Args args3({ "test1" });
    EXPECT_TRUE(Command::run(dt, args3));
    EXPECT_TRUE(ran_test1);
    EXPECT_FALSE(ran_test2);
    ran_test1 = ran_test2 = false;

    // Run the second command.
    Args args4({ "test2", "arg1" });
    EXPECT_TRUE(Command::run(dt, args4));
    EXPECT_FALSE(ran_test1);
    EXPECT_TRUE(ran_test2);
    ran_test1 = ran_test2 = false;
}

TEST(CommandTest, Usage)
{
    std::string name1 = "test1";
    std::string usage1 = "first test usage string";
    std::string name2 = "test2";
    std::string usage2 = "second test usage string";

    Command test1(name1, 0, 0, do_test1, usage1);
    Command test2(name2, 0, 0, do_test2, usage2);

    auto summary = Command::usageSummary();

    EXPECT_THAT(summary, testing::HasSubstr(name1));
    EXPECT_THAT(summary, testing::HasSubstr(usage1));

    EXPECT_THAT(summary, testing::HasSubstr(name2));
    EXPECT_THAT(summary, testing::HasSubstr(usage2));
}
