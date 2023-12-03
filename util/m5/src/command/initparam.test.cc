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

#include "args.hh"
#include "command.hh"
#include "dispatch_table.hh"

uint64_t test_key_str1;
uint64_t test_key_str2;
uint64_t test_result;

uint64_t
test_m5_init_param(uint64_t key_str1, uint64_t key_str2)
{
    test_key_str1 = key_str1;
    test_key_str2 = key_str2;

    return test_result;
}

DispatchTable dt = { .m5_init_param = &test_m5_init_param };

std::string cout_output;

bool
run(std::initializer_list<std::string> arg_args)
{
    Args args(arg_args);

    // Redirect cout into a stringstream.
    std::stringstream buffer;
    std::streambuf *orig = std::cout.rdbuf(buffer.rdbuf());

    bool res = Command::run(dt, args);

    // Capture the contents of the stringstream and restore cout.
    cout_output = buffer.str();
    std::cout.rdbuf(orig);

    return res;
}

TEST(Fail, Arguments)
{
    // Called with no arguments.
    test_key_str1 = 0;
    test_key_str2 = 0;
    EXPECT_FALSE(run({ "initparam" }));
    EXPECT_EQ(cout_output, "");

    // Called with an empty argument.
    test_key_str1 = 1;
    test_key_str2 = 1;
    test_result = 5;
    EXPECT_TRUE(run({ "initparam", "" }));
    EXPECT_EQ(test_key_str1, 0);
    EXPECT_EQ(test_key_str2, 0);
    EXPECT_EQ(cout_output, "5");

    // Called with a short argument.
    test_key_str1 = 1;
    test_key_str2 = 1;
    test_result = 4;
    EXPECT_TRUE(run({ "initparam", "shrt" }));
    EXPECT_EQ(test_key_str1, ((uint64_t)'s' << 0) | ((uint64_t)'h' << 8) |
                                 ((uint64_t)'r' << 16) |
                                 ((uint64_t)'t' << 24));
    EXPECT_EQ(test_key_str2, 0);
    EXPECT_EQ(cout_output, "4");

    // Call with a longer argument.
    test_key_str1 = 1;
    test_key_str2 = 1;
    test_result = 3;
    EXPECT_TRUE(run({ "initparam", "longer arg" }));
    EXPECT_EQ(test_key_str1,
              ((uint64_t)'l' << 0) | ((uint64_t)'o' << 8) |
                  ((uint64_t)'n' << 16) | ((uint64_t)'g' << 24) |
                  ((uint64_t)'e' << 32) | ((uint64_t)'r' << 40) |
                  ((uint64_t)' ' << 48) | ((uint64_t)'a' << 56));
    EXPECT_EQ(test_key_str2, ((uint64_t)'r' << 0) | ((uint64_t)'g' << 8));
    EXPECT_EQ(cout_output, "3");

    // Call with an almost too long argument.
    test_key_str1 = 1;
    test_key_str2 = 1;
    test_result = 2;
    EXPECT_TRUE(run({ "initparam", "1234567887654321" }));
    EXPECT_EQ(test_key_str1,
              ((uint64_t)'1' << 0) | ((uint64_t)'2' << 8) |
                  ((uint64_t)'3' << 16) | ((uint64_t)'4' << 24) |
                  ((uint64_t)'5' << 32) | ((uint64_t)'6' << 40) |
                  ((uint64_t)'7' << 48) | ((uint64_t)'8' << 56));
    EXPECT_EQ(test_key_str2,
              ((uint64_t)'8' << 0) | ((uint64_t)'7' << 8) |
                  ((uint64_t)'6' << 16) | ((uint64_t)'5' << 24) |
                  ((uint64_t)'4' << 32) | ((uint64_t)'3' << 40) |
                  ((uint64_t)'2' << 48) | ((uint64_t)'1' << 56));
    EXPECT_EQ(cout_output, "2");

    // Call with an argument that is too long.
    EXPECT_FALSE(run({ "initparam", "12345678876543210" }));
    EXPECT_EQ(cout_output, "");

    // Call with a valid argument and then one extra.
    EXPECT_FALSE(run({ "valid", "extra" }));
    EXPECT_EQ(cout_output, "");
}
