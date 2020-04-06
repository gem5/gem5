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

#include <cstring>

#include "args.hh"

TEST(ArgsTest, Stoi)
{
    // Successful conversion of a decimal number.
    uint64_t val = 0;
    EXPECT_TRUE(Args::stoi("7", val));
    EXPECT_EQ(val, 7);

    // Successful conversion of a hex number.
    val = 0;
    EXPECT_TRUE(Args::stoi("0xf", val));
    EXPECT_EQ(val, 15);

    // Successful conversion with a default.
    val = 0;
    EXPECT_TRUE(Args::stoi("8", val, 9));
    EXPECT_EQ(val, 8);

    // Failed conversion.
    val = 0;
    EXPECT_FALSE(Args::stoi("not a number", val));

    // Failed conversion with a default.
    val = 0;
    EXPECT_FALSE(Args::stoi("not a number", val, 4));
    EXPECT_EQ(val, 4);
}

TEST(ArgsTest, Pack)
{
    uint64_t regs[32];
    std::memset(regs, 0, sizeof(regs));

    // Too long a string.
    EXPECT_FALSE(Args::pack("Test", regs, 0));
    // Verify that regs was not written to.
    EXPECT_EQ(regs[0], 0);

    // Success with one register.
    EXPECT_TRUE(Args::pack("Test", regs, 1));
    EXPECT_EQ(regs[0], ('T' << 0) | ('e' << 8) | ('s' << 16) | ('t' << 24));

    // Success with two registers.
    EXPECT_TRUE(Args::pack("A longer string", regs, 2));
    EXPECT_EQ(regs[0], ((uint64_t)'A' << 0) | ((uint64_t)' ' << 8) |
                       ((uint64_t)'l' << 16) | ((uint64_t)'o' << 24) |
                       ((uint64_t)'n' << 32) | ((uint64_t)'g' << 40) |
                       ((uint64_t)'e' << 48) | ((uint64_t)'r' << 56));
    EXPECT_EQ(regs[1], ((uint64_t)' ' << 0) | ((uint64_t)'s' << 8) |
                       ((uint64_t)'t' << 16) | ((uint64_t)'r' << 24) |
                       ((uint64_t)'i' << 32) | ((uint64_t)'n' << 40) |
                       ((uint64_t)'g' << 48));

    // Success with exactly the right number of characters.
    EXPECT_TRUE(Args::pack("12345678", regs, 1));
    EXPECT_EQ(regs[0], ((uint64_t)'1' << 0) | ((uint64_t)'2' << 8) |
                       ((uint64_t)'3' << 16) | ((uint64_t)'4' << 24) |
                       ((uint64_t)'5' << 32) | ((uint64_t)'6' << 40) |
                       ((uint64_t)'7' << 48) | ((uint64_t)'8' << 56));

    // Failure with exactly one too many characters.
    EXPECT_FALSE(Args::pack("123456789", regs, 1));
    EXPECT_EQ(regs[0], 0);
}

TEST(ArgsTest, Pop)
{
    const char *test_argv[] = { "arg0", "0x1", "2" };
    const int test_argc = sizeof(test_argv) / sizeof(test_argv[0]);

    uint64_t val = 0;

    Args args(test_argc, test_argv);
    const Args reset_args = args;
    EXPECT_EQ(args.size(), test_argc);

    for (int i = 0; i < test_argc; i++)
        EXPECT_EQ(args[i], test_argv[i]);

    // Initializer list constructor with no elements.
    args = Args({});
    EXPECT_EQ(args.size(), 0);

    // Initializer list with a few elements.
    args = Args({ "arg0", "0x1", "arg2" });
    EXPECT_EQ(args.size(), 3);

    // Pop as an integer and fail with no default.
    args = reset_args;
    val = 0;
    EXPECT_FALSE(args.pop(val));
    EXPECT_EQ(args.size(), test_argc);

    // Pop as an integer and fail with a default.
    args = reset_args;
    val = 0;
    EXPECT_FALSE(args.pop(val, 5));
    EXPECT_EQ(val, 5);

    // Pop as a string successfully.
    EXPECT_EQ(args.pop(), test_argv[0]);
    EXPECT_EQ(args.size(), test_argc - 1);

    // Pop as a string successfully with a default.
    args = reset_args;
    EXPECT_EQ(args.pop("something else"), "arg0");
    EXPECT_EQ(args.size(), test_argc - 1);

    // Pop as an integer and succeed.
    val = 0;
    EXPECT_TRUE(args.pop(val));
    EXPECT_EQ(val, 1);
    EXPECT_EQ(args.size(), test_argc - 2);

    // Pop as an integer and succeed with a default.
    val = 0;
    EXPECT_TRUE(args.pop(val, 5));
    EXPECT_EQ(val, 2);
    EXPECT_EQ(args.size(), test_argc - 3);

    Args empty({});

    // Pop from an empty list.
    EXPECT_EQ(empty.size(), 0);
    EXPECT_EQ(empty.pop("the default"), "the default");
    EXPECT_EQ(empty.size(), 0);

    // Pop an integer from an empty list.
    val = 0;
    EXPECT_FALSE(empty.pop(val));
    EXPECT_EQ(empty.size(), 0);

    // Pop an integer from an empty list with a default.
    val = 0;
    EXPECT_TRUE(empty.pop(val, 5));
    EXPECT_EQ(val, 5);
    EXPECT_EQ(empty.size(), 0);

    Args short_args({ "short" });
    Args long_args({ "A really long argument that won't fit." });
    uint64_t regs[1];

    // Pop into a list of registers and succeed.
    EXPECT_EQ(short_args.size(), 1);
    EXPECT_TRUE(short_args.pop(regs, 1));
    EXPECT_EQ(short_args.size(), 0);
    EXPECT_EQ(regs[0], ((uint64_t)'s' << 0) | ((uint64_t)'h' << 8) |
                       ((uint64_t)'o' << 16) | ((uint64_t)'r' << 24) |
                       ((uint64_t)'t' << 32));

    // Pop into a list of register and fail.
    EXPECT_EQ(long_args.size(), 1);
    EXPECT_FALSE(long_args.pop(regs, 1));
    EXPECT_EQ(long_args.size(), 1);
}
