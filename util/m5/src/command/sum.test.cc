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

#include <sstream>

#include "args.hh"
#include "command.hh"
#include "dispatch_table.hh"

unsigned test_a, test_b, test_c, test_d, test_e, test_f;
unsigned test_result;

unsigned
test_m5_sum(unsigned a, unsigned b, unsigned c, unsigned d, unsigned e,
            unsigned f)
{
    test_a = a;
    test_b = b;
    test_c = c;
    test_d = d;
    test_e = e;
    test_f = f;

    return test_result;
}

void
check_args(unsigned a, unsigned b, unsigned c, unsigned d, unsigned e,
           unsigned f)
{
    EXPECT_EQ(test_a, a);
    EXPECT_EQ(test_b, b);
    EXPECT_EQ(test_c, c);
    EXPECT_EQ(test_d, d);
    EXPECT_EQ(test_e, e);
    EXPECT_EQ(test_f, f);
}

DispatchTable dt = { .m5_sum = &test_m5_sum };

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

TEST(Sum, Arguments)
{
    // Called with no arguments.
    EXPECT_FALSE(run({ "sum" }));

    // Called with one argument.
    EXPECT_FALSE(run({ "sum", "1" }));

    // Called with two arguments.
    test_result = 42;
    EXPECT_TRUE(run({ "sum", "1", "2" }));
    check_args(1, 2, 0, 0, 0, 0);
    EXPECT_EQ(cout_output, "Sum is 42.\n");

    // Call with all arguments.
    test_result = 314159;
    EXPECT_TRUE(run({ "sum", "6", "5", "4", "3", "2", "1" }));
    check_args(6, 5, 4, 3, 2, 1);
    EXPECT_EQ(cout_output, "Sum is 314159.\n");
}
