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

uint64_t test_ns_delay;
uint64_t test_code;

void
test_m5_fail(uint64_t ns_delay, uint64_t code)
{
    test_ns_delay = ns_delay;
    test_code = code;
}

DispatchTable dt = { .m5_fail = &test_m5_fail };

bool
run(std::initializer_list<std::string> arg_args)
{
    Args args(arg_args);
    return Command::run(dt, args);
}

TEST(Fail, Arguments)
{
    // Called with no arguments.
    test_ns_delay = 50;
    test_code = 40;
    EXPECT_FALSE(run({ "fail" }));

    // Called with one argument.
    test_ns_delay = 50;
    test_code = 40;
    EXPECT_TRUE(run({ "fail", "1" }));
    EXPECT_EQ(test_ns_delay, 0);
    EXPECT_EQ(test_code, 1);

    // Called with two arguments.
    test_ns_delay = 50;
    test_code = 40;
    EXPECT_TRUE(run({ "fail", "1", "2" }));
    EXPECT_EQ(test_ns_delay, 2);
    EXPECT_EQ(test_code, 1);

    // Called with three arguments.
    EXPECT_FALSE(run({ "fail", "1", "2", "3" }));
}
