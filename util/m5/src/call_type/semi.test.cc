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

#include <csetjmp>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "args.hh"
#include "call_type.hh"
#include "call_type/verify_semi.hh"
#include "dispatch_table.hh"

class DefaultCallType : public CallType
{
  private:
    DispatchTable dt;

  public:
    DefaultCallType() : CallType("default") {}

    bool init_called = false;
    void init() override { init_called = true; }

    bool isDefault() const override { return true; }
    void printDesc(std::ostream &os) const override {}
    const DispatchTable &getDispatch() const override { return dt; }
};

DefaultCallType default_call_type;

TEST(SemiCallType, Detect)
{
    CallType *ct;

    // Semi should not be selected if there are no arguments.
    Args empty({});
    default_call_type.init_called = false;
    ct = CallType::detect(empty);
    EXPECT_EQ(ct, &default_call_type);
    EXPECT_TRUE(default_call_type.init_called);

    // Inst should not be selected if --semi isn't the first argument.
    Args one_arg({"one"});
    default_call_type.init_called = false;
    ct = CallType::detect(one_arg);
    EXPECT_EQ(ct, &default_call_type);
    EXPECT_TRUE(default_call_type.init_called);

    // Semi should be selected if --semi is the first argument.
    Args selected({"--semi"});
    default_call_type.init_called = false;
    ct = CallType::detect(selected);
    EXPECT_NE(ct, &default_call_type);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(default_call_type.init_called);

    Args extra({"--semi", "foo"});
    default_call_type.init_called = false;
    ct = CallType::detect(extra);
    EXPECT_NE(ct, &default_call_type);
    EXPECT_NE(ct, nullptr);
    EXPECT_FALSE(default_call_type.init_called);

    // Semi should not be selected if --semi isn't first.
    Args not_first({"foo", "--semi"});
    default_call_type.init_called = false;
    ct = CallType::detect(not_first);
    EXPECT_EQ(ct, &default_call_type);
    EXPECT_TRUE(default_call_type.init_called);
}

sigjmp_buf intercept_env;
siginfo_t intercept_siginfo;

void
sigill_handler(int sig, siginfo_t *info, void *ucontext)
{
    std::memcpy(&intercept_siginfo, info, sizeof(intercept_siginfo));
    siglongjmp(intercept_env, 1);
}

TEST(SemiCallType, Sum)
{
    // Get the semi call type, which is in an anonymous namespace.
    Args args({"--semi"});
    CallType *semi_call_type = CallType::detect(args);
    EXPECT_NE(semi_call_type, nullptr);

    // Get the dispatch table associated with it.
    const auto &dt = semi_call_type->getDispatch();

    // Determine if we're running within gem5 by checking whether a flag is
    // set in the environment.
    bool in_gem5 = (std::getenv("RUNNING_IN_GEM5") != nullptr);
    if (in_gem5)
        std::cout << "In gem5, m5 ops should work." << std::endl;
    else
        std::cout << "Not in gem5, m5 ops won't work." << std::endl;

    // If it is, then we should be able to run the "sum" command.
    if (in_gem5) {
        EXPECT_EQ((*dt.m5_sum)(2, 2, 0, 0, 0, 0), 4);
        return;
    }

    // If not, then we'll need to try to catch the fall out from trying to run
    // an m5 op and verify that what we were trying looks correct.

    struct sigaction sigill_action;
    std::memset(&sigill_action, 0, sizeof(sigill_action));
    sigill_action.sa_sigaction = &sigill_handler;
    sigill_action.sa_flags = SA_SIGINFO | SA_RESETHAND;

    struct sigaction old_sigill_action;

    sigaction(SIGILL, &sigill_action, &old_sigill_action);

    if (!sigsetjmp(intercept_env, 1)) {
        (*dt.m5_sum)(2, 2, 0, 0, 0, 0);
        sigaction(SIGILL, &old_sigill_action, nullptr);
        ADD_FAILURE() << "Didn't die when attempting to run \"sum\".";
        return;
    }

    // Back from siglongjump.
    auto &info = intercept_siginfo;

    EXPECT_EQ(info.si_signo, SIGILL);
    EXPECT_TRUE(info.si_code == ILL_ILLOPC || info.si_code == ILL_ILLOPN);

    abi_verify_semi(info, M5OP_SUM, {2, 2, 0, 0, 0, 0});
}
