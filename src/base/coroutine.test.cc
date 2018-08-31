/*
 * Copyright (c) 2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Giacomo Travaglini
 */

#include <gtest/gtest.h>

#include "base/coroutine.hh"

using namespace m5;

/**
 * This test is checking if the Coroutine, once it's created
 * it doesn't start since the second argument of the constructor
 * (run_coroutine) is set to false
 */
TEST(Coroutine, Unstarted)
{
    auto yielding_task =
    [] (Coroutine<void, void>::CallerType& yield)
    {
        yield();
    };

    const bool start_upon_creation = false;
    Coroutine<void, void> coro(yielding_task, start_upon_creation);

    ASSERT_FALSE(coro.started());
}

/**
 * This test is checking if the Coroutine, once it yields
 * back to the caller, it is still marked as not finished.
 */
TEST(Coroutine, Unfinished)
{
    auto yielding_task =
    [] (Coroutine<void, void>::CallerType& yield)
    {
        yield();
    };

    Coroutine<void, void> coro(yielding_task);
    ASSERT_TRUE(coro);
}

/**
 * This test is checking the parameter passing interface of a
 * coroutine which takes an integer as an argument.
 * Coroutine::operator() and CallerType::get() are the tested
 * APIS.
 */
TEST(Coroutine, Passing)
{
    const std::vector<int> input{ 1, 2, 3 };
    const std::vector<int> expected_values = input;

    auto passing_task =
    [&expected_values] (Coroutine<int, void>::CallerType& yield)
    {
        int argument;

        for (const auto expected : expected_values) {
            argument = yield.get();
            ASSERT_EQ(argument, expected);
        }
    };

    Coroutine<int, void> coro(passing_task);
    ASSERT_TRUE(coro);

    for (const auto val : input) {
        coro(val);
    }
}

/**
 * This test is checking the yielding interface of a coroutine
 * which takes no argument and returns integers.
 * Coroutine::get() and CallerType::operator() are the tested
 * APIS.
 */
TEST(Coroutine, Returning)
{
    const std::vector<int> output{ 1, 2, 3 };
    const std::vector<int> expected_values = output;

    auto returning_task =
    [&output] (Coroutine<void, int>::CallerType& yield)
    {
        for (const auto ret : output) {
            yield(ret);
        }
    };

    Coroutine<void, int> coro(returning_task);
    ASSERT_TRUE(coro);

    for (const auto expected : expected_values) {
        int returned = coro.get();
        ASSERT_EQ(returned, expected);
    }
}

/**
 * This test is still supposed to test the returning interface
 * of the the Coroutine, proving how coroutine can be used
 * for generators.
 * The coroutine is computing the first #steps of the fibonacci
 * sequence and it is yielding back results one number per time.
 */
TEST(Coroutine, Fibonacci)
{
    const std::vector<int> expected_values{
        1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233 };

    const int steps = expected_values.size();

    auto fibonacci_task =
    [steps] (Coroutine<void, int>::CallerType& yield)
    {
        int prev = 0;
        int current = 1;

        for (auto iter = 0; iter < steps; iter++) {
            int sum = prev + current;
            yield(sum);

            prev = current;
            current = sum;
        }
    };

    Coroutine<void, int> coro(fibonacci_task);
    ASSERT_TRUE(coro);

    for (const auto expected : expected_values) {
        ASSERT_TRUE(coro);
        int returned = coro.get();
        ASSERT_EQ(returned, expected);
    }
}

/**
 * This test is using a bi-channel coroutine (accepting and
 * yielding values) for testing a cooperative task.
 * The caller and the coroutine have a string each; they are
 * composing a new string by merging the strings together one
 * character per time.
 * The result string is hence passed back and forth between the
 * coroutine and the caller.
 */
TEST(Coroutine, Cooperative)
{
    const std::string caller_str("HloWrd");
    const std::string coro_str("el ol!");
    const std::string expected("Hello World!");

    auto cooperative_task =
    [&coro_str] (Coroutine<std::string, std::string>::CallerType& yield)
    {
        for (auto& appended_c : coro_str) {
            auto old_str = yield.get();
            yield(old_str + appended_c);
        }
    };

    Coroutine<std::string, std::string> coro(cooperative_task);

    std::string result;
    for (auto& c : caller_str) {
        ASSERT_TRUE(coro);
        result += c;
        result = coro(result).get();
    }

    ASSERT_EQ(result, expected);
}

/**
 * This test is testing nested coroutines by using one inner and one
 * outer coroutine. It basically ensures that yielding from the inner
 * coroutine returns to the outer coroutine (mid-layer of execution) and
 * not to the outer caller.
 */
TEST(Coroutine, Nested)
{
    const std::string wrong("Inner");
    const std::string expected("Inner + Outer");

    auto inner_task =
    [] (Coroutine<void, std::string>::CallerType& yield)
    {
        std::string inner_string("Inner");
        yield(inner_string);
    };

    auto outer_task =
    [&inner_task] (Coroutine<void, std::string>::CallerType& yield)
    {
        Coroutine<void, std::string> coro(inner_task);
        std::string inner_string = coro.get();

        std::string outer_string("Outer");
        yield(inner_string + " + " + outer_string);
    };


    Coroutine<void, std::string> coro(outer_task);
    ASSERT_TRUE(coro);

    std::string result = coro.get();

    ASSERT_NE(result, wrong);
    ASSERT_EQ(result, expected);
}

/**
 * This test is stressing the scenario where two distinct fibers are
 * calling the same coroutine.  First the test instantiates (and runs) a
 * coroutine, then spawns another one and it passes it a reference to
 * the first coroutine. Once the new coroutine calls the first coroutine
 * and the first coroutine yields, we are expecting execution flow to
 * be yielded to the second caller (the second coroutine) and not the
 * original caller (the test itself)
 */
TEST(Coroutine, TwoCallers)
{
    bool valid_return = false;

    Coroutine<void, void> callee{[]
        (Coroutine<void, void>::CallerType& yield)
    {
        yield();
        yield();
    }};

    Coroutine<void, void> other_caller{[&callee, &valid_return]
        (Coroutine<void, void>::CallerType& yield)
    {
        callee();
        valid_return = true;
        yield();
    }};

    ASSERT_TRUE(valid_return);
}
