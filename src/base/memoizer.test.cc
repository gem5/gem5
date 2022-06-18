/*
 * Copyright (c) 2022 Arm Limited
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
 */

#include <gtest/gtest.h>

#include "base/memoizer.hh"

using namespace gem5;

namespace
{

uint32_t
fibonacci(uint32_t n)
{
    if (n == 0) return 0;
    if (n == 1) return 1;

    return fibonacci(n-1) + fibonacci(n-2);
}

using FibonacciMemoizer = decltype(Memoizer(fibonacci));

class FibonacciMemoizerFixture : public FibonacciMemoizer,
                                 public ::testing::Test
{
  public:
    FibonacciMemoizerFixture()
      : FibonacciMemoizer(fibonacci)
    {}

};

}

/**
 * Testing result cache before and after a memoized call
 */
TEST_F(FibonacciMemoizerFixture, Uncached)
{
    const auto res10 = fibonacci(10);

    // Fresh memoizer, input = 10 shouldn't be present
    ASSERT_FALSE(cached(10));

    // We are now memoizing the result and making sure
    // it provides the same value
    EXPECT_EQ((*this)(10), res10);

    // Now the fibonacci output for input = 10 should be cached
    ASSERT_TRUE(cached(10));
}

/**
 * Just checking memoization works for multiple values
 */
TEST_F(FibonacciMemoizerFixture, MultipleValues)
{
    const auto res0 = fibonacci(0);
    const auto res10 = fibonacci(10);
    const auto res20 = fibonacci(20);

    EXPECT_EQ((*this)(0), res0);
    EXPECT_EQ((*this)(10), res10);
    EXPECT_EQ((*this)(20), res20);

    EXPECT_EQ(cacheSize(), 3);

    EXPECT_TRUE(cached(0));
    EXPECT_TRUE(cached(10));
    EXPECT_TRUE(cached(20));

    // fibonacci(30) shouldn't be cached
    EXPECT_FALSE(cached(30));
}

/**
 * Testing the Memoizer::flush method
 */
TEST_F(FibonacciMemoizerFixture, CacheFlush)
{
    const auto res10 = fibonacci(10);

    ASSERT_EQ(cacheSize(), 0);

    // Memoizing fibonacci(10)
    EXPECT_EQ((*this)(10), res10);
    ASSERT_EQ(cacheSize(), 1);

    // Flushing the cache
    flush();

    // Cache should be empty now
    ASSERT_EQ(cacheSize(), 0);
}
