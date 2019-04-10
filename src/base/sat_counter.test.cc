/*
 * Copyright (c) 2019 Inria
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
 *
 * Authors: Daniel Carvalho
 */

#include <gtest/gtest.h>

#include "base/sat_counter.hh"

/**
 * Test if the maximum value is indeed the maximum value reachable.
 */
TEST(SatCounterTest, MaximumValue)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter(bits);

    for (int i = 0; i < 2*max_value; i++) {
        counter++;
    }

    ASSERT_EQ(counter, max_value);
}

/**
 * Test if the minimum value is indeed the mimimum value reachable.
 */
TEST(SatCounterTest, MinimumValue)
{
    const unsigned bits = 3;
    SatCounter counter(bits);

    for (int i = 0; i < 2; i++) {
        counter--;
    }

    ASSERT_EQ(counter, 0);
}

/**
 * Test initializing the counter with a value, updating it and then resetting.
 */
TEST(SatCounterTest, InitialValue)
{
    const unsigned bits = 3;
    const unsigned initial_value = 4;
    SatCounter counter(bits, initial_value);
    ASSERT_EQ(counter, initial_value);
    counter++;
    counter.reset();
    ASSERT_EQ(counter, initial_value);
}

/**
 * Test back and forth against an int.
 */
TEST(SatCounterTest, IntComparison)
{
    const unsigned bits = 3;
    SatCounter counter(bits);
    int value = 0;

    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter++, value++);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter--, value--);
    ASSERT_EQ(counter, 0);
}

/**
 * Test both pre and post operators.
 */
TEST(SatCounterTest, PrePostOperators)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter_pre(bits);
    SatCounter counter_post(bits);

    for (int i = 0; i < 2*max_value; i++) {
        counter_post++;
        SatCounter value_pre = ++counter_pre;
        ASSERT_EQ(counter_post, value_pre);
    }

    ASSERT_EQ(counter_pre, max_value);
    ASSERT_EQ(counter_post, max_value);

    for (int i = 0; i < 2*max_value; i++) {
        counter_post--;
        SatCounter value_pre = --counter_pre;
        ASSERT_EQ(counter_post, value_pre);
    }

    ASSERT_EQ(counter_pre, 0);
    ASSERT_EQ(counter_post, 0);
}
