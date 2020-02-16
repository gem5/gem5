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
 */

#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>

#include <utility>

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
 * Test calculating saturation percentile.
 */
TEST(SatCounterTest, SaturationPercentile)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter(bits);

    ASSERT_FALSE(counter.isSaturated());
    for (double value = 0.0; value <= max_value; value++, counter++) {
        const double saturation = value / max_value;
        ASSERT_DOUBLE_EQ(counter.calcSaturation(), saturation);
    }
    ASSERT_TRUE(counter.isSaturated());
}

/**
 * Test abrupt saturation.
 */
TEST(SatCounterTest, Saturate)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter(bits);
    counter++;
    ASSERT_FALSE(counter.isSaturated());

    // Make sure the value added is what was missing to saturate
    const unsigned diff = counter.saturate();
    ASSERT_EQ(diff, max_value - 1);
    ASSERT_TRUE(counter.isSaturated());
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
 * Test shift operators.
 */
TEST(SatCounterTest, Shift)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    const unsigned initial_value = 1;
    SatCounter counter(bits, initial_value);
    SatCounter other(bits, initial_value);
    // The saturated shift value is just enough to saturate, since greater
    // values could generate undefined behavior
    SatCounter saturated_counter(bits, bits);
    int value = initial_value;

    // Test random shifts
    counter <<= 2;
    value <<= 2;
    ASSERT_EQ(counter, value);
    counter >>= 1;
    value >>= 1;
    ASSERT_EQ(counter, value);

    // Test saturation
    counter <<= bits;
    ASSERT_EQ(counter, max_value);

    // Test zeroing
    counter >>= bits;
    ASSERT_EQ(counter, 0);

    // Test saturation against other saturating counter
    counter.reset();
    value = initial_value;
    counter <<= other;
    value <<= other;
    ASSERT_EQ(counter, value);
    counter <<= saturated_counter;
    value = max_value;
    ASSERT_EQ(counter, max_value);

    // Test zeroing against other saturating counter
    counter >>= other;
    value >>= other;
    ASSERT_EQ(counter, value);
    counter >>= saturated_counter;
    ASSERT_EQ(counter, 0);

    // Make sure the counters cannot be shifted by negative numbers, since
    // that is undefined behaviour
    ASSERT_DEATH(counter >>= -1, "");
    ASSERT_DEATH(counter <<= -1, "");
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

/**
 * Test copy and move for both constructor and assignment.
 */
TEST(SatCounterTest, CopyMove)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    const unsigned initial_value = 1;
    SatCounter counter(bits, initial_value);
    SatCounter deep_copy(1);
    SatCounter counter_copy(2);

    // Increase counter value so that we can check if the inner counter is
    // being copied
    counter++;

    // Copy counter using both the copy constructor and the copy assignment
    SatCounter counter_copy_constructor(counter);
    deep_copy = counter_copy = counter;
    ASSERT_EQ(counter_copy_constructor, initial_value + 1);
    ASSERT_EQ(counter_copy, initial_value + 1);
    ASSERT_EQ(deep_copy, initial_value + 1);

    // Make sure max value is the same for all of them, and that modifying
    // the copies does not modify the original
    for (int i = 0; i < 2*max_value; i++) {
        counter_copy_constructor++;
        counter_copy++;
        deep_copy++;
    }
    ASSERT_EQ(counter, initial_value + 1);
    ASSERT_EQ(counter_copy_constructor, max_value);
    ASSERT_EQ(counter_copy, max_value);
    ASSERT_EQ(deep_copy, max_value);

    // Make sure initial value is the same for all of them
    counter_copy_constructor.reset();
    counter_copy.reset();
    deep_copy.reset();
    ASSERT_EQ(counter_copy_constructor, initial_value);
    ASSERT_EQ(counter_copy, initial_value);
    ASSERT_EQ(deep_copy, initial_value);

    // Now check move
    SatCounter counter_move_constructor(std::move(counter));
    ASSERT_EQ(counter, 0);
    ASSERT_EQ(counter_move_constructor, initial_value + 1);

    SatCounter counter_move(bits);
    counter_move = std::move(counter_move_constructor);
    ASSERT_EQ(counter_move_constructor, 0);
    ASSERT_EQ(counter_move, initial_value + 1);
}

/**
 * Test add-assignment and subtract assignment.
 */
TEST(SatCounterTest, AddSubAssignment)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter(bits);
    SatCounter other(bits, 2);
    SatCounter saturated_counter(bits, max_value);
    int value = 0;

    // Test add-assignment for a few random values and then saturate
    counter += 2;
    value += 2;
    ASSERT_EQ(counter, value);
    counter += 3;
    value += 3;
    ASSERT_EQ(counter, value);
    counter += max_value;
    value = max_value;
    ASSERT_EQ(counter, value);

    // Test subtract-assignment for a few random values until back to zero
    counter -= 2;
    value -= 2;
    ASSERT_EQ(counter, value);
    counter -= 3;
    value -= 3;
    ASSERT_EQ(counter, value);
    counter -= max_value;
    value = 0;
    ASSERT_EQ(counter, value);

    // Test add-assignment of other saturating counter
    counter += other;
    value += other;
    ASSERT_EQ(counter, value);
    counter += saturated_counter;
    value = max_value;
    ASSERT_EQ(counter, saturated_counter);

    // Test subtract-assignment of other saturating counter
    counter -= other;
    value -= other;
    ASSERT_EQ(counter, value);
    counter -= saturated_counter;
    ASSERT_EQ(counter, 0);
}

/**
 * Test add-assignment and subtract assignment using negative numbers.
 */
TEST(SatCounterTest, NegativeAddSubAssignment)
{
    const unsigned bits = 3;
    const unsigned max_value = (1 << bits) - 1;
    SatCounter counter(bits, max_value);
    int value = max_value;

    // Test add-assignment for a few negative values until zero is reached
    counter += -2;
    value += -2;
    ASSERT_EQ(counter, value);
    counter += -3;
    value += -3;
    ASSERT_EQ(counter, value);
    counter += (int)-max_value;
    value = 0;
    ASSERT_EQ(counter, value);

    // Test subtract-assignment for a few negative values until saturation
    counter -= -2;
    value -= -2;
    ASSERT_EQ(counter, value);
    counter -= -3;
    value -= -3;
    ASSERT_EQ(counter, value);
    counter -= (int)-max_value;
    value = max_value;
    ASSERT_EQ(counter, value);
}

