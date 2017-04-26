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

#include "base/circular_queue.hh"

/** Testing that once instantiated with a fixed size,
 * the queue is still empty */
TEST(CircularQueueTest, Empty)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    ASSERT_EQ(cq.capacity(), cq_size);
    ASSERT_EQ(cq.size(), 0);
    ASSERT_TRUE(cq.empty());
}

/** Testing that once instantiated with a fixed size,
 * the queue has Head = Tail + 1 */
TEST(CircularQueueTest, HeadTailEmpty)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);
    ASSERT_EQ(cq.head(), cq.tail() + 1);
}

/** Adding elements to the circular queue.
 * Once an element has been added we test the new value
 * of front() and back() (head an tail). Since we are just
 * adding elements and not removing them, we expect the front
 * value to be fixed and the back value to change, matching
 * the latest pushed value.*/
TEST(CircularQueueTest, AddingElements)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    const auto first_element = 0xAAAAAAAA;
    cq.push_back(first_element);
    ASSERT_EQ(cq.front(), first_element);
    ASSERT_EQ(cq.back(), first_element);

    const auto second_element = 0x55555555;
    cq.push_back(second_element);
    ASSERT_EQ(cq.front(), first_element);
    ASSERT_EQ(cq.back(), second_element);

    ASSERT_EQ(cq.size(), 2);
}

/** Removing elements from the circular queue.
 * We add two elements and we consequently remove them.
 * After removing them we check that the elements have been
 * effectively removed, which means the circular queue is
 * empty */
TEST(CircularQueueTest, RemovingElements)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    // Adding first element
    const auto first_element = 0xAAAAAAAA;
    cq.push_back(first_element);

    // Adding second element
    const auto second_element = 0x55555555;
    cq.push_back(second_element);

    auto initial_head = cq.head();
    auto initial_tail = cq.tail();

    // Removing first and second element
    cq.pop_front();
    ASSERT_EQ(cq.head(), initial_head + 1);
    ASSERT_EQ(cq.tail(), initial_tail);

    cq.pop_front();
    ASSERT_EQ(cq.head(), initial_head + 2);
    ASSERT_EQ(cq.tail(), initial_tail);

    ASSERT_EQ(cq.size(), 0);
    ASSERT_TRUE(cq.empty());
}

/** Testing CircularQueue::full
 * This tests adds elements to the queue and checks that it is full,
 * which means:
 *    - CircularQueue::full == true
 *    - Head = Tail + 1
 */
TEST(CircularQueueTest, Full)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    const auto value = 0xAAAAAAAA;
    for (auto idx = 0; idx < cq_size; idx++) {
        cq.push_back(value);
    }

    ASSERT_TRUE(cq.full());
    ASSERT_EQ(cq.head(), cq.tail() + 1);
}

/** Testing CircularQueue::begin(), CircularQueue::end()
 * This tests the following:
 *     - In an empty queue, begin() == end()
 *     - After pushing some elements in the queue, the begin()
 *       and end() iterators are correctly misaligned
 */
TEST(CircularQueueTest, BeginEnd)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    // Begin/End are the same (empty)
    ASSERT_EQ(cq.begin(), cq.end());

    const auto first_value = 0xAAAAAAAA;
    const auto second_value = 0x55555555;

    cq.push_back(first_value);
    cq.push_back(second_value);

    // End = Begin + 2
    ASSERT_EQ(cq.begin() + 2, cq.end());
}

/** Testing that begin() and end() (-1) iterators
 * actually point to the correct values
 * so that dereferencing them leads to a match with the
 * values of (front() and back())
 */
TEST(CircularQueueTest, BeginFrontEndBack)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    const auto front_value = 0xAAAAAAAA;
    const auto back_value = 0x55555555;

    cq.push_back(front_value);
    cq.push_back(back_value);

    ASSERT_EQ(*(cq.begin()), cq.front());
    ASSERT_EQ(*(cq.end() - 1), cq.back());
}

/** Testing circular queue iterators:
 * By allocating two iterators to a queue we test several
 * operators.
 */
TEST(CircularQueueTest, IteratorsOp)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    const auto first_value = 0xAAAAAAAA;
    const auto second_value = 0x55555555;
    cq.push_back(first_value);
    cq.push_back(second_value);

    auto it_1 = cq.begin();
    auto it_2 = cq.begin() + 1;

    // Operators test
    ASSERT_TRUE(it_1 != it_2);
    ASSERT_FALSE(it_1 == it_2);
    ASSERT_FALSE(it_1 > it_2);
    ASSERT_FALSE(it_1 >= it_2);
    ASSERT_TRUE(it_1 < it_2);
    ASSERT_TRUE(it_1 <= it_2);
    ASSERT_EQ(*it_1, first_value);
    ASSERT_EQ(it_1 + 1, it_2);
    ASSERT_EQ(it_1, it_2 - 1);
    ASSERT_EQ(it_2 - it_1, 1);

    auto temp_it = it_1;
    ASSERT_EQ(++temp_it, it_2);
    ASSERT_EQ(--temp_it, it_1);
    ASSERT_EQ(temp_it++, it_1);
    ASSERT_EQ(temp_it, it_2);
    ASSERT_EQ(temp_it--, it_2);
    ASSERT_EQ(temp_it, it_1);
}

/**
 * Testing a full loop, which is incrementing one iterator until
 * it wraps and has the same index as the starting iterator.
 * This test checks that even if they have the same index, they are
 * not the same iterator since they have different round.
 */
TEST(CircularQueueTest, FullLoop)
{
    const auto cq_size = 8;
    CircularQueue<uint32_t> cq(cq_size);

    // ending_it does a full loop and points at the same
    // index as starting_it but with a different round
    auto starting_it = cq.begin();
    auto ending_it = starting_it + cq_size;

    ASSERT_EQ(starting_it._idx, ending_it._idx);
    ASSERT_TRUE(starting_it != ending_it);
}
