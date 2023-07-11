/*
 * Copyright (c) 2015, 2018 ARM Limited
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vector>

#include "base/circlebuf.hh"

using testing::ElementsAreArray;
using namespace gem5;

const char data[] = {
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
    0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
};

// A better way to implement this would be with std::span, but that is only
// available starting in c++20.
template <typename T>
std::vector<T>
subArr(T *arr, int size, int offset=0)
{
    return std::vector<T>(arr + offset, arr + offset + size);
}

// Basic non-overflow functionality
TEST(CircleBufTest, BasicReadWriteNoOverflow)
{
    CircleBuf<char> buf(8);
    char foo[16];

    // Write empty buffer, no overflow
    buf.write(data, 8);
    EXPECT_EQ(buf.size(), 8);
    buf.peek(foo, 8);
    EXPECT_THAT(subArr(foo, 8), ElementsAreArray(data, 8));

    // Read 2
    buf.read(foo, 2);
    EXPECT_THAT(subArr(foo, 2), ElementsAreArray(data, 2));
    EXPECT_EQ(buf.size(), 6);
    buf.read(foo, 6);
    EXPECT_THAT(subArr(foo, 6), ElementsAreArray(data + 2, 6));
    EXPECT_EQ(buf.size(), 0);
}

// Basic single write overflow functionality
TEST(CircleBufTest, SingleWriteOverflow)
{
    CircleBuf<char> buf(8);
    char foo[16];

    buf.write(data, 16);
    EXPECT_EQ(buf.size(), 8);
    buf.peek(foo, 8);
    EXPECT_THAT(subArr(foo, 8), ElementsAreArray(data + 8, 8));
}


// Multi-write overflow functionality
TEST(CircleBufTest, MultiWriteOverflow)
{
    CircleBuf<char> buf(8);
    char foo[16];

    // Write, no overflow, write overflow
    buf.write(data, 6);
    buf.write(data + 8, 6);
    EXPECT_EQ(buf.size(), 8);
    buf.peek(foo, 8);
    EXPECT_THAT(subArr(foo, 2), ElementsAreArray(data + 4, 2));
    EXPECT_THAT(subArr(foo, 6, 2), ElementsAreArray(data + 8, 6));
}

// Pointer wrap around
TEST(CircleBufTest, PointerWrapAround)
{
    CircleBuf<char> buf(8);
    char foo[16];

    // _start == 0, _stop = 8
    buf.write(data, 8);
    // _start == 4, _stop = 8
    buf.read(foo, 4);
    // _start == 4, _stop = 12
    buf.write(data + 8, 4);
    EXPECT_EQ(buf.size(), 8);
    // _start == 10, _stop = 12
    // Normalized: _start == 2, _stop = 4
    buf.read(foo + 4, 6);
    EXPECT_EQ(buf.size(), 2);
    EXPECT_THAT(subArr(foo, 10), ElementsAreArray(data, 10));
    // Normalized: _start == 4, _stop = 4
    buf.read(foo + 10, 2);
    EXPECT_EQ(buf.size(), 0);
    EXPECT_THAT(subArr(foo, 12), ElementsAreArray(data, 12));
}

// Consume after produce empties queue
TEST(CircleBufTest, ProduceConsumeEmpty)
{
    CircleBuf<char> buf(8);
    char foo[1] = {'a'};

    // buf is empty to begin with.
    EXPECT_TRUE(buf.empty());
    // Produce one element.
    buf.write(foo, 1);
    EXPECT_FALSE(buf.empty());

    // Read it back out.
    buf.read(foo, 1);

    // Now the buffer should be empty again.
    EXPECT_TRUE(buf.empty());
}
