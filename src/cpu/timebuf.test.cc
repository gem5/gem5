/*
 * Copyright (c) 2026 The Regents of the University of California
 * All rights reserved.
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

#include "base/gtest/logging.hh"
#include "cpu/timebuf.hh"

namespace gem5
{
namespace
{

TEST(TimeBufferTest, ValidRangeAndAdvance)
{
    TimeBuffer<int> buffer(2, 3);
    buffer[-2] = 10;
    buffer[0] = 20;
    buffer[3] = 30;
    EXPECT_EQ(*buffer.access(-2), 10);
    EXPECT_EQ(*buffer.access(0), 20);
    EXPECT_EQ(*buffer.access(3), 30);

    buffer.advance();
    EXPECT_EQ(buffer[-1], 20);
    EXPECT_EQ(buffer[2], 30);
    EXPECT_EQ(buffer[3], 0);
}

TEST(TimeBufferTest, AccessOutsideRangePanics)
{
    TimeBuffer<int> buffer(2, 3);
    EXPECT_THROW(buffer.access(-3), GTestException);
    EXPECT_THROW(buffer.access(4), GTestException);
    EXPECT_THROW(buffer[-3], GTestException);
    const auto &const_buffer = buffer;
    EXPECT_THROW(const_buffer[4], GTestException);
}

TEST(TimeBufferTest, WireOutsideRangePanics)
{
    TimeBuffer<int> buffer(2, 3);
    auto wire = buffer.getWire(0);
    wire = -2;
    EXPECT_THROW(--wire, GTestException);
    wire = 3;
    EXPECT_THROW(++wire, GTestException);
    wire = 0;
    EXPECT_THROW(wire += 4, GTestException);
    EXPECT_THROW(wire -= 3, GTestException);
}

TEST(TimeBufferTest, ZeroLengthHistoryAndFuture)
{
    TimeBuffer<int> buffer(0, 0);
    buffer[0] = 7;
    EXPECT_EQ(*buffer.getWire(0), 7);
    EXPECT_THROW(buffer.access(-1), GTestException);
    EXPECT_THROW(buffer.access(1), GTestException);
    buffer.advance();
    EXPECT_EQ(buffer[0], 0);
}

} // anonymous namespace
} // namespace gem5
