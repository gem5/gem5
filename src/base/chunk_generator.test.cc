/*
 * Copyright (c) 2019 The Regents of the University of California
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

#include <gtest/gtest.h>

#include "chunk_generator.hh"

using namespace gem5;

/*
 * A test to ensure the object is in a sane state after initialization.
 */
TEST(ChunkGeneratorTest, StartingConditions)
{
    ChunkGenerator chunk_generator(0, 1024, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());
}

/*
 * A simple test to check the move to the next chunk under normal conditions.
 */
TEST(ChunkGeneratorTest, AdvanceToNextChunk)
{
    ChunkGenerator chunk_generator(0, 1024, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(8, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());
}

/*
 * A test to check skipping over bytes.
 */
TEST(ChunkGeneratorTest, SkipBytes)
{
    ChunkGenerator chunk_generator(0, 1024, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());

    chunk_generator.setNext(23);
    EXPECT_EQ(23 - 8, chunk_generator.size());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(23, chunk_generator.addr());
    EXPECT_EQ(1, chunk_generator.size());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(24, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());

    chunk_generator.setNext(32);
    EXPECT_EQ(32 - 24, chunk_generator.size());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(32, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());

    chunk_generator.setNext(64);
    EXPECT_EQ(64 - 32, chunk_generator.size());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(64, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());

    chunk_generator.setNext(2048);
    EXPECT_EQ(1024 - 64, chunk_generator.size());
    EXPECT_TRUE(chunk_generator.last());
    EXPECT_FALSE(chunk_generator.next());
    EXPECT_TRUE(chunk_generator.done());
}

/*
 * A test to consume chunks until the last chunk.
 */
TEST(ChunkGeneratorTest, AdvanceToLastChunk)
{
    ChunkGenerator chunk_generator(0, 32, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(16, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(24, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(24, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}

/*
 * A test to consume chunks, inclusive of the last chunk.
 */
TEST(ChunkGeneratorTest, AdvanceToTheEnd)
{
    ChunkGenerator chunk_generator(0, 32, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(16, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(24, chunk_generator.addr());
    /* The following returns false because we cannot advance to the next to
     * the next chunk (it does not exist). However, we still process the last
     * chunk. It is therefore not indicative of failure to change the
     * state of the object.
     */
    EXPECT_FALSE(chunk_generator.next());
    EXPECT_EQ(24, chunk_generator.addr());
    EXPECT_EQ(0, chunk_generator.size());
    EXPECT_EQ(24, chunk_generator.complete());
    EXPECT_TRUE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}

/*
 * A region does is not necessisarily divisable by the chunk size. This will
 * will result in the final chunk being smaller than the rest.
 */
TEST(ChunkGeneratorTest, SmallerLastChunk)
{
    // There are two chunks. The last will be 6 bytes.
    ChunkGenerator chunk_generator(0, 14, 8);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());
    EXPECT_EQ(6, chunk_generator.size());
    EXPECT_EQ(8, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}

/*
 * When a chunk size is greater than the total size, the chunk size
 * is effectively that of the region size. This test will verify this
 * corner-case.
 */
TEST(ChunkGeneratorTest, ChunkSizeGreaterThanTotalSize)
{
    ChunkGenerator chunk_generator(0, 32, 64);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_EQ(32, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());

    // Process the entire region.
    EXPECT_FALSE(chunk_generator.next());
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_EQ(0, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_TRUE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}

/*
 * As a special case, we assume there is no chunking when the chunk size is
 * zero. Processing a chunk (i.e., execution of "next()"). should progress to
 * the end of the region.
 */
TEST(ChunkGeneratorTest, ChunkSizeZero)
{
    ChunkGenerator chunk_generator(0, 64, 0);
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_EQ(64, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());

    //Process the entire region.
    EXPECT_FALSE(chunk_generator.next());
    EXPECT_EQ(0, chunk_generator.addr());
    EXPECT_EQ(0, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_TRUE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}

/*
 * A test to ensure a non-zero start functions correctly.
 */
TEST(ChunkGeneratorTest, StartAtNonZero)
{
    ChunkGenerator chunk_generator(4, 32, 8); //End address: 36.
    EXPECT_EQ(4, chunk_generator.addr());
    EXPECT_EQ(4, chunk_generator.size());
    EXPECT_EQ(0, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());

    /*
     * As the starting position is 4, moving to the next bit should move to
     * 8 (i.e., process the remainder of the first chunk in the region).
     */
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(8, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(4, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());

    // Process the rest of the region.
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(16, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(12, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());

    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(24, chunk_generator.addr());
    EXPECT_EQ(8, chunk_generator.size());
    EXPECT_EQ(20, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_FALSE(chunk_generator.last());

    // The last chunk is also only 4 bytes.
    EXPECT_TRUE(chunk_generator.next());
    EXPECT_EQ(32, chunk_generator.addr());
    EXPECT_EQ(4, chunk_generator.size());
    EXPECT_EQ(28, chunk_generator.complete());
    EXPECT_FALSE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());

    EXPECT_FALSE(chunk_generator.next());
    EXPECT_EQ(32, chunk_generator.addr());
    EXPECT_EQ(0, chunk_generator.size());
    EXPECT_EQ(28, chunk_generator.complete());
    EXPECT_TRUE(chunk_generator.done());
    EXPECT_TRUE(chunk_generator.last());
}
