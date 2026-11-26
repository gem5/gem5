/*
 * Copyright (c) 2026 Barcelona Supercomputing Center
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

#include <algorithm>
#include <vector>

#include "base/types.hh"
#include "cpu/o3/lsq_segments.hh"

using namespace gem5;
using namespace gem5::o3;

namespace
{

/** Returns a segment whose bytes are all enabled. */
MemSegment
segment(Addr vaddr, unsigned size)
{
    return {vaddr, size, size, {}};
}

/** Returns a segment with the given byte enables. */
MemSegment
segment(Addr vaddr, const std::vector<bool> &byte_enable)
{
    std::vector<MemSegment> segments;
    appendSegment(segments, vaddr, byte_enable.size(), byte_enable);
    return segments.at(0);
}

/** Returns size byte enables, of which only [first, last) are enabled. */
std::vector<bool>
enabled(unsigned size, unsigned first, unsigned last)
{
    std::vector<bool> byte_enable(size, false);
    std::fill(byte_enable.begin() + first, byte_enable.begin() + last, true);
    return byte_enable;
}

} // anonymous namespace

TEST(LSQSegmentsTest, AppendWithoutByteEnable)
{
    std::vector<MemSegment> segments;
    appendSegment(segments, 0x1004, 8, {});
    ASSERT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].vaddr, 0x1004);
    EXPECT_EQ(segments[0].size, 8);
    EXPECT_EQ(segments[0].activeBytes, 8);
    EXPECT_TRUE(segments[0].byteEnable.empty());
}

TEST(LSQSegmentsTest, AppendCountsEnabledBytes)
{
    std::vector<MemSegment> segments;
    const std::vector<bool> byte_enable{true, false, false, true};
    appendSegment(segments, 0x1000, 4, byte_enable);
    ASSERT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].size, 4);
    EXPECT_EQ(segments[0].activeBytes, 2);
    EXPECT_EQ(segments[0].byteEnable, byte_enable);
}

TEST(LSQSegmentsTest, AppendDropsFullByteEnable)
{
    // Byte enables are only kept if some byte is disabled.
    std::vector<MemSegment> segments;
    appendSegment(segments, 0x1000, 4, {true, true, true, true});
    ASSERT_EQ(segments.size(), 1);
    EXPECT_EQ(segments[0].activeBytes, 4);
    EXPECT_TRUE(segments[0].byteEnable.empty());
}

TEST(LSQSegmentsTest, AppendSkipsDisabledSegment)
{
    std::vector<MemSegment> segments;
    appendSegment(segments, 0x1000, 4, {false, false, false, false});
    EXPECT_TRUE(segments.empty());
}

TEST(LSQSegmentsTest, EmptySegmentList)
{
    const std::vector<MemSegment> segments;
    EXPECT_EQ(sumBlocks(segments, 16), 0);
    EXPECT_EQ(distinctBlocks(segments, 16), 0);
    EXPECT_EQ(crossingSegments(segments, 16), 0);
    EXPECT_EQ(totalActiveBytes(segments), 0);
    EXPECT_EQ(chunks(segments, 16, true), 0);
    EXPECT_EQ(chunks(segments, 16, false), 0);
}

/** Segments that start or end exactly on a block boundary. */
TEST(LSQSegmentsTest, BlockBoundaries)
{
    // A whole aligned block.
    EXPECT_EQ(sumBlocks(std::vector{segment(0x10, 16)}, 16), 1);
    // Ends on the last byte of a block.
    EXPECT_EQ(sumBlocks(std::vector{segment(0x8, 8)}, 16), 1);
    // Ends on the first byte of the next block.
    EXPECT_EQ(sumBlocks(std::vector{segment(0x8, 9)}, 16), 2);
    // Starts on the last byte of a block.
    EXPECT_EQ(sumBlocks(std::vector{segment(0xf, 2)}, 16), 2);
    // Exactly two aligned blocks.
    EXPECT_EQ(sumBlocks(std::vector{segment(0x20, 32)}, 16), 2);
    // A single byte.
    EXPECT_EQ(sumBlocks(std::vector{segment(0x1f, 1)}, 16), 1);
}

TEST(LSQSegmentsTest, BlockAtTopOfAddressSpace)
{
    const std::vector segments{segment(MaxAddr - 15, 16)};
    EXPECT_EQ(sumBlocks(segments, 16), 1);
    EXPECT_EQ(distinctBlocks(segments, 16), 1);
    EXPECT_EQ(crossingSegments(segments, 16), 0);
}

TEST(LSQSegmentsTest, OverlappingSegments)
{
    // Blocks 0 and 0-1: block 0 is shared.
    const std::vector segments{segment(0x0, 16), segment(0x8, 16)};
    EXPECT_EQ(sumBlocks(segments, 16), 3);
    EXPECT_EQ(distinctBlocks(segments, 16), 2);
}

TEST(LSQSegmentsTest, SegmentsInTheSameBlock)
{
    const std::vector segments{segment(0x0, 4), segment(0x4, 4),
                               segment(0xc, 4)};
    EXPECT_EQ(sumBlocks(segments, 16), 3);
    EXPECT_EQ(distinctBlocks(segments, 16), 1);
}

TEST(LSQSegmentsTest, ContainedSegment)
{
    // Blocks 0-3 and 1: the second segment adds no block.
    const std::vector segments{segment(0x0, 64), segment(0x10, 8)};
    EXPECT_EQ(sumBlocks(segments, 16), 5);
    EXPECT_EQ(distinctBlocks(segments, 16), 4);
}

TEST(LSQSegmentsTest, AdjacentSegments)
{
    // Blocks 0 and 1 are consecutive but not shared.
    const std::vector segments{segment(0x0, 16), segment(0x10, 16)};
    EXPECT_EQ(sumBlocks(segments, 16), 2);
    EXPECT_EQ(distinctBlocks(segments, 16), 2);
}

TEST(LSQSegmentsTest, UnsortedSegments)
{
    // Blocks 8, 0 and 7-8, in that order.
    const std::vector segments{segment(0x80, 8), segment(0x0, 8),
                               segment(0x78, 16)};
    EXPECT_EQ(sumBlocks(segments, 16), 4);
    EXPECT_EQ(distinctBlocks(segments, 16), 3);
}

/** Blocks whose bytes are all disabled are not touched. */
TEST(LSQSegmentsTest, DisabledTailInNextBlock)
{
    // 8 bytes over two 64-byte blocks. Only the 4 bytes in the first block
    // are enabled.
    const std::vector segments{segment(0x103c, enabled(8, 0, 4))};
    EXPECT_EQ(sumBlocks(segments, 64), 1);
    EXPECT_EQ(distinctBlocks(segments, 64), 1);
    EXPECT_EQ(crossingSegments(segments, 64), 0);
}

TEST(LSQSegmentsTest, DisabledHeadInPreviousBlock)
{
    // Only the 4 bytes in the second block are enabled.
    const std::vector segments{segment(0x103c, enabled(8, 4, 8))};
    EXPECT_EQ(sumBlocks(segments, 64), 1);
    EXPECT_EQ(distinctBlocks(segments, 64), 1);
    EXPECT_EQ(crossingSegments(segments, 64), 0);
}

TEST(LSQSegmentsTest, EnabledBytesInBothBlocks)
{
    // One enabled byte on each side of the boundary, with disabled bytes
    // in between.
    std::vector<bool> byte_enable(8, false);
    byte_enable.front() = byte_enable.back() = true;
    const std::vector segments{segment(0x103c, byte_enable)};
    EXPECT_EQ(sumBlocks(segments, 64), 2);
    EXPECT_EQ(distinctBlocks(segments, 64), 2);
    EXPECT_EQ(crossingSegments(segments, 64), 1);
}

TEST(LSQSegmentsTest, DisabledBlockBetweenEnabledBlocks)
{
    // Three blocks, of which the middle one has no enabled byte.
    std::vector<bool> byte_enable(192, false);
    byte_enable.front() = byte_enable.back() = true;
    const std::vector segments{segment(0x1000, byte_enable)};
    EXPECT_EQ(sumBlocks(segments, 64), 2);
    EXPECT_EQ(distinctBlocks(segments, 64), 2);
    EXPECT_EQ(crossingSegments(segments, 64), 1);
}

TEST(LSQSegmentsTest, DisabledBytesInsideABlock)
{
    // Disabled bytes do not hide a block that has other enabled bytes.
    const std::vector segments{segment(0x8, enabled(16, 7, 9))};
    EXPECT_EQ(sumBlocks(segments, 16), 2);
    EXPECT_EQ(distinctBlocks(segments, 16), 2);
    EXPECT_EQ(crossingSegments(segments, 16), 1);
    EXPECT_EQ(chunks(segments, 16, true), 1);
}

TEST(LSQSegmentsTest, DistinctBlocksOfMaskedSegments)
{
    // Blocks 0 and 2, block 2, and blocks 1-2. The first segment has no
    // enabled byte in block 1, and the second one has none in block 3.
    std::vector<bool> ends(48, false);
    ends.front() = ends.back() = true;
    const std::vector segments{segment(0x0, ends),
                               segment(0x20, enabled(32, 0, 16)),
                               segment(0x10, 32)};
    EXPECT_EQ(sumBlocks(segments, 16), 5);
    EXPECT_EQ(distinctBlocks(segments, 16), 3);
    EXPECT_EQ(crossingSegments(segments, 16), 2);
}

TEST(LSQSegmentsTest, MaskedBlockAtTopOfAddressSpace)
{
    const std::vector segments{segment(MaxAddr - 31, enabled(32, 16, 32))};
    EXPECT_EQ(sumBlocks(segments, 16), 1);
    EXPECT_EQ(distinctBlocks(segments, 16), 1);
    EXPECT_EQ(crossingSegments(segments, 16), 0);
}

TEST(LSQSegmentsTest, CrossingSegments)
{
    EXPECT_EQ(crossingSegments(std::vector{segment(0x0, 64)}, 64), 0);
    EXPECT_EQ(crossingSegments(std::vector{segment(0x3f, 1)}, 64), 0);
    EXPECT_EQ(crossingSegments(std::vector{segment(0x3f, 2)}, 64), 1);
    EXPECT_EQ(crossingSegments(std::vector{segment(0x40, 64)}, 64), 0);

    const std::vector segments{segment(0x38, 16), segment(0x80, 8),
                               segment(0xf0, 64)};
    EXPECT_EQ(crossingSegments(segments, 64), 2);
}

TEST(LSQSegmentsTest, TotalActiveBytes)
{
    const std::vector segments{segment(0x0, 8),
                               segment(0x40, enabled(16, 0, 3))};
    EXPECT_EQ(totalActiveBytes(segments), 11);
}

TEST(LSQSegmentsTest, Chunks)
{
    // A multiple of the chunk size, and one byte more.
    EXPECT_EQ(chunks(std::vector{segment(0x0, 16)}, 8, true), 2);
    EXPECT_EQ(chunks(std::vector{segment(0x0, 17)}, 8, true), 3);
    // Chunks only depend on the number of enabled bytes, not on the
    // alignment.
    EXPECT_EQ(chunks(std::vector{segment(0x7, 8)}, 8, true), 1);
}

TEST(LSQSegmentsTest, ChunksCoalesce)
{
    const std::vector segments{segment(0x0, enabled(8, 0, 4)),
                               segment(0x100, enabled(8, 4, 8)),
                               segment(0x200, enabled(16, 0, 9))};
    // 17 enabled bytes together, or 4, 4 and 9 bytes separately.
    EXPECT_EQ(chunks(segments, 8, true), 3);
    EXPECT_EQ(chunks(segments, 8, false), 4);
}
