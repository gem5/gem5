/*
 * Copyright (c) 2026 Adarsh Kumar
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

#include <memory>

#include "mem/cache/tags/super_blk.hh"

using namespace gem5;

class SuperBlkTest : public ::testing::Test
{
  protected:
    static constexpr std::size_t BlkSize = 64;
    static constexpr unsigned NumSubBlks = 4;

    SuperBlk superBlk;
    std::unique_ptr<CompressionBlk[]> subBlks;

    void
    SetUp() override
    {
        superBlk.setBlkSize(BlkSize);
        subBlks.reset(new CompressionBlk[NumSubBlks]);
        superBlk.blks.resize(NumSubBlks);
        for (unsigned k = 0; k < NumSubBlks; ++k) {
            superBlk.blks[k] = &subBlks[k];
            subBlks[k].setSectorBlock(&superBlk);
            subBlks[k].setSectorOffset(k);
            subBlks[k].registerTagExtractor([](Addr addr) { return addr; });
        }
        superBlk.registerTagExtractor([](Addr addr) { return addr; });
    }
};

/**
 * Test that CompressionBlk correctly sets compression status for uncompressed
 * vs compressed sizes.
 */
TEST_F(SuperBlkTest, UncompressedSubBlockDetection)
{
    CompressionBlk &blk = subBlks[0];
    blk.setDecompressionLatency(Cycles(2));

    // Uncompressed line (512 bits for a 64-byte line)
    blk.setSizeBits(512);
    EXPECT_FALSE(blk.isCompressed());
    EXPECT_EQ(blk.getSizeBits(), 512);

    // Compressed line (256 bits)
    blk.setSizeBits(256);
    EXPECT_TRUE(blk.isCompressed());
    EXPECT_EQ(blk.getSizeBits(), 256);
}

TEST_F(SuperBlkTest, SetUncompressedClearsCompressed)
{
    CompressionBlk &blk = subBlks[0];
    blk.setSizeBits(256);
    EXPECT_TRUE(blk.isCompressed());

    blk.setUncompressed();
    EXPECT_FALSE(blk.isCompressed());
}

using CanCoAllocateTest = SuperBlkTest;

TEST_F(CanCoAllocateTest, CumulativeBitSumCheck)
{
    // Initially empty superblock
    EXPECT_TRUE(superBlk.canCoAllocate(128));
    EXPECT_FALSE(superBlk.canCoAllocate(512)); // uncompressed rejected

    // Add 1st sub-block (128 bits) -> sum = 128
    subBlks[0].insert({0x1000, false});
    subBlks[0].setSizeBits(128);

    // Add 2nd sub-block (128 bits) -> sum = 256
    subBlks[1].insert({0x1000, false});
    subBlks[1].setSizeBits(128);

    // 128 + 128 + 128 = 384 <= 512 bits -> should allow co-allocation
    EXPECT_TRUE(superBlk.canCoAllocate(128));

    // Add 3rd sub-block (128 bits) -> sum = 384
    subBlks[2].insert({0x1000, false});
    subBlks[2].setSizeBits(128);

    // 384 + 128 = 512 <= 512 bits -> should allow 4th 128-bit sub-block
    EXPECT_TRUE(superBlk.canCoAllocate(128));

    // 384 + 256 = 640 > 512 bits -> should reject 256-bit sub-block
    EXPECT_FALSE(superBlk.canCoAllocate(256));
}

TEST_F(CanCoAllocateTest, HeterogeneousSubBlockCoAllocation)
{
    // Add 1st sub-block (256 bits, CF=2)
    subBlks[0].insert({0x2000, false});
    subBlks[0].setSizeBits(256);

    // Add 2nd sub-block (128 bits, CF=4) -> sum = 384 bits, min_cf = 2
    subBlks[1].insert({0x2000, false});
    subBlks[1].setSizeBits(128);

    // Under discrete count cap (numValid < target_cf), numValid = 2 < 2 would
    // be false. Under cumulative bit sum check (384 + 128 = 512 <= 512), this
    // should succeed!
    EXPECT_TRUE(superBlk.canCoAllocate(128));

    // Exceeding 512 bits (384 + 256 = 640 > 512) must be rejected
    EXPECT_FALSE(superBlk.canCoAllocate(256));
}
