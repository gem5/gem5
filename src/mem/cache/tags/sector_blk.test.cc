/*
 * Copyright (c) 2026
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "base/gtest/logging.hh"
#include "mem/cache/tags/sector_blk.hh"
#include "sim/eventq.hh"

using namespace gem5;

// Initialize an event queue so that curTick() is valid during CacheBlk moves
EventQueue eventQueue("SectorBlkTest Queue");

class SectorBlkTestF : public ::testing::Test
{
  protected:
    void
    SetUp() override
    { curEventQueue(&eventQueue); }
};

/**
 * Test moving a sub-block into an empty (invalid) destination sector.
 * The destination sector must be initialized with the source's tag directly.
 */
TEST_F(SectorBlkTestF, MoveSubBlockEmptySectorSucceeds)
{
    auto tag_extractor = [](Addr addr) { return addr >> 12; };

    // Destination sector (initially invalid)
    SectorBlk dest_sector;
    dest_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk dest_sub;
    dest_sub.setSectorBlock(&dest_sector);
    dest_sub.registerTagExtractor(tag_extractor);

    ASSERT_FALSE(dest_sector.isValid());
    ASSERT_FALSE(dest_sub.isValid());

    // Source sub-block in another sector
    SectorBlk src_sector;
    src_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk src_sub;
    src_sub.setSectorBlock(&src_sector);
    src_sub.registerTagExtractor(tag_extractor);

    TaggedEntry::KeyType src_key{0x12345040, false};
    src_sub.insert(src_key);

    ASSERT_TRUE(src_sector.isValid());
    ASSERT_EQ(src_sector.getTag(), 0x12345);
    ASSERT_TRUE(src_sub.isValid());
    ASSERT_EQ(src_sub.getTag(), 0x12345);

    // Move to empty sector
    dest_sub = std::move(src_sub);

    EXPECT_TRUE(dest_sector.isValid());
    EXPECT_EQ(dest_sector.getTag(), 0x12345);
    EXPECT_TRUE(dest_sub.isValid());
    EXPECT_EQ(dest_sub.getTag(), 0x12345);
    EXPECT_FALSE(src_sub.isValid());
}

/**
 * Test moving a sub-block into another sub-block of a sector that is
 * already valid and shares the exact same sector tag (co-allocation).
 * This must succeed without triggering double tag extraction panic.
 */
TEST_F(SectorBlkTestF, MoveSubBlockSameSectorSucceeds)
{
    auto tag_extractor = [](Addr addr) { return addr >> 12; };

    SectorBlk dest_sector;
    dest_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk dest_sub0;
    dest_sub0.setSectorBlock(&dest_sector);
    dest_sub0.registerTagExtractor(tag_extractor);

    SectorSubBlk dest_sub1;
    dest_sub1.setSectorBlock(&dest_sector);
    dest_sub1.registerTagExtractor(tag_extractor);

    // Make dest_sector valid with tag 0x12345
    TaggedEntry::KeyType key0{0x12345040, false};
    dest_sub0.insert(key0);

    ASSERT_TRUE(dest_sector.isValid());
    ASSERT_EQ(dest_sector.getTag(), 0x12345);

    // Source sub-block sharing tag 0x12345
    SectorBlk src_sector;
    src_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk src_sub;
    src_sub.setSectorBlock(&src_sector);
    src_sub.registerTagExtractor(tag_extractor);

    TaggedEntry::KeyType src_key{0x12345080, false};
    src_sub.insert(src_key);

    // Co-allocate into dest_sub1
    dest_sub1 = std::move(src_sub);

    EXPECT_TRUE(dest_sector.isValid());
    EXPECT_EQ(dest_sector.getTag(), 0x12345);
    EXPECT_TRUE(dest_sub1.isValid());
    EXPECT_EQ(dest_sub1.getTag(), 0x12345);
    EXPECT_FALSE(src_sub.isValid());
}

/**
 * Test that moving a sub-block with a mismatched tag into an already-valid
 * sector with a different tag correctly panics with "Overwriting valid
 * sector!".
 */
TEST_F(SectorBlkTestF, MoveSubBlockDifferentSectorPanics)
{
    auto tag_extractor = [](Addr addr) { return addr >> 12; };

    SectorBlk dest_sector;
    dest_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk dest_sub0;
    dest_sub0.setSectorBlock(&dest_sector);
    dest_sub0.registerTagExtractor(tag_extractor);

    SectorSubBlk dest_sub1;
    dest_sub1.setSectorBlock(&dest_sector);
    dest_sub1.registerTagExtractor(tag_extractor);

    // dest_sector has tag 0x12345
    TaggedEntry::KeyType key0{0x12345040, false};
    dest_sub0.insert(key0);

    ASSERT_TRUE(dest_sector.isValid());
    ASSERT_EQ(dest_sector.getTag(), 0x12345);

    // src_sub has a completely DIFFERENT tag: 0x99999
    SectorBlk src_sector;
    src_sector.registerTagExtractor(tag_extractor);

    SectorSubBlk src_sub;
    src_sub.setSectorBlock(&src_sector);
    src_sub.registerTagExtractor(tag_extractor);

    TaggedEntry::KeyType src_key{0x99999040, false};
    src_sub.insert(src_key);

    ASSERT_TRUE(src_sub.isValid());
    ASSERT_EQ(src_sub.getTag(), 0x99999);

    // Attempting to move tag 0x99999 into sector with tag 0x12345 must panic!
    EXPECT_THROW(dest_sub1 = std::move(src_sub), gem5::GTestException);
    EXPECT_THAT(gtestLogOutput.str(),
                ::testing::HasSubstr("Overwriting valid sector!"));
}
