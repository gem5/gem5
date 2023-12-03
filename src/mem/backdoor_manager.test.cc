/*
 * Copyright 2023 Google, Inc.
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
#include <vector>

#include "base/addr_range.hh"
#include "base/gtest/logging.hh"
#include "mem/backdoor.hh"
#include "mem/backdoor_manager.hh"

namespace gem5
{
namespace backdoor_manager_test
{
const std::vector<AddrRange> kOriginalRange({ AddrRange(0x0, 0x1000) });
const std::vector<AddrRange> kRemappedRange({ AddrRange(0x1000, 0x2000) });

class BackdoorManagerTest : public BackdoorManager, public ::testing::Test
{
  public:
    BackdoorManagerTest() : BackdoorManager(kOriginalRange, kRemappedRange) {}
};

TEST_F(BackdoorManagerTest, BasicRemapTest)
{
    /**
     * The backdoor range is remappedRanges[0], and should be reverted into
     * originalRanges[0].
     */
    AddrRange pkt_range = originalRanges[0];

    uint8_t *ptr = nullptr;
    MemBackdoor remapped_backdoor(remappedRanges[0], ptr,
                                  MemBackdoor::Flags::Readable);
    MemBackdoorPtr reverted_backdoor =
        getRevertedBackdoor(&remapped_backdoor, pkt_range);

    EXPECT_EQ(reverted_backdoor->range(), originalRanges[0]);
    EXPECT_EQ(reverted_backdoor->ptr(), ptr);
    ASSERT_EQ(backdoorLists[0].size(), 1);
    EXPECT_EQ(backdoorLists[0].begin()->get(), reverted_backdoor);

    /**
     * After the target backdoor is invalidated, the new created backdoor
     * should be freed and removed from the backdoor list.
     */
    remapped_backdoor.invalidate();
    EXPECT_EQ(backdoorLists[0].size(), 0);
}

TEST_F(BackdoorManagerTest, ShrinkTest)
{
    AddrRange pkt_range = originalRanges[0];

    /**
     * The backdoor range is larger than the address remapper's address range.
     * Backdoor is expected to be shrinked.
     */
    Addr diff = 0x1000;
    AddrRange remapped_backdoor_range(remappedRanges[0].start() - diff, // 0x0
                                      remappedRanges[0].end() +
                                          diff); // 0x3000

    uint8_t *ptr = nullptr;
    MemBackdoor remapped_backdoor(remapped_backdoor_range, ptr,
                                  MemBackdoor::Flags::Readable);
    MemBackdoorPtr reverted_backdoor =
        getRevertedBackdoor(&remapped_backdoor, pkt_range);

    EXPECT_EQ(reverted_backdoor->range(), originalRanges[0]);
    EXPECT_EQ(reverted_backdoor->ptr(), ptr + diff);

    remapped_backdoor.invalidate();
}

TEST_F(BackdoorManagerTest, ReuseTest)
{
    /**
     * The two packets have different address range, but both contained in the
     * original address range.
     */
    Addr mid = originalRanges[0].start() + originalRanges[0].size() / 2;
    AddrRange pkt_range_0 = AddrRange(originalRanges[0].start(), mid);
    AddrRange pkt_range_1 = AddrRange(mid, originalRanges[0].end());

    /**
     * The address range of the backdoor covers the whole address range, so
     * both packets can be fulfilled by this backdoor.
     */
    uint8_t *ptr = nullptr;
    MemBackdoor remapped_backdoor(remappedRanges[0], ptr,
                                  MemBackdoor::Flags::Readable);
    /**
     * For the first packet, a new backdoor should be constructed.
     */
    MemBackdoorPtr reverted_backdoor_0 =
        getRevertedBackdoor(&remapped_backdoor, pkt_range_0);
    EXPECT_EQ(backdoorLists[0].size(), 1);

    /**
     * For the second packet, it should return the same backdoor as previous
     * one, and no new backdoor should be constructed.
     */
    MemBackdoorPtr reverted_backdoor_1 =
        getRevertedBackdoor(&remapped_backdoor, pkt_range_1);
    EXPECT_EQ(reverted_backdoor_0, reverted_backdoor_1);
    EXPECT_EQ(backdoorLists[0].size(), 1);

    remapped_backdoor.invalidate();
}

} // namespace backdoor_manager_test
} // namespace gem5
