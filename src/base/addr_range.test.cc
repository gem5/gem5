/*
 * Copyright (c) 2019 The Regents of the University of California
 * Copyright (c) 2018-2019, 2021 Arm Limited
 * Copyright (c) 2026 Google Inc
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

#include <cmath>

#include "base/addr_range.hh"
#include "base/bitfield.hh"

using namespace gem5;

using testing::ElementsAre;

TEST(AddrRangeTest, ValidRange)
{
    AddrRange r;
    EXPECT_FALSE(r.valid());
}

/*
 * This following tests check the behavior of AddrRange when initialized with
 * a start and end address. The expected behavior is that the first address
 * within the range will be the start address, and the last address in the
 * range will be the (end - 1) address.
 */
TEST(AddrRangeTest, EmptyRange)
{
    AddrRange r(0x0, 0x0);

    /*
     * Empty ranges are valid.
     */
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(0x0, r.start());
    EXPECT_EQ(0x0, r.end());
    EXPECT_EQ(0, r.size());

    /*
     * With no masks, granularity equals the size of the range.
     */
    EXPECT_EQ(0, r.granularity());

    /*
     * With no masks, "interleaved()" returns false.
     */
    EXPECT_FALSE(r.interleaved());

    /*
     * With no masks, "stripes()" returns 1ULL.
     */
    EXPECT_EQ(1ULL, r.stripes());
    EXPECT_EQ("[0:0]", r.to_string());
}

TEST(AddrRangeTest, RangeSizeOfOne)
{
    AddrRange r(0x0, 0x1);
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(0x0, r.start());
    EXPECT_EQ(0x1, r.end());
    EXPECT_EQ(1, r.size());
    EXPECT_EQ(1, r.granularity());
    EXPECT_FALSE(r.interleaved());
    EXPECT_EQ(1ULL, r.stripes());
    EXPECT_EQ("[0:0x1]", r.to_string());
}

TEST(AddrRangeTest, Range16Bit)
{
    AddrRange r(0xF000, 0xFFFF);
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(0xF000, r.start());
    EXPECT_EQ(0xFFFF, r.end());
    EXPECT_EQ(0x0FFF, r.size());
    EXPECT_EQ(0x0FFF, r.granularity());
    EXPECT_FALSE(r.interleaved());
    EXPECT_EQ(1ULL, r.stripes());
    EXPECT_EQ("[0xf000:0xffff]", r.to_string());
}

TEST(AddrRangeTest, InvalidRange)
{
    AddrRange r(0x1, 0x0);
    EXPECT_FALSE(r.valid());
}

TEST(AddrRangeTest, LessThan)
{
    /*
     * The less-than override is a bit unintuitive and does not have a
     * corresponding greater than. It compares the AddrRange.start() values.
     * If they are equal, the "intlvMatch" values are compared. This is
     * zero when AddRange is initialized with a just a start and end address.
     */
    AddrRange r1(0xF000, 0xFFFF);
    AddrRange r2(0xF001, 0xFFFF);
    AddrRange r3(0xF000, 0xFFFF);

    EXPECT_TRUE(r1 < r2);
    EXPECT_FALSE(r2 < r1);
    EXPECT_FALSE(r1 < r3);
    EXPECT_FALSE(r3 < r1);
}

TEST(AddrRangeTest, EqualToNotEqualTo)
{
    AddrRange r1(0x1234, 0x5678);
    AddrRange r2(0x1234, 0x5678);
    AddrRange r3(0x1234, 0x5679);

    EXPECT_TRUE(r1 == r2);
    EXPECT_FALSE(r1 == r3);
    EXPECT_FALSE(r1 != r2);
    EXPECT_TRUE(r1 != r3);

    EXPECT_TRUE(r2 == r1);
    EXPECT_FALSE(r3 == r1);
    EXPECT_FALSE(r2 != r1);
    EXPECT_TRUE(r3 != r1);
}

TEST(AddrRangeTest, MergesWith)
{
    /*
     * AddrRange.mergesWith will return true if the start, end, and masks
     * are the same.
     */
    AddrRange r1(0x10, 0x1F);
    AddrRange r2(0x10, 0x1F);

    EXPECT_TRUE(r1.mergesWith(r2));
    EXPECT_TRUE(r2.mergesWith(r1));
}

TEST(AddrRangeTest, DoesNotMergeWith)
{
    AddrRange r1(0x10, 0x1E);
    AddrRange r2(0x10, 0x1F);

    EXPECT_FALSE(r1.mergesWith(r2));
    EXPECT_FALSE(r2.mergesWith(r1));
}

TEST(AddrRangeTest, IntersectsCompleteOverlap)
{
    AddrRange r1(0x21, 0x30);
    AddrRange r2(0x21, 0x30);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}

TEST(AddrRangeTest, IntersectsAddressWithin)
{
    AddrRange r1(0x0, 0xF);
    AddrRange r2(0x1, 0xE);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}

TEST(AddrRangeTest, IntersectsPartialOverlap)
{
    AddrRange r1(0x0F0, 0x0FF);
    AddrRange r2(0x0F5, 0xF00);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}

TEST(AddrRangeTest, IntersectsNoOverlap)
{
    AddrRange r1(0x00, 0x10);
    AddrRange r2(0x11, 0xFF);

    EXPECT_FALSE(r1.intersects(r2));
    EXPECT_FALSE(r2.intersects(r1));
}

TEST(AddrRangeTest, IntersectsFirstLastAddressOverlap)
{
    AddrRange r1(0x0, 0xF);
    AddrRange r2(0xF, 0xF0);

    /*
     * The "end address" is not in the range. Therefore, if
     * r1.end() == r2.start(), the ranges do not intersect.
     */
    EXPECT_FALSE(r1.intersects(r2));
    EXPECT_FALSE(r2.intersects(r1));
}

TEST(AddrRangeTest, isSubsetCompleteOverlap)
{
    AddrRange r1(0x10, 0x20);
    AddrRange r2(0x10, 0x20);

    EXPECT_TRUE(r1.isSubset(r2));
    EXPECT_TRUE(r2.isSubset(r1));
}

TEST(AddrRangeTest, isSubsetNoOverlap)
{
    AddrRange r1(0x10, 0x20);
    AddrRange r2(0x20, 0x22);

    EXPECT_FALSE(r1.isSubset(r2));
    EXPECT_FALSE(r2.isSubset(r1));
}

TEST(AddrRangeTest, isSubsetTrueSubset)
{
    AddrRange r1(0x10, 0x20);
    AddrRange r2(0x15, 0x17);

    EXPECT_TRUE(r2.isSubset(r1));
    EXPECT_FALSE(r1.isSubset(r2));
}

TEST(AddrRangeTest, isSubsetPartialSubset)
{
    AddrRange r1(0x20, 0x30);
    AddrRange r2(0x26, 0xF0);

    EXPECT_FALSE(r1.isSubset(r2));
    EXPECT_FALSE(r2.isSubset(r1));
}

TEST(AddrRangeTest, isSubsetInterleavedCompleteOverlap)
{
    AddrRange r1(0x00, 0x100, std::vector<Addr>{0x40}, 0);
    AddrRange r2(0x00, 0x40);

    EXPECT_TRUE(r2.isSubset(r1));
}

TEST(AddrRangeTest, isSubsetInterleavedNoOverlap)
{
    AddrRange r1(0x00, 0x100, std::vector<Addr>{0x40}, 1);
    AddrRange r2(0x00, 0x40);

    EXPECT_FALSE(r2.isSubset(r1));
}

TEST(AddrRangeTest, isSubsetInterleavedPartialOverlap)
{
    AddrRange r1(0x00, 0x100, std::vector<Addr>{0x40}, 0);
    AddrRange r2(0x10, 0x50);

    EXPECT_FALSE(r2.isSubset(r1));
}

TEST(AddrRangeTest, Contains)
{
    AddrRange r(0xF0, 0xF5);

    EXPECT_FALSE(r.contains(0xEF));
    EXPECT_TRUE(r.contains(0xF0));
    EXPECT_TRUE(r.contains(0xF1));
    EXPECT_TRUE(r.contains(0xF2));
    EXPECT_TRUE(r.contains(0xF3));
    EXPECT_TRUE(r.contains(0xF4));
    EXPECT_FALSE(r.contains(0xF5));
    EXPECT_FALSE(r.contains(0xF6));
}

TEST(AddrRangeTest, ContainsInAnEmptyRange)
{
    AddrRange r(0x1, 0x1);

    EXPECT_FALSE(r.contains(0x1));
}

TEST(AddrRangeTest, RemoveIntlvBits)
{
    AddrRange r(0x01, 0x10);

    /*
     * When there are no masks, AddrRange.removeIntlBits just returns the
     * address parameter.
     */
    Addr a(56);
    a = r.removeIntlvBits(a);
    EXPECT_EQ(56, a);
}

TEST(AddrRangeTest, addIntlvBits)
{
    AddrRange r(0x01, 0x10);

    /*
     * As with AddrRange.removeIntlBits, when there are no masks,
     * AddrRange.addIntlvBits just returns the address parameter.
     */
    Addr a(56);
    a = r.addIntlvBits(a);
    EXPECT_EQ(56, a);
}

TEST(AddrRangeTest, OffsetInRange)
{
    AddrRange r(0x01, 0xF0);
    EXPECT_EQ(0x04, r.getOffset(0x5));
}

TEST(AddrRangeTest, OffsetOutOfRangeAfter)
{
    /*
     * If the address is less than the range, MaxAddr is returned.
     */
    AddrRange r(0x01, 0xF0);
    EXPECT_EQ(MaxAddr, r.getOffset(0xF0));
}

TEST(AddrRangeTest, OffsetOutOfRangeBefore)
{
    AddrRange r(0x05, 0xF0);
    EXPECT_EQ(MaxAddr, r.getOffset(0x04));
}

/*
 * The following tests check the behavior of AddrRange when initialized with
 * a start and end address, as well as masks to distinguish interleaving bits.
 */
TEST(AddrRangeTest, LsbInterleavingMask)
{
    Addr start = 0x00;
    Addr end   = 0xFF;
    std::vector<Addr> masks;
    /*
     * The address is in range if the LSB is set, i.e. is the value is odd.
     */
    masks.push_back(1);
    uint8_t intlv_match = 1;

    AddrRange r(start, end, masks, intlv_match);
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(start, r.start());
    EXPECT_EQ(end, r.end());
    /*
     * With interleaving, it's assumed the size is equal to
     * start - end >> [number of masks].
     */
    EXPECT_EQ(0x7F, r.size());
    /*
     * The Granularity, the size of regions created by the interleaving bits,
     * which, in this case, is one.
     */
    EXPECT_EQ(1, r.granularity());
    EXPECT_TRUE(r.interleaved());
    EXPECT_EQ(2ULL, r.stripes());
    EXPECT_EQ("[0:0xff] a[0]^\b=1", r.to_string());
}

TEST(AddrRangeTest, TwoInterleavingMasks)
{
    Addr start = 0x0000;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    /*
     * There are two marks, the two LSBs.
     */
    masks.push_back(1);
    masks.push_back((1 << 1));
    uint8_t intlv_match = (1 << 1) | 1;

    AddrRange r(start, end, masks, intlv_match);
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(start, r.start());
    EXPECT_EQ(end, r.end());

    EXPECT_EQ(0x3FFF, r.size());
    EXPECT_TRUE(r.interleaved());
    EXPECT_EQ(4ULL, r.stripes());
    EXPECT_EQ("[0:0xffff] a[0]^\b=1 a[1]^\b=1", r.to_string());
}

TEST(AddrRangeTest, ComplexInterleavingMasks)
{
    Addr start = 0x0000;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back((1 << 1) | 1);
    masks.push_back((1ULL << 63) | (1ULL << 62));
    uint8_t intlv_match = 0;

    AddrRange r(start, end, masks, intlv_match);
    EXPECT_TRUE(r.valid());
    EXPECT_EQ(start, r.start());
    EXPECT_EQ(end, r.end());

    EXPECT_EQ(0x3FFF, r.size());
    EXPECT_TRUE(r.interleaved());
    EXPECT_EQ(4ULL, r.stripes());
    EXPECT_EQ("[0:0xffff] a[0]^a[1]^\b=0 a[62]^a[63]^\b=0", r.to_string());
}

TEST(AddrRangeTest, InterleavingAddressesMergesWith)
{
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks.push_back((1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks, intlv_match1);

    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    uint8_t intlv_match2 = 1; // intlv_match may differ.
    AddrRange r2(start2, end2, masks, intlv_match2);

    EXPECT_TRUE(r1.mergesWith(r2));
    EXPECT_TRUE(r2.mergesWith(r1));
}

TEST(AddrRangeTest, InterleavingAddressesDoNotMergeWith)
{
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks1.push_back((1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks2.push_back((1 << 3)); // Different mask here.
    uint8_t intlv_match2 = 1; // intlv_match may differ.
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_FALSE(r1.mergesWith(r2));
    EXPECT_FALSE(r2.mergesWith(r1));
}

TEST(AddrRangeTest, InterleavingAddressesDoNotIntersect)
{
    /*
     * Range 1: all the odd addresses between 0x0000 and 0xFFFF.
     */
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back(1);
    uint8_t intlv_match1 = 1;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    /*
     * Range 2: all the even addresses between 0x0000 and 0xFFFF. These
     * addresses should thereby not intersect.
     */
    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back(1);
    uint8_t intv_match2 = 0;
    AddrRange r2(start2, end2, masks2, intv_match2);

    EXPECT_FALSE(r1.intersects(r2));
    EXPECT_FALSE(r2.intersects(r1));
}

TEST(AddrRangeTest, InterleavingAddressesIntersectsViaMerging)
{
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks1.push_back((1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks2.push_back((1 << 2));
    uint8_t intlv_match2 = 0;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}

TEST(AddrRangeTest, InterleavingAddressesDoesNotIntersectViaMerging)
{
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks1.push_back((1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 29) | (1 << 20) | (1 << 10) | 1);
    masks2.push_back((1 << 2));
    /*
     * These addresses can merge, but their intlv_match values differ. They
     * therefore do not intersect.
     */
    uint8_t intlv_match2 = 1;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_FALSE(r1.intersects(r2));
    EXPECT_FALSE(r2.intersects(r1));
}

/*
 * The following tests were created to test more complex cases where
 * interleaving addresses may intersect. However, the "intersects" function
 * does not cover all cases (a "Cannot test intersection..." exception will
 * be thrown outside of very simple checks to see if an intersection occurs).
 * The tests below accurately test whether two ranges intersect but, for now,
 * code has yet to be implemented to utilize these tests. They are therefore
 * disabled, but may be enabled at a later date if/when the "intersects"
 * function is enhanced.
 */
TEST(AddrRangeTest, DISABLED_InterleavingAddressesIntersect)
{
    /*
     * Range 1: all the odd addresses between 0x0000 and 0xFFFF.
     */
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back(1);
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    /*
     * Range 2: all the addresses divisible by 4 between 0x0000 and
     * 0xFFFF. These addresses should thereby intersect.
     */
    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back(1 << 2);
    uint8_t intlv_match2 = 1;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}

TEST(AddrRangeTest, DISABLED_InterleavingAddressesIntersectsOnOneByteAddress)
{
    /*
     * Range: all the odd addresses between 0x0000 and 0xFFFF.
     */
    Addr start = 0x0000;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back(1);
    uint8_t intlv_match = 1;
    AddrRange r1(start, end, masks, intlv_match);

    AddrRange r2(0x0000, 0x0001);

    EXPECT_FALSE(r1.intersects(r2));
    EXPECT_FALSE(r2.intersects(r1));
}

TEST(AddrRangeTest,
    DISABLED_InterleavingAddressesDoesNotIntersectOnOneByteAddress)
{
    /*
     * Range: all the odd addresses between 0x0000 and 0xFFFF.
     */
    Addr start = 0x0000;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back(1);
    uint8_t intlv_match = 1;
    AddrRange r1(start, end, masks, intlv_match);

    AddrRange r2(0x0001, 0x0002);

    EXPECT_TRUE(r1.intersects(r2));
    EXPECT_TRUE(r2.intersects(r1));
}


/*
 * The following three tests were created to test the addr_range.isSubset
 * function for Interleaving address ranges. However, for now, this
 * functionality has not been implemented. These tests are therefore disabled.
 */
TEST(AddrRangeTest, DISABLED_InterleavingAddressIsSubset)
{
    // Range 1: all the even addresses between 0x0000 and 0xFFFF.
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back(1);
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    // Range 2: all the even addresses between 0xF000 and 0x0FFF, this is
    // a subset of Range 1.
    Addr start2 = 0xF000;
    Addr end2   = 0x0FFF;
    std::vector<Addr> masks2;
    masks2.push_back(1);
    uint8_t intlv_match2 = 0;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_TRUE(r1.isSubset(r2));
    EXPECT_TRUE(r2.isSubset(r1));
}

TEST(AddrRangeTest, DISABLED_InterleavingAddressIsNotSubset)
{
    //Range 1: all the even addresses between 0x0000 and 0xFFFF.
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back(1);
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);


    // Range 2: all the odd addresses between 0xF000 and 0x0FFF, this is
    //a subset of Range 1.
    Addr start2 = 0xF000;
    Addr end2   = 0x0FFF;
    std::vector<Addr> masks2;
    masks2.push_back(1);
    uint8_t intlv_match2 = 1;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_FALSE(r1.isSubset(r2));
    EXPECT_FALSE(r2.isSubset(r1));
}

TEST(AddrRangeTest, DISABLED_InterleavingAddressContains)
{
    /*
     * Range: all the address between 0x0 and 0xFF which have both the 1st
     * and 5th bits 1, or both are 0
     */
    Addr start = 0x00;
    Addr end   = 0xFF;
    std::vector<Addr> masks;
    masks.push_back((1 << 4) | 1);
    uint8_t intlv_match = 0;
    AddrRange r(start, end, masks, intlv_match);

    for (Addr addr = start; addr < end; addr++) {
        if (((addr & 1) && ((1 << 4) & addr)) || // addr[0] && addr[4]
            (!(addr & 1) && !((1 << 4) & addr))) { //!addr[0] && !addr[4]
            EXPECT_TRUE(r.contains(addr));
        } else {
            EXPECT_FALSE(r.contains(addr));
        }
    }
}

TEST(AddrRangeTest, InterleavingAddressAddRemoveInterlvBits)
{
    Addr start = 0x00000;
    Addr end   = 0x10000;
    std::vector<Addr> masks;
    masks.push_back(1);
    uint8_t intlv_match = 1;
    AddrRange r(start, end, masks, intlv_match);

    Addr input = 0xFFFF;
    Addr output = r.removeIntlvBits(input);

    /*
     * The removeIntlvBits function removes the LSB from each mask from the
     * input address. For example, two masks:
     * 00000001 and,
     * 10000100
     * with an input address of:
     * 10101010
     *
     * we would remove bit at position 0, and at position 2, resulting in:
     * 00101011
     *
     * In this test there is is one mask, with a LSB at position 0.
     * Therefore, removing the interleaving bits is equivilant to bitshifting
     * the input to the right.
     */
    EXPECT_EQ(input >> 1, output);

    /*
     * The addIntlvBits function will re-insert bits at the removed locations
     */
    EXPECT_EQ(input, r.addIntlvBits(output));
}

TEST(AddrRangeTest, InterleavingAddressAddRemoveInterlvBitsTwoMasks)
{
    Addr start = 0x00000;
    Addr end   = 0x10000;
    std::vector<Addr> masks;
    masks.push_back((1 << 3) | (1 << 2) | (1 << 1) | 1);
    masks.push_back((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
    uint8_t intlv_match = 1;
    AddrRange r(start, end, masks, intlv_match);

    Addr input = (1 << 9) | (1 << 8) | 1;
    /*
     * (1 << 8) and 1 are interleaving bits to be removed.
     */
    Addr output = r.removeIntlvBits(input);

    /*
     * The bit, formally at position 9, is now at 7.
     */
    EXPECT_EQ((1 << 7), output);

    /*
     * Re-adding the interleaving.
     */
    EXPECT_EQ(input, r.addIntlvBits(output));
}

TEST(AddrRangeTest, AddRemoveInterleavBitsAcrossRange)
{
    /*
     * This purpose of this test is to ensure that removing then adding
     * interleaving bits has no net effect.
     * E.g.:
     * addr_range.addIntlvBits(add_range.removeIntlvBits(an_address)) should
     * always return an_address.
     */
    Addr start = 0x00000;
    Addr end   = 0x10000;
    std::vector<Addr> masks;
    masks.push_back(1 << 2);
    masks.push_back(1 << 3);
    masks.push_back(1 << 7);
    masks.push_back(1 << 11);
    uint8_t intlv_match = 0xF;
    AddrRange r(start, end, masks, intlv_match);

    for (Addr i = 0; i < 0xFFF; i++) {
        Addr removedBits = r.removeIntlvBits(i);
        /*
         * As intlv_match = 0xF, all the interleaved bits should be set.
         */
        EXPECT_EQ(i | (1 << 2) | (1 << 3) | (1 << 7) | (1 << 11),
                  r.addIntlvBits(removedBits));
    }
}

TEST(AddrRangeTest, AddRemoveInterleavBitsAcrossContiguousRange)
{
    /*
     * This purpose of this test is to ensure that removing then adding
     * interleaving bits has no net effect.
     * E.g.:
     * addr_range.addIntlvBits(add_range.removeIntlvBits(an_address)) should
     * always return an_address.
     */
    Addr start = 0x00000;
    Addr end   = 0x10000;
    std::vector<Addr> masks;
    masks.push_back(1 << 2);
    masks.push_back(1 << 3);
    masks.push_back(1 << 4);
    uint8_t intlv_match = 0x7;
    AddrRange r(start, end, masks, intlv_match);

    for (Addr i = 0; i < 0xFFF; i++) {
        Addr removedBits = r.removeIntlvBits(i);
        /*
         * As intlv_match = 0x7, all the interleaved bits should be set.
         */
        EXPECT_EQ(i | (1 << 2) | (1 << 3) | (1 << 4),
                  r.addIntlvBits(removedBits));
    }
}

TEST(AddrRangeTest, InterleavingAddressesGetOffset)
{
    Addr start = 0x0002;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match = 0;
    AddrRange r(start, end, masks, intlv_match);

    Addr value = ((1 << 10) | (1 << 9) | (1 <<  8) | (1 << 2) | (1 << 1) | 1);
    Addr value_interleaving_bits_removed =
                            ((1 << 9) | (1 << 8) | (1 << 7) | (1 << 1) | 1);

    Addr expected_output = value_interleaving_bits_removed - start;

    EXPECT_EQ(expected_output, r.getOffset(value));
}

TEST(AddrRangeTest, InterleavingLessThanStartEquals)
{
    Addr start1 = 0x0000FFFF;
    Addr end1   = 0xFFFF0000;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000FFFF;
    Addr end2   = 0x000F0000;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 4) | (1 << 2));
    masks2.push_back((1 << 10));
    uint8_t intlv_match2 = 2;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    /*
     * When The start addresses are equal, the intlv_match values are
     * compared.
     */
    EXPECT_TRUE(r1 < r2);
    EXPECT_FALSE(r2 < r1);
}

TEST(AddrRangeTest, InterleavingLessThanStartNotEquals)
{
    Addr start1 = 0x0000FFFF;
    Addr end1   = 0xFFFF0000;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000FFFE;
    Addr end2   = 0x000F0000;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 4) | (1 << 2));
    masks2.push_back((1 << 10));
    uint8_t intlv_match2 = 2;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_TRUE(r2 < r1);
    EXPECT_FALSE(r1 < r2);
}

TEST(AddrRangeTest, InterleavingEqualTo)
{
    Addr start1 = 0x0000FFFF;
    Addr end1   = 0xFFFF0000;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000FFFF;
    Addr end2   = 0xFFFF0000;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match2 = 0;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    EXPECT_TRUE(r1 == r2);
}

TEST(AddrRangeTest, InterleavingNotEqualTo)
{
    Addr start1 = 0x0000FFFF;
    Addr end1   = 0xFFFF0000;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000FFFF;
    Addr end2   = 0xFFFF0000;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 4) | (1 << 2));
    masks2.push_back((1 << 10));
    uint8_t intlv_match2 = 2;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    /*
     * These ranges are not equal due to having different masks.
     */
    EXPECT_FALSE(r1 == r2);
}

/*
 * The AddrRange(AddrRangeList) constructor "merges" the interleaving
 * address ranges. It should be noted that this constructor simply checks that
 * these interleaving addresses can be merged then creates a new address from
 * the start and end addresses of the first address range in the list.
 */
TEST(AddrRangeTest, MergingInterleavingAddressRanges)
{
    Addr start1 = 0x0000;
    Addr end1   = 0xFFFF;
    std::vector<Addr> masks1;
    masks1.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match1 = 0;
    AddrRange r1(start1, end1, masks1, intlv_match1);

    Addr start2 = 0x0000;
    Addr end2   = 0xFFFF;
    std::vector<Addr> masks2;
    masks2.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match2 = 1;
    AddrRange r2(start2, end2, masks2, intlv_match2);

    AddrRangeList to_merge;
    to_merge.push_back(r1);
    to_merge.push_back(r2);

    AddrRange output(to_merge);

    EXPECT_EQ(0x0000, output.start());
    EXPECT_EQ(0xFFFF, output.end());
    EXPECT_FALSE(output.interleaved());
}

TEST(AddrRangeTest, MergingInterleavingAddressRangesOneRange)
{
    /*
     * In the case where there is just one range in the list, the merged
     * address range is equal to that range.
     */
    Addr start = 0x0000;
    Addr end   = 0xFFFF;
    std::vector<Addr> masks;
    masks.push_back((1 << 4) | (1 << 2));
    uint8_t intlv_match = 0;
    AddrRange r(start, end, masks, intlv_match);

    AddrRangeList to_merge;
    to_merge.push_back(r);

    AddrRange output(to_merge);

    EXPECT_EQ(r, output);
}

/*
 * The following tests verify the soundness of the "legacy constructor",
 * AddrRange(Addr, Addr, uint8_t, uint8_t, uint8_t, uint8_t).
 *
 * The address is assumed to contain two ranges; the interleaving bits, and
 * the xor bits. The first two arguments of this constructor specify the
 * start and end addresses. The third argument specifies the MSB of the
 * interleaving bits. The fourth argument specifies the MSB of the xor bits.
 * The firth argument specifies the size (in bits) of the xor and interleaving
 * bits. These cannot overlap. The sixth argument specifies the value the
 * XORing of the xor and interleaving bits should equal to be considered in
 * range.
 *
 * This constructor does a lot of complex translation to migrate this
 * constructor to the masks/intlv_match format.
 */
TEST(AddrRangeTest, LegacyConstructorNoInterleaving)
{
    /*
     * This constructor should create a range with no interleaving.
     */
    AddrRange range(0x0000, 0xFFFF, 0, 0, 0 ,0);
    AddrRange expected(0x0000, 0xFFFF);

    EXPECT_EQ(expected, range);
}

TEST(AddrRangeTest, LegacyConstructorOneBitMask)
{
    /*
     * In this test, the LSB of the address determines whether an address is
     * in range. If even, it's in range, if not, it's out of range. the XOR
     * bit range is not used.
     */
    AddrRange range(0x00000000, 0xFFFFFFFF, 0, 0, 1, 0);

    std::vector<Addr> masks;
    masks.push_back(1);
    AddrRange expected(0x00000000, 0xFFFFFFFF, masks, 0);

    EXPECT_TRUE(expected == range);
}

TEST(AddrRangeTest, LegacyConstructorTwoBitMask)
{
    /*
     * In this test, the two LSBs of the address determines whether an address
     * is in range. If the two are set, the address is in range. The XOR bit
     * range is not used.
     */
    AddrRange range(0x00000000, 0xFFFFFFFF, 1, 0, 2, 3);

    std::vector<Addr> masks;
    masks.push_back(1);
    masks.push_back((1 << 1));
    AddrRange expected(0x00000000, 0xFFFFFFFF, masks, 3);

    EXPECT_TRUE(expected == range);
}

TEST(AddrRangeTest, LegacyConstructorTwoBitMaskWithXOR)
{
    /*
     * In this test, the two LSBs of the address determine wether an address
     * is in range. They are XORed to the 10th and 11th bits in the address.
     * If XORed value is equal to 3, then the address is in range.
     */

    AddrRange range(0x00000000, 0xFFFFFFFF, 1, 11, 2,  3);

    /*
     * The easiest way to ensure this range is correct is to iterate throguh
     * the address range and ensure the correct set of addresses are contained
     * within the range.
     *
     * We start with the xor_mask: a mask to select the 10th and 11th bits.
     */
    Addr xor_mask = (1 << 11) | (1 << 10);
    for (Addr i = 0; i < 0x0000FFFF; i++) {
        // Get xor bits.
        Addr xor_value = (xor_mask & i) >> 10;
        /* If the XOR of xor_bits and the intlv bits (the 0th and 1st bits) is
         * equal to intlv_match (3, i.e., the 0th and 1st bit is set),then the
         * address is within range.
         */
        if (((xor_value ^ i) & 3) == 3) {
            EXPECT_TRUE(range.contains(i));
        } else {
            EXPECT_FALSE(range.contains(i));
        }
    }
}

/*
 * addr_range.hh contains some convenience constructors. The following tests
 * verify they construct AddrRange correctly.
 */
TEST(AddrRangeTest, RangeExConstruction)
{
    AddrRange r = RangeEx(0x6, 0xE);
    EXPECT_EQ(0x6, r.start());
    EXPECT_EQ(0xE, r.end());
}

TEST(AddrRangeTest, RangeInConstruction)
{
    AddrRange r = RangeIn(0x6, 0xE);
    EXPECT_EQ(0x6, r.start());
    EXPECT_EQ(0xF, r.end());
}

TEST(AddrRangeTest, RangeSizeConstruction){
    AddrRange r = RangeSize(0x5, 5);
    EXPECT_EQ(0x5, r.start());
    EXPECT_EQ(0xA, r.end());
}

/*
 * The exclude list is excluding the entire range: return an empty
 * list of ranges
 *
 * |---------------------|
 * |       range         |
 * |---------------------|
 *
 * |------------------------------|
 * |       exclude_range          |
 * |------------------------------|
 */
TEST(AddrRangeTest, ExcludeAll)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x0, 0x200)
    };

    AddrRange r(0x00, 0x100);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_TRUE(ranges.empty());
}

/*
 * The exclude list is excluding the entire range: return an empty
 * list of ranges. The exclude_range = range
 *
 * |---------------------|
 * |       range         |
 * |---------------------|
 *
 * |---------------------|
 * |    exclude_range    |
 * |---------------------|
 */
TEST(AddrRangeTest, ExcludeAllEqual)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x0, 0x100)
    };

    AddrRange r(0x00, 0x100);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_TRUE(ranges.empty());
}

/*
 * The exclude list is made of multiple adjacent ranges covering the entire
 * interval: return an empty list of ranges.
 *
 * |---------------------------------------------------------------|
 * |                            range                              |
 * |---------------------------------------------------------------|
 *
 * |--------------------------|---------------|--------------------------|
 * |       exclude_range      | exclude_range |       exclude_range      |
 * |--------------------------|---------------|--------------------------|
 */
TEST(AddrRangeTest, ExcludeAllMultiple)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x0, 0x30),
        AddrRange(0x30, 0x40),
        AddrRange(0x40, 0x120)
    };

    AddrRange r(0x00, 0x100);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_TRUE(ranges.empty());
}

/*
 * ExcludeAllOverlapping:
 * The exclude list is made of multiple overlapping ranges covering the entire
 * interval: return an empty list of ranges.
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *  |-----------------------------|
 *  |       exclude_range         |
 *  |-----------------------------|
 *                          |-----------------------------|
 *                          |       exclude_range         |
 *                          |-----------------------------|
 */
TEST(AddrRangeTest, ExcludeAllOverlapping)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x0, 0x150),
        AddrRange(0x140, 0x220)
    };

    AddrRange r(0x100, 0x200);

    auto ranges = r.exclude(exclude_ranges);

    EXPECT_TRUE(ranges.empty());
}

/*
 * The exclude list is empty:
 * the return list contains the unmodified range
 *
 * |---------------------|
 * |       range         |
 * |---------------------|
 *
 */
TEST(AddrRangeTest, ExcludeEmpty)
{
    const AddrRangeList exclude_ranges;

    AddrRange r(0x00, 0x100);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 1);
    EXPECT_EQ(ranges.front(), r);
}


/*
 * Ranges do not overlap:
 * the return list contains the unmodified range
 *
 * |---------------------|
 * |       range         |
 * |---------------------|
 *
 *                       |------------------------------|
 *                       |       exclude_range          |
 *                       |------------------------------|
 */
TEST(AddrRangeTest, NoExclusion)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x100, 0x200)
    };

    AddrRange r(0x00, 0x100);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 1);
    EXPECT_EQ(ranges.front(), r);
}

/*
 * DoubleExclusion:
 * The exclusion should return two ranges:
 * AddrRange(0x130, 0x140)
 * AddrRange(0x170, 0x200)
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *  |-----------------|  |-----------------|
 *  |  exclude_range  |  |  exclude_range  |
 *  |-----------------|  |-----------------|
 */
TEST(AddrRangeTest, DoubleExclusion)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x000, 0x130),
        AddrRange(0x140, 0x170),
    };

    const AddrRange expected_range1(0x130, 0x140);
    const AddrRange expected_range2(0x170, 0x200);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

/*
 * MultipleExclusion:
 * The exclusion should return two ranges:
 * AddrRange(0x130, 0x140)
 * AddrRange(0x170, 0x180)
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *  |-----------------|  |-----------------|  |-----------------|
 *  |  exclude_range  |  |  exclude_range  |  |  exclude_range  |
 *  |-----------------|  |-----------------|  |-----------------|
 */
TEST(AddrRangeTest, MultipleExclusion)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x000, 0x130),
        AddrRange(0x140, 0x170),
        AddrRange(0x180, 0x210)
    };

    const AddrRange expected_range1(0x130, 0x140);
    const AddrRange expected_range2(0x170, 0x180);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

/*
 * MultipleExclusionOverlapping:
 * The exclusion should return one range:
 * AddrRange(0x130, 0x140)
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *  |-----------------|  |-----------------|
 *  |  exclude_range  |  |  exclude_range  |
 *  |-----------------|  |-----------------|
 *                                 |-----------------|
 *                                 |  exclude_range  |
 *                                 |-----------------|
 */
TEST(AddrRangeTest, MultipleExclusionOverlapping)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x000, 0x130),
        AddrRange(0x140, 0x170),
        AddrRange(0x150, 0x210)
    };

    const AddrRange expected_range1(0x130, 0x140);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 1);
    EXPECT_THAT(ranges, ElementsAre(expected_range1));
}

/*
 * ExclusionOverlapping:
 * The exclusion should return two range:
 * AddrRange(0x100, 0x120)
 * AddrRange(0x180, 0x200)
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *                   |--------------------|
 *                   |    exclude_range   |
 *                   |--------------------|
 *
 *                      |---------------|
 *                      | exclude_range |
 *                      |---------------|
 */
TEST(AddrRangeTest, ExclusionOverlapping)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x120, 0x180),
        AddrRange(0x130, 0x170)
    };

    const AddrRange expected_range1(0x100, 0x120);
    const AddrRange expected_range2(0x180, 0x200);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

/*
 * MultipleExclusionUnsorted:
 * The exclusion should return two ranges:
 * AddrRange(0x130, 0x140)
 * AddrRange(0x170, 0x180)
 * Same as MultipleExclusion, but the exclude list is provided
 * in unsorted order
 *
 *           |-----------------------------------|
 *           |              range                |
 *           |-----------------------------------|
 *
 *  |-----------------|  |-----------------|  |-----------------|
 *  |  exclude_range  |  |  exclude_range  |  |  exclude_range  |
 *  |-----------------|  |-----------------|  |-----------------|
 */
TEST(AddrRangeTest, MultipleExclusionUnsorted)
{
    const AddrRangeList exclude_ranges{
        AddrRange(0x180, 0x210),
        AddrRange(0x000, 0x130),
        AddrRange(0x140, 0x170)
    };

    const AddrRange expected_range1(0x130, 0x140);
    const AddrRange expected_range2(0x170, 0x180);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(exclude_ranges);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, ExclusionOfSingleRange)
{
    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x1c0, 0x200);

    AddrRange r(0x100, 0x200);
    auto ranges = r.exclude(AddrRange(0x140, 0x1c0));

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, ExclusionOfRangeFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x180);
    const AddrRange expected_range2(0x380, 0x400);

    auto ranges = exclude(base, AddrRange(0x180, 0x380));

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, ExclusionOfRangeListFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x180, 0x200);
    const AddrRange expected_range3(0x300, 0x340);
    const AddrRange expected_range4(0x380, 0x400);

    const AddrRangeList to_exclude({
            AddrRange(0x140, 0x180), AddrRange(0x340, 0x380)});
    auto ranges = exclude(base, to_exclude);

    EXPECT_EQ(ranges.size(), 4);
    EXPECT_THAT(ranges, ElementsAre(
                expected_range1, expected_range2,
                expected_range3, expected_range4));
}

TEST(AddrRangeTest, SubtractionOperatorRange)
{
    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x1c0, 0x200);

    AddrRange r(0x100, 0x200);
    auto ranges = r - AddrRange(0x140, 0x1c0);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, SubtractionOperatorRangeList)
{
    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x160, 0x180);
    const AddrRange expected_range3(0x1a0, 0x200);

    AddrRange r(0x100, 0x200);
    auto ranges = r - AddrRangeList(
            {AddrRange(0x140, 0x160), AddrRange(0x180, 0x1a0)});

    EXPECT_EQ(ranges.size(), 3);
    EXPECT_THAT(ranges, ElementsAre(
                expected_range1, expected_range2, expected_range3));
}

TEST(AddrRangeTest, SubtractionOfRangeFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x180);
    const AddrRange expected_range2(0x380, 0x400);

    auto ranges = base - AddrRange(0x180, 0x380);

    EXPECT_EQ(ranges.size(), 2);
    EXPECT_THAT(ranges, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, SubtractionOfRangeListFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x180, 0x200);
    const AddrRange expected_range3(0x300, 0x340);
    const AddrRange expected_range4(0x380, 0x400);

    const AddrRangeList to_exclude({
            AddrRange(0x140, 0x180), AddrRange(0x340, 0x380)});
    auto ranges = base - to_exclude;

    EXPECT_EQ(ranges.size(), 4);
    EXPECT_THAT(ranges, ElementsAre(
                expected_range1, expected_range2,
                expected_range3, expected_range4));
}

TEST(AddrRangeTest, SubtractionAssignmentOfRangeFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x180);
    const AddrRange expected_range2(0x380, 0x400);

    base -= AddrRange(0x180, 0x380);

    EXPECT_EQ(base.size(), 2);
    EXPECT_THAT(base, ElementsAre(expected_range1, expected_range2));
}

TEST(AddrRangeTest, SubtractionAssignmentOfRangeListFromRangeList)
{
    AddrRangeList base({AddrRange(0x100, 0x200), AddrRange(0x300, 0x400)});

    const AddrRange expected_range1(0x100, 0x140);
    const AddrRange expected_range2(0x180, 0x200);
    const AddrRange expected_range3(0x300, 0x340);
    const AddrRange expected_range4(0x380, 0x400);

    const AddrRangeList to_exclude({
            AddrRange(0x140, 0x180), AddrRange(0x340, 0x380)});
    base -= to_exclude;

    EXPECT_EQ(base.size(), 4);
    EXPECT_THAT(base, ElementsAre(
                expected_range1, expected_range2,
                expected_range3, expected_range4));
}

TEST(AddrRangeTest, isNotSubsetLastByte)
{
    /* An issue raised in https://github.com/gem5/gem5/issues/240 where if an
     * address range ends at the last byte of a 64 bit address space, it will
     * be considered a subset of any other address range that starts at the
     * first byte of the range.
     */

    AddrRange first_four_bytes = RangeSize(0x0, 4);
    AddrRange last_four_bytes = RangeSize(0xfffffffffffffffc, 4);

    EXPECT_FALSE(last_four_bytes.isSubset(first_four_bytes));
}

TEST(AddrRangeTest, isSubsetLastByte)
{
    /* An issue raised in https://github.com/gem5/gem5/issues/240 where if an
     * address range ends at the last byte of a 64 bit address space, it will
     * be considered a subset of any other address range that starts at the
     * first byte of the range.
     *
     * This test checks it subset works correctly when the range is the last
     * byte of the address space.
     */
    AddrRange last_four_bytes = RangeSize(0xfffffffffffffffc, 4);
    AddrRange not_wrapped_last_bytes = RangeSize(0xfffffffffffffffc, 3);

    EXPECT_TRUE(not_wrapped_last_bytes.isSubset(last_four_bytes));
}

/*
 * InterleavingRanges:
 * The exclude method does not support interleaving ranges
 */
TEST(AddrRangeDeathTest, ExcludeInterleavingRanges)
{
  /* An `assert(!interleaved());` exists at the top of the `exclude(...)`
   * method. This means EXPECT_DEATH will only function when DEBUG is enabled
   * (as when compiled to `.opt`). When disabled (as when compiled to `.fast`),
   * `r.exclude` fails more catastrophically via a `panic` which GTest cannot
   * handle correctly. We therefore include a `#ifdef NDEBUG` guard so this
   * test is skipped when DEBUG is disabled.
   */
#ifdef NDEBUG
    GTEST_SKIP() << "Skipping as assetions are stripped from fast builds.";
#endif
    const AddrRangeList exclude_ranges{
        AddrRange(0x180, 0x210),
    };

    AddrRange r(0x100, 0x200, std::vector<Addr>{1}, 0);

    EXPECT_TRUE(r.interleaved());
    EXPECT_DEATH(r.exclude(exclude_ranges), "");
}

/*
 * Modulo Interleaving Tests
 */

TEST(AddrRangeTest, ModuloSimple)
{
    // Range 0-300, 3 stripes.
    // Stripe 0: 0, 3, 6...
    // Stripe 1: 1, 4, 7...
    // Stripe 2: 2, 5, 8...
    AddrRange r0 = AddrRange::withModuloInterleaving(0, 300, 3, 0);
    AddrRange r1 = AddrRange::withModuloInterleaving(0, 300, 3, 1);
    AddrRange r2 = AddrRange::withModuloInterleaving(0, 300, 3, 2);

    EXPECT_TRUE(r0.contains(0));
    EXPECT_FALSE(r0.contains(1));
    EXPECT_FALSE(r0.contains(2));
    EXPECT_TRUE(r0.contains(3));
    EXPECT_FALSE(r0.contains(4));
    EXPECT_FALSE(r0.contains(5));

    EXPECT_FALSE(r1.contains(0));
    EXPECT_TRUE(r1.contains(1));
    EXPECT_FALSE(r1.contains(2));
    EXPECT_FALSE(r1.contains(3));
    EXPECT_TRUE(r1.contains(4));
    EXPECT_FALSE(r1.contains(5));

    EXPECT_FALSE(r2.contains(0));
    EXPECT_FALSE(r2.contains(1));
    EXPECT_TRUE(r2.contains(2));
    EXPECT_FALSE(r2.contains(3));
    EXPECT_FALSE(r2.contains(4));
    EXPECT_TRUE(r2.contains(5));

    EXPECT_EQ(100, r0.size());
    EXPECT_EQ(100, r1.size());
    EXPECT_EQ(100, r2.size());
}

TEST(AddrRangeTest, ModuloOffset)
{
    // Range 0-300, 3 stripes, intlv_bit=0.
    AddrRange r = AddrRange::withModuloInterleaving(0, 300, 3, 1);

    // 1 is the 0th element in this range.
    EXPECT_EQ(0, r.getOffset(1));
    // 4 is the 1st element.
    EXPECT_EQ(1, r.getOffset(4));
    // 7 is the 2nd element.
    EXPECT_EQ(2, r.getOffset(7));
}

TEST(AddrRangeTest, ModuloGranularity)
{
    // 2 stripes, granularity 4 (bit 2).
    // range 0-96.
    // stripe 0: 0-3, 8-11, 16-19...
    // stripe 1: 4-7, 12-15, 20-23...
    AddrRange r = AddrRange::withModuloInterleaving(0, 96, 2, 1, 2);

    EXPECT_FALSE(r.contains(0)); // 0>>2 = 0. 0%2 = 0 != 1.
    EXPECT_FALSE(r.contains(3));
    EXPECT_TRUE(r.contains(4)); // 4>>2 = 1. 1%2 = 1 == 1.
    EXPECT_TRUE(r.contains(7));
    EXPECT_FALSE(r.contains(8));

    // Size: 50?
    // 0-100 = 100 bytes.
    // 100 / 4 = 25 blocks.
    // 25 / 2 = 12 blocks for stripe 1 (since 25 is odd, 12.5? No integers).
    // Blocks: 0, 1, 2, ... 24.
    // Stripe 0: 0, 2, 4... 24 (13 blocks)
    // Stripe 1: 1, 3, 5... 23 (12 blocks)
    // Size = 12 * 4 = 48 bytes?
    // Let's check calculation: (100 - 0) -> compact(100) - compact(0).
    // compact(0) = 0.
    // compact(100): 100>>2 = 25. 25/2 = 12. 12<<2 = 48. 100&3 = 0. Total 48.
    // Correct.
    EXPECT_EQ(48, r.size());

    // Offset of 4 should be 0.
    EXPECT_EQ(0, r.getOffset(4));
    // Offset of 5 should be 1.
    EXPECT_EQ(1, r.getOffset(5));
    // Offset of 12 (block 3, matches 3%2=1)
    // Block 1 (4-7) -> local block 0 -> offset 0
    // Block 3 (12-15) -> local block 1 -> offset 4
    EXPECT_EQ(4, r.getOffset(12));
}

/*
 * Sparse Policy Tests
 */

TEST(AddrRangeTest, SparseSimple)
{
    // Ranges: 0-100, 200-300.
    std::vector<std::pair<Addr, Addr>> ranges = {{0, 100}, {200, 300}};
    AddrRange r(ranges);

    EXPECT_EQ(0, r.start());
    EXPECT_EQ(300, r.end());
    EXPECT_EQ(200, r.size()); // 100 + 100
    EXPECT_FALSE(r.interleaved());
    EXPECT_TRUE(r.isSparse());

    EXPECT_TRUE(r.contains(0));
    EXPECT_TRUE(r.contains(99));
    EXPECT_FALSE(r.contains(100)); // Hole check
    EXPECT_FALSE(r.contains(199));
    EXPECT_TRUE(r.contains(200));
    EXPECT_TRUE(r.contains(299));
    EXPECT_FALSE(r.contains(300));
}

TEST(AddrRangeTest, SparseOffset)
{
    // Ranges: 0-100, 200-300.
    std::vector<std::pair<Addr, Addr>> ranges = {{0, 100}, {200, 300}};
    AddrRange r(ranges);

    EXPECT_EQ(0, r.getOffset(0));
    EXPECT_EQ(99, r.getOffset(99));
    // 200 should map to 100 (start of second chunk maps to end of first
    // offsetwise)
    EXPECT_EQ(100, r.getOffset(200));
    EXPECT_EQ(101, r.getOffset(201));
    EXPECT_EQ(199, r.getOffset(299));
}

TEST(AddrRangeTest, SparseIntersection)
{
    // Sparse range: 0-100, 200-300
    std::vector<std::pair<Addr, Addr>> ranges = {{0, 100}, {200, 300}};
    AddrRange rSparse(ranges);

    // Flat range in hole: 100-200
    AddrRange rHole(100, 200);
    EXPECT_FALSE(rSparse.intersects(rHole));
    EXPECT_FALSE(rHole.intersects(rSparse));

    // Flat range overlapping first chunk
    AddrRange rOverlap1(50, 150);
    EXPECT_TRUE(rSparse.intersects(rOverlap1));
    EXPECT_TRUE(rOverlap1.intersects(rSparse));

    // Flat range overlapping second chunk
    AddrRange rOverlap2(250, 350);
    EXPECT_TRUE(rSparse.intersects(rOverlap2));
    EXPECT_TRUE(rOverlap2.intersects(rSparse));

    // Flat range fully containing sparse
    AddrRange rContainer(0, 400);
    EXPECT_TRUE(rSparse.intersects(rContainer));

    // Sparse vs Sparse disjoint
    // Sparse2: 100-200 (in hole of rSparse)
    std::vector<std::pair<Addr, Addr>> ranges2 = {{100, 200}};
    AddrRange rSparse2(ranges2);
    EXPECT_FALSE(rSparse.intersects(rSparse2));

    // Sparse vs Sparse overlapping
    // Sparse3: 50-150
    std::vector<std::pair<Addr, Addr>> ranges3 = {{50, 150}};
    AddrRange rSparse3(ranges3);
    EXPECT_TRUE(rSparse.intersects(rSparse3));
}

TEST(AddrRangeTest, ConstructorMergeTest)
{
    // 1. Compatible policies (Modulo) - Forward order
    // 2 stripes, bit 6.
    // Range: 0-256. Stripes: 0 (0-128 interleaved), 1 (0-128 interleaved).
    // Actually stripes split the address space.
    // 2 stripes, bit 0 (interleave bit).
    // range 0-100.
    // r1: match 0. (0, 2, 4...)
    // r2: match 1. (1, 3, 5...)
    // Combined: 0-100 flat.
    AddrRange m1 = AddrRange::withModuloInterleaving(0, 100, 2, 0, 0);
    AddrRange m2 = AddrRange::withModuloInterleaving(0, 100, 2, 1, 0);
    std::vector<AddrRange> ranges = {m1, m2};
    AddrRange merged(ranges);
    EXPECT_FALSE(merged.interleaved());
    EXPECT_EQ(merged.start(), 0);
    EXPECT_EQ(merged.end(), 100);

    // 2. Reverse order
    std::vector<AddrRange> rangesRev = {m2, m1};
    AddrRange mergedRev(rangesRev);
    EXPECT_FALSE(mergedRev.interleaved());
    EXPECT_EQ(mergedRev.start(), 0);
    EXPECT_EQ(mergedRev.end(), 100);

    // 3. Incompatible policies (Masked + Modulo)
    // Masked: 0-100, masks={1}, match=0
    AddrRange k1(0, 100, std::vector<Addr>{1}, 0);
    std::vector<AddrRange> mixed = {m1, k1};
    // Expected to fail
    EXPECT_ANY_THROW({ AddrRange r(mixed); });

    // 4. Incompatible policies (Modulo mismatch)
    // Different stripes
    AddrRange m3 = AddrRange::withModuloInterleaving(0, 99, 3, 0, 0);
    std::vector<AddrRange> mismatch = {m1, m3};
    EXPECT_ANY_THROW({ AddrRange r(mismatch); });
}

TEST(AddrRangeTest, NestedSparseTest)
{
    // 1. Sparse + Modulo
    // Range: 0-0x3000. Hole: 0x1000-0x2000.
    // Valid chunks: [0, 0x1002), [0x2000, 0x3002)
    // Interleaving: 3 stripes, match 0.
    // Interleaving: 3 stripes, match 0.
    std::vector<std::pair<Addr, Addr>> chunks = {{0, 0x1002},
                                                 {0x2000, 0x3002}};
    AddrRange r = AddrRange::withModuloInterleaving(chunks, 3, 0);

    // Check contains
    // System Interleaving:
    // 0x0 -> System 0. 0%3=0. Match.
    EXPECT_TRUE(r.contains(0x0));
    EXPECT_FALSE(r.contains(0x1));    // 1 mod 3 != 0
    EXPECT_FALSE(r.contains(0x1005)); // Hole
    EXPECT_FALSE(r.contains(0x1500)); // Hole
    // 0x2000 -> System 8192. 8192 mod 3 = 2. Match 0 -> False.
    EXPECT_FALSE(r.contains(0x2000));
    // 0x2001 -> System 8193. 8193 mod 3 = 0. Match 0 -> True.
    EXPECT_TRUE(r.contains(0x2001));
    // 0x2002 -> System 8194. 8194 mod 3 = 1. Match 0 -> False.
    EXPECT_FALSE(r.contains(0x2002));

    // Check size
    // Note: AddrRange delegates size calculation to the policy over the
    // individual sparse chunks. The size is the sum of sizes of all chunks.
    // Chunk 1: [0, 4098) -> size = toCompact(4098) - toCompact(0) = 1366.
    // Chunk 2: [8192, 12290)
    //          -> size = toCompact(12290) - toCompact(8192) = 1366.
    // Total size = 1366 + 1366 = 2732.
    EXPECT_EQ(r.size(), 2732);

    // 2. Sparse + Masked
    // Mask: bit 0 (value 1). Match 1 (odd addresses).
    // Valid chunks: [0, 10), [20, 30).
    std::vector<std::pair<Addr, Addr>> chunks2 = {{0, 10}, {20, 30}};
    AddrRange r2(chunks2, std::vector<Addr>{1}, 1);

    EXPECT_FALSE(r2.contains(0));  // Even
    EXPECT_TRUE(r2.contains(1));   // Odd
    EXPECT_FALSE(r2.contains(15)); // Hole
    EXPECT_TRUE(r2.contains(21));  // Odd in chunk 2

    // Size: 5 + 5 = 10.
    EXPECT_EQ(r2.size(), 10);
}

TEST(AddrRangeTest, SparseMergeTest)
{
    // Define chunks [0, 100), [200, 300)
    std::vector<std::pair<Addr, Addr>> chunks = {{0, 100}, {200, 300}};

    // Create 2 ranges with Modulo(2) interleaving
    // range1: Match 0
    AddrRange r1 = AddrRange::withModuloInterleaving(chunks, 2, 0);
    // range2: Match 1
    AddrRange r2 = AddrRange::withModuloInterleaving(chunks, 2, 1);

    // Merge them
    AddrRange merged(std::vector<AddrRange>{r1, r2});

    // Check if merged range is Sparse + Flat (no sub-policy)
    // Logical addresses:
    // [0, 100) -> Logical [0, 100)
    // [200, 300) -> Logical [100, 200)
    // Total logical size = 200.

    EXPECT_EQ(merged.size(), 200);
    EXPECT_TRUE(merged.contains(0));    // Chunk 1 start
    EXPECT_TRUE(merged.contains(99));   // Chunk 1 end
    EXPECT_FALSE(merged.contains(150)); // Hole
    EXPECT_TRUE(merged.contains(200));  // Chunk 2 start
    EXPECT_TRUE(merged.contains(299));  // Chunk 2 end

    // Verify interleaving property
    // Merged Modulo(2,0) + Modulo(2,1) -> Flat.
    // So "interleaved()" should be false (no interleaving policy).
    EXPECT_FALSE(merged.interleaved());
    // But isSparse() should be true
    EXPECT_TRUE(merged.isSparse());
}

TEST(AddrRangeTest, Phase8RegressionTest)
{
    // Simulate DualChannel Setup: 0-3GB, 4GB-End (16GB total -> 33GB Top)
    // 0-3GB: 0x0 - 0xC0000000
    // 4GB-34GB: 0x100000000 - 0x880000000 (Size 30GB)

    std::vector<std::pair<Addr, Addr>> chunks;
    chunks.emplace_back(0, 0xc0000000);
    chunks.emplace_back(0x100000000, 0x840000000);

    // Create Masks for 2 channels (1 bit, bit 6 for atom size 64)
    std::vector<Addr> masks = {1ULL << 6}; // 64-byte interleaving

    // Ctrl 0 (Match 0)
    AddrRange r0(chunks, masks, 0);
    // Ctrl 1 (Match 1)
    AddrRange r1(chunks, masks, 1);

    // 1. Verify Properties
    EXPECT_TRUE(r0.isSparse()) << "r0 should be sparse";
    EXPECT_TRUE(r1.isSparse()) << "r1 should be sparse";
    EXPECT_TRUE(r0.interleaved());
    EXPECT_TRUE(r1.interleaved());
    EXPECT_EQ(r0.decompose().size(), 2);

    // 2. Verify Merge
    EXPECT_TRUE(r0.mergesWith(r1)) << "Ranges should strictly merge";

    std::vector<AddrRange> rangeList = {r0, r1};
    AddrRange merged(rangeList);

    // The merged range should be Sparse (because it has chunks)
    // and Flat (because masks merged).
    EXPECT_TRUE(merged.isSparse())
        << "Merged result MUST be sparse if chunks exist";
    EXPECT_EQ(merged.decompose().size(), 2);
    EXPECT_FALSE(merged.interleaved());

    // Verify String output format
    // Expected: Sparse[0:0x840000000]:[0:0xc0000000]:[0x100000000:0x840000000]
    std::cout << "Merged String: " << merged.to_string() << std::endl;
}
