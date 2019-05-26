/*
 * Copyright (c) 2018-2019 ARM Limited
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
 * Authors: Nikos Nikoleris
 */

#include <gtest/gtest.h>

#include "base/addr_range.hh"
#include "base/bitfield.hh"

TEST(AddrRangeComp, AddrRangeIsSubset)
{
    AddrRange r, r1, r2;

    // Test non-interleaved ranges
    r1 = AddrRange(0x0, 0x7f);
    r2 = AddrRange(0x80, 0xff);

    r = AddrRange(0x0, 0xf);
    EXPECT_TRUE(r.isSubset(r1));
    EXPECT_FALSE(r.isSubset(r2));

    r = AddrRange(0x80, 0x8f);
    EXPECT_FALSE(r.isSubset(r1));
    EXPECT_TRUE(r.isSubset(r2));

    // Test interleaved ranges
    r1 = AddrRange(0x0, 0xff, 6, 0, 1, 0);
    r2 = AddrRange(0x0, 0xff, 6, 0, 1, 1);

    r = AddrRange(0x0, 0xf);
    EXPECT_TRUE(r.isSubset(r1));
    EXPECT_FALSE(r.isSubset(r2));

    r = AddrRange(0x40, 0x4f);
    EXPECT_FALSE(r.isSubset(r1));
    EXPECT_TRUE(r.isSubset(r2));

    r = AddrRange(0xbf, 0xc0);
    EXPECT_FALSE(r.isSubset(r1));
    EXPECT_FALSE(r.isSubset(r2));

    // Test interleaved ranges with hashing
    r1 = AddrRange(0x0, 0xff, 6, 7, 1, 0);
    r2 = AddrRange(0x0, 0xff, 6, 7, 1, 1);

    r = AddrRange(0x0, 0xf);
    EXPECT_TRUE(r.isSubset(r1));
    EXPECT_FALSE(r.isSubset(r2));

    r = AddrRange(0x40, 0x4f);
    EXPECT_FALSE(r.isSubset(r1));
    EXPECT_TRUE(r.isSubset(r2));

    r = AddrRange(0xbf, 0xc0);
    EXPECT_FALSE(r.isSubset(r1));
    EXPECT_FALSE(r.isSubset(r2));
}

class AddrRangeBase : public testing::Test {
  protected:

    virtual int getIndex(Addr addr) = 0;

    void testContains()
    {
        for (Addr addr = start; addr <= end; addr++) {
            int i = getIndex(addr);
            ASSERT_TRUE(range[i].contains(addr));
            for (int j = 1; j < intlvSize; j++) {
                ASSERT_FALSE(range[(i + j) % intlvSize].contains(addr));
            }
        }
    }

    void testGetOffset()
    {
        Addr offsets[intlvSize] = {0, 0, 0, 0};
        for (Addr addr = start; addr <= end; addr++) {
            int i = getIndex(addr);
            Addr offset = range[i].getOffset(addr);
            ASSERT_EQ(offsets[i], offset);
            offsets[i]++;
        }
        for (Addr offset: offsets) {
            ASSERT_EQ(offset, (end - start + 1) / intlvSize);
        }
    }

    static const Addr end = 0x1ffff;
    static const Addr start = 0x0;
    static const int intlvSize = 4;

    AddrRange range[intlvSize];
};


class AddrRangeCont : public AddrRangeBase {
  protected:
    void SetUp() override
    {
        std::vector<Addr> masks = {
            1UL << xorBits0[0] | 1UL << xorBits0[1],
            1UL << xorBits1[0] | 1UL << xorBits1[1]
        };
        for (auto i = 0; i < intlvSize; i++) {
            range[i] = AddrRange(start, end, masks, i);
        }
    }

    int getIndex(Addr addr) override
    {
        return bits(addr, xorBits1[1], xorBits0[1]) ^
            bits(addr, xorBits1[0], xorBits0[0]);
    }

    const int xorBits0[2] = {8, 14};
    const int xorBits1[2] = {9, 15};
};

TEST_F(AddrRangeCont, AddrRangeContains)
{
    testContains();
}

TEST_F(AddrRangeCont, AddrRangeGetOffset)
{
    testGetOffset();
}


class AddrRangeContLegacy : public AddrRangeCont {
  protected:
    void SetUp() override
    {
        // Test interleaved ranges with hashing
        for (auto i = 0; i < intlvSize; i++) {
            range[i] = AddrRange(start, end, xorBits1[0], xorBits1[1],
                                 2, i);
        }
    }
};

TEST_F(AddrRangeContLegacy, AddrRangeContains)
{
    testContains();
}

TEST_F(AddrRangeContLegacy, AddrRangeGetOffset)
{
    testGetOffset();
}


class AddrRangeArb : public AddrRangeBase {
  protected:
    void SetUp() override
    {
        std::vector<Addr> masks = {
            1UL << xorBits0[0] | 1UL << xorBits0[1],
            1UL << xorBits1[0] | 1UL << xorBits1[1]
        };
        for (auto i = 0; i < intlvSize; i++) {
            range[i] = AddrRange(start, end, masks, i);
        }
    }

    int getIndex(Addr addr) override
    {
        return (bits(addr, xorBits0[0]) ^ bits(addr, xorBits0[1])) |
            (bits(addr, xorBits1[0]) ^ bits(addr, xorBits1[1])) << 1;
    }

    const int xorBits0[2] = {11, 12};
    const int xorBits1[2] = {8, 15};
};

TEST_F(AddrRangeArb, AddrRangeContains)
{
    testContains();
}

TEST_F(AddrRangeArb, AddrRangeGetOffset)
{
    testGetOffset();
}
