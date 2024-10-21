/*
 * Copyright (c) 2012, 2018 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include <vector>

#include "base/addr_range_map.hh"

using namespace gem5;

// Converted from legacy unit test framework
TEST(AddrRangeMapTest, LegacyTests)
{
    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    i = r.insert(RangeIn(10, 40), 5);
    ASSERT_NE(i, r.end());

    i = r.insert(RangeIn(60, 90), 3);
    ASSERT_NE(i, r.end());

    EXPECT_NE(r.intersects(RangeIn(20, 30)), r.end());
    EXPECT_EQ(r.contains(RangeIn(55, 55)), r.end());
    EXPECT_EQ(r.intersects(RangeIn(55, 55)), r.end());

    i = r.insert(RangeIn(0, 12), 1);
    EXPECT_EQ(i, r.end());

    i = r.insert(RangeIn(0, 9), 1);
    ASSERT_NE(i, r.end());

    EXPECT_NE(r.contains(RangeIn(20, 30)), r.end());
}

/**
 * Test AddrRangeMap with interleaved address ranges defined by bitmasks.
 * An AddrRangeMap containing a set of N interleaved address ranges,
 * defined with the same start and end address, and including all possible
 * intlvMatch values 0..N-1, must contain the start address.
 * For N-way interleaving, log2(N) selection masks are needed.
 * For N = 16, define the masks as follows,
 *
 * masks[0] = 1 << 6
 * masks[1] = 1 << 7
 * masks[2] = 1 << 8
 * masks[3] = 1 << 9
 *
 */
TEST(AddrRangeMapTest, InterleavedTest1)
{
    const auto N = 16;
    const auto masks = std::vector<Addr>{0x40, 0x80, 0x100, 0x200};
    const Addr start = 0x80000000;
    const Addr end = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, masks, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 0 for start = 0x80000000
    EXPECT_EQ(i->second, 0);
}

/**
 * Test AddrRangeMap with interleaved address ranges defined by bitmasks.
 * An AddrRangeMap containing a set of N interleaved address ranges,
 * defined with the same start and end address, and including all possible
 * intlvMatch values 0..N-1, must contain the start address.
 * For N-way interleaving, log2(N) selection masks are needed.
 * For N = 16, define the masks as described in the
 * CMN-600 Technical Reference Manual [1], section 2.17.3
 *
 * masks[0] = 1 << 6 | 1 << 10 | 1 << 14 | .. | 1 << 50
 * masks[1] = 1 << 7 | 1 << 11 | 1 << 15 | .. | 1 << 51
 * masks[2] = 1 << 8 | 1 << 12 | 1 << 16 | .. | 1 << 48
 * masks[3] = 1 << 9 | 1 << 13 | 1 << 17 | .. | 1 << 49
 *
 * [1] https://developer.arm.com/documentation/100180/0302
 */
TEST(AddrRangeMapTest, InterleavedTest2)
{
    const auto N = 16;
    const auto masks = std::vector<Addr>{0x4444444444440, 0x8888888888880,
                                         0x1111111111100, 0x2222222222200};
    const Addr start = 0x80000000;
    const Addr end = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, masks, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 2 for start = 0x80000000
    EXPECT_EQ(i->second, 2);
}

TEST(AddrRangeMapTest, ModuloInterleavedTest1)
{
    const auto N = 16;

    const Addr start = 0x80000000;
    const Addr end = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, N, 6, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 0 for start = 0x80000000
    EXPECT_EQ(i->second, 0);
}

TEST(AddrRangeMapTest, ModuloInterleavedTest2)
{
    // testing prime number interleaving
    const auto N = 17;

    const Addr start = 0x80000000;
    const Addr end = 0xc0000000;

    AddrRangeMap<int> r;
    AddrRangeMap<int>::const_iterator i;

    // populate AddrRangeMap with N-way interleaved address ranges
    // for all intlvMatch values 0..N-1
    for (int k=0; k < N; k++) {
        r.insert(AddrRange(start, end, N, 6, k), k);
    }
    // find AddrRange element containing start address
    i = r.contains(start);
    // i must not be the past-the-end iterator
    ASSERT_NE(i, r.end()) << "start address not found in AddrRangeMap";
    // intlvMatch = 0 for start = 0x80000000
    EXPECT_EQ(i->second, 2);

    // we are interleaving every 64 bytes,
    // due to the modulo starting 6 bits from the LSB
    // test this by increasing the address by 64 bytes

    Addr test_addr = start + 0x40;
    // initial value found by start shifted 6 bits then % by N, should be 2
    int base_index = start >> 6 % N;
    for (int index = 1; index < N; index++) {
        i = r.contains(test_addr);
        // i must not be the past-the-end iterator
        ASSERT_NE(i, r.end()) << "address not found in AddrRangeMap";
        // intlvMatch = should increment every test_addr+=0x40
        EXPECT_EQ(i->second, (index + base_index) % N);
        test_addr += 0x40;
    }
}
