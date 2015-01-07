/*
 * Copyright 2014 Google, Inc.
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
 * Authors: Gabe Black
 */

#include <cassert>
#include <iostream>

#include "base/bitunion.hh"
#include "base/cprintf.hh"
#include "unittest/unittest.hh"

using namespace std;
using UnitTest::setCase;

namespace {

BitUnion64(SixtyFour)
    Bitfield<39, 32> byte5;
    Bitfield<2> bit2;
    BitfieldRO<39, 32> byte5RO;
    BitfieldWO<39, 32> byte5WO;
    SubBitUnion(byte6, 47, 40)
        Bitfield<43, 42> bits43To42;
        Bitfield<41> bit41;
        SignedBitfield<41> bit41Signed;
    EndSubBitUnion(byte6)
    SignedBitfield<47, 40> byte6Signed;
    SignedBitfieldRO<47, 40> byte6SignedRO;
    SignedBitfieldWO<47, 40> byte6SignedWO;
EndBitUnion(SixtyFour)

BitUnion64(EmptySixtyFour)
EndBitUnion(EmptySixtyFour)

BitUnion32(EmptyThirtyTwo)
EndBitUnion(EmptyThirtyTwo)

BitUnion16(EmptySixteen)
EndBitUnion(EmptySixteen)

BitUnion8(EmptyEight)
EndBitUnion(EmptyEight)

struct ContainingStruct
{
    BitUnion64(Contained)
        Bitfield<63, 60> topNibble;
    EndBitUnion(Contained)

    Contained contained;
};

uint64_t
containingFunc(uint64_t init_val, uint64_t fieldVal)
{
    BitUnion32(Contained)
        Bitfield<16, 15> field;
    EndBitUnion(Contained)

    Contained contained = init_val;
    contained.field = fieldVal;
    return contained;
}

} // anonymous namespace

// Declare these as global so g++ doesn't ignore them. Initialize them in
// various ways.
EmptySixtyFour emptySixtyFour = 0;
EmptyThirtyTwo emptyThirtyTwo;
EmptySixteen emptySixteen;
EmptyEight emptyEight(0);

int
main()
{
    SixtyFour sixtyFour = 0;

    setCase("normal bitfield");
    EXPECT_EQ(sixtyFour.byte5, 0);
    sixtyFour.byte5 = 0xff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
    sixtyFour.byte5 = 0xfff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
    EXPECT_EQ(sixtyFour.byte5, 0xff);
    sixtyFour = 0;

    setCase("single bitfield");
    EXPECT_EQ(sixtyFour.bit2, 0);
    sixtyFour.bit2 = 0x1;
    EXPECT_EQ(sixtyFour, 0x4);
    EXPECT_EQ(sixtyFour.bit2, 0x1);
    sixtyFour = 0;

    setCase("read only bitfield");
    EXPECT_EQ(sixtyFour.byte5RO, 0);
    sixtyFour.byte5 = 0xff;
    EXPECT_EQ(sixtyFour.byte5RO, 0xff);
    sixtyFour = 0;

    setCase("write only bitfield");
    sixtyFour.byte5WO = 0xff;
    EXPECT_EQ(sixtyFour, 0xff00000000);
    sixtyFour = 0;

    setCase("sub bitunions and their bitfields");
    EXPECT_EQ(sixtyFour.byte6.bit41, 0);
    sixtyFour.byte6 = 0x2;
    EXPECT_EQ(sixtyFour.byte6.bit41, 1);
    sixtyFour.byte6.bits43To42 = 0x3;
    EXPECT_EQ(sixtyFour.byte6, 0xe);
    sixtyFour.byte6 = 0xff;
    sixtyFour.byte6.bit41 = 0;
    EXPECT_EQ(sixtyFour, 0xfd0000000000);
    sixtyFour = 0;

    setCase("normal, read only, and write only signed bitfields");
    sixtyFour.byte6 = 0xff;
    EXPECT_EQ(sixtyFour.byte6Signed, -1);
    EXPECT_EQ(sixtyFour.byte6SignedRO, -1);
    sixtyFour.byte6SignedWO = 0;
    EXPECT_EQ(sixtyFour.byte6Signed, 0);
    EXPECT_EQ(sixtyFour.byte6SignedRO, 0);
    EXPECT_EQ(sixtyFour.byte6, 0);
    sixtyFour = 0;

    setCase("bitunion declared inside a struct");
    ContainingStruct containing;
    containing.contained = 0;
    containing.contained.topNibble = 0xd;
    EXPECT_EQ(containing.contained, 0xd000000000000000);

    setCase("bitunion declared inside a function");
    EXPECT_EQ(containingFunc(0xfffff, 0), 0xe7fff);

    setCase("assigning bitfields to other bitfields");
    SixtyFour otherSixtyFour = 0;
    sixtyFour.bit2 = 1;
    otherSixtyFour.byte6.bit41 = sixtyFour.bit2;
    EXPECT_EQ(otherSixtyFour, 0x20000000000);
    otherSixtyFour.bit2 = sixtyFour.bit2;
    EXPECT_EQ(otherSixtyFour, 0x20000000004);

    setCase("bitunion operators");
    sixtyFour = 0;
    otherSixtyFour = 0x4;
    sixtyFour = otherSixtyFour;
    EXPECT_EQ(sixtyFour, 0x4);
    sixtyFour = 0;
    EXPECT_TRUE(sixtyFour < otherSixtyFour);
    EXPECT_TRUE(otherSixtyFour > sixtyFour);
    EXPECT_TRUE(sixtyFour != otherSixtyFour);
    sixtyFour = otherSixtyFour;
    EXPECT_TRUE(sixtyFour == otherSixtyFour);

    return UnitTest::printResults();
}
