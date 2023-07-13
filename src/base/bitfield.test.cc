/*
 * Copyright (c) 2019 The Regents of the University of California
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

#include <gtest/gtest.h>

#include "base/bitfield.hh"

using namespace gem5;

/*
 * The following tests the "mask(N)" function. It is assumed that the mask
 * returned is a 64 bit value with the N LSBs set to one.
 */
TEST(BitfieldTest, Mask0Bits)
{
    EXPECT_EQ(0x0, mask(0));
}

TEST(BitfieldTest, Mask1Bit)
{
    EXPECT_EQ(0x1, mask(1));
}

TEST(BitfieldTest, Mask8Bits)
{
    EXPECT_EQ(0xFF, mask(8));
}

TEST(BitfieldTest, Mask16Bits)
{
    EXPECT_EQ(0xFFFF, mask(16));
}

TEST(BitfieldTest, Mask32Bits)
{
    EXPECT_EQ(0xFFFFFFFF, mask(32));
}

TEST(BitfieldTest, MaskAllBits)
{
    EXPECT_EQ(0xFFFFFFFFFFFFFFFF, mask(64));
}

TEST(BitfieldTest, MaskAllBitsGreaterThan64)
{
    /* We cannot create a mask greater than 64 bits. It should default to 64
     * bits if this occurs.
     */
    EXPECT_EQ(0xFFFFFFFFFFFFFFFF, mask(70));
}

/*
 * The following tests "mask(X, Y)". mask will create a 64 bit value with bits
 * X to Y (inclusive) set to one.
 */
TEST(BitfieldTest, MaskOneBit)
{
    EXPECT_EQ(1, mask(0, 0));
}

TEST(BitfieldTest, MaskTwoBits)
{
    EXPECT_EQ((1 << 1) + 1, mask(1, 0));
}

TEST(BitfieldTest, MaskThreeBits)
{
    EXPECT_EQ((1 << 5) + (1 << 4) + (1 << 3), mask(5, 3));
}

TEST(BitfieldTest, MaskEntireRange)
{
    EXPECT_EQ(0xFFFFFFFFFFFFFFFF, mask(63, 0));
}

TEST(BitfieldTest, MaskOutsideOfRange)
{
    // Masking >64 bits is not possible. The maximum is a 64 bit mask.
    EXPECT_EQ(0xFFFFFFFFFFFFFFFF, mask(100, 0));
}

/*
 * The following tests "bits". This function extracts bit/bits from the input
 * value and return them as the LSBs. The remaining bits are set to zero.
 */
TEST(BitfieldTest, ExtractOneBit)
{
    int32_t x = 1 << 31;
    EXPECT_EQ(1, bits(x, 31));
}

TEST(BitfieldTest, Extract63rdBit)
{
    int64_t x = 1ULL << 63;
    EXPECT_EQ(1, bits(x, 63));
}

TEST(BitfieldTest, ExtractFirstBit)
{
    int64_t x = 1;
    EXPECT_EQ(1, bits(x, 0));
}

TEST(BitfieldTest, ExtractFirstBitFirstBitZero)
{
    int64_t x = 1 << 1;
    EXPECT_EQ(0, bits(x, 0));
}

TEST(BitfieldTest, ExtractThreeBits)
{
    uint64_t x = 1 << 31;
    EXPECT_EQ((1 << 2), bits(x, 31, 29));
}


/*
 * The following tests "mbits(X, Y, Z)". mbits returns a value with bits Y to
 * Z from X (in position Y to Z).
 */
TEST(BitfieldTest, MbitsStandardCase)
{
    uint64_t x = (1 << 10) + (1 << 1);
    EXPECT_EQ((1 << 10), mbits(x, 10, 8));
}

TEST(BitfieldTest, MbitsEntireRange)
{
    uint64_t x = (1ULL << 63) + 1;
    EXPECT_EQ((1ULL << 63) + 1, mbits(x, 63, 0));
}

/*
 * The following tests the "sext<N>(X)" function. sext carries out a sign
 * extention from N bits to 64 bits on value X. It does not zero bits past the
 * sign bit if it was zero.
 */
TEST(BitfieldTest, SignExtendPositiveInput)
{
    int8_t val = 14;
    int64_t output = 14;
    EXPECT_EQ(output, sext<8>(val));
}

TEST(BitfieldTest, SignExtendNegativeInput)
{
    int8_t val = -14;
    uint64_t output = -14;
    EXPECT_EQ(output, sext<8>(val));
}

TEST(BitfieldTest, SignExtendPositiveInputOutsideRange)
{
    EXPECT_EQ((1 << 10), sext<8>(1 << 10));
}

TEST(BitfieldTest, SignExtendNegativeInputOutsideRange)
{
    uint64_t val = 0x4800000010000008;
    uint64_t output = 0xF800000010000008;
    EXPECT_EQ(output, sext<60>(val));
}
/*
 * The following tests the "szext<N>(X)" function. szext carries out a sign
 * extention from N bits to 64 bits on value X. Will zero bits past the sign
 * bit if it was zero.
 */
TEST(BitfieldTest, SignZeroExtendPositiveInput)
{
    int8_t val = 14;
    int64_t output = 14;
    EXPECT_EQ(output, szext<8>(val));
}

TEST(BitfieldTest, SignZeroExtendNegativeInput)
{
    int8_t val = -14;
    uint64_t output = -14;
    EXPECT_EQ(output, szext<8>(val));
}

TEST(BitfieldTest, SignZeroExtendPositiveInputOutsideRange)
{
    EXPECT_EQ(0, szext<8>(1 << 10));
}

TEST(BitfieldTest, SignZeroExtendNegativeInputOutsideRange)
{
    uint64_t val = 0x4800000010000008;
    uint64_t output = 0xF800000010000008;
    EXPECT_EQ(output, szext<60>(val));
}

/* The following tests "insertBits(A, B, C, D)". insertBits returns A
 * with bits B to C set to D's (B - C) LSBs. "insertBits(A, B, D)" overrides
 * the function to insert only B's LSB to position B.
 */
TEST(BitfieldTest, InsertOneBitTo3)
{
    int64_t val = 0;
    int64_t bits = (1 << 3) + (1 << 2) + (1 << 1) + 1;
    EXPECT_EQ((1 << 3), insertBits(val, 3, bits));
}

TEST(BitfieldTest, InsertOneBitTo18)
{
    int64_t val = 0;
    int64_t bits = (1 << 3) + (1 << 2) + (1 << 1) + 1;
    EXPECT_EQ((1 << 18), insertBits(val, 18, bits));
}

TEST(BitfieldTest, InsertOneBitTo3LsbZero)
{
    int64_t val = 0;
    int64_t bits = (1 << 3) + (1 << 2) + (1 << 1);
    EXPECT_EQ(0, insertBits(val, 3, bits));
}

TEST(BitfieldTest, InsertOneBitTo18LsbZero)
{
    int64_t val = 0;
    int64_t bits = (1 << 3) + (1 << 2) + (1 << 1);
    EXPECT_EQ(0, insertBits(val, 18, bits));
}

TEST(BitfieldTest, InsertOnBitTo8LsbZero)
{
    int64_t val = (1 << 8);
    int64_t bits = (1 << 3) + (1 << 2) + (1 << 1);
    EXPECT_EQ(0, insertBits(val, 8, bits));
}

TEST(BitfieldTest, InsertMultipleBits)
{
    int64_t val = (1ULL << 63);
    int64_t bits = (1 << 2) + 1;
    EXPECT_EQ(val + (1 << 5) + (1 << 3), insertBits(val, 5, 3, bits));
}

TEST(BitfieldTest, InsertMultipleBitsOverwrite)
{
    int64_t val = (1 << 29);
    int64_t bits = (1 << 2) + 1;
    EXPECT_EQ((1 << 30) + (1 << 28), insertBits(val, 30, 28, bits));
}

// The following tests the "reverseBits" function.
TEST(BitfieldTest, ReverseBits8Bit)
{
    uint8_t value = (1 << 7);
    EXPECT_EQ(1, reverseBits(value));
}

TEST(BitfieldTest, ReverseBits64Bit)
{
    uint64_t value = 0xF0F0F0F0F0F0F0F1;
    EXPECT_EQ(0x8F0F0F0F0F0F0F0F, reverseBits(value));
}

/* The following tests "findMsb" and "findLsb". These return the most position
 * of the MSBs/LSBs of the input value.
 */
TEST(BitfieldTest, FindMsb29)
{
    uint64_t val = (1 << 29) + (1 << 1);
    EXPECT_EQ(29, findMsbSet(val));
}

TEST(BitfieldTest, FindMsb63)
{
    uint64_t val = (1ULL << 63) + (1ULL << 60) + (1 << 1);
    EXPECT_EQ(63, findMsbSet(val));
}


TEST(BitfieldTest, FindMsbZero)
{
    EXPECT_EQ(0, findMsbSet(0));
}

TEST(BitfieldTest, FindLsb)
{
    uint64_t val = (1ULL << 63) + (1 << 1);
    EXPECT_EQ(1, findLsbSet(val));
    EXPECT_EQ(1, findLsbSetFallback(val));
}

TEST(BitfieldTest, FindLsbZero)
{
    EXPECT_EQ(64, findLsbSet(0));
}

TEST(BitfieldTest, FindLsbGeneralized)
{
    static constexpr size_t N{1000};
    std::bitset<N> bs{0};
    EXPECT_EQ(findLsbSet(bs), N);
    for (size_t i{0}; i < N ; ++i) {
        bs = std::bitset<N>{1} << i;
        ASSERT_EQ(findLsbSet(bs), i);
    }

    const auto leadingOne = std::bitset<N>{1} << (N-1);
    for (size_t i{0}; i < N ; ++i) {
        bs = leadingOne | (std::bitset<N>{1} << i);
        ASSERT_EQ(findLsbSet(bs), i);
    }
}

/*
 * The following tests "popCount(X)". popCount counts the number of bits set to
 * one.
 */
TEST(BitfieldTest, PopCountNoBits)
{
    EXPECT_EQ(0, popCount(0));
}

TEST(BitfieldTest, PopCountOneBit)
{
    int64_t val = (1 << 9);
    EXPECT_EQ(1, popCount(val));
}

TEST(BitfieldTest, PopCountManyBits)
{
    int64_t val = (1 << 22) + (1 << 21) + (1 << 15) + (1 << 9) + 1;
    EXPECT_EQ(5, popCount(val));
}

TEST(BitfieldTest, PopCountAllOnes)
{
    int64_t val = 0xFFFFFFFFFFFFFFFF;
    EXPECT_EQ(64, popCount(val));
}

/*
 * The following tests the "alignToPowerOfTwo(x)" function which rounds
 * uint64_t x up to the nearest power of two. If x is already a power
 * of two, that power is returned.
 */
TEST(BitfieldTest, AlignToPowerOfTwo0)
{
    EXPECT_EQ(0, alignToPowerOfTwo(0));
}

TEST(BitfieldTest, AlignToPowerOfTwo3)
{
    EXPECT_EQ(4, alignToPowerOfTwo(3));
}

TEST(BitfieldTest, AlignToPowerOfTwo5)
{
    EXPECT_EQ(8, alignToPowerOfTwo(5));
}

TEST(BitfieldTest, AlignToPowerOfTwo10)
{
    EXPECT_EQ(16, alignToPowerOfTwo(10));
}

TEST(BitfieldTest, AlignToPowerOfTwo16)
{
    EXPECT_EQ(16, alignToPowerOfTwo(16));
}

TEST(BitfieldTest, AlignToPowerOfTwo31)
{
    EXPECT_EQ(32, alignToPowerOfTwo(31));
}

/*
 * The following tests test ctz32/64. The value returned in all cases should
 * be equal to the number of trailing zeros (i.e., the number before the first
 * bit set to one).
 */

TEST(BitfieldTest, CountTrailingZeros32BitsNoTrailing)
{
    int32_t value = 1;
    EXPECT_EQ(0, ctz32(value));
}

TEST(BitfieldTest, CountTrailingZeros32Bits)
{
    uint32_t value = (1 << 30) + (1 << 29);
    EXPECT_EQ(29, ctz32(value));
}

TEST(BitfieldTest, CountTrailingZeros64BitsNoTrailing)
{
    uint64_t value = (1 << 29) + 1;
    EXPECT_EQ(0, ctz64(value));
}

TEST(BitfieldTest, CountTrailingZeros64Bits)
{
    uint64_t value = 1ULL << 63;
    EXPECT_EQ(63, ctz64(value));
}

TEST(BitfieldTest, CountTrailingZero64AllZeros)
{
    uint64_t value = 0;
    EXPECT_EQ(64, ctz64(value));
}

/*
 * The following tests test clz32/64. The value returned in all cases should
 * be equal to the number of leading zeros (i.e., the number of zeroes before
 * the first bit set to one counting from the MSB).
 */

TEST(BitfieldTest, CountLeadingZeros32BitsNoTrailing)
{
    int32_t value = 1;
    EXPECT_EQ(31, clz32(value));
}

TEST(BitfieldTest, CountLeadingZeros32Bits)
{
    uint32_t value = (1 << 30) + (1 << 29);
    EXPECT_EQ(1, clz32(value));
}

TEST(BitfieldTest, CountLeadingZeros64BitsNoTrailing)
{
    uint64_t value = (1 << 29) + 1;
    EXPECT_EQ(34, clz64(value));
}

TEST(BitfieldTest, CountLeadingZeros64Bits)
{
    uint64_t value = 1ULL << 63;
    EXPECT_EQ(0, clz64(value));
}

TEST(BitfieldTest, CountLeadingZero64AllZeros)
{
    uint64_t value = 0;
    EXPECT_EQ(64, clz64(value));
}
