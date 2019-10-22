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
 *
 * Authors: Mahyar Samani
 */

#include <gtest/gtest.h>

#include "sim/byteswap.hh"

TEST(ByteswapTest, swap_byte64)
{
    EXPECT_EQ(0x0123456789abcdef, swap_byte64(0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210, swap_byte64(0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1, swap_byte64(0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff, swap_byte64(0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555, swap_byte64(0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, swap_byte64(0xa0a0a0a0a0a0a0a0));
}

TEST(ByteswapTest, swap_byte32)
{
    EXPECT_EQ(0x0123cdef, swap_byte32(0xefcd2301));
    EXPECT_EQ(0xfedc3210, swap_byte32(0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, swap_byte32(0xb1411c0b));
    EXPECT_EQ(0x0000ffff, swap_byte32(0xffff0000));
    EXPECT_EQ(0x55555555, swap_byte32(0x55555555));
    EXPECT_EQ(0xa0a0a0a0, swap_byte32(0xa0a0a0a0));
}

TEST(ByteswapTest, swap_byte16)
{
    EXPECT_EQ(0x01ef, swap_byte16(0xef01));
    EXPECT_EQ(0xfe10, swap_byte16(0x10fe));
    EXPECT_EQ(0x0bb1, swap_byte16(0xb10b));
    EXPECT_EQ(0x00ff, swap_byte16(0xff00));
    EXPECT_EQ(0x5555, swap_byte16(0x5555));
    EXPECT_EQ(0xa0a0, swap_byte16(0xa0a0));
}

TEST(ByteswapTest, swap_byte)
{
    EXPECT_EQ(0x0123456789abcdef, swap_byte((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210, swap_byte((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1, swap_byte((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff, swap_byte((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555, swap_byte((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, swap_byte((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, swap_byte((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, swap_byte((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, swap_byte((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, swap_byte((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, swap_byte((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, swap_byte((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, swap_byte((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, swap_byte((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, swap_byte((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, swap_byte((uint16_t)0xff00));
    EXPECT_EQ(0x5555, swap_byte((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, swap_byte((uint16_t)0xa0a0));
}

TEST(ByteswapTest, htog)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0xefcdab8967452301, htog((uint64_t)0xefcdab8967452301,
                                BigEndianByteOrder));
    EXPECT_EQ(0x1032547698badcfe, htog((uint64_t)0x1032547698badcfe,
                                BigEndianByteOrder));
    EXPECT_EQ(0xb14171b771b01c0b, htog((uint64_t)0xb14171b771b01c0b,
                                BigEndianByteOrder));
    EXPECT_EQ(0xffffffff00000000, htog((uint64_t)0xffffffff00000000,
                                BigEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                BigEndianByteOrder));
    EXPECT_EQ(0xefcd2301, htog((uint32_t)0xefcd2301, BigEndianByteOrder));
    EXPECT_EQ(0x1032dcfe, htog((uint32_t)0x1032dcfe, BigEndianByteOrder));
    EXPECT_EQ(0xb1411c0b, htog((uint32_t)0xb1411c0b, BigEndianByteOrder));
    EXPECT_EQ(0xffff0000, htog((uint32_t)0xffff0000, BigEndianByteOrder));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, BigEndianByteOrder));
    EXPECT_EQ(0xef01, htog((uint16_t)0xef01, BigEndianByteOrder));
    EXPECT_EQ(0x10fe, htog((uint16_t)0x10fe, BigEndianByteOrder));
    EXPECT_EQ(0xb10b, htog((uint16_t)0xb10b, BigEndianByteOrder));
    EXPECT_EQ(0xff00, htog((uint16_t)0xff00, BigEndianByteOrder));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, BigEndianByteOrder));
    EXPECT_EQ(0x0123456789abcdef, htog((uint64_t)0xefcdab8967452301,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xfedcba9876543210, htog((uint64_t)0x1032547698badcfe,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x0b1cb071b77141b1, htog((uint64_t)0xb14171b771b01c0b,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x00000000ffffffff, htog((uint64_t)0xffffffff00000000,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x0123cdef, htog((uint32_t)0xefcd2301, LittleEndianByteOrder));
    EXPECT_EQ(0xfedc3210, htog((uint32_t)0x1032dcfe, LittleEndianByteOrder));
    EXPECT_EQ(0x0b1c41b1, htog((uint32_t)0xb1411c0b, LittleEndianByteOrder));
    EXPECT_EQ(0x0000ffff, htog((uint32_t)0xffff0000, LittleEndianByteOrder));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, LittleEndianByteOrder));
    EXPECT_EQ(0x01ef, htog((uint16_t)0xef01, LittleEndianByteOrder));
    EXPECT_EQ(0xfe10, htog((uint16_t)0x10fe, LittleEndianByteOrder));
    EXPECT_EQ(0x0bb1, htog((uint16_t)0xb10b, LittleEndianByteOrder));
    EXPECT_EQ(0x00ff, htog((uint16_t)0xff00, LittleEndianByteOrder));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, LittleEndianByteOrder));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef, htog((uint64_t)0xefcdab8967452301,
                                BigEndianByteOrder));
    EXPECT_EQ(0xfedcba9876543210, htog((uint64_t)0x1032547698badcfe,
                                BigEndianByteOrder));
    EXPECT_EQ(0x0b1cb071b77141b1, htog((uint64_t)0xb14171b771b01c0b,
                                BigEndianByteOrder));
    EXPECT_EQ(0x00000000ffffffff, htog((uint64_t)0xffffffff00000000,
                                BigEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                BigEndianByteOrder));
    EXPECT_EQ(0x0123cdef, htog((uint32_t)0xefcd2301, BigEndianByteOrder));
    EXPECT_EQ(0xfedc3210, htog((uint32_t)0x1032dcfe, BigEndianByteOrder));
    EXPECT_EQ(0x0b1c41b1, htog((uint32_t)0xb1411c0b, BigEndianByteOrder));
    EXPECT_EQ(0x0000ffff, htog((uint32_t)0xffff0000, BigEndianByteOrder));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, BigEndianByteOrder));
    EXPECT_EQ(0x01ef, htog((uint16_t)0xef01, BigEndianByteOrder));
    EXPECT_EQ(0xfe10, htog((uint16_t)0x10fe, BigEndianByteOrder));
    EXPECT_EQ(0x0bb1, htog((uint16_t)0xb10b, BigEndianByteOrder));
    EXPECT_EQ(0x00ff, htog((uint16_t)0xff00, BigEndianByteOrder));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, BigEndianByteOrder));
    EXPECT_EQ(0xefcdab8967452301, htog((uint64_t)0xefcdab8967452301,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x1032547698badcfe, htog((uint64_t)0x1032547698badcfe,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xb14171b771b01c0b, htog((uint64_t)0xb14171b771b01c0b,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xffffffff00000000, htog((uint64_t)0xffffffff00000000,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xefcd2301, htog((uint32_t)0xefcd2301, LittleEndianByteOrder));
    EXPECT_EQ(0x1032dcfe, htog((uint32_t)0x1032dcfe, LittleEndianByteOrder));
    EXPECT_EQ(0xb1411c0b, htog((uint32_t)0xb1411c0b, LittleEndianByteOrder));
    EXPECT_EQ(0xffff0000, htog((uint32_t)0xffff0000, LittleEndianByteOrder));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, LittleEndianByteOrder));
    EXPECT_EQ(0xef01, htog((uint16_t)0xef01, LittleEndianByteOrder));
    EXPECT_EQ(0x10fe, htog((uint16_t)0x10fe, LittleEndianByteOrder));
    EXPECT_EQ(0xb10b, htog((uint16_t)0xb10b, LittleEndianByteOrder));
    EXPECT_EQ(0xff00, htog((uint16_t)0xff00, LittleEndianByteOrder));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, LittleEndianByteOrder));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, gtoh)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0xefcdab8967452301, gtoh((uint64_t)0xefcdab8967452301,
                                BigEndianByteOrder));
    EXPECT_EQ(0x1032547698badcfe, gtoh((uint64_t)0x1032547698badcfe,
                                BigEndianByteOrder));
    EXPECT_EQ(0xb14171b771b01c0b, gtoh((uint64_t)0xb14171b771b01c0b,
                                BigEndianByteOrder));
    EXPECT_EQ(0xffffffff00000000, gtoh((uint64_t)0xffffffff00000000,
                                BigEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                BigEndianByteOrder));
    EXPECT_EQ(0xefcd2301, gtoh((uint32_t)0xefcd2301, BigEndianByteOrder));
    EXPECT_EQ(0x1032dcfe, gtoh((uint32_t)0x1032dcfe, BigEndianByteOrder));
    EXPECT_EQ(0xb1411c0b, gtoh((uint32_t)0xb1411c0b, BigEndianByteOrder));
    EXPECT_EQ(0xffff0000, gtoh((uint32_t)0xffff0000, BigEndianByteOrder));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, BigEndianByteOrder));
    EXPECT_EQ(0xef01, gtoh((uint16_t)0xef01, BigEndianByteOrder));
    EXPECT_EQ(0x10fe, gtoh((uint16_t)0x10fe, BigEndianByteOrder));
    EXPECT_EQ(0xb10b, gtoh((uint16_t)0xb10b, BigEndianByteOrder));
    EXPECT_EQ(0xff00, gtoh((uint16_t)0xff00, BigEndianByteOrder));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, BigEndianByteOrder));
    EXPECT_EQ(0x0123456789abcdef, gtoh((uint64_t)0xefcdab8967452301,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xfedcba9876543210, gtoh((uint64_t)0x1032547698badcfe,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x0b1cb071b77141b1, gtoh((uint64_t)0xb14171b771b01c0b,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x00000000ffffffff, gtoh((uint64_t)0xffffffff00000000,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x0123cdef, gtoh((uint32_t)0xefcd2301, LittleEndianByteOrder));
    EXPECT_EQ(0xfedc3210, gtoh((uint32_t)0x1032dcfe, LittleEndianByteOrder));
    EXPECT_EQ(0x0b1c41b1, gtoh((uint32_t)0xb1411c0b, LittleEndianByteOrder));
    EXPECT_EQ(0x0000ffff, gtoh((uint32_t)0xffff0000, LittleEndianByteOrder));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, LittleEndianByteOrder));
    EXPECT_EQ(0x01ef, gtoh((uint16_t)0xef01, LittleEndianByteOrder));
    EXPECT_EQ(0xfe10, gtoh((uint16_t)0x10fe, LittleEndianByteOrder));
    EXPECT_EQ(0x0bb1, gtoh((uint16_t)0xb10b, LittleEndianByteOrder));
    EXPECT_EQ(0x00ff, gtoh((uint16_t)0xff00, LittleEndianByteOrder));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, LittleEndianByteOrder));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef, gtoh((uint64_t)0xefcdab8967452301,
                                BigEndianByteOrder));
    EXPECT_EQ(0xfedcba9876543210, gtoh((uint64_t)0x1032547698badcfe,
                                BigEndianByteOrder));
    EXPECT_EQ(0x0b1cb071b77141b1, gtoh((uint64_t)0xb14171b771b01c0b,
                                BigEndianByteOrder));
    EXPECT_EQ(0x00000000ffffffff, gtoh((uint64_t)0xffffffff00000000,
                                BigEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                BigEndianByteOrder));
    EXPECT_EQ(0x0123cdef, gtoh((uint32_t)0xefcd2301, BigEndianByteOrder));
    EXPECT_EQ(0xfedc3210, gtoh((uint32_t)0x1032dcfe, BigEndianByteOrder));
    EXPECT_EQ(0x0b1c41b1, gtoh((uint32_t)0xb1411c0b, BigEndianByteOrder));
    EXPECT_EQ(0x0000ffff, gtoh((uint32_t)0xffff0000, BigEndianByteOrder));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, BigEndianByteOrder));
    EXPECT_EQ(0x01ef, gtoh((uint16_t)0xef01, BigEndianByteOrder));
    EXPECT_EQ(0xfe10, gtoh((uint16_t)0x10fe, BigEndianByteOrder));
    EXPECT_EQ(0x0bb1, gtoh((uint16_t)0xb10b, BigEndianByteOrder));
    EXPECT_EQ(0x00ff, gtoh((uint16_t)0xff00, BigEndianByteOrder));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, BigEndianByteOrder));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, BigEndianByteOrder));
    EXPECT_EQ(0xefcdab8967452301, gtoh((uint64_t)0xefcdab8967452301,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x1032547698badcfe, gtoh((uint64_t)0x1032547698badcfe,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xb14171b771b01c0b, gtoh((uint64_t)0xb14171b771b01c0b,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xffffffff00000000, gtoh((uint64_t)0xffffffff00000000,
                                LittleEndianByteOrder));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                LittleEndianByteOrder));
    EXPECT_EQ(0xefcd2301, gtoh((uint32_t)0xefcd2301, LittleEndianByteOrder));
    EXPECT_EQ(0x1032dcfe, gtoh((uint32_t)0x1032dcfe, LittleEndianByteOrder));
    EXPECT_EQ(0xb1411c0b, gtoh((uint32_t)0xb1411c0b, LittleEndianByteOrder));
    EXPECT_EQ(0xffff0000, gtoh((uint32_t)0xffff0000, LittleEndianByteOrder));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, LittleEndianByteOrder));
    EXPECT_EQ(0xef01, gtoh((uint16_t)0xef01, LittleEndianByteOrder));
    EXPECT_EQ(0x10fe, gtoh((uint16_t)0x10fe, LittleEndianByteOrder));
    EXPECT_EQ(0xb10b, gtoh((uint16_t)0xb10b, LittleEndianByteOrder));
    EXPECT_EQ(0xff00, gtoh((uint16_t)0xff00, LittleEndianByteOrder));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, LittleEndianByteOrder));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, LittleEndianByteOrder));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, betole)
{
    EXPECT_EQ(0x0123456789abcdef, betole((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210, betole((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1, betole((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff, betole((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555, betole((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, betole((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, betole((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, betole((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, betole((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, betole((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, betole((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, betole((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, betole((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, betole((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, betole((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, betole((uint16_t)0xff00));
    EXPECT_EQ(0x5555, betole((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, betole((uint16_t)0xa0a0));
}

TEST(ByteswapTest, letobe)
{
    EXPECT_EQ(0x0123456789abcdef, letobe((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210, letobe((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1, letobe((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff, letobe((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555, letobe((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, letobe((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, letobe((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, letobe((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, letobe((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, letobe((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, letobe((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, letobe((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, letobe((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, letobe((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, letobe((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, letobe((uint16_t)0xff00));
    EXPECT_EQ(0x5555, letobe((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, letobe((uint16_t)0xa0a0));
}

TEST(ByteswapTest, beg_gtole)
{
    EXPECT_EQ(0x0123456789abcdef,
        BigEndianGuest::gtole((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        BigEndianGuest::gtole((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        BigEndianGuest::gtole((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        BigEndianGuest::gtole((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::gtole((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::gtole((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, BigEndianGuest::gtole((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, BigEndianGuest::gtole((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, BigEndianGuest::gtole((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, BigEndianGuest::gtole((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::gtole((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::gtole((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, BigEndianGuest::gtole((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, BigEndianGuest::gtole((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, BigEndianGuest::gtole((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, BigEndianGuest::gtole((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::gtole((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::gtole((uint16_t)0xa0a0));
}

TEST(ByteswapTest, beg_letog)
{
    EXPECT_EQ(0x0123456789abcdef,
        BigEndianGuest::letog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        BigEndianGuest::letog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        BigEndianGuest::letog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        BigEndianGuest::letog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::letog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::letog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, BigEndianGuest::letog((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, BigEndianGuest::letog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, BigEndianGuest::letog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, BigEndianGuest::letog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::letog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::letog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, BigEndianGuest::letog((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, BigEndianGuest::letog((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, BigEndianGuest::letog((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, BigEndianGuest::letog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::letog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::letog((uint16_t)0xa0a0));
}

TEST(ByteswapTest, beg_gtobe)
{
    EXPECT_EQ(0xefcdab8967452301,
        BigEndianGuest::gtobe((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        BigEndianGuest::gtobe((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        BigEndianGuest::gtobe((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        BigEndianGuest::gtobe((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::gtobe((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::gtobe((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, BigEndianGuest::gtobe((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, BigEndianGuest::gtobe((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, BigEndianGuest::gtobe((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, BigEndianGuest::gtobe((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::gtobe((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::gtobe((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, BigEndianGuest::gtobe((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, BigEndianGuest::gtobe((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, BigEndianGuest::gtobe((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, BigEndianGuest::gtobe((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::gtobe((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::gtobe((uint16_t)0xa0a0));
}

TEST(ByteswapTest, beg_betog)
{
    EXPECT_EQ(0xefcdab8967452301,
        BigEndianGuest::betog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        BigEndianGuest::betog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        BigEndianGuest::betog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        BigEndianGuest::betog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::betog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::betog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, BigEndianGuest::betog((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, BigEndianGuest::betog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, BigEndianGuest::betog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, BigEndianGuest::betog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::betog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::betog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, BigEndianGuest::betog((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, BigEndianGuest::betog((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, BigEndianGuest::betog((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, BigEndianGuest::betog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::betog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::betog((uint16_t)0xa0a0));
}

TEST(ByteswapTest, beg_htog)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0xefcdab8967452301,
        BigEndianGuest::htog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        BigEndianGuest::htog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        BigEndianGuest::htog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        BigEndianGuest::htog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::htog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::htog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, BigEndianGuest::htog((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, BigEndianGuest::htog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, BigEndianGuest::htog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, BigEndianGuest::htog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::htog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::htog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, BigEndianGuest::htog((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, BigEndianGuest::htog((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, BigEndianGuest::htog((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, BigEndianGuest::htog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::htog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::htog((uint16_t)0xa0a0));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef,
        BigEndianGuest::htog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        BigEndianGuest::htog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        BigEndianGuest::htog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        BigEndianGuest::htog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::htog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::htog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, BigEndianGuest::htog((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, BigEndianGuest::htog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, BigEndianGuest::htog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, BigEndianGuest::htog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::htog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::htog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, BigEndianGuest::htog((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, BigEndianGuest::htog((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, BigEndianGuest::htog((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, BigEndianGuest::htog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::htog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::htog((uint16_t)0xa0a0));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, beg_gtoh)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0xefcdab8967452301,
        BigEndianGuest::gtoh((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        BigEndianGuest::gtoh((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        BigEndianGuest::gtoh((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        BigEndianGuest::gtoh((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::gtoh((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::gtoh((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, BigEndianGuest::gtoh((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, BigEndianGuest::gtoh((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, BigEndianGuest::gtoh((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, BigEndianGuest::gtoh((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::gtoh((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::gtoh((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, BigEndianGuest::gtoh((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, BigEndianGuest::gtoh((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, BigEndianGuest::gtoh((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, BigEndianGuest::gtoh((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::gtoh((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::gtoh((uint16_t)0xa0a0));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef,
        BigEndianGuest::gtoh((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        BigEndianGuest::gtoh((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        BigEndianGuest::gtoh((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        BigEndianGuest::gtoh((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        BigEndianGuest::gtoh((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        BigEndianGuest::gtoh((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, BigEndianGuest::gtoh((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, BigEndianGuest::gtoh((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, BigEndianGuest::gtoh((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, BigEndianGuest::gtoh((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, BigEndianGuest::gtoh((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, BigEndianGuest::gtoh((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, BigEndianGuest::gtoh((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, BigEndianGuest::gtoh((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, BigEndianGuest::gtoh((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, BigEndianGuest::gtoh((uint16_t)0xff00));
    EXPECT_EQ(0x5555, BigEndianGuest::gtoh((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, BigEndianGuest::gtoh((uint16_t)0xa0a0));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, leg_gtole)
{
    EXPECT_EQ(0xefcdab8967452301,
        LittleEndianGuest::gtole((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        LittleEndianGuest::gtole((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        LittleEndianGuest::gtole((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        LittleEndianGuest::gtole((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::gtole((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::gtole((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, LittleEndianGuest::gtole((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, LittleEndianGuest::gtole((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, LittleEndianGuest::gtole((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, LittleEndianGuest::gtole((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::gtole((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::gtole((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, LittleEndianGuest::gtole((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, LittleEndianGuest::gtole((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, LittleEndianGuest::gtole((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, LittleEndianGuest::gtole((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::gtole((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::gtole((uint16_t)0xa0a0));
}

TEST(ByteswapTest, leg_letog)
{
    EXPECT_EQ(0xefcdab8967452301,
        LittleEndianGuest::letog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        LittleEndianGuest::letog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        LittleEndianGuest::letog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        LittleEndianGuest::letog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::letog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::letog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, LittleEndianGuest::letog((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, LittleEndianGuest::letog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, LittleEndianGuest::letog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, LittleEndianGuest::letog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::letog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::letog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, LittleEndianGuest::letog((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, LittleEndianGuest::letog((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, LittleEndianGuest::letog((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, LittleEndianGuest::letog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::letog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::letog((uint16_t)0xa0a0));
}

TEST(ByteswapTest, leg_gtobe)
{
    EXPECT_EQ(0x0123456789abcdef,
        LittleEndianGuest::gtobe((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        LittleEndianGuest::gtobe((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        LittleEndianGuest::gtobe((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        LittleEndianGuest::gtobe((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::gtobe((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::gtobe((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, LittleEndianGuest::gtobe((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, LittleEndianGuest::gtobe((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, LittleEndianGuest::gtobe((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, LittleEndianGuest::gtobe((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::gtobe((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::gtobe((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, LittleEndianGuest::gtobe((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, LittleEndianGuest::gtobe((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, LittleEndianGuest::gtobe((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, LittleEndianGuest::gtobe((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::gtobe((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::gtobe((uint16_t)0xa0a0));
}

TEST(ByteswapTest, leg_betog)
{
    EXPECT_EQ(0x0123456789abcdef,
        LittleEndianGuest::betog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        LittleEndianGuest::betog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        LittleEndianGuest::betog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        LittleEndianGuest::betog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::betog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::betog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, LittleEndianGuest::betog((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, LittleEndianGuest::betog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, LittleEndianGuest::betog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, LittleEndianGuest::betog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::betog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::betog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, LittleEndianGuest::betog((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, LittleEndianGuest::betog((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, LittleEndianGuest::betog((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, LittleEndianGuest::betog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::betog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::betog((uint16_t)0xa0a0));
}

TEST(ByteswapTest, leg_htog)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0x0123456789abcdef,
        LittleEndianGuest::htog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        LittleEndianGuest::htog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        LittleEndianGuest::htog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        LittleEndianGuest::htog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::htog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::htog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, LittleEndianGuest::htog((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, LittleEndianGuest::htog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, LittleEndianGuest::htog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, LittleEndianGuest::htog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::htog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::htog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, LittleEndianGuest::htog((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, LittleEndianGuest::htog((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, LittleEndianGuest::htog((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, LittleEndianGuest::htog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::htog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::htog((uint16_t)0xa0a0));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0xefcdab8967452301,
        LittleEndianGuest::htog((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        LittleEndianGuest::htog((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        LittleEndianGuest::htog((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        LittleEndianGuest::htog((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::htog((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::htog((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, LittleEndianGuest::htog((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, LittleEndianGuest::htog((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, LittleEndianGuest::htog((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, LittleEndianGuest::htog((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::htog((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::htog((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, LittleEndianGuest::htog((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, LittleEndianGuest::htog((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, LittleEndianGuest::htog((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, LittleEndianGuest::htog((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::htog((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::htog((uint16_t)0xa0a0));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, leg_gtoh)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0x0123456789abcdef,
        LittleEndianGuest::gtoh((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0xfedcba9876543210,
        LittleEndianGuest::gtoh((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0x0b1cb071b77141b1,
        LittleEndianGuest::gtoh((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0x00000000ffffffff,
        LittleEndianGuest::gtoh((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::gtoh((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::gtoh((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0x0123cdef, LittleEndianGuest::gtoh((uint32_t)0xefcd2301));
    EXPECT_EQ(0xfedc3210, LittleEndianGuest::gtoh((uint32_t)0x1032dcfe));
    EXPECT_EQ(0x0b1c41b1, LittleEndianGuest::gtoh((uint32_t)0xb1411c0b));
    EXPECT_EQ(0x0000ffff, LittleEndianGuest::gtoh((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::gtoh((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::gtoh((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0x01ef, LittleEndianGuest::gtoh((uint16_t)0xef01));
    EXPECT_EQ(0xfe10, LittleEndianGuest::gtoh((uint16_t)0x10fe));
    EXPECT_EQ(0x0bb1, LittleEndianGuest::gtoh((uint16_t)0xb10b));
    EXPECT_EQ(0x00ff, LittleEndianGuest::gtoh((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::gtoh((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::gtoh((uint16_t)0xa0a0));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0xefcdab8967452301,
        LittleEndianGuest::gtoh((uint64_t)0xefcdab8967452301));
    EXPECT_EQ(0x1032547698badcfe,
        LittleEndianGuest::gtoh((uint64_t)0x1032547698badcfe));
    EXPECT_EQ(0xb14171b771b01c0b,
        LittleEndianGuest::gtoh((uint64_t)0xb14171b771b01c0b));
    EXPECT_EQ(0xffffffff00000000,
        LittleEndianGuest::gtoh((uint64_t)0xffffffff00000000));
    EXPECT_EQ(0x5555555555555555,
        LittleEndianGuest::gtoh((uint64_t)0x5555555555555555));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0,
        LittleEndianGuest::gtoh((uint64_t)0xa0a0a0a0a0a0a0a0));
    EXPECT_EQ(0xefcd2301, LittleEndianGuest::gtoh((uint32_t)0xefcd2301));
    EXPECT_EQ(0x1032dcfe, LittleEndianGuest::gtoh((uint32_t)0x1032dcfe));
    EXPECT_EQ(0xb1411c0b, LittleEndianGuest::gtoh((uint32_t)0xb1411c0b));
    EXPECT_EQ(0xffff0000, LittleEndianGuest::gtoh((uint32_t)0xffff0000));
    EXPECT_EQ(0x55555555, LittleEndianGuest::gtoh((uint32_t)0x55555555));
    EXPECT_EQ(0xa0a0a0a0, LittleEndianGuest::gtoh((uint32_t)0xa0a0a0a0));
    EXPECT_EQ(0xef01, LittleEndianGuest::gtoh((uint16_t)0xef01));
    EXPECT_EQ(0x10fe, LittleEndianGuest::gtoh((uint16_t)0x10fe));
    EXPECT_EQ(0xb10b, LittleEndianGuest::gtoh((uint16_t)0xb10b));
    EXPECT_EQ(0xff00, LittleEndianGuest::gtoh((uint16_t)0xff00));
    EXPECT_EQ(0x5555, LittleEndianGuest::gtoh((uint16_t)0x5555));
    EXPECT_EQ(0xa0a0, LittleEndianGuest::gtoh((uint16_t)0xa0a0));
#else
    #error Invalid Endianess
#endif
}
