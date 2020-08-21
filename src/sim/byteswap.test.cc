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
                                ByteOrder::big));
    EXPECT_EQ(0x1032547698badcfe, htog((uint64_t)0x1032547698badcfe,
                                ByteOrder::big));
    EXPECT_EQ(0xb14171b771b01c0b, htog((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::big));
    EXPECT_EQ(0xffffffff00000000, htog((uint64_t)0xffffffff00000000,
                                ByteOrder::big));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::big));
    EXPECT_EQ(0xefcd2301, htog((uint32_t)0xefcd2301, ByteOrder::big));
    EXPECT_EQ(0x1032dcfe, htog((uint32_t)0x1032dcfe, ByteOrder::big));
    EXPECT_EQ(0xb1411c0b, htog((uint32_t)0xb1411c0b, ByteOrder::big));
    EXPECT_EQ(0xffff0000, htog((uint32_t)0xffff0000, ByteOrder::big));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, ByteOrder::big));
    EXPECT_EQ(0xef01, htog((uint16_t)0xef01, ByteOrder::big));
    EXPECT_EQ(0x10fe, htog((uint16_t)0x10fe, ByteOrder::big));
    EXPECT_EQ(0xb10b, htog((uint16_t)0xb10b, ByteOrder::big));
    EXPECT_EQ(0xff00, htog((uint16_t)0xff00, ByteOrder::big));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, ByteOrder::big));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, ByteOrder::big));
    EXPECT_EQ(0x0123456789abcdef, htog((uint64_t)0xefcdab8967452301,
                                ByteOrder::little));
    EXPECT_EQ(0xfedcba9876543210, htog((uint64_t)0x1032547698badcfe,
                                ByteOrder::little));
    EXPECT_EQ(0x0b1cb071b77141b1, htog((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::little));
    EXPECT_EQ(0x00000000ffffffff, htog((uint64_t)0xffffffff00000000,
                                ByteOrder::little));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::little));
    EXPECT_EQ(0x0123cdef, htog((uint32_t)0xefcd2301, ByteOrder::little));
    EXPECT_EQ(0xfedc3210, htog((uint32_t)0x1032dcfe, ByteOrder::little));
    EXPECT_EQ(0x0b1c41b1, htog((uint32_t)0xb1411c0b, ByteOrder::little));
    EXPECT_EQ(0x0000ffff, htog((uint32_t)0xffff0000, ByteOrder::little));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, ByteOrder::little));
    EXPECT_EQ(0x01ef, htog((uint16_t)0xef01, ByteOrder::little));
    EXPECT_EQ(0xfe10, htog((uint16_t)0x10fe, ByteOrder::little));
    EXPECT_EQ(0x0bb1, htog((uint16_t)0xb10b, ByteOrder::little));
    EXPECT_EQ(0x00ff, htog((uint16_t)0xff00, ByteOrder::little));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, ByteOrder::little));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, ByteOrder::little));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef, htog((uint64_t)0xefcdab8967452301,
                                ByteOrder::big));
    EXPECT_EQ(0xfedcba9876543210, htog((uint64_t)0x1032547698badcfe,
                                ByteOrder::big));
    EXPECT_EQ(0x0b1cb071b77141b1, htog((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::big));
    EXPECT_EQ(0x00000000ffffffff, htog((uint64_t)0xffffffff00000000,
                                ByteOrder::big));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::big));
    EXPECT_EQ(0x0123cdef, htog((uint32_t)0xefcd2301, ByteOrder::big));
    EXPECT_EQ(0xfedc3210, htog((uint32_t)0x1032dcfe, ByteOrder::big));
    EXPECT_EQ(0x0b1c41b1, htog((uint32_t)0xb1411c0b, ByteOrder::big));
    EXPECT_EQ(0x0000ffff, htog((uint32_t)0xffff0000, ByteOrder::big));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, ByteOrder::big));
    EXPECT_EQ(0x01ef, htog((uint16_t)0xef01, ByteOrder::big));
    EXPECT_EQ(0xfe10, htog((uint16_t)0x10fe, ByteOrder::big));
    EXPECT_EQ(0x0bb1, htog((uint16_t)0xb10b, ByteOrder::big));
    EXPECT_EQ(0x00ff, htog((uint16_t)0xff00, ByteOrder::big));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, ByteOrder::big));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, ByteOrder::big));
    EXPECT_EQ(0xefcdab8967452301, htog((uint64_t)0xefcdab8967452301,
                                ByteOrder::little));
    EXPECT_EQ(0x1032547698badcfe, htog((uint64_t)0x1032547698badcfe,
                                ByteOrder::little));
    EXPECT_EQ(0xb14171b771b01c0b, htog((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::little));
    EXPECT_EQ(0xffffffff00000000, htog((uint64_t)0xffffffff00000000,
                                ByteOrder::little));
    EXPECT_EQ(0x5555555555555555, htog((uint64_t)0x5555555555555555,
                                ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, htog((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::little));
    EXPECT_EQ(0xefcd2301, htog((uint32_t)0xefcd2301, ByteOrder::little));
    EXPECT_EQ(0x1032dcfe, htog((uint32_t)0x1032dcfe, ByteOrder::little));
    EXPECT_EQ(0xb1411c0b, htog((uint32_t)0xb1411c0b, ByteOrder::little));
    EXPECT_EQ(0xffff0000, htog((uint32_t)0xffff0000, ByteOrder::little));
    EXPECT_EQ(0x55555555, htog((uint32_t)0x55555555, ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0, htog((uint32_t)0xa0a0a0a0, ByteOrder::little));
    EXPECT_EQ(0xef01, htog((uint16_t)0xef01, ByteOrder::little));
    EXPECT_EQ(0x10fe, htog((uint16_t)0x10fe, ByteOrder::little));
    EXPECT_EQ(0xb10b, htog((uint16_t)0xb10b, ByteOrder::little));
    EXPECT_EQ(0xff00, htog((uint16_t)0xff00, ByteOrder::little));
    EXPECT_EQ(0x5555, htog((uint16_t)0x5555, ByteOrder::little));
    EXPECT_EQ(0xa0a0, htog((uint16_t)0xa0a0, ByteOrder::little));
#else
    #error Invalid Endianess
#endif
}

TEST(ByteswapTest, gtoh)
{
#if (defined(_BIG_ENDIAN)||!defined(_LITTLE_ENDIAN)) && BYTE_ORDER==BIG_ENDIAN
    EXPECT_EQ(0xefcdab8967452301, gtoh((uint64_t)0xefcdab8967452301,
                                ByteOrder::big));
    EXPECT_EQ(0x1032547698badcfe, gtoh((uint64_t)0x1032547698badcfe,
                                ByteOrder::big));
    EXPECT_EQ(0xb14171b771b01c0b, gtoh((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::big));
    EXPECT_EQ(0xffffffff00000000, gtoh((uint64_t)0xffffffff00000000,
                                ByteOrder::big));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::big));
    EXPECT_EQ(0xefcd2301, gtoh((uint32_t)0xefcd2301, ByteOrder::big));
    EXPECT_EQ(0x1032dcfe, gtoh((uint32_t)0x1032dcfe, ByteOrder::big));
    EXPECT_EQ(0xb1411c0b, gtoh((uint32_t)0xb1411c0b, ByteOrder::big));
    EXPECT_EQ(0xffff0000, gtoh((uint32_t)0xffff0000, ByteOrder::big));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, ByteOrder::big));
    EXPECT_EQ(0xef01, gtoh((uint16_t)0xef01, ByteOrder::big));
    EXPECT_EQ(0x10fe, gtoh((uint16_t)0x10fe, ByteOrder::big));
    EXPECT_EQ(0xb10b, gtoh((uint16_t)0xb10b, ByteOrder::big));
    EXPECT_EQ(0xff00, gtoh((uint16_t)0xff00, ByteOrder::big));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, ByteOrder::big));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, ByteOrder::big));
    EXPECT_EQ(0x0123456789abcdef, gtoh((uint64_t)0xefcdab8967452301,
                                ByteOrder::little));
    EXPECT_EQ(0xfedcba9876543210, gtoh((uint64_t)0x1032547698badcfe,
                                ByteOrder::little));
    EXPECT_EQ(0x0b1cb071b77141b1, gtoh((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::little));
    EXPECT_EQ(0x00000000ffffffff, gtoh((uint64_t)0xffffffff00000000,
                                ByteOrder::little));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::little));
    EXPECT_EQ(0x0123cdef, gtoh((uint32_t)0xefcd2301, ByteOrder::little));
    EXPECT_EQ(0xfedc3210, gtoh((uint32_t)0x1032dcfe, ByteOrder::little));
    EXPECT_EQ(0x0b1c41b1, gtoh((uint32_t)0xb1411c0b, ByteOrder::little));
    EXPECT_EQ(0x0000ffff, gtoh((uint32_t)0xffff0000, ByteOrder::little));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, ByteOrder::little));
    EXPECT_EQ(0x01ef, gtoh((uint16_t)0xef01, ByteOrder::little));
    EXPECT_EQ(0xfe10, gtoh((uint16_t)0x10fe, ByteOrder::little));
    EXPECT_EQ(0x0bb1, gtoh((uint16_t)0xb10b, ByteOrder::little));
    EXPECT_EQ(0x00ff, gtoh((uint16_t)0xff00, ByteOrder::little));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, ByteOrder::little));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, ByteOrder::little));
#elif defined(_LITTLE_ENDIAN) || BYTE_ORDER==LITTLE_ENDIAN
    EXPECT_EQ(0x0123456789abcdef, gtoh((uint64_t)0xefcdab8967452301,
                                ByteOrder::big));
    EXPECT_EQ(0xfedcba9876543210, gtoh((uint64_t)0x1032547698badcfe,
                                ByteOrder::big));
    EXPECT_EQ(0x0b1cb071b77141b1, gtoh((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::big));
    EXPECT_EQ(0x00000000ffffffff, gtoh((uint64_t)0xffffffff00000000,
                                ByteOrder::big));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::big));
    EXPECT_EQ(0x0123cdef, gtoh((uint32_t)0xefcd2301, ByteOrder::big));
    EXPECT_EQ(0xfedc3210, gtoh((uint32_t)0x1032dcfe, ByteOrder::big));
    EXPECT_EQ(0x0b1c41b1, gtoh((uint32_t)0xb1411c0b, ByteOrder::big));
    EXPECT_EQ(0x0000ffff, gtoh((uint32_t)0xffff0000, ByteOrder::big));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, ByteOrder::big));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, ByteOrder::big));
    EXPECT_EQ(0x01ef, gtoh((uint16_t)0xef01, ByteOrder::big));
    EXPECT_EQ(0xfe10, gtoh((uint16_t)0x10fe, ByteOrder::big));
    EXPECT_EQ(0x0bb1, gtoh((uint16_t)0xb10b, ByteOrder::big));
    EXPECT_EQ(0x00ff, gtoh((uint16_t)0xff00, ByteOrder::big));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, ByteOrder::big));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, ByteOrder::big));
    EXPECT_EQ(0xefcdab8967452301, gtoh((uint64_t)0xefcdab8967452301,
                                ByteOrder::little));
    EXPECT_EQ(0x1032547698badcfe, gtoh((uint64_t)0x1032547698badcfe,
                                ByteOrder::little));
    EXPECT_EQ(0xb14171b771b01c0b, gtoh((uint64_t)0xb14171b771b01c0b,
                                ByteOrder::little));
    EXPECT_EQ(0xffffffff00000000, gtoh((uint64_t)0xffffffff00000000,
                                ByteOrder::little));
    EXPECT_EQ(0x5555555555555555, gtoh((uint64_t)0x5555555555555555,
                                ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0a0a0a0a0, gtoh((uint64_t)0xa0a0a0a0a0a0a0a0,
                                ByteOrder::little));
    EXPECT_EQ(0xefcd2301, gtoh((uint32_t)0xefcd2301, ByteOrder::little));
    EXPECT_EQ(0x1032dcfe, gtoh((uint32_t)0x1032dcfe, ByteOrder::little));
    EXPECT_EQ(0xb1411c0b, gtoh((uint32_t)0xb1411c0b, ByteOrder::little));
    EXPECT_EQ(0xffff0000, gtoh((uint32_t)0xffff0000, ByteOrder::little));
    EXPECT_EQ(0x55555555, gtoh((uint32_t)0x55555555, ByteOrder::little));
    EXPECT_EQ(0xa0a0a0a0, gtoh((uint32_t)0xa0a0a0a0, ByteOrder::little));
    EXPECT_EQ(0xef01, gtoh((uint16_t)0xef01, ByteOrder::little));
    EXPECT_EQ(0x10fe, gtoh((uint16_t)0x10fe, ByteOrder::little));
    EXPECT_EQ(0xb10b, gtoh((uint16_t)0xb10b, ByteOrder::little));
    EXPECT_EQ(0xff00, gtoh((uint16_t)0xff00, ByteOrder::little));
    EXPECT_EQ(0x5555, gtoh((uint16_t)0x5555, ByteOrder::little));
    EXPECT_EQ(0xa0a0, gtoh((uint16_t)0xa0a0, ByteOrder::little));
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
