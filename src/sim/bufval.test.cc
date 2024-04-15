/*
 * Copyright 2022 Google Inc
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

#include <array>

#include "sim/bufval.hh"

using namespace gem5;

using testing::ElementsAre;

template <class First>
bool
pairFail(const std::pair<First, bool> &p)
{
    return !p.second;
}

bool
pairFailU64(const std::pair<std::uint64_t, bool> &p)
{
    return pairFail(p);
}

bool
pairFailStr(const std::pair<std::string, bool> &p)
{
    return pairFail(p);
}

template <class First>
bool
pairVal(const std::pair<First, bool> &p, const First &expected)
{
    auto &[first, success] = p;
    return success && (first == expected);
}

bool
pairValU64(const std::pair<std::uint64_t, bool> &p, std::uint64_t expected)
{
    return pairVal(p, expected);
}

bool
pairValStr(const std::pair<std::string, bool> &p, const std::string &expected)
{
    return pairVal(p, expected);
}

TEST(GetUintX, BadSize)
{
    uint8_t dummy{};
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 0, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 0, ByteOrder::big));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 3, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 3, ByteOrder::big));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 5, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 5, ByteOrder::big));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 6, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 6, ByteOrder::big));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 7, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 7, ByteOrder::big));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 9, ByteOrder::little));
    EXPECT_PRED1(pairFailU64, getUintX(&dummy, 9, ByteOrder::big));
}

TEST(GetUintX, LittleEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0x9 };
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 1, ByteOrder::little), 0x01);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 2, ByteOrder::little),
                 0x0201);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 4, ByteOrder::little),
                 0x04030201);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 8, ByteOrder::little),
                 0x0807060504030201);
}

TEST(GetUintX, BigEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0x9 };
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 1, ByteOrder::big), 0x01);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 2, ByteOrder::big), 0x0102);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 4, ByteOrder::big),
                 0x01020304);
    EXPECT_PRED2(pairValU64, getUintX(buf.data(), 8, ByteOrder::big),
                 0x0102030405060708);
}

TEST(SetUintX, BadSize)
{
    uint8_t dummy{};
    EXPECT_FALSE(setUintX(0, &dummy, 0, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 0, ByteOrder::big));
    EXPECT_FALSE(setUintX(0, &dummy, 3, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 3, ByteOrder::big));
    EXPECT_FALSE(setUintX(0, &dummy, 5, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 5, ByteOrder::big));
    EXPECT_FALSE(setUintX(0, &dummy, 6, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 6, ByteOrder::big));
    EXPECT_FALSE(setUintX(0, &dummy, 7, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 7, ByteOrder::big));
    EXPECT_FALSE(setUintX(0, &dummy, 9, ByteOrder::little));
    EXPECT_FALSE(setUintX(0, &dummy, 9, ByteOrder::big));
}

TEST(SetUintX, LittleEndian)
{
    const std::array<uint8_t, 9> orig_buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                              0x6, 0x7, 0x8, 0x9 };
    auto buf = orig_buf;

    EXPECT_TRUE(setUintX(0xf1, buf.data(), 1, ByteOrder::little));
    EXPECT_THAT(buf,
                ElementsAre(0xf1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(setUintX(0xe2e1, buf.data(), 2, ByteOrder::little));
    EXPECT_THAT(buf,
                ElementsAre(0xe1, 0xe2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(setUintX(0xd4d3d2d1, buf.data(), 4, ByteOrder::little));
    EXPECT_THAT(buf,
                ElementsAre(0xd1, 0xd2, 0xd3, 0xd4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(
        setUintX(0xc8c7c6c5c4c3c2c1, buf.data(), 8, ByteOrder::little));
    EXPECT_THAT(
        buf, ElementsAre(0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0x9));
}

TEST(SetUintX, BigEndian)
{
    const std::array<uint8_t, 9> orig_buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                              0x6, 0x7, 0x8, 0x9 };
    auto buf = orig_buf;

    EXPECT_TRUE(setUintX(0xf1, buf.data(), 1, ByteOrder::big));
    EXPECT_THAT(buf,
                ElementsAre(0xf1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(setUintX(0xe1e2, buf.data(), 2, ByteOrder::big));
    EXPECT_THAT(buf,
                ElementsAre(0xe1, 0xe2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(setUintX(0xd1d2d3d4, buf.data(), 4, ByteOrder::big));
    EXPECT_THAT(buf,
                ElementsAre(0xd1, 0xd2, 0xd3, 0xd4, 0x5, 0x6, 0x7, 0x8, 0x9));
    EXPECT_TRUE(setUintX(0xc1c2c3c4c5c6c7c8, buf.data(), 8, ByteOrder::big));
    EXPECT_THAT(
        buf, ElementsAre(0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0x9));
}

TEST(PrintUintX, BadSize)
{
    uint8_t dummy{};
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 0, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 0, ByteOrder::big));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 3, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 3, ByteOrder::big));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 5, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 5, ByteOrder::big));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 6, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 6, ByteOrder::big));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 7, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 7, ByteOrder::big));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 9, ByteOrder::little));
    EXPECT_PRED1(pairFailStr, printUintX(&dummy, 9, ByteOrder::big));
}

TEST(PrintUintX, LittleEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0x9 };
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 1, ByteOrder::little),
                 "0x01");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 2, ByteOrder::little),
                 "0x0201");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 4, ByteOrder::little),
                 "0x04030201");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 8, ByteOrder::little),
                 "0x0807060504030201");
}

TEST(PrintUintX, BigEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0x9 };
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 1, ByteOrder::big),
                 "0x01");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 2, ByteOrder::big),
                 "0x0102");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 4, ByteOrder::big),
                 "0x01020304");
    EXPECT_PRED2(pairValStr, printUintX(buf.data(), 8, ByteOrder::big),
                 "0x0102030405060708");
}

TEST(PrintByteBuf, LittleEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0xa };

    EXPECT_EQ(printByteBuf(buf.data(), 0, ByteOrder::little, 3), "[]");
    EXPECT_EQ(printByteBuf(buf.data(), 1, ByteOrder::little, 3), "[01]");
    EXPECT_EQ(printByteBuf(buf.data(), 2, ByteOrder::little, 3), "[0102]");
    EXPECT_EQ(printByteBuf(buf.data(), 3, ByteOrder::little, 3), "[010203]");
    EXPECT_EQ(printByteBuf(buf.data(), 4, ByteOrder::little, 3),
              "[010203 04]");
    EXPECT_EQ(printByteBuf(buf.data(), 5, ByteOrder::little, 3),
              "[010203 0405]");
    EXPECT_EQ(printByteBuf(buf.data(), 6, ByteOrder::little, 3),
              "[010203 040506]");
    EXPECT_EQ(printByteBuf(buf.data(), 7, ByteOrder::little, 3),
              "[010203 040506 07]");
    EXPECT_EQ(printByteBuf(buf.data(), 8, ByteOrder::little, 3),
              "[010203 040506 0708]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 3),
              "[010203 040506 07080a]");
}

TEST(PrintByteBuf, BigEndian)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0xa };

    EXPECT_EQ(printByteBuf(buf.data(), 0, ByteOrder::big, 3), "[]");
    EXPECT_EQ(printByteBuf(buf.data(), 1, ByteOrder::big, 3), "[01]");
    EXPECT_EQ(printByteBuf(buf.data(), 2, ByteOrder::big, 3), "[0201]");
    EXPECT_EQ(printByteBuf(buf.data(), 3, ByteOrder::big, 3), "[030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 4, ByteOrder::big, 3), "[04 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 5, ByteOrder::big, 3), "[0504 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 6, ByteOrder::big, 3),
              "[060504 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 7, ByteOrder::big, 3),
              "[07 060504 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 8, ByteOrder::big, 3),
              "[0807 060504 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 3),
              "[0a0807 060504 030201]");
}

TEST(PrintByteBuf, ChunkSize)
{
    const std::array<uint8_t, 9> buf = { 0x1, 0x2, 0x3, 0x4, 0x5,
                                         0x6, 0x7, 0x8, 0xa };

    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 1),
              "[01 02 03 04 05 06 07 08 0a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 2),
              "[0102 0304 0506 0708 0a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 3),
              "[010203 040506 07080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 4),
              "[01020304 05060708 0a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 5),
              "[0102030405 0607080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 6),
              "[010203040506 07080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 7),
              "[01020304050607 080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 8),
              "[0102030405060708 0a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 9),
              "[01020304050607080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 10),
              "[01020304050607080a]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::little, 100),
              "[01020304050607080a]");

    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 1),
              "[0a 08 07 06 05 04 03 02 01]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 2),
              "[0a 0807 0605 0403 0201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 3),
              "[0a0807 060504 030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 4),
              "[0a 08070605 04030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 5),
              "[0a080706 0504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 6),
              "[0a0807 060504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 7),
              "[0a08 07060504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 8),
              "[0a 0807060504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 9),
              "[0a0807060504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 10),
              "[0a0807060504030201]");
    EXPECT_EQ(printByteBuf(buf.data(), 9, ByteOrder::big, 100),
              "[0a0807060504030201]");
}
