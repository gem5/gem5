/*
 * Copyright (c) 2026 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
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

#include <array>
#include <memory>

#include "salam/storage_access.hh"
#include "sim/cur_tick.hh"

TEST(SalamStorageAccess, AlignedAndUnalignedWords)
{
    auto aligned = SALAM::storageAccess(0, 8, 4);
    EXPECT_EQ(aligned.words, 2);
    EXPECT_EQ(aligned.bits, 64);
    EXPECT_EQ(aligned.partialWords, 0);
    auto unaligned = SALAM::storageAccess(3, 8, 4);
    EXPECT_EQ(unaligned.words, 3);
    EXPECT_EQ(unaligned.bits, 64);
    EXPECT_EQ(unaligned.partialWords, 2);
}

TEST(SalamStorageAccess, PartialAccessCostsAWord)
{
    auto access = SALAM::storageAccess(1, 1, 4);
    EXPECT_EQ(access.words, 1);
    EXPECT_EQ(access.bits, 8);
    EXPECT_EQ(access.partialWords, 1);
}

TEST(SalamStorageAccess, MaskedWritesCountOnlyEnabledWords)
{
    auto access = SALAM::storageAccess(
        0, 8, 4, {true, false, false, true, false, false, false, false});
    EXPECT_EQ(access.words, 1);
    EXPECT_EQ(access.bits, 16);
    EXPECT_EQ(access.partialWords, 1);
    access = SALAM::storageAccess(0, 4, 4, {false, false, false, false});
    EXPECT_EQ(access.words, 0);
    EXPECT_EQ(access.bits, 0);
}

TEST(SalamStorageAccess, SplitRequestsKeepPhysicalAccessCosts)
{
    auto a = SALAM::storageAccess(0, 2, 4);
    auto b = SALAM::storageAccess(2, 2, 4);
    EXPECT_EQ(a.words + b.words, 2);
    EXPECT_EQ(a.partialWords + b.partialWords, 2);
    EXPECT_EQ(a.bits + b.bits, 32);
}

TEST(SalamStorageAccess, RejectsInvalidGeometry)
{
    EXPECT_THROW(SALAM::storageAccess(0, 4, 0), std::invalid_argument);
    EXPECT_THROW(SALAM::storageAccess(0, 4, 4, {true}), std::invalid_argument);
}

namespace
{
class SalamStoragePacket : public ::testing::Test
{
  protected:
    gem5::Tick tick = 0;
    gem5::Tick *previous = nullptr;
    void
    SetUp() override
    {
        previous = gem5::Gem5Internal::_curTickPtr;
        gem5::Gem5Internal::_curTickPtr = &tick;
    }
    void
    TearDown() override
    {
        gem5::Gem5Internal::_curTickPtr = previous;
    }
};
} // anonymous namespace

TEST_F(SalamStoragePacket, RealMaskedPacketKeepsDisabledBytes)
{
    constexpr gem5::Addr base = 0x4000;
    auto request = std::make_shared<gem5::Request>(base + 3, 8, 0, 0);
    request->setByteEnable(
        {true, true, false, false, false, true, false, false});
    gem5::Packet packet(request, gem5::MemCmd::WriteReq);
    std::array<uint8_t, 8> payload{0xa0, 0xa1, 0xa2, 0xa3,
                                   0xa4, 0xa5, 0xa6, 0xa7};
    packet.dataStatic(payload.data());
    std::array<uint8_t, 16> memory;
    memory.fill(0x55);
    packet.writeData(memory.data() + 3);
    const std::array<uint8_t, 16> expected{0x55, 0x55, 0x55, 0xa0, 0xa1, 0x55,
                                           0x55, 0x55, 0xa5, 0x55, 0x55, 0x55,
                                           0x55, 0x55, 0x55, 0x55};
    EXPECT_EQ(memory, expected);
    ASSERT_TRUE(SALAM::hasStorageTransfer(packet));
    const auto access = SALAM::storageAccess(packet, base, 4);
    EXPECT_EQ(access.words, 3);
    EXPECT_EQ(access.bits, 24);
    EXPECT_EQ(access.partialWords, 3);
    EXPECT_EQ(packet.getSize(), 8);
    EXPECT_TRUE(packet.isWrite());
    EXPECT_TRUE(packet.needsResponse());
}

TEST_F(SalamStoragePacket, AllMaskedPacketHasNoPhysicalTransfer)
{
    constexpr gem5::Addr base = 0x4000;
    auto request = std::make_shared<gem5::Request>(base + 3, 8, 0, 0);
    request->setByteEnable(std::vector<bool>(8, false));
    gem5::Packet packet(request, gem5::MemCmd::WriteReq);
    std::array<uint8_t, 8> payload{};
    packet.dataStatic(payload.data());
    std::array<uint8_t, 8> memory;
    memory.fill(0x55);
    const auto unchanged = memory;
    packet.writeData(memory.data());
    EXPECT_EQ(memory, unchanged);
    // This guard is used before word-width lookup in both real devices,
    // so an unknown word width still observes no access event here.
    EXPECT_FALSE(SALAM::hasStorageTransfer(packet));
    const auto access = SALAM::storageAccess(packet, base, 4);
    EXPECT_EQ(access.words, 0);
    EXPECT_EQ(access.bits, 0);
    EXPECT_EQ(access.partialWords, 0);
    // The original device byte accounting and response semantics use
    // the untouched packet size/command, including for an all-masked write.
    EXPECT_EQ(packet.getSize(), 8);
    EXPECT_TRUE(packet.isWrite());
    EXPECT_TRUE(packet.needsResponse());
}

TEST_F(SalamStoragePacket, DefaultMaskAndCopiedRequestStayExplicit)
{
    constexpr gem5::Addr base = 0x4000;
    auto request = std::make_shared<gem5::Request>(base + 3, 8, 0, 0);
    gem5::Packet ordinary(request, gem5::MemCmd::WriteReq);
    ASSERT_TRUE(SALAM::hasStorageTransfer(ordinary));
    auto access = SALAM::storageAccess(ordinary, base, 4);
    EXPECT_EQ(access.words, 3);
    EXPECT_EQ(access.bits, 64);
    EXPECT_EQ(access.partialWords, 2);
    request->setByteEnable(
        {false, true, false, false, false, false, false, false});
    auto copied = std::make_shared<gem5::Request>(*request);
    gem5::Packet copy(copied, gem5::MemCmd::WriteReq);
    EXPECT_EQ(copied->getByteEnable(), request->getByteEnable());
    access = SALAM::storageAccess(copy, base, 4);
    EXPECT_EQ(access.words, 1);
    EXPECT_EQ(access.bits, 8);
    EXPECT_EQ(access.partialWords, 1);
}

TEST_F(SalamStoragePacket, ReadServiceIgnoresWriteByteEnables)
{
    constexpr gem5::Addr base = 0x4000;
    auto request = std::make_shared<gem5::Request>(base, 8, 0, 0);
    request->setByteEnable(std::vector<bool>(8, false));
    gem5::Packet packet(request, gem5::MemCmd::ReadReq);
    ASSERT_TRUE(SALAM::hasStorageTransfer(packet));
    const auto access = SALAM::storageAccess(packet, base, 4);
    EXPECT_EQ(access.words, 2);
    EXPECT_EQ(access.bits, 64);
    EXPECT_EQ(access.partialWords, 0);
}
