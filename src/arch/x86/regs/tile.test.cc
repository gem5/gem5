/*
 * Copyright (c) 2026 Jiaqi Wang
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

#include <cstring>
#include <fstream>
#include <string>

#include "arch/x86/regs/tile.hh"
#include "base/gtest/serialization_fixture.hh"
#include "sim/serialize.hh"

using namespace gem5;
using namespace gem5::X86ISA;

namespace
{

AmxRawTileConfig
makePalette1Raw()
{
    AmxRawTileConfig raw{};
    raw[0] = 1;
    raw[16] = 64;
    raw[48] = 8;
    return raw;
}

void
fillTilePattern(AmxTileRegister &tile, uint8_t value)
{
    std::memset(tile.data, value, sizeof(tile.data));
}

bool
tileAllZero(const AmxTileRegister &tile)
{
    for (const auto &row : tile.data) {
        for (const auto byte : row) {
            if (byte != 0) {
                return false;
            }
        }
    }
    return true;
}

bool
tileAllEqual(const AmxTileRegister &tile, uint8_t value)
{
    for (const auto &row : tile.data) {
        for (const auto byte : row) {
            if (byte != value) {
                return false;
            }
        }
    }
    return true;
}

} // anonymous namespace

TEST(AmxTile, TileZeroClearsTargetOnly)
{
    AmxTileState state{};
    state.configured = true;
    state.config.paletteId = 1;
    state.config.startRow = 3;

    for (unsigned tile = 0; tile < AmxNumTiles; ++tile) {
        fillTilePattern(state.tmm[tile], static_cast<uint8_t>(0x10 + tile));
    }

    amxTileZeroState(state, 2);

    EXPECT_TRUE(tileAllZero(state.tmm[2]));
    EXPECT_EQ(state.config.startRow, 0);
    for (unsigned tile = 0; tile < AmxNumTiles; ++tile) {
        if (tile == 2) {
            continue;
        }
        EXPECT_TRUE(
            tileAllEqual(state.tmm[tile], static_cast<uint8_t>(0x10 + tile)))
            << "tile " << tile << " changed unexpectedly";
    }
}

TEST(AmxTile, ValidatePalette1Accepts)
{
    const AmxTileConfig config = decodeAmxTileConfig(makePalette1Raw());
    std::string reason;
    EXPECT_TRUE(validateAmxTileConfig(config, reason)) << reason;
}

TEST(AmxTile, RejectsIllegalPalette)
{
    auto raw = makePalette1Raw();
    raw[0] = 2;
    const AmxTileConfig config = decodeAmxTileConfig(raw);
    std::string reason;
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
    EXPECT_NE(reason.find("palette"), std::string::npos);
}

TEST(AmxTile, RejectsReservedNonZero)
{
    auto raw = makePalette1Raw();
    raw[2] = 1;
    auto config = decodeAmxTileConfig(raw);
    std::string reason;
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
    EXPECT_NE(reason.find("reserved"), std::string::npos);

    raw = makePalette1Raw();
    raw[32] = 1;
    config = decodeAmxTileConfig(raw);
    EXPECT_FALSE(validateAmxTileConfig(config, reason));

    raw = makePalette1Raw();
    raw[56] = 1;
    config = decodeAmxTileConfig(raw);
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
}

TEST(AmxTile, RejectsRowsOrColumnsOutOfRange)
{
    auto raw = makePalette1Raw();
    raw[48] = 17;
    auto config = decodeAmxTileConfig(raw);
    std::string reason;
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
    EXPECT_NE(reason.find("16 rows"), std::string::npos);

    raw = makePalette1Raw();
    raw[16] = 65;
    config = decodeAmxTileConfig(raw);
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
    EXPECT_NE(reason.find("64 column"), std::string::npos);
}

TEST(AmxTile, RejectsMismatchedRowsAndColumns)
{
    auto raw = makePalette1Raw();
    raw[16] = 0;
    raw[17] = 0;
    const AmxTileConfig config = decodeAmxTileConfig(raw);
    std::string reason;
    EXPECT_FALSE(validateAmxTileConfig(config, reason));
    EXPECT_NE(reason.find("mismatched"), std::string::npos);
}

TEST(AmxTile, RejectsReservedEncodingFields)
{
    ExtMachInst machInst{};
    machInst.mode.submode = SixtyFourBitMode;

    EXPECT_TRUE(amxEncodingValid(machInst, false));

    machInst.rex.w = 1;
    EXPECT_FALSE(amxEncodingValid(machInst, false));
    machInst.rex.w = 0;

    machInst.rex.r = 1;
    EXPECT_FALSE(amxEncodingValid(machInst, false));
    machInst.rex.r = 0;

    machInst.vex.v = 1;
    EXPECT_FALSE(amxEncodingValid(machInst, false));
    machInst.vex.v = 0;

    machInst.rex.b = 1;
    EXPECT_TRUE(amxEncodingValid(machInst, false));
    EXPECT_FALSE(amxEncodingValid(machInst, true));
    machInst.rex.b = 0;

    machInst.rex.x = 1;
    EXPECT_FALSE(amxEncodingValid(machInst, true));
    machInst.rex.x = 0;

    machInst.mode.submode = CompatabilityMode;
    EXPECT_FALSE(amxEncodingValid(machInst, false));
}

class AmxCheckpointFixture : public SerializationFixture
{
};

TEST_F(AmxCheckpointFixture, AmxStateRoundTrip)
{
    AmxTileState before{};
    before.seEnabled = true;
    before.configured = true;
    before.config = decodeAmxTileConfig(makePalette1Raw());
    for (unsigned tile = 0; tile < AmxNumTiles; ++tile) {
        fillTilePattern(before.tmm[tile], static_cast<uint8_t>(0x20 + tile));
    }

    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
        before.serialize(cpt_stream);
    }
    cpt_stream.close();

    AmxTileState after{};
    after.seEnabled = true;
    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        after.unserialize(cpt_in);
    }

    EXPECT_TRUE(after.seEnabled);
    EXPECT_EQ(after.configured, before.configured);
    EXPECT_EQ(after.config.paletteId, before.config.paletteId);
    EXPECT_EQ(after.config.startRow, before.config.startRow);
    EXPECT_EQ(after.config.columnBytes[0], before.config.columnBytes[0]);
    EXPECT_EQ(after.config.rows[0], before.config.rows[0]);
    EXPECT_EQ(
        std::memcmp(after.tmm.data(), before.tmm.data(), sizeof(before.tmm)),
        0);
}

TEST_F(AmxCheckpointFixture, DisabledStateIsOmitted)
{
    AmxTileState state{};

    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
        state.serialize(cpt_stream);
    }
    cpt_stream.close();

    CheckpointIn cpt_in(getDirName());
    bool configured = true;
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        EXPECT_FALSE(optParamIn(cpt_in, "amx_configured", configured));
        EXPECT_FALSE(cpt_in.entryExists("AmxState", "amxConfig"));
        EXPECT_FALSE(cpt_in.entryExists("AmxState", "amxTmm"));
    }
}

TEST_F(AmxCheckpointFixture, EnabledUnconfiguredStateRoundTrip)
{
    AmxTileState before{};
    before.seEnabled = true;
    before.config.paletteId = 0x2b;
    fillTilePattern(before.tmm[0], 0x91);

    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
        before.serialize(cpt_stream);
    }
    cpt_stream.close();

    AmxTileState after{};
    after.seEnabled = true;
    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        after.unserialize(cpt_in);
    }

    EXPECT_TRUE(after.seEnabled);
    EXPECT_FALSE(after.configured);
    EXPECT_EQ(after.config.paletteId, 0x2b);
    EXPECT_TRUE(tileAllEqual(after.tmm[0], 0x91));
}

TEST_F(AmxCheckpointFixture, MissingLegacyStateResetsAndKeepsEnable)
{
    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
    }
    cpt_stream.close();

    AmxTileState state{};
    state.seEnabled = true;
    state.configured = true;
    state.config.paletteId = 1;
    fillTilePattern(state.tmm[0], 0xa5);

    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        state.unserialize(cpt_in);
    }

    EXPECT_TRUE(state.seEnabled);
    EXPECT_FALSE(state.configured);
    EXPECT_EQ(state.config.paletteId, 0);
    EXPECT_TRUE(tileAllZero(state.tmm[0]));
}

TEST_F(AmxCheckpointFixture, CompleteLegacyFalseConfiguredStateIsRead)
{
    AmxTileState before{};
    before.config.paletteId = 0x5a;
    fillTilePattern(before.tmm[0], 0xc3);

    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
        bool amx_configured = false;
        paramOut(cpt_stream, "amx_configured", amx_configured);
        arrayParamOut(cpt_stream, "amxConfig",
                      reinterpret_cast<const uint8_t *>(&before.config),
                      AmxTileConfigBytes);
        arrayParamOut(cpt_stream, "amxTmm",
                      reinterpret_cast<const uint8_t *>(before.tmm.data()),
                      sizeof(before.tmm));
    }
    cpt_stream.close();

    AmxTileState after{};
    after.seEnabled = true;
    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        after.unserialize(cpt_in);
    }

    EXPECT_TRUE(after.seEnabled);
    EXPECT_FALSE(after.configured);
    EXPECT_EQ(after.config.paletteId, 0x5a);
    EXPECT_TRUE(tileAllEqual(after.tmm[0], 0xc3));
}

TEST_F(AmxCheckpointFixture, ConfiguredStateSurvivesOnDisabledInstance)
{
    AmxTileState before{};
    before.configured = true;
    before.config = decodeAmxTileConfig(makePalette1Raw());
    fillTilePattern(before.tmm[0], 0x6d);

    std::ofstream cpt_stream;
    Serializable::generateCheckpointOut(getDirName(), cpt_stream);
    {
        Serializable::ScopedCheckpointSection sec(cpt_stream, "AmxState");
        before.serialize(cpt_stream);
    }
    cpt_stream.close();

    AmxTileState after{};
    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        after.unserialize(cpt_in);
    }

    EXPECT_FALSE(after.seEnabled);
    EXPECT_TRUE(after.configured);
    EXPECT_EQ(after.config.paletteId, 1);
    EXPECT_TRUE(tileAllEqual(after.tmm[0], 0x6d));
}

TEST(AmxTile, CopyArchStateKeepsDestinationEnable)
{
    AmxTileState source{};
    source.seEnabled = true;
    source.configured = true;
    source.config = decodeAmxTileConfig(makePalette1Raw());
    fillTilePattern(source.tmm[0], 0x47);

    AmxTileState destination{};
    destination.copyArchStateFrom(source);

    EXPECT_FALSE(destination.seEnabled);
    EXPECT_TRUE(destination.configured);
    EXPECT_EQ(destination.config.paletteId, 1);
    EXPECT_TRUE(tileAllEqual(destination.tmm[0], 0x47));
}

TEST(AmxTile, SeXcr0EnablesOnlyRequiredLegacyAndTileState)
{
    constexpr RegVal unrelated = (1ULL << 9) | (1ULL << 42);

    const RegVal without_avx = amxSeXcr0(unrelated, true);
    EXPECT_EQ(without_avx & AmxLegacyXcr0Mask, AmxLegacyXcr0Mask);
    EXPECT_EQ(without_avx & AmxTileXcr0Mask, AmxTileXcr0Mask);
    EXPECT_EQ(without_avx & (1ULL << 2), 0);
    EXPECT_EQ(without_avx & unrelated, unrelated);

    const RegVal with_avx = amxSeXcr0(unrelated | (1ULL << 2), true);
    EXPECT_NE(with_avx & (1ULL << 2), 0);
    EXPECT_EQ(with_avx & unrelated, unrelated);

    EXPECT_EQ(amxSeXcr0(unrelated, false), unrelated);
}

TEST(AmxTile, PaletteZeroIgnoresPayloadAndResetsState)
{
    AmxTileState state{};
    state.seEnabled = true;
    std::string reason;
    ASSERT_TRUE(state.loadConfig(makePalette1Raw(), reason));
    for (auto &tile : state.tmm) {
        fillTilePattern(tile, 0xa5);
    }
    // Invalid dimensions, reserved bytes and start_row are all ignored.
    AmxRawTileConfig raw{};
    raw.fill(0xff);
    raw[0] = 0;
    ASSERT_TRUE(state.loadConfig(raw, reason)) << reason;
    EXPECT_FALSE(state.configured);
    EXPECT_TRUE(state.seEnabled);
    const AmxTileConfig zero{};
    EXPECT_EQ(std::memcmp(&state.config, &zero, sizeof(zero)), 0);
    for (const auto &tile : state.tmm) {
        EXPECT_TRUE(tileAllZero(tile));
    }
}

TEST(AmxTile, PaletteOnePreservesStartRow)
{
    AmxTileState state{};
    auto raw = makePalette1Raw();
    raw[1] = 3;
    std::string reason;
    ASSERT_TRUE(state.loadConfig(raw, reason)) << reason;
    EXPECT_TRUE(state.configured);
    EXPECT_EQ(state.config.startRow, 3);
    EXPECT_EQ(state.config.rows[0], 8);
    EXPECT_EQ(state.config.columnBytes[0], 64);
}

TEST(AmxTile, InvalidConfigPreservesState)
{
    AmxTileState state{};
    auto raw = makePalette1Raw();
    raw[1] = 3;
    std::string reason;
    ASSERT_TRUE(state.loadConfig(raw, reason));
    fillTilePattern(state.tmm[0], 0xa5);
    const auto before = state.config;
    raw[2] = 1;
    EXPECT_FALSE(state.loadConfig(raw, reason));
    EXPECT_EQ(std::memcmp(&state.config, &before, sizeof(before)), 0);
    EXPECT_TRUE(state.configured);
    EXPECT_TRUE(tileAllEqual(state.tmm[0], 0xa5));
}

TEST(AmxTile, RipRelativeAddressSizeOverride)
{
    ExtMachInst inst{};
    inst.mode.submode = SixtyFourBitMode;
    inst.modRM.mod = 0;
    inst.modRM.rm = 5;
    inst.displacement = 0x100;
    inst.addrSize = 8;
    EXPECT_EQ(amxAddressOffset(inst, 0x401009, 0, 0, 0), 0x401109);
    inst.addrSize = 4;
    EXPECT_EQ(amxAddressOffset(inst, 0x401009, 0, 0, 0), 0x401109);
    // Truncate and zero-extend the result, including overflow past 4 GiB.
    EXPECT_EQ(amxAddressOffset(inst, 0xfffffff0, 0, 0, 0), 0xf0);
    EXPECT_EQ(amxAddressOffset(inst, 0x100401009ULL, 0, 0, 0), 0x401109);
    inst.displacement = static_cast<Addr>(-0x100);
    EXPECT_EQ(amxAddressOffset(inst, 0x80, 0, 0, 0), 0xffffff80ULL);
}

TEST(AmxTile, NonRipRelativeAddressForms)
{
    ExtMachInst inst{};
    inst.mode.submode = SixtyFourBitMode;
    inst.addrSize = 4;
    inst.modRM.mod = 0;
    inst.modRM.rm = 4; // SIB absolute addressing does not add RIP.
    inst.displacement = 0x100;
    EXPECT_EQ(amxAddressOffset(inst, 0x401009, 0, 0, 1), 0x100);
    inst.modRM.mod = 1;
    inst.modRM.rm = 5;
    EXPECT_EQ(amxAddressOffset(inst, 0x401009, 0x1000, 0x20, 4), 0x1180);
    inst.mode.submode = CompatabilityMode;
    inst.modRM.mod = 0;
    EXPECT_EQ(amxAddressOffset(inst, 0x401009, 0, 0, 0), 0x100);
}
