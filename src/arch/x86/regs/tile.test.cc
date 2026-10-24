/*
 * Copyright (c) 2026 UserNameForGithub03
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

void
serializeAmxState(CheckpointOut &cp, const AmxTileState &state)
{
    bool amx_configured = state.configured;
    SERIALIZE_SCALAR(amx_configured);
    arrayParamOut(cp, "amxConfig",
                  reinterpret_cast<const uint8_t *>(&state.config),
                  AmxTileConfigBytes);
    arrayParamOut(cp, "amxTmm",
                  reinterpret_cast<const uint8_t *>(state.tmm.data()),
                  sizeof(state.tmm));
}

void
unserializeAmxState(CheckpointIn &cp, AmxTileState &state)
{
    bool amx_configured = false;
    UNSERIALIZE_SCALAR(amx_configured);
    state.configured = amx_configured;
    arrayParamIn(cp, "amxConfig", reinterpret_cast<uint8_t *>(&state.config),
                 AmxTileConfigBytes);
    arrayParamIn(cp, "amxTmm", reinterpret_cast<uint8_t *>(state.tmm.data()),
                 sizeof(state.tmm));
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
        serializeAmxState(cpt_stream, before);
    }
    cpt_stream.close();

    AmxTileState after{};
    after.seEnabled = true;
    CheckpointIn cpt_in(getDirName());
    {
        Serializable::ScopedCheckpointSection sec(cpt_in, "AmxState");
        unserializeAmxState(cpt_in, after);
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
