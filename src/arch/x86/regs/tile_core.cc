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

#include <algorithm>

#include "arch/x86/regs/tile.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/AMX.hh"

namespace gem5
{
namespace X86ISA
{
namespace
{

bool
allZero(const uint8_t *bytes, size_t size)
{
    return std::all_of(bytes, bytes + size,
                       [](uint8_t byte) { return byte == 0; });
}

} // anonymous namespace

void
AmxTileState::reset()
{
    const bool keep_enable = seEnabled;
    *this = AmxTileState{};
    seEnabled = keep_enable;
}

bool
amxEncodingValid(const ExtMachInst &machInst, bool tile_rm)
{
    return machInst.mode.submode == SixtyFourBitMode && !machInst.rex.w &&
           !machInst.rex.r &&
           (!tile_rm || (!machInst.rex.b && !machInst.rex.x)) &&
           machInst.vex.v == 0;
}

AmxTileConfig
decodeAmxTileConfig(const AmxRawTileConfig &raw)
{
    AmxTileConfig config = {};
    config.paletteId = raw[0];
    config.startRow = raw[1];
    std::copy_n(raw.begin() + 2, sizeof(config.reserved0), config.reserved0);

    for (int tile = 0; tile < AmxNumTiles; ++tile) {
        const size_t offset = 16 + tile * 2;
        config.columnBytes[tile] =
            static_cast<uint16_t>(raw[offset]) |
            (static_cast<uint16_t>(raw[offset + 1]) << 8);
    }

    std::copy_n(raw.begin() + 32, sizeof(config.reserved1), config.reserved1);
    std::copy_n(raw.begin() + 48, AmxNumTiles, config.rows);
    std::copy_n(raw.begin() + 56, sizeof(config.reserved2), config.reserved2);
    return config;
}

bool
validateAmxTileConfig(const AmxTileConfig &config, std::string &reason)
{
    const auto reject = [&reason](const std::string &message) {
        reason = message;
        return false;
    };

    if (config.paletteId != 0 && config.paletteId != 1) {
        return reject("unsupported palette " +
                      std::to_string(config.paletteId));
    }
    if (config.startRow != 0) {
        return reject("nonzero start_row is not supported");
    }
    if (!allZero(config.reserved0, sizeof(config.reserved0))) {
        return reject("reserved bytes 2-15 must be zero");
    }
    if (!allZero(config.reserved1, sizeof(config.reserved1))) {
        return reject("reserved bytes 32-47 must be zero");
    }
    if (!allZero(config.reserved2, sizeof(config.reserved2))) {
        return reject("reserved bytes 56-63 must be zero");
    }

    for (int tile = 0; tile < AmxNumTiles; ++tile) {
        const uint16_t rows = config.rows[tile];
        const uint16_t columns = config.columnBytes[tile];

        if (rows > AmxMaxRows) {
            return reject("tile " + std::to_string(tile) +
                          " has more than 16 rows");
        }
        if (columns > AmxMaxColumnBytes) {
            return reject("tile " + std::to_string(tile) +
                          " has more than 64 column bytes");
        }
        if ((rows == 0) != (columns == 0)) {
            return reject("tile " + std::to_string(tile) +
                          " has mismatched row and column dimensions");
        }
        if (config.paletteId == 0 && (rows != 0 || columns != 0)) {
            return reject("palette zero requires an empty tile layout");
        }
    }
    return true;
}

void
amxTileZeroState(AmxTileState &state, unsigned tmm_idx)
{
    panic_if(tmm_idx >= AmxNumTiles, "TILEZERO: bad tmm %u", tmm_idx);
    state.tmm[tmm_idx] = {};
    state.config.startRow = 0;
    DPRINTF(AMX, "TILEZERO tmm%u\n", tmm_idx);
}

} // namespace X86ISA
} // namespace gem5
