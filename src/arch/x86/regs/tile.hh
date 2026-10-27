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

#ifndef __ARCH_X86_REGS_TILE_HH__
#define __ARCH_X86_REGS_TILE_HH__

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include "arch/x86/types.hh"
#include "base/types.hh"
#include "sim/faults.hh"

namespace gem5
{

class ExecContext;
class ThreadContext;

namespace X86ISA
{

inline constexpr int AmxMaxRows = 16;
inline constexpr int AmxMaxColumnBytes = 64;
inline constexpr int AmxNumTiles = 8;
inline constexpr size_t AmxTileConfigBytes = 64;

using AmxRawTileConfig = std::array<uint8_t, AmxTileConfigBytes>;

struct AmxTileConfig
{
    uint8_t paletteId = 0;
    uint8_t startRow = 0;
    uint8_t reserved0[14] = {};
    uint16_t columnBytes[AmxNumTiles] = {};
    uint8_t reserved1[16] = {};
    uint8_t rows[AmxNumTiles] = {};
    uint8_t reserved2[8] = {};
};

static_assert(sizeof(AmxTileConfig) == AmxTileConfigBytes);

struct AmxTileRegister
{
    uint8_t data[AmxMaxRows][AmxMaxColumnBytes] = {};
};

struct AmxTileState
{
    bool seEnabled = false;
    bool configured = false;
    AmxTileConfig config = {};
    std::array<AmxTileRegister, AmxNumTiles> tmm = {};

    void reset();
};

AmxTileConfig decodeAmxTileConfig(const AmxRawTileConfig &raw);
bool validateAmxTileConfig(const AmxTileConfig &config, std::string &reason);

AmxTileState &getAmxState(ThreadContext *tc);
Fault amxRequireEnabled(ThreadContext *tc);
bool amxEncodingValid(const ExtMachInst &machInst, bool tile_rm);
Fault amxCheckEncoding(const ExtMachInst &machInst, bool tile_rm);
Fault amxApplyTileConfig(ThreadContext *tc, const uint8_t *buf);
void amxStoreTileConfig(ThreadContext *tc, uint8_t *buf);
void amxTileRelease(ThreadContext *tc);
void amxTileZeroState(AmxTileState &state, unsigned tmm_idx);
Fault amxTileZero(ThreadContext *tc, const ExtMachInst &machInst);
Addr amxComputeEA(ExecContext *xc, const StaticInst *si,
                  const ExtMachInst &machInst);

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_REGS_TILE_HH__
