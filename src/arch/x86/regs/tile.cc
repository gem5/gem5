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

#include "arch/x86/regs/tile.hh"

#include <cstring>

#include "arch/x86/emulenv.hh"
#include "arch/x86/faults.hh"
#include "arch/x86/isa.hh"
#include "arch/x86/regs/misc.hh"
#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "cpu/thread_context.hh"
#include "debug/AMX.hh"
#include "sim/full_system.hh"

namespace gem5
{
namespace X86ISA
{

AmxTileState &
getAmxState(ThreadContext *tc)
{
    auto *isa = dynamic_cast<ISA *>(tc->getIsaPtr());
    panic_if(!isa, "AMX requires an X86ISA instance");
    return isa->amxState;
}

Fault
amxRequireEnabled(ThreadContext *tc)
{
    const RegVal xcr0 = tc->readMiscRegNoEffect(misc_reg::Xcr0);
    constexpr RegVal tile_xcr0 = (1ULL << 17) | (1ULL << 18);
    if (FullSystem || !getAmxState(tc).seEnabled ||
        (xcr0 & tile_xcr0) != tile_xcr0) {
        return std::make_shared<InvalidOpcode>();
    }
    return NoFault;
}

Fault
amxCheckEncoding(const ExtMachInst &machInst, bool tile_rm)
{
    if (!amxEncodingValid(machInst, tile_rm)) {
        return std::make_shared<InvalidOpcode>();
    }
    return NoFault;
}

Fault
amxApplyTileConfig(ThreadContext *tc, const uint8_t *buf)
{
    AmxRawTileConfig raw{};
    std::memcpy(raw.data(), buf, AmxTileConfigBytes);
    AmxTileConfig decoded = decodeAmxTileConfig(raw);

    std::string reason;
    if (!validateAmxTileConfig(decoded, reason)) {
        DPRINTF(AMX, "LDTILECFG rejected: %s\n", reason.c_str());
        return std::make_shared<GeneralProtection>(0);
    }

    AmxTileState &state = getAmxState(tc);
    state.tmm = {};
    state.config = decoded;
    state.configured = (decoded.paletteId != 0);

    DPRINTF(AMX, "LDTILECFG palette=%u configured=%d\n", decoded.paletteId,
            state.configured);
    return NoFault;
}

void
amxStoreTileConfig(ThreadContext *tc, uint8_t *buf)
{
    AmxRawTileConfig raw{};
    const AmxTileState &state = getAmxState(tc);
    if (!state.configured) {
        std::memset(buf, 0, AmxTileConfigBytes);
        return;
    }

    raw[0] = state.config.paletteId;
    raw[1] = state.config.startRow;
    for (int tile = 0; tile < AmxNumTiles; ++tile) {
        const size_t offset = 16 + tile * 2;
        raw[offset] = state.config.columnBytes[tile] & 0xff;
        raw[offset + 1] = (state.config.columnBytes[tile] >> 8) & 0xff;
        raw[48 + tile] = state.config.rows[tile];
    }
    std::memcpy(buf, raw.data(), AmxTileConfigBytes);
    DPRINTF(AMX, "STTILECFG\n");
}

void
amxTileRelease(ThreadContext *tc)
{
    AmxTileState &state = getAmxState(tc);
    state.configured = false;
    state.config = {};
    state.tmm = {};
    DPRINTF(AMX, "TILERELEASE\n");
}

Fault
amxTileZero(ThreadContext *tc, const ExtMachInst &machInst)
{
    Fault fault = amxCheckEncoding(machInst, true);
    if (fault != NoFault) {
        return fault;
    }

    const unsigned tmm_idx = machInst.modRM.reg;
    AmxTileState &state = getAmxState(tc);
    if (!state.configured || state.config.rows[tmm_idx] == 0 ||
        state.config.columnBytes[tmm_idx] == 0) {
        return std::make_shared<InvalidOpcode>();
    }
    amxTileZeroState(state, tmm_idx);
    return NoFault;
}

Addr
amxComputeEA(ExecContext *xc, const StaticInst *si,
             const ExtMachInst &machInst)
{
    EmulEnv env(0, 0, machInst.opSize, machInst.addrSize, machInst.stackSize);
    env.doModRM(machInst);
    env.setSeg(machInst);

    unsigned operand = 0;
    const RegVal base =
        env.base == int_reg::T0 ? 0 : xc->getRegOperand(si, operand++);
    const RegVal index =
        env.index == int_reg::T0 ? 0 : xc->getRegOperand(si, operand++);
    const Addr disp = machInst.displacement;
    const bool rip_relative =
        machInst.modRM.mod == 0 && machInst.modRM.rm == 5 &&
        machInst.addrSize == 8 && machInst.mode.submode == SixtyFourBitMode;
    const Addr rip = rip_relative ? xc->pcState().as<PCState>().npc() : 0;
    const Addr seg_base =
        xc->tcBase()->readMiscReg(misc_reg::segBase(env.seg));

    Addr offset = rip + base + index * env.scale + disp;
    if (machInst.addrSize == 4) {
        offset &= mask(32);
    }
    const Addr ea = seg_base + offset;
    DPRINTF(AMX,
            "AMX EA=%#x (seg=%#x rip=%#x base=%#x "
            "index=%#x*%u disp=%#x)\n",
            ea, seg_base, rip, base, index, env.scale, disp);
    return ea;
}

} // namespace X86ISA
} // namespace gem5
