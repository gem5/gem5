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
    const RegVal cr4 = tc->readMiscRegNoEffect(misc_reg::Cr4);
    if (FullSystem || !getAmxState(tc).seEnabled ||
        !amxControlsEnabled(cr4, xcr0)) {
        return std::make_shared<InvalidOpcode>();
    }
    return NoFault;
}

Fault
amxCheckEncoding(const ExtMachInst &machInst, bool tile_dest)
{
    if (!amxEncodingValid(machInst, tile_dest)) {
        return std::make_shared<InvalidOpcode>();
    }
    return NoFault;
}

Fault
amxApplyTileConfig(ThreadContext *tc, const uint8_t *buf)
{
    AmxRawTileConfig raw{};
    std::memcpy(raw.data(), buf, AmxTileConfigBytes);
    AmxTileState &state = getAmxState(tc);
    std::string reason;
    if (!state.loadConfig(raw, reason)) {
        DPRINTF(AMX, "LDTILECFG rejected: %s\n", reason.c_str());
        return std::make_shared<GeneralProtection>(0);
    }

    DPRINTF(AMX, "LDTILECFG palette=%u configured=%d\n",
            state.config.paletteId, state.configured);
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

Fault
amxComputeEA(ExecContext *xc, const StaticInst *si,
             const ExtMachInst &machInst, Addr &ea)
{
    EmulEnv env(0, 0, machInst.opSize, machInst.addrSize, machInst.stackSize);
    env.doModRM(machInst);

    unsigned operand = 0;
    const RegVal base =
        env.base == int_reg::T0 ? 0 : xc->getRegOperand(si, operand++);
    const RegVal index =
        env.index == int_reg::T0 ? 0 : xc->getRegOperand(si, operand++);
    const Addr offset = amxAddressOffset(
        machInst, xc->pcState().as<PCState>().npc(), base, index, env.scale);
    const Addr seg_base =
        xc->tcBase()->readMiscReg(misc_reg::segBase(env.seg));
    ea = seg_base + offset;
    if (!amxConfigAddressValid(ea)) {
        if (env.seg == segment_idx::Ss) {
            return std::make_shared<StackFault>(0);
        }
        return std::make_shared<GeneralProtection>(0);
    }
    DPRINTF(AMX, "AMX EA=%#x (seg=%#x offset=%#x)\n", ea, seg_base, offset);
    return NoFault;
}

} // namespace X86ISA
} // namespace gem5
