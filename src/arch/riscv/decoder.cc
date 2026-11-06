/*
 * Copyright (c) 2012 Google
 * Copyright (c) The University of Virginia
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

#include "arch/riscv/decoder.hh"
#include "arch/riscv/insts/zcmt.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/types.hh"
#include "base/bitfield.hh"
#include "cpu/thread_context.hh"
#include "debug/Decode.hh"

namespace gem5
{

namespace RiscvISA
{

Decoder::Decoder(const RiscvDecoderParams &p) : InstDecoder(p, &machInst)
{
    ISA *isa = dynamic_cast<ISA*>(p.isa);
    vlen = isa->getVecLenInBits();
    elen = isa->getVecElemLenInBits();
    _hasZcd = isa->reportsExtension("Zcd");
    reset();
}

void Decoder::reset()
{
    InstDecoder::reset();
    aligned = true;
    mid = false;
    machInst = 0;
    emi = 0;
    jvtEntry = 0;
    // Also runs on faults and interrupts. A finished vset stays in
    // machVtype. The spec stack is consumed by squashSpec.
    pendingSpec = false;
    pendingPublished = false;
    vtypeReady = true;
}

void
Decoder::moreBytes(const PCStateBase &pc, Addr fetchPC)
{
    // The MSB of the upper and lower halves of a machine instruction.
    constexpr size_t max_bit = sizeof(machInst) * 8 - 1;
    constexpr size_t mid_bit = sizeof(machInst) * 4 - 1;

    auto inst = letoh(machInst);
    DPRINTF(Decode, "Requesting bytes 0x%08x from address %#x\n", inst,
            fetchPC);

    PCState pc_state = pc.as<PCState>();

    if (GEM5_UNLIKELY(pc_state.zcmtSecondFetch())) {
        if (mid) {
            replaceBits(jvtEntry, sizeof(jvtEntry) * 8 - 1, max_bit + 1, inst);
            mid = false;
            instDone = true;
            outOfBytes = true;
        } else {
            replaceBits(jvtEntry, max_bit, 0, inst);
            mid = (pc_state.rvType() != RV32);
            instDone = (pc_state.rvType() == RV32);
            outOfBytes = true;
        }

        if (instDone && pc_state.rvType() == RV32) {
            jvtEntry = sext<32>(jvtEntry);
        }
        return;
    }

    bool aligned = pc.instAddr() % sizeof(machInst) == 0;
    if (aligned) {
        emi.instBits = inst;
        if (compressed(inst))
            emi.instBits = bits(inst, mid_bit, 0);
        outOfBytes = !compressed(emi);
        instDone = true;
    } else {
        if (mid) {
            assert(bits(emi.instBits, max_bit, mid_bit + 1) == 0);
            replaceBits(emi.instBits, max_bit, mid_bit + 1, inst);
            mid = false;
            outOfBytes = false;
            instDone = true;
        } else {
            emi.instBits = bits(inst, max_bit, mid_bit + 1);
            mid = !compressed(emi);
            outOfBytes = true;
            instDone = compressed(emi);
        }
    }
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    DPRINTF(Decode, "Decoding instruction 0x%08x at address %#x\n",
            mach_inst.instBits, addr);

    auto it = instMap.find(mach_inst);
    if (it != instMap.end()) {
        it->second->size(compressed(mach_inst) ? 2 : 4);
        DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
                it->second->getName(), mach_inst);
        return it->second;
    }

    StaticInstPtr si = decodeInst(mach_inst);
    si->size(compressed(mach_inst) ? 2 : 4);
    instMap[mach_inst] = si;

    DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
            si->getName(), mach_inst);
    return si;
}

StaticInstPtr
Decoder::decode(PCStateBase &_next_pc)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    auto &next_pc = _next_pc.as<PCState>();

    if (GEM5_UNLIKELY(next_pc.zcmtSecondFetch())) {
        return new ZcmtSecondFetchInst(emi, jvtEntry);
    }

    if (compressed(emi)) {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst) / 2);
        next_pc.compressed(true);
    } else {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst));
        next_pc.compressed(false);
    }

    pendingSpec = false;
    pendingPublished = false;
    const VTYPE before = machVtype;
    emi.vtype8 = machVtype & 0xff;
    emi.vill = machVtype.vill;
    emi.rv_type = static_cast<int>(next_pc.rvType());
    emi.has_zcd = _hasZcd;

    StaticInstPtr inst = decode(emi, next_pc.instAddr());
    if (inst->isVectorConfig()) {
        auto *vset = static_cast<VConfOp *>(inst.get());
        pendingSpec = true;
        pendingPrev = before;
        if (vset->vtypeIsImm) {
            pendingPublished = true;
            pendingValue = legalizeVtype(vset->earlyVtype);
            setVtype(pendingValue);
            releaseFetch();
        } else {
            vtypeReady = false;
        }
    }
    return inst;
}

VTYPE
Decoder::legalizeVtype(RegVal requested) const
{
    VTYPE new_vtype = requested;
    const float vflmul = getVflmul(new_vtype.vlmul);
    const uint32_t sew = getSew(new_vtype.vsew);
    const bool vill = !(vflmul >= 0.125f && vflmul <= 8.0f) ||
                      static_cast<float>(sew) >
                          std::min(vflmul, 1.0f) * static_cast<float>(elen) ||
                      bits(requested, 62, 8) != 0;
    if (vill) {
        new_vtype = 0;
        new_vtype.vill = 1;
    }
    return new_vtype;
}

void
Decoder::setVtype(RegVal vtype)
{
    machVtype = vtype;
}

void
Decoder::installYoungestPublished()
{
    for (auto it = specStack.rbegin(); it != specStack.rend(); ++it) {
        if (!it->published) {
            continue;
        }
        machVtype = it->value;
        return;
    }
    if (!specStack.empty()) {
        machVtype = specStack.front().prev;
    }
}

void
Decoder::dropSuperseded()
{
    bool keep = false;
    for (size_t i = specStack.size(); i-- > 0;) {
        if (!specStack[i].published) {
            continue;
        }
        if (!keep) {
            keep = true;
            continue;
        }
        specStack.erase(specStack.begin() + i);
    }
}

void
Decoder::publishExecuted(ThreadContext *tc, InstSeqNum seq, RegVal vtype,
                         uint32_t vl)
{
    for (auto &entry : specStack) {
        if (entry.seq != seq) {
            continue;
        }
        entry.published = true;
        entry.value = vtype;
        // Defer architectural CSR writes until commit so a squashed
        // wrong-path vset cannot pollute vl/vtype (see commitVtype).
        pendingCsrWrites[seq] = {vl, static_cast<RegVal>(vtype)};
        installYoungestPublished();
        dropSuperseded();
        return;
    }
    // Atomic/Timing: no decode-time spec stack entry; execute commits.
    setVtype(vtype);
    // Write via ISA to avoid O3 ThreadContext::conditionalSquash.
    tc->getIsaPtr()->setMiscReg(MISCREG_VL, vl);
    tc->getIsaPtr()->setMiscReg(MISCREG_VTYPE, static_cast<RegVal>(vtype));
}

void
Decoder::noteDecodedSeq(InstSeqNum seq)
{
    if (!pendingSpec) {
        return;
    }
    specStack.push_back({seq, pendingPrev, pendingPublished, pendingValue});
    pendingSpec = false;
    pendingPublished = false;
}

void
Decoder::squashSpec(InstSeqNum seq)
{
    if (pendingSpec) {
        machVtype = pendingPrev;
        pendingSpec = false;
    }
    while (!specStack.empty() && specStack.back().seq > seq) {
        machVtype = specStack.back().prev;
        specStack.pop_back();
    }
    for (auto it = pendingCsrWrites.begin(); it != pendingCsrWrites.end();) {
        if (it->first > seq) {
            it = pendingCsrWrites.erase(it);
        } else {
            ++it;
        }
    }
    vtypeReady = true;
}

void
Decoder::discardSpec(ThreadContext *tc)
{
    pendingSpec = false;
    pendingPublished = false;
    specStack.clear();
    pendingCsrWrites.clear();
    // Resync decode vtype from the committed CSR (also used after cpt
    // restore when machVtype was not serialized).
    setVtype(tc->readMiscReg(MISCREG_VTYPE));
    releaseFetch();
}

void
Decoder::commitVtype(ThreadContext *tc, InstSeqNum seq)
{
    RegVal vtype = machVtype;
    auto it = pendingCsrWrites.find(seq);
    if (it != pendingCsrWrites.end()) {
        // Architectural update happens only for instructions that commit.
        // Use ISA::setMiscReg: O3's ThreadContext::setMiscReg would call
        // conditionalSquash() mid-commit and trip commit asserts.
        tc->getIsaPtr()->setMiscReg(MISCREG_VL, it->second.vl);
        tc->getIsaPtr()->setMiscReg(MISCREG_VTYPE, it->second.vtype);
        vtype = it->second.vtype;
        pendingCsrWrites.erase(it);
    }

    while (!specStack.empty() && specStack.front().seq <= seq) {
        specStack.erase(specStack.begin());
    }
    // A younger register vset may still be waiting on its source.
    // Fetch stays stalled until that vset executes.
    if (specStack.empty()) {
        setVtype(vtype);
        releaseFetch();
    }
}

} // namespace RiscvISA
} // namespace gem5
