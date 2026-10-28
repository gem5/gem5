/*
 * Copyright (c) 2026 The gem5 Authors
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

#include "arch/riscv/fof.hh"

#include "arch/riscv/isa.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"

namespace gem5
{
namespace RiscvISA
{
namespace
{

FoFTable &
fofTable(ThreadContext *tc)
{
    return static_cast<ISA *>(tc->getIsaPtr())->fof();
}

bool
useSeqKey(InstSeqNum sn)
{
    return sn != InvalidInstSeqNum;
}

} // anonymous namespace

void
FoFTable::set(InstSeqNum sn, const StaticInst *si, uint32_t fault_idx,
              bool trim_vl)
{
    // Re-initiate with NoFault must not erase an already-recorded FoF trim
    // for this dynamic / static micro (#3362 class of bug).
    if (useSeqKey(sn)) {
        auto it = bySeq.find(sn);
        if (it != bySeq.end() && it->second.trimVl && !trim_vl)
            return;
        bySeq[sn] = Entry{si, fault_idx, trim_vl};
    } else {
        auto it = bySi.find(si);
        if (it != bySi.end() && it->second.trimVl && !trim_vl)
            return;
        bySi[si] = Entry{si, fault_idx, trim_vl};
    }
}

uint32_t
FoFTable::faultIdx(InstSeqNum sn, const StaticInst *si) const
{
    if (useSeqKey(sn)) {
        auto it = bySeq.find(sn);
        if (it != bySeq.end())
            return it->second.faultIdx;
    } else {
        auto it = bySi.find(si);
        if (it != bySi.end())
            return it->second.faultIdx;
    }
    // Unset: do not truncate the memacc element loop.
    return (uint32_t)-1;
}

uint32_t
FoFTable::reduce(InstSeqNum trim_sn,
                 const std::vector<StaticInstPtr> &load_micros,
                 uint32_t num_load_micros)
{
    const uint32_t n = std::min<uint32_t>(num_load_micros, load_micros.size());
    uint32_t vl = 0;

    if (useSeqKey(trim_sn)) {
        for (uint32_t i = 0; i < n; ++i) {
            InstSeqNum best_sn = 0;
            const Entry *best = nullptr;
            for (const auto &kv : bySeq) {
                // Per-thread table: other harts never appear here. Same-thread
                // sibling micros may have *gaps* in seqNum (SMT fetch), so do
                // not assume they occupy [trim_sn-n, trim_sn). Take the
                // youngest matching StaticInst still older than this trim.
                if (kv.first >= trim_sn)
                    continue;
                if (kv.second.si != load_micros[i].get())
                    continue;
                if (!best || kv.first > best_sn) {
                    best_sn = kv.first;
                    best = &kv.second;
                }
            }
            if (!best)
                continue;
            vl += best->faultIdx;
            const bool stop = best->trimVl;
            bySeq.erase(best_sn);
            if (stop)
                break;
        }
    } else {
        for (uint32_t i = 0; i < n; ++i) {
            const StaticInst *si = load_micros[i].get();
            auto it = bySi.find(si);
            if (it == bySi.end())
                continue;
            vl += it->second.faultIdx;
            const bool stop = it->second.trimVl;
            bySi.erase(it);
            if (stop)
                break;
        }
    }

    return vl;
}

void
FoFTable::clearSeq(InstSeqNum sn)
{
    if (useSeqKey(sn))
        bySeq.erase(sn);
}

void
FoFTable::clear()
{
    bySeq.clear();
    bySi.clear();
}

void
setFoFState(ExecContext *xc, const StaticInst *si, uint32_t fault_idx,
            bool trim_vl)
{
    fofTable(xc->tcBase()).set(xc->getSeqNum(), si, fault_idx, trim_vl);
}

uint32_t
getFoFFaultIdx(ExecContext *xc, const StaticInst *si)
{
    return fofTable(xc->tcBase()).faultIdx(xc->getSeqNum(), si);
}

uint32_t
reduceFoFVl(ExecContext *xc, const std::vector<StaticInstPtr> &load_micros,
            uint32_t num_load_micros)
{
    return fofTable(xc->tcBase())
            .reduce(xc->getSeqNum(), load_micros, num_load_micros);
}

void
clearFoFSeq(ThreadContext *tc, InstSeqNum sn)
{
    fofTable(tc).clearSeq(sn);
}

} // namespace RiscvISA
} // namespace gem5
