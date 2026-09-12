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

#ifndef __ARCH_RISCV_FOF_HH__
#define __ARCH_RISCV_FOF_HH__

#include <algorithm>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "cpu/inst_seq.hh"
#include "cpu/static_inst_fwd.hh"

namespace gem5
{

class ExecContext;
class ThreadContext;

namespace RiscvISA
{

/**
 * Fault-only-first (FoF) results for vector unit-stride / segment loads.
 *
 * Lives on RiscvISA::ISA (not ExecContext / StaticInst) so:
 * - Generic CPU interfaces stay ISA-agnostic
 * - O3 keys by DynInst seqNum (survives commit; cleared on squash)
 * - In-order CPUs (getSeqNum() == InvalidInstSeqNum) key by StaticInst*
 *
 * O3 seqNums are CPU-wide: SMT fetch can insert another hart's instructions
 * between micros of the same macro, so sibling FoF micros are not necessarily
 * consecutive. Isolation from other harts comes from a per-thread ISA/table,
 * not from assuming sn in [trim-n, trim).
 */
class FoFTable
{
  public:
    struct Entry
    {
        const StaticInst *si = nullptr;
        uint32_t faultIdx = 0;
        bool trimVl = false;
    };

    void set(InstSeqNum sn, const StaticInst *si, uint32_t fault_idx,
             bool trim_vl);
    uint32_t faultIdx(InstSeqNum sn, const StaticInst *si) const;
    uint32_t reduce(InstSeqNum trim_sn,
                    const std::vector<StaticInstPtr> &load_micros,
                    uint32_t num_load_micros);
    void clearSeq(InstSeqNum sn);
    void clear();

  private:
    // O3 / any model that reports a real dynamic seqNum.
    std::unordered_map<InstSeqNum, Entry> bySeq;
    // In-order models: StaticInst* is unique per micro within a macro.
    std::unordered_map<const StaticInst *, Entry> bySi;
};

/** Sentinel: ExecContext has no dynamic sequence number (Simple/Minor/...). */
static constexpr InstSeqNum InvalidInstSeqNum =
    std::numeric_limits<InstSeqNum>::max();

void setFoFState(ExecContext *xc, const StaticInst *si, uint32_t fault_idx,
                 bool trim_vl);
uint32_t getFoFFaultIdx(ExecContext *xc, const StaticInst *si);
uint32_t reduceFoFVl(ExecContext *xc,
                     const std::vector<StaticInstPtr> &load_micros,
                     uint32_t num_load_micros);
void clearFoFSeq(ThreadContext *tc, InstSeqNum sn);

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_FOF_HH__
